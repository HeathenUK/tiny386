// incomplete x87 emulation, use at your own risk!
// no exception, no tag word, no float80
#include "fpu.h"
#include "i386.h"
#include <string.h>
#include <stdlib.h>
#include <assert.h>

#include "esp_attr.h"

typedef struct {
	uint32_t mant0;
	uint32_t mant1;
	uint16_t high;
} F80;

union union64 {
	double f;
	uint64_t i;
};

union union32 {
	float f;
	uint32_t i;
};

// Use float32 on ESP32 for hardware FPU acceleration (no double-precision HW)
#define USE_FLOAT32_FPU

#ifdef USE_FLOAT32_FPU
// Float32-based FPU emulation - faster on chips without double-precision HW
// Trades precision for speed (32-bit mantissa vs 64-bit)
#include <math.h>
#define BIAS80 16383
#define BIAS32 127
#define fpreal float
#define FPCONST(x) x##f
// Math function wrappers for float32
#define fppow powf
#define fplog2 log2f
#define fpsin sinf
#define fpcos cosf
#define fptan tanf
#define fpatan2 atan2f
#define fpsqrt sqrtf
#define fpfabs fabsf
#define fpfrexp frexpf
#define fpcopysign copysignf
#define fpsignbit(x) signbit(x)
#define fpisnan(x) isnan(x)
#define fpisfinite(x) isfinite(x)
#define fpisunordered(a,b) isunordered(a,b)
#define fptrunc truncf
#define fpround_fn roundf

// Fast polynomial approximations for transcendental functions
// Trading precision for speed - acceptable for DOS-era software

// Fast 2^x approximation using bit manipulation
// Valid for reasonable input range, avoids slow powf()
static inline float IRAM_ATTR fast_pow2f(float x)
{
	// Handle edge cases
	if (x < -126.0f) return 0.0f;
	if (x > 128.0f) return INFINITY;

	int i = (int)x;
	float f = x - (float)i;
	if (f < 0) { f += 1.0f; i--; }  // Handle negative fractional part

	// Polynomial approximation for 2^f where 0 <= f < 1
	// 2^f ≈ 1 + f*(ln2 + f*(ln2^2/2 + f*ln2^3/6))
	float p = 1.0f + f * (0.693147f + f * (0.240226f + f * 0.055504f));

	// Scale by 2^i using IEEE float bit manipulation
	union { float f; uint32_t i; } u = { .f = p };
	u.i += (uint32_t)i << 23;
	return u.f;
}

// Fast log2 approximation
static inline float IRAM_ATTR fast_log2f(float x)
{
	if (x <= 0.0f) return -INFINITY;

	union { float f; uint32_t i; } u = { .f = x };
	int exp = ((u.i >> 23) & 0xFF) - 127;
	u.i = (u.i & 0x7FFFFF) | (127 << 23);  // Extract mantissa as 1.xxx

	float m = u.f;
	// Polynomial approximation for log2(m) where 1 <= m < 2
	// log2(m) ≈ (m-1) * (1.4426950408 - 0.7213475 * (m-1) + 0.4809 * (m-1)^2)
	float t = m - 1.0f;
	return (float)exp + t * (1.4426950f - t * (0.7213475f - t * 0.4809f));
}

// Fast sine approximation using Bhaskara I formula (7th century)
// Very fast, ~1.8% max error, good enough for DOS games
static inline float IRAM_ATTR fast_sinf(float x)
{
	// Reduce to [0, 2*pi]
	const float PI = 3.14159265f;
	const float TWO_PI = 6.28318531f;
	const float INV_TWO_PI = 0.15915494f;

	x = x - TWO_PI * floorf(x * INV_TWO_PI);

	// Reduce to [0, pi] and track sign
	int sign = 1;
	if (x > PI) {
		x -= PI;
		sign = -1;
	}

	// Bhaskara I formula: sin(x) ≈ 16x(π-x) / (5π² - 4x(π-x))
	float xpi = x * (PI - x);
	return sign * (16.0f * xpi) / (49.348f - 4.0f * xpi);
}

// Fast cosine via phase-shifted sine
static inline float IRAM_ATTR fast_cosf(float x)
{
	return fast_sinf(x + 1.5707963f);  // cos(x) = sin(x + π/2)
}

// Fast tangent
static inline float IRAM_ATTR fast_tanf(float x)
{
	float s = fast_sinf(x);
	float c = fast_cosf(x);
	if (fabsf(c) < 1e-10f) return copysignf(INFINITY, s);
	return s / c;
}

// Override libm calls with fast versions for ESP32
#undef fppow
#undef fplog2
#undef fpsin
#undef fpcos
#undef fptan
#define fppow(base, exp) fast_pow2f((exp) * fast_log2f(base))
#define fplog2 fast_log2f
#define fpsin fast_sinf
#define fpcos fast_cosf
#define fptan fast_tanf

static F80 tof80(float val)
{
	union union32 u = { .f = val };
	uint32_t v = u.i;
	int sign = v >> 31;
	int exp = (v >> 23) & 0xFF;
	uint64_t mant80 = (uint64_t)(v & 0x7FFFFF) << 40;  // 23-bit to 63-bit
	if (exp == 0) {
		// zero or subnormal
		if (mant80 != 0) {
			// subnormal
			int shift = 64 - __builtin_ffsll(mant80);
			mant80 <<= shift;
			exp += BIAS80 - BIAS32 + 1 - shift;
		}
	} else if (exp == 0xFF) {
		// inf or nan
		mant80 |= (uint64_t) 1 << 63;
		exp = 0x7fff;
	} else {
		// normal
		mant80 |= (uint64_t) 1 << 63;
		exp += BIAS80 - BIAS32;
	}

	F80 res;
	res.high = (sign << 15) | (uint16_t) exp;
	res.mant1 = mant80 >> 32;
	res.mant0 = mant80;
	return res;
}

static float fromf80(F80 f80)
{
	int sign = f80.high >> 15;
	int exp = f80.high & 0x7fff;
	uint64_t mant80 = f80.mant1;
	mant80 = (mant80 << 32) | f80.mant0;

	uint32_t mant32;
	if (exp == 0) {
		// f80 subnormal/zero => f32 zero
		mant32 = 0;
		exp = 0;
	} else if (exp == 0x7fff) {
		// inf or nan
		exp = 0xFF;
		mant32 = mant80 >> 40;
		if (mant32 == 0 && mant80 != 0)
			mant32 = 1;
	} else {
		// normal
		exp += BIAS32 - BIAS80;
		if (exp <= -23) {
			// => f32 zero
			exp = 0;
			mant32 = 0;
		} else if (exp <= 0) {
			// => f32 subnormal
			mant32 = mant80 >> (41 - exp);
			exp = 0;
		} else if (exp >= 0xFF) {
			// => f32 inf
			exp = 0xFF;
			mant32 = 0;
		} else {
			mant32 = mant80 >> 40;
			mant32 &= 0x7FFFFF;
		}
	}

	uint32_t res = ((uint32_t) sign << 31) | ((uint32_t) exp << 23) | mant32;
	union union32 u = { .i = res };
	return u.f;
}

#elif !defined(USE_FLOAT80)
// Float64-based FPU emulation - good precision, slow on chips without double HW
#include <math.h>
#define BIAS80 16383
#define BIAS64 1023
#define fpreal double
#define FPCONST(x) x
// Math function wrappers for float64
#define fppow pow
#define fplog2 log2
#define fpsin sin
#define fpcos cos
#define fptan tan
#define fpatan2 atan2
#define fpsqrt sqrt
#define fpfabs fabs
#define fpfrexp frexp
#define fpcopysign copysign
#define fpsignbit(x) signbit(x)
#define fpisnan(x) isnan(x)
#define fpisfinite(x) isfinite(x)
#define fpisunordered(a,b) isunordered(a,b)
#define fptrunc trunc
#define fpround_fn round

static F80 tof80(double val)
{
	union union64 u = { .f = val };
	uint64_t v = u.i;
	int sign = v >> 63;
	int exp = (v >> 52) & ((1 << 11) - 1);
	uint64_t mant80 = (v & (((uint64_t) 1 << 52) - 1)) << 11;
	if (exp == 0) {
		// zero or subnormal
		if (mant80 != 0) {
			// subnormal
			int shift = 64 - __builtin_ffsll(mant80);
			mant80 <<= shift;
			exp += BIAS80 - BIAS64 + 1 - shift;
		}
	} else if (exp == (1 << 11) - 1) {
		// inf or nan
		mant80 |= (uint64_t) 1 << 63;
		exp = 0x7fff;
	} else {
		// normal
		mant80 |= (uint64_t) 1 << 63;
		exp += BIAS80 - BIAS64;
	}

	F80 res;
	res.high = (sign << 15) | (uint16_t) exp;
	res.mant1 = mant80 >> 32;
	res.mant0 = mant80;
	return res;
}

static double fromf80(F80 f80)
{
	int sign = f80.high >> 15;
	int exp = f80.high & 0x7fff;
	uint64_t mant80 = f80.mant1;
	mant80 = (mant80 << 32) | f80.mant0;

	uint64_t mant64;
	if (exp == 0) {
		// f80 subnormal/zero => f64 zero
		mant64 = 0;
	} else if (exp == 0x7fff) {
		// inf or nan
		exp = (1 << 11) - 1;
		mant64 = mant80 >> 11;
		if (mant64 == 0 && mant80 != 0)
			mant64 = 1;
	} else {
		// normal
		exp += BIAS64 - BIAS80;
		if (exp <= -52) {
			// => f64 zero
			exp = 0;
			mant64 = 0;
		} else if (exp <= 0) {
			// => f64 subnormal
			mant64 = mant80 >> (12 - exp);
			exp = 0;
		} else if (exp >= (1 << 11) - 1) {
			// => f64 inf
			exp = (1 << 11) - 1;
			mant64 = 0;
		} else {
			mant64 = mant80 >> 11;
			mant64 &= ((uint64_t) 1 << 52) - 1;
		}
	}

	uint64_t res = ((uint64_t) sign << 63) | ((uint64_t) exp << 52) | mant64;
	union union64 u = { .i = res };
	return u.f;
}
#else
// if USE_FLOAT80 is defined, we have native f80 support (e.g. x87)
#include <tgmath.h>
#define fpreal long double
#define FPCONST(x) x##L
// Math function wrappers for long double
#define fppow powl
#define fplog2 log2l
#define fpsin sinl
#define fpcos cosl
#define fptan tanl
#define fpatan2 atan2l
#define fpsqrt sqrtl
#define fpfabs fabsl
#define fpfrexp frexpl
#define fpcopysign copysignl
#define fpsignbit(x) signbit(x)
#define fpisnan(x) isnan(x)
#define fpisfinite(x) isfinite(x)
#define fpisunordered(a,b) isunordered(a,b)
#define fptrunc truncl
#define fpround_fn roundl

union union80 {
	__float80 f;
	F80 f80;
};

static F80 tof80(long double val)
{
	union union80 u = { .f = val };
	return u.f80;
}

static long double fromf80(F80 f80)
{
	union union80 u = { .f80 = f80 };
	return u.f;
}

#define sincos sincosl
#endif

typedef union {
	uint32_t u32v[4];
	float f32v[4];
} UXMM;

struct FPU {
	u16 cw, sw; // TODO: tag word
	unsigned int top;

	fpreal st[8];
	F80 rawst[8];
	u8 rawtagr;
	u8 rawtagw;

#ifdef I386_ENABLE_SSE
	u32 mxcsr;
	UXMM xmm[8];
#endif
};

static u16 getsw(FPU *fpu)
{
	return (fpu->sw & 0xc7ff) | (fpu->top << 11);
}

static void setsw(FPU *fpu, u16 sw)
{
	fpu->sw = sw;
	fpu->top = (sw >> 11) & 7;
}

FPU *fpu_new()
{
	FPU *fpu = malloc(sizeof(FPU));
	memset(fpu, 0, sizeof(FPU));
	fpu->cw = 0x40;
	return fpu;
}

void fpu_delete(FPU *fpu)
{
	free(fpu);
}

#ifdef USE_FLOAT32_FPU
// Fast path for ESP32 - skip F80 tracking, just use float32 directly
static inline fpreal fpget(FPU *fpu, int i)
{
	return fpu->st[(fpu->top + i) & 7];
}

static inline void fpset(FPU *fpu, int i, fpreal val)
{
	fpu->st[(fpu->top + i) & 7] = val;
}
#else
static fpreal fpget(FPU *fpu, int i)
{
	unsigned int idx = (fpu->top + i) & 7;
	unsigned int mask = 1 << idx;
	if (!(fpu->rawtagr & mask) && !(fpu->rawtagw & mask)) {
		fpu->st[idx] = fromf80(fpu->rawst[idx]);
		fpu->rawtagr |= mask;
	}
	return fpu->st[idx];
}

static void fpset(FPU *fpu, int i, fpreal val)
{
	unsigned int idx = (fpu->top + i) & 7;
	unsigned int mask = 1 << idx;
	fpu->st[idx] = val;
	fpu->rawtagw |= mask;
}
#endif

static inline void fppush(FPU *fpu, fpreal val)
{
	fpu->top = (fpu->top - 1) & 7;
	// set tw
	fpset(fpu, 0, val);
}

static inline void fppop(FPU *fpu)
{
	fpu->top = (fpu->top + 1) & 7;
	// set tw
}

static bool IRAM_ATTR fploadf32(void *cpu, int seg, uword addr, fpreal *res)
{
	union union32 u;
	if(!cpu_load32(cpu, seg, addr, &u.i))
		return false;
	*res = u.f;
	return true;
}

static bool IRAM_ATTR fploadf64(void *cpu, int seg, uword addr, fpreal *res)
{
	uint64_t v;
	if(!cpu_load64(cpu, seg, addr, &v))
		return false;
	union union64 u = { .i = v };
	*res = u.f;
	return true;
}

static bool fploadf80(void *cpu, int seg, uword addr, fpreal *res)
{
	F80 f80;
	if(!cpu_load32(cpu, seg, addr, &f80.mant0))
		return false;
	if(!cpu_load32(cpu, seg, addr + 4, &f80.mant1))
		return false;
	if(!cpu_load16(cpu, seg, addr + 8, &f80.high))
		return false;
	*res = fromf80(f80);
	return true;
}

static bool fploadi16(void *cpu, int seg, uword addr, fpreal *res)
{
	u16 v;
	if(!cpu_load16(cpu, seg, addr, &v))
		return false;
	*res = (s16) v;
	return true;
}

static bool IRAM_ATTR fploadi32(void *cpu, int seg, uword addr, fpreal *res)
{
	u32 v;
	if(!cpu_load32(cpu, seg, addr, &v))
		return false;
	*res = (s32) v;
	return true;
}

static bool IRAM_ATTR fploadi64(void *cpu, int seg, uword addr, fpreal *res)
{
	uint64_t v;
	if(!cpu_load64(cpu, seg, addr, &v))
		return false;
	*res = (int64_t)v;
	return true;
}

static int bcd100(u8 b)
{
	return b - (6 * (b >> 4));
}

static bool fploadbcd(void *cpu, int seg, uword addr, fpreal *res)
{
	u32 lo, mi;
	u16 hi;
	if(!cpu_load32(cpu, seg, addr, &lo))
		return false;
	if(!cpu_load32(cpu, seg, addr + 4, &mi))
		return false;
	if(!cpu_load16(cpu, seg, addr + 8, &hi))
		return false;

	uint64_t val = 0;
	int sign = hi & 0x8000;
	hi &= 0x7FFF;
	for (int i = 0; i < 4; i++) {
		val = val * 100 + bcd100(lo);
		lo >>= 8;
	}
	for (int i = 0; i < 4; i++) {
		val = val * 100 + bcd100(mi);
		mi >>= 8;
	}
	for (int i = 0; i < 2; i++) {
		val = val * 100 + bcd100(hi);
		hi >>= 8;
	}
	*res = copysignf((float)val, sign ? -1.0f : 1.0f);
	return true;
}

static bool IRAM_ATTR fpstoref32(void *cpu, int seg, uword addr, fpreal val)
{
	union union32 u = { .f = val };
	u32 v = u.i;
	if (!cpu_store32(cpu, seg, addr, v))
		return false;
	return true;
}

static bool IRAM_ATTR fpstoref64(void *cpu, int seg, uword addr, fpreal val)
{
	union union64 u = { .f = val };
	return cpu_store64(cpu, seg, addr, u.i);
}

static bool fpstoref80(void *cpu, int seg, uword addr, fpreal val)
{
	F80 f80 = tof80(val);
	if (!cpu_store32(cpu, seg, addr, f80.mant0))
		return false;
	if (!cpu_store32(cpu, seg, addr + 4, f80.mant1))
		return false;
	if (!cpu_store16(cpu, seg, addr + 8, f80.high))
		return false;
	return true;
}

static fpreal IRAM_ATTR fpround(fpreal x, int rc)
{
	switch (rc) {
	case 0:
#ifdef USE_FLOAT32_FPU
		return nearbyintf(x);
#else
		return nearbyint(x);
#endif
	case 1:
#ifdef USE_FLOAT32_FPU
		return floorf(x);
#else
		return floor(x);
#endif
	case 2:
#ifdef USE_FLOAT32_FPU
		return ceilf(x);
#else
		return ceil(x);
#endif
	case 3:
#ifdef USE_FLOAT32_FPU
		return truncf(x);
#else
		return trunc(x);
#endif
	}
	return x;
}

static bool fpstorei16(void *cpu, int seg, uword addr, fpreal val)
{
	s16 v = val < FPCONST(32768.0) && val >= FPCONST(-32768.0) ? (s16) val : 0x8000;
	if (!cpu_store16(cpu, seg, addr, v))
		return false;
	return true;
}

static bool IRAM_ATTR fpstorei32(void *cpu, int seg, uword addr, fpreal val)
{
	s32 v = val < FPCONST(2147483648.0) && val >= FPCONST(-2147483648.0) ? (s32) val : 0x80000000;
	if (!cpu_store32(cpu, seg, addr, v))
		return false;
	return true;
}

static bool IRAM_ATTR fpstorei64(void *cpu, int seg, uword addr, fpreal val)
{
	int64_t v = val < FPCONST(9223372036854775808.0) && val >= FPCONST(-9223372036854775808.0) ?
		(int64_t) val : 0x8000000000000000ll;
	return cpu_store64(cpu, seg, addr, (uint64_t)v);
}

static bool fpstorebcd(void *cpu, int seg, uword addr, fpreal val)
{
	int64_t v = val < FPCONST(9223372036854775808.0) && val >= FPCONST(-9223372036854775808.0) ?
		(int64_t) val : 0x8000000000000000ll;
	int sign = 0;
	if (v < 0) {
		sign = 1;
		v = -v;
	}

	for (int i = 0; i < 9; i++) {
		int res = (v % 10) | ((v / 10) % 10) << 4;
		v /= 100;
		if (!cpu_store8(cpu, seg, addr + i, res))
			return false;
	}

	int res = (v % 10) | ((v / 10) % 10) << 4;
	if (sign)
		res |= 0x80;
	if (!cpu_store8(cpu, seg, addr + 9, res))
		return false;
	return true;
}

enum {
	C0 = 0x100,
	C1 = 0x200,
	C2 = 0x400,
	C3 = 0x4000,
};

static void IRAM_ATTR fparith(FPU *fpu, int group, unsigned int d, fpreal a, fpreal b)
{
	fpreal c;
	switch (group) {
	case 0: // FADD
		c = a + b;
		break;
	case 1: // FMUL
		c = a * b;
		break;
	case 2: // FCOM
	case 3: // FCOMP
		if (fpisunordered(a, b)) {
			fpu->sw |= C0 | C2 | C3;
		} else if (a == b) {
			fpu->sw |= C3;
			fpu->sw &= ~(C0 | C2);
		} else if (a < b) {
			fpu->sw |= C0;
			fpu->sw &= ~(C2 | C3);
		} else {
			fpu->sw &= ~(C0 | C2 | C3);
		}
		break;
	case 4: // FSUB
		c = a - b;
		break;
	case 5: // FSUBR
		c = b - a;
		break;
	case 6: // FDIV
		c = a / b;
		break;
	case 7: // FDIVR
		c = b / a;
		break;
	default:
		assert(false);
	}
	if (group == 3) {
		fppop(fpu);
	} else if (group != 2) {
		fpset(fpu, d, c);
	}
}

bool IRAM_ATTR fpu_exec2(FPU *fpu, void *cpu, bool opsz16, int op, int group, int seg, uint32_t addr)
{
	fpreal a;
	switch (op) {
	case 0: {
		a = fpget(fpu, 0);
		fpreal b;
		if (!fploadf32(cpu, seg, addr, &b)) {
			return false;
		}
		fparith(fpu, group, 0, a, b);
		break;
	}
	case 1: {
		switch (group) {
		case 0: { // FLD float32
			if (!fploadf32(cpu, seg, addr, &a)) {
				return false;
			}
			fppush(fpu, a);
			break;
		}
		case 1:
			cpu_setexc(cpu, 6, 0);
			return false;
		case 2: { // FST float32
			a = fpget(fpu, 0);
			if (!fpstoref32(cpu, seg, addr, a)) {
				return false;
			}
			break;
		}
		case 3: { // FSTP float32
			a = fpget(fpu, 0);
			if (!fpstoref32(cpu, seg, addr, a)) {
				return false;
			}
			fppop(fpu);
			break;
		}
		case 4: // FLDENV
			if (opsz16) {
				if(!cpu_load16(cpu, seg, addr, &(fpu->cw)))
					return false;
				u16 sw;
				if(!cpu_load16(cpu, seg, addr + 2, &sw))
					return false;
				setsw(fpu, sw);
			} else {
				if(!cpu_load16(cpu, seg, addr, &(fpu->cw)))
					return false;
				u16 sw;
				if(!cpu_load16(cpu, seg, addr + 4, &sw))
					return false;
				setsw(fpu, sw);
			}
			break;
		case 5: // FLDCW
			if(!cpu_load16(cpu, seg, addr, &(fpu->cw)))
				return false;
			break;
		case 6: // FNSTENV
			if (opsz16) {
				if(!cpu_store16(cpu, seg, addr, fpu->cw))
					return false;
				u16 sw = getsw(fpu);
				if(!cpu_store16(cpu, seg, addr + 2, sw))
					return false;
				if(!cpu_store16(cpu, seg, addr + 4, 0 /* tw */))
					return false;
			} else {
				if(!cpu_store32(cpu, seg, addr, fpu->cw))
					return false;
				u16 sw = getsw(fpu);
				if(!cpu_store32(cpu, seg, addr + 4, sw))
					return false;
				if(!cpu_store32(cpu, seg, addr + 8, 0 /* tw */))
					return false;
			}
			break;
		case 7: // FNSTCW
			if(!cpu_store16(cpu, seg, addr, fpu->cw))
				return false;
			break;
		}
		break;
	}
	case 2: {
		a = fpget(fpu, 0);
		fpreal b;
		bool r = fploadi32(cpu, seg, addr, &b);
		if (!r)
			return false;
		fparith(fpu, group, 0, a, b);
		break;
	}
	case 3:
		switch (group) {
		case 0: { // FILD int32
			if (!fploadi32(cpu, seg, addr, &a)) {
				return false;
			}
			fppush(fpu, a);
			break;
		}
		case 1: // FISTTP int32
		case 2: // FIST int32
		case 3: // FISTP int32
		{
			a = fpget(fpu, 0);
			if (group != 1) {
				int rc = (fpu->cw >> 10) & 3;
				a = fpround(a, rc);
			}
			if (!fpstorei32(cpu, seg, addr, a)) {
				return false;
			}
			if (group != 2)
				fppop(fpu);
			break;
		}
		case 4:
			cpu_setexc(cpu, 6, 0);
			return false;
		case 5: { // FLD float80
			if (!fploadf80(cpu, seg, addr, &a)) {
				return false;
			}
			fppush(fpu, a);
			break;
		}
		case 6:
			cpu_setexc(cpu, 6, 0);
			return false;
		case 7: { // FSTP float80
			a = fpget(fpu, 0);
			if (!fpstoref80(cpu, seg, addr, a)) {
				return false;
			}
			fppop(fpu);
			break;
		}
		}
		break;
	case 4: {
		a = fpget(fpu, 0);
		fpreal b;
		bool r = fploadf64(cpu, seg, addr, &b);
		if (!r)
			return false;
		fparith(fpu, group, 0, a, b);
		break;
	}
	case 5:
		switch (group) {
		case 0: { // FLD float64
			if (!fploadf64(cpu, seg, addr, &a)) {
				return false;
			}
			fppush(fpu, a);
			break;
		}
		case 1: { // FISTTP int64
			a = fpget(fpu, 0);
			if (!fpstorei64(cpu, seg, addr, a)) {
				return false;
			}
			fppop(fpu);
			break;
		}
		case 2: { // FST float64
			a = fpget(fpu, 0);
			if (!fpstoref64(cpu, seg, addr, a)) {
				return false;
			}
			break;
		}
		case 3: { // FSTP float64
			a = fpget(fpu, 0);
			if (!fpstoref64(cpu, seg, addr, a)) {
				return false;
			}
			fppop(fpu);
			break;
		}
		case 4: { // FRSTOR
			uword start = addr;
			if (opsz16) {
				if (!cpu_load16(cpu, seg, addr, &(fpu->cw)))
					return false;
				u16 sw;
				if (!cpu_load16(cpu, seg, addr + 2, &sw))
					return false;
				setsw(fpu, sw);
				start += 14;
			} else {
				if (!cpu_load16(cpu, seg, addr, &(fpu->cw)))
					return false;
				u16 sw;
				if (!cpu_load16(cpu, seg, addr + 4, &sw))
					return false;
				setsw(fpu, sw);
				start += 28;
			}
			for (int j = 0; j < 8; j++) {
				if (!cpu_load32(cpu, seg, start,
						&fpu->rawst[j].mant0))
					return false;
				if (!cpu_load32(cpu, seg, start + 4,
						&fpu->rawst[j].mant1))
					return false;
				if (!cpu_load16(cpu, seg, start + 8,
						&fpu->rawst[j].high))
					return false;
				fpu->rawtagr &= ~(1 << j);
				fpu->rawtagw &= ~(1 << j);
				start += 10;
			}
			break;
		}
		case 5:
			cpu_setexc(cpu, 6, 0);
			return false;
		case 6: { // FNSAVE
			uword start = addr;
			if (opsz16) {
				if(!cpu_store16(cpu, seg, addr, fpu->cw))
					return false;
				u16 sw = getsw(fpu);
				if(!cpu_store16(cpu, seg, addr + 2, sw))
					return false;
				if(!cpu_store16(cpu, seg, addr + 4, 0 /* tw */))
					return false;
				start += 14;
			} else {
				if(!cpu_store32(cpu, seg, addr, fpu->cw))
					return false;
				u16 sw = getsw(fpu);
				if(!cpu_store32(cpu, seg, addr + 4, sw))
					return false;
				if(!cpu_store32(cpu, seg, addr + 8, 0 /* tw */))
					return false;
				start += 28;
			}
			for (int j = 0; j < 8; j++) {
				if (fpu->rawtagw & (1 << j)) {
					fpu->rawst[j] = tof80(fpu->st[j]);
					fpu->rawtagw &= ~(1 << j);
				}
				if (!cpu_store32(cpu, seg, start,
						 fpu->rawst[j].mant0))
					return false;
				if (!cpu_store32(cpu, seg, start + 4,
						 fpu->rawst[j].mant1))
					return false;
				if (!cpu_store16(cpu, seg, start + 8,
						 fpu->rawst[j].high))
					return false;
				start += 10;
			}
			fpu->sw = 0;
			fpu->top = 0;
			fpu->cw = 0x37f;
			break;
		}
		case 7: // FNSTSW
			if (!cpu_store16(cpu, seg, addr, getsw(fpu)))
				return false;
			break;
		}
		break;
	case 6: {
		a = fpget(fpu, 0);
		fpreal b;
		bool r = fploadi16(cpu, seg, addr, &b);
		if (!r)
			return false;
		fparith(fpu, group, 0, a, b);
		break;
	}
	case 7:
		switch (group) {
		case 0: { // FILD int16
			if (!fploadi16(cpu, seg, addr, &a)) {
				return false;
			}
			fppush(fpu, a);
			break;
		}
		case 1: // FISTTP int16
		case 2: // FIST int16
		case 3: // FISTP int16
		{
			a = fpget(fpu, 0);
			if (group != 1) {
				int rc = (fpu->cw >> 10) & 3;
				a = fpround(a, rc);
			}
			if (!fpstorei16(cpu, seg, addr, a)) {
				return false;
			}
			if (group != 2)
				fppop(fpu);
			break;
		}
		case 4: { // FBLD
			if (!fploadbcd(cpu, seg, addr, &a)) {
				return false;
			}
			fppush(fpu, a);
			break;
		}
		case 5: { // FILD int64
			if (!fploadi64(cpu, seg, addr, &a)) {
				return false;
			}
			fppush(fpu, a);
			break;
		}
		case 6: { // FBSTP
			int rc = (fpu->cw >> 10) & 3;
			a = fpget(fpu, 0);
			a = fpround(a, rc);
			if (!fpstorebcd(cpu, seg, addr, a)) {
				return false;
			}
			fppop(fpu);
			break;
		}
		case 7: { // FISTP int64
			int rc = (fpu->cw >> 10) & 3;
			a = fpget(fpu, 0);
			a = fpround(a, rc);
			if (!fpstorei64(cpu, seg, addr, a)) {
				return false;
			}
			fppop(fpu);
			break;
		}
		}
		break;
	}
	return true;
}

#define PI		FPCONST(3.141592653589793238462)
#define L2E		FPCONST(1.4426950408889634073605)
#define L2T		FPCONST(3.3219280948873623478693)
#define LN2		FPCONST(0.69314718055994530941683)
#define LG2		FPCONST(0.30102999566398119521379)

enum {
	CF = 0x1,
	PF = 0x4,
	AF = 0x10,
	ZF = 0x40,
	SF = 0x80,
	OF = 0x800,
};

static bool cmov_cond(FPU *fpu, void *cpu, int i)
{
	uword flags = cpu_getflags(cpu);
	switch(i) {
	case 0x0: return flags & CF;
	case 0x1: return flags & ZF;
	case 0x2: return flags & (CF | ZF);
	case 0x3: return flags & PF;
	default: assert(false);
	}
}

static void ucomi(FPU *fpu, void *cpu, int i)
{
	fpreal a = fpget(fpu, 0);
	fpreal b = fpget(fpu, i);
	if (fpisunordered(a, b)) {
		cpu_setflags(cpu, ZF | PF | CF, 0);
	} else if (a == b) {
		cpu_setflags(cpu, ZF, PF | CF);
	} else if (a < b) {
		cpu_setflags(cpu, CF, ZF | PF);
	} else {
		cpu_setflags(cpu, 0, ZF | PF | CF);
	}
}

bool IRAM_ATTR fpu_exec1(FPU *fpu, void *cpu, int op, int group, unsigned int i)
{
	switch (op) {
	case 0: {
		fpreal a = fpget(fpu, 0);
		fpreal b = fpget(fpu, i);
		fparith(fpu, group, 0, a, b);
		break;
	}
	case 1: {
		fpreal temp, temp2;
		switch (group) {
		case 0: // FLD
			temp = fpget(fpu, i);
			fppush(fpu, temp);
			break;
		case 1: // FXCH
			temp = fpget(fpu, i);
			temp2 = fpget(fpu, 0);
			fpset(fpu, i, temp2);
			fpset(fpu, 0, temp);
			break;
		case 2: // FNOP
			break;
		case 3: // FSTP
			temp = fpget(fpu, 0);
			fpset(fpu, i, temp);
			fppop(fpu);
			break;
		case 4:
			temp = fpget(fpu, 0);
			switch (i) {
			case 0: // FCHS
				fpset(fpu, 0, fpcopysign(temp,
						       fpsignbit(temp) ? FPCONST(1.0) : FPCONST(-1.0)));
				break;
			case 1: // FABS
				fpset(fpu, 0, fpfabs(temp));
				break;
			case 2:
			case 3:
			case 6:
			case 7:
				cpu_setexc(cpu, 6, 0);
				return false;
			case 4: // FTST
				fparith(fpu, 2, 0, temp, 0.0);
				break;
			case 5: // FXAM
				if (fpsignbit(temp)) {
					fpu->sw |= C1;
				} else {
					fpu->sw &= ~C1;
				}
				if(temp == 0.0) {
					fpu->sw |= C3;
					fpu->sw &= ~(C0 | C2);
				} else if (fpisnan(temp)) {
					fpu->sw |= C0;
					fpu->sw &= ~(C2 | C3);
				} else if (fpisfinite(temp)) {
					fpu->sw |= C2;
					fpu->sw &= ~(C0 | C3);
				} else {
					fpu->sw |= (C0 | C2);
					fpu->sw &= ~C3;
				}
				break;
			}
			break;
		case 5:
			switch (i) {
			case 0: // FLD1
				fppush(fpu, 1.0);
				break;
			case 1: // FLDL2T
				fppush(fpu, L2T);
				break;
			case 2: // FLDL2E
				fppush(fpu, L2E);
				break;
			case 3: // FLDPI
				fppush(fpu, PI);
				break;
			case 4: // FLDLG2
				fppush(fpu, LG2);
				break;
			case 5: // FLDLN2
				fppush(fpu, LN2);
				break;
			case 6: // FLDZ
				fppush(fpu, 0.0);
				break;
			case 7:
				cpu_setexc(cpu, 6, 0);
				return false;
			}
			break;
		case 6:
			temp = fpget(fpu, 0);
			switch (i) {
			case 0: // F2XM1
				fpset(fpu, 0, fppow(FPCONST(2.0), temp) - 1);
				break;
			case 1: // FYL2X
				temp2 = fpget(fpu, 1);
				fpset(fpu, 1, temp2 * fplog2(temp));
				fppop(fpu);
				break;
			case 2: // FPTAN
				fpset(fpu, 0, fptan(temp));
				fppush(fpu, 1.0);
				fpu->sw &= ~C2;
				break;
			case 3: // FPATAN
				temp2 = fpget(fpu, 1);
				fpset(fpu, 1, fpatan2(temp2, temp));
				fppop(fpu);
				break;
			case 4: { // FXTRACT
				int exp;
				fpreal mant = fpfrexp(temp, &exp);
				mant *= 2;
				exp--;
				fpset(fpu, 0, exp);
				fppush(fpu, mant);
				break;
			}
			case 5: { // FPREM1
				temp2 = fpget(fpu, 1);
				int64_t q = nearbyint(temp / temp2); // TODO: overflow
				fpset(fpu, 0, temp - q * temp2);
				fpu->sw &= ~C2;
				if (q & 1) fpu->sw |= C1; else fpu->sw &= ~C1;
				if (q & 2) fpu->sw |= C3; else fpu->sw &= ~C3;
				if (q & 4) fpu->sw |= C0; else fpu->sw &= ~C0;
				break;
			}
			case 6: // FDECSTP
				fpu->top = (fpu->top - 1) & 7;
				break;
			case 7: // FINCSTP
				fpu->top = (fpu->top + 1) & 7;
				break;
			}
			break;
		case 7:
			temp = fpget(fpu, 0);
			switch (i) {
			case 0: { // FPREM
				temp2 = fpget(fpu, 1);
				int64_t q = temp / temp2; // TODO: overflow
				fpset(fpu, 0, temp - q * temp2);
				fpu->sw &= ~C2;
				if (q & 1) fpu->sw |= C1; else fpu->sw &= ~C1;
				if (q & 2) fpu->sw |= C3; else fpu->sw &= ~C3;
				if (q & 4) fpu->sw |= C0; else fpu->sw &= ~C0;
				break;
			}
			case 1: // FYL2XP1
				temp2 = fpget(fpu, 1);
				fpset(fpu, 1, temp2 * fplog2(FPCONST(1.0) + temp));
				fppop(fpu);
				break;
			case 2: // FSQRT
				fpset(fpu, 0, fpsqrt(temp));
				break;
			case 3: { // FSINCOS
				fpreal s, c;
#if defined(_GNU_SOURCE) && !defined(USE_FLOAT32_FPU)
				sincos(temp, &s, &c);
#else
				s = fpsin(temp);
				c = fpcos(temp);
#endif
				fpset(fpu, 0, s);
				fppush(fpu, c);
				fpu->sw &= ~C2;
				break;
			}
			case 4: { // FRNDINT
				int rc = (fpu->cw >> 10) & 3;
				fpset(fpu, 0, fpround(temp, rc));
				break;
			}
			case 5: // FSCALE
				fpset(fpu, 0,
				      temp * fppow(FPCONST(2.0), fptrunc(fpget(fpu, 1))));
				break;
			case 6: // FSIN
				fpset(fpu, 0, fpsin(temp));
				fpu->sw &= ~C2;
				break;
			case 7: // FCOS
				fpset(fpu, 0, fpcos(temp));
				fpu->sw &= ~C2;
				break;
			}
			break;
		}
		break;
	}
	case 2:
		switch (group) {
		case 5:
			if (i == 1) { // FUCOMPP
				fpreal a = fpget(fpu, 0);
				fpreal b = fpget(fpu, 1);
				if (fpisunordered(a, b)) {
					fpu->sw |= C0 | C2 | C3;
				} else if (a == b) {
					fpu->sw |= C3;
					fpu->sw &= ~(C0 | C2);
				} else if (a < b) {
					fpu->sw |= C0;
					fpu->sw &= ~(C2 | C3);
				} else {
					fpu->sw &= ~(C0 | C2 | C3);
				}
				fppop(fpu);
				fppop(fpu);
				break;
			}
			cpu_setexc(cpu, 6, 0);
			return false;
		case 0: case 1: case 2: case 3: // FCMOV
			if (cmov_cond(fpu, cpu, group))
				fpset(fpu, 0, fpget(fpu, i));
			break;
		default:
			cpu_setexc(cpu, 6, 0);
			return false;
		}
		break;
	case 3:
		switch (group) {
		case 4:
			switch (i) {
			case 0:
			case 1: // 8087 only
			case 4: // FNSETPM
			case 5: // FRSTPM
				/* nothing */
				break;
			case 2: // FNCLEX
				fpu->sw &= ~0x80ff;
				break;
			case 3: // FNINIT
				fpu->sw = 0;
				fpu->top = 0;
				fpu->cw = 0x37f;
				break;
			case 6:
			case 7:
				cpu_setexc(cpu, 6, 0);
				return false;
			}
			break;
		case 0: case 1: case 2: case 3: // FCMOV
			if (!cmov_cond(fpu, cpu, group))
				fpset(fpu, 0, fpget(fpu, i));
			break;
		case 5: // FUCOMI
		case 6: // FCOMI TODO: raise IA
			ucomi(fpu, cpu, i);
			break;
		default:
			cpu_setexc(cpu, 6, 0);
			return false;
		}
		break;
	case 4: {
		fpreal a = fpget(fpu, 0);
		fpreal b = fpget(fpu, i);
		fparith(fpu, group, i, a, b);
		break;
	}
	case 5: {
		fpreal temp, temp2;
		switch (group) {
		case 0: // FFREE
			// tw
			break;
		case 1: // FXCH
			temp = fpget(fpu, i);
			temp2 = fpget(fpu, 0);
			fpset(fpu, i, temp2);
			fpset(fpu, 0, temp);
			break;
		case 2: // FST
			temp = fpget(fpu, 0);
			fpset(fpu, i, temp);
			break;
		case 3: // FSTP
			temp = fpget(fpu, 0);
			fpset(fpu, i, temp);
			fppop(fpu);
			break;
		case 4: // FUCOM
		case 5: // FUCOMP
		{
			fpreal a = fpget(fpu, 0);
			fpreal b = fpget(fpu, i);
			if (fpisunordered(a, b)) {
				fpu->sw |= C0 | C2 | C3;
			} else if (a == b) {
				fpu->sw |= C3;
				fpu->sw &= ~(C0 | C2);
			} else if (a < b) {
				fpu->sw |= C0;
				fpu->sw &= ~(C2 | C3);
			} else {
				fpu->sw &= ~(C0 | C2 | C3);
			}
			if (group == 5)
				fppop(fpu);
			break;
		}
		case 6:
		case 7:
			cpu_setexc(cpu, 6, 0);
			return false;
		}
		break;
	}
	case 6: {
		fpreal a = fpget(fpu, 0);
		fpreal b = fpget(fpu, i);
		fparith(fpu, group, i, a, b);
		fppop(fpu);
		break;
	}
	case 7:
		switch (group) {
		case 0: // FFREEP
			// tw
			fppop(fpu);
			break;
		case 1: { // FXCH
			fpreal temp = fpget(fpu, i);
			fpreal temp2 = fpget(fpu, 0);
			fpset(fpu, i, temp2);
			fpset(fpu, 0, temp);
			break;
		}
		case 2: // FSTP
		case 3: { // FSTP
			fpreal temp;
			temp = fpget(fpu, 0);
			fpset(fpu, i, temp);
			fppop(fpu);
			break;
		}
		case 4:
			if (i == 0) { // FNSTSW
				u16 sw = getsw(fpu);
				cpu_setax(cpu, sw);
			} else {
				cpu_setexc(cpu, 6, 0);
				return false;
			}
			break;
		case 5: // FUCOMIP
		case 6: // FCOMIP TODO: raise IA
			ucomi(fpu, cpu, i);
			fppop(fpu);
			break;
		case 7:
			cpu_setexc(cpu, 6, 0);
			return false;
		}
		break;
	}
	return true;
}

#if defined(I386_ENABLE_MMX) || defined(I386_ENABLE_SSE)
#define SIMD_fpu_c
#include "simd.inc"
#undef SIMD_fpu_c
#endif
