Q = @
CC_ = gcc
CC_win32 = i686-w64-mingw32-gcc
CC = ${CC_${PLAT}}
CXX_ = g++
CXX_win32 = i686-w64-mingw32-g++
CXX = ${CXX_${PLAT}}
HOSTCC = ${CC}

SDL_CONFIG = sdl-config
SLIRP_INC =
SLIRP_LIB = -lslirp

CFLAGS = -I . -Wall -O3 -ffunction-sections -fdata-sections -g -Wl,--gc-sections ${SLIRP_INC}
CFLAGS += -DI386_ENABLE_MMX -DI386_ENABLE_SSE
CXXFLAGS = ${CFLAGS} -std=c++14 -fno-exceptions -fno-rtti
CFLAGS_SDL = ${CFLAGS} `${SDL_CONFIG} --cflags`
CXXFLAGS_SDL = ${CXXFLAGS} `${SDL_CONFIG} --cflags`

LDFLAGS_ =
LDFLAGS_win32 = -mconsole
LDFLAGS = ${LDFLAGS_${PLAT}}

LIBS_ = -lm ${SLIRP_LIB}
LIBS_win32 = -lm ${SLIRP_LIB} -lws2_32 -liphlpapi
LIBS = ${LIBS_${PLAT}}

LIBS_SDL_ = `${SDL_CONFIG} --libs` ${LIBS}
LIBS_SDL_win32 = `${SDL_CONFIG} --libs` ${LIBS}
LIBS_SDL = ${LIBS_SDL_${PLAT}}

LIBS_RAWDRAW_ = -lX11 -lasound ${LIBS}
LIBS_RAWDRAW_win32 = -lgdi32 -lwinmm ${LIBS}
LIBS_RAWDRAW = ${LIBS_RAWDRAW_${PLAT}}

#USE_SDL = y/n
USE_SDL = n

SUFF_SDL_SDL_y =
SUFF_SDL_SDL_n = _sdl
SUFF_RAWDRAW_SDL_y = _rawdraw
SUFF_RAWDRAW_SDL_n =
SUFF_SDL = ${SUFF_SDL_SDL_${USE_SDL}}
SUFF_RAWDRAW = ${SUFF_RAWDRAW_SDL_${USE_SDL}}

PROGS_ = tiny386 tiny386_nosdl tiny386_kvm wifikbd initnet
PROGS_win32 = tiny386 tiny386_nosdl wifikbd
PROGS = ${PROGS_${PLAT}}

SRCS = ini.c i386.c fpu.c i8259.c i8254.c ide.c vga.c i8042.c misc.c adlib.c ne2000.c i8257.c sb16.c pcspk.c
SRCS += pci.c
SRCS += win32.c
# OSD
SRCS += osd/microui.c osd/osd.c

# ymfm OPL emulation (replaces fmopl.c)
YMFM_SRCS = ymfm_wrapper.cpp thirdparty/ymfm/src/ymfm_opl.cpp thirdparty/ymfm/src/ymfm_adpcm.cpp thirdparty/ymfm/src/ymfm_pcm.cpp thirdparty/ymfm/src/ymfm_ssg.cpp

OBJS = ${SRCS:.c=.o}
YMFM_OBJS = ${YMFM_SRCS:.cpp=.o}

.PHONY: all clean dep prepare
.SUFFIXES: .c .cpp
.c.o:
	@/bin/echo -e " \e[1;32mCC\e[0m\t\e[1;37m$<\e[0m \e[1;32m->\e[0m \e[1;37m$@\e[0m"
	${Q}${CC} ${CFLAGS} -c $< -o $@

.cpp.o:
	@/bin/echo -e " \e[1;32mCXX\e[0m\t\e[1;37m$<\e[0m \e[1;32m->\e[0m \e[1;37m$@\e[0m"
	${Q}${CXX} ${CXXFLAGS} -c $< -o $@

all: ${PROGS}

win32:
	make -C . PLAT=win32

clean:
	rm -f ${OBJS} ${YMFM_OBJS} .depends ${PROGS}

prepare:
	@echo "No preparation needed (ymfm has no lookup tables to generate)"

tiny386${SUFF_SDL}: sdl/main.c pc.c ${OBJS} ${YMFM_OBJS}
	@/bin/echo -e " \e[1;32mCXXLD\e[0m\t\e[1;32m->\e[0m \e[1;37m$@\e[0m"
	${Q}${CXX} ${LDFLAGS} ${CFLAGS_SDL} -o $@ $^$> ${LIBS_SDL}

tiny386${SUFF_RAWDRAW}: rawdraw/main.c pc.c ${OBJS} ${YMFM_OBJS}
	@/bin/echo -e " \e[1;32mCXXLD\e[0m\t\e[1;32m->\e[0m \e[1;37m$@\e[0m"
	${Q}${CXX} ${LDFLAGS} ${CFLAGS} -o $@ $^$> ${LIBS_RAWDRAW}

tiny386_nosdl: main.c pc.c ${OBJS} ${YMFM_OBJS}
	@/bin/echo -e " \e[1;32mCXXLD\e[0m\t\e[1;32m->\e[0m \e[1;37m$@\e[0m"
	${Q}${CXX} ${LDFLAGS} ${CFLAGS} -o $@ $^$> ${LIBS}

tiny386_kvm${SUFF_SDL}: sdl/main.c kvm.c pc.c ${OBJS} ${YMFM_OBJS}
	@/bin/echo -e " \e[1;32mCXXLD\e[0m\t\e[1;32m->\e[0m \e[1;37m$@\e[0m"
	${Q}${CXX} -DUSEKVM ${LDFLAGS} ${CFLAGS_SDL} -o $@ $^$> ${LIBS_SDL}

tiny386_kvm${SUFF_RAWDRAW}: rawdraw/main.c kvm.c pc.c ${OBJS} ${YMFM_OBJS}
	@/bin/echo -e " \e[1;32mCXXLD\e[0m\t\e[1;32m->\e[0m \e[1;37m$@\e[0m"
	${Q}${CXX} -DUSEKVM ${LDFLAGS} ${CFLAGS} -o $@ $^$> ${LIBS_RAWDRAW}

wifikbd${SUFF_SDL}: tools/wifikbd.c win32.c
	@/bin/echo -e " \e[1;32mCCLD\e[0m\t\e[1;32m->\e[0m \e[1;37m$@\e[0m"
	${Q}${CC} ${LDFLAGS} ${CFLAGS_SDL} -o $@ $^$> ${LIBS_SDL}

wifikbd${SUFF_RAWDRAW}: rawdraw/wifikbd.c win32.c
	@/bin/echo -e " \e[1;32mCCLD\e[0m\t\e[1;32m->\e[0m \e[1;37m$@\e[0m"
	${Q}${CC} ${LDFLAGS} ${CFLAGS} -o $@ $^$> ${LIBS_RAWDRAW}

initnet: tools/initnet.c
	@/bin/echo -e " \e[1;32mCCLD\e[0m\t\e[1;32m->\e[0m \e[1;37m$@\e[0m"
	${Q}${CC} -o $@ $^$>

.depends: ${SRCS} ${YMFM_SRCS}
	@/bin/echo -e " \e[1;32mDEP\e[0m\t\e[1;37mC/C++ sources\e[0m \e[1;32m->\e[0m \e[1;37m$@\e[0m"
	${Q}rm -f $@
	${Q}for i in ${SRCS}; do ${CC} ${CFLAGS} -MT $$(dirname $$i)/$$(basename -s .c $$i).o -MM $$i 2> /dev/null >> $@ || exit 0; done
	${Q}for i in ${YMFM_SRCS}; do ${CXX} ${CXXFLAGS} -MT $$(dirname $$i)/$$(basename -s .cpp $$i).o -MM $$i 2> /dev/null >> $@ || exit 0; done

dep: .depends
-include .depends
