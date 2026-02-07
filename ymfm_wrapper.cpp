/*
 * ymfm wrapper - C interface for ymfm OPL3 emulation
 * Provides OPL3 (YMF262) emulation with stereo output
 */

#include <cstdint>

// ymfm headers
#include "thirdparty/ymfm/src/ymfm_opl.h"

// C interface header
extern "C" {
#include "fmopl.h"
}

// OPL3 clock frequency (standard)
static constexpr uint32_t OPL_CLOCK = 14318180;

// Wrapper class implementing ymfm_interface for OPL3
class opl3_wrapper : public ymfm::ymfm_interface
{
public:
    opl3_wrapper(int clock, int rate) :
        m_chip(*this),
        m_clock(clock),
        m_rate(rate),
        m_timer_handler(nullptr),
        m_timer_param(nullptr)
    {
        m_timer_running[0] = false;
        m_timer_running[1] = false;
        m_chip.reset();
    }

    // Timer callback from ymfm
    virtual void ymfm_set_timer(uint32_t tnum, int32_t duration_in_clocks) override
    {
        if (tnum >= 2)
            return;

        if (duration_in_clocks < 0) {
            // Cancel timer
            m_timer_running[tnum] = false;
            if (m_timer_handler)
                m_timer_handler(m_timer_param, tnum, 0.0f);
        } else {
            // Start timer - convert clocks to seconds
            m_timer_running[tnum] = true;
            if (m_timer_handler) {
                float interval = static_cast<float>(duration_in_clocks) / static_cast<float>(m_clock);
                m_timer_handler(m_timer_param, tnum, interval);
            }
        }
    }

    void set_timer_handler(OPL_TIMERHANDLER handler, void *param)
    {
        m_timer_handler = handler;
        m_timer_param = param;
    }

    void write(int addr, int data)
    {
        // OPL3 address mapping:
        // addr 0 = address port for bank 0 (0x388)
        // addr 1 = data port for bank 0 (0x389)
        // addr 2 = address port for bank 1 (0x38A)
        // addr 3 = data port for bank 1 (0x38B)
        switch (addr & 3) {
        case 0:  // Address port, low bank
            m_chip.write_address(static_cast<uint8_t>(data));
            break;
        case 1:  // Data port, low bank
            m_chip.write_data(static_cast<uint8_t>(data));
            break;
        case 2:  // Address port, high bank
            m_chip.write_address_hi(static_cast<uint8_t>(data));
            break;
        case 3:  // Data port, high bank (OPL3 uses same write_data)
            m_chip.write_data(static_cast<uint8_t>(data));
            break;
        }
    }

    uint8_t read(int addr)
    {
        (void)addr;
        return m_chip.read_status();
    }

    void timer_over(int timer)
    {
        if (timer < 2 && m_timer_running[timer]) {
            m_engine->engine_timer_expired(timer);
        }
    }

    // Generate stereo samples
    void generate_stereo(int16_t *buffer, int samples)
    {
        for (int i = 0; i < samples; i++) {
            ymfm::ymf262::output_data output;
            m_chip.generate(&output, 1);

            // OPL3 outputs 4 channels, mix to stereo
            // Channels 0,1 = left, channels 2,3 = right (typical config)
            int32_t left = output.data[0] + output.data[1];
            int32_t right = output.data[2] + output.data[3];

            // Clamp to 16-bit
            if (left > 32767) left = 32767;
            if (left < -32768) left = -32768;
            if (right > 32767) right = 32767;
            if (right < -32768) right = -32768;

            buffer[i * 2] = static_cast<int16_t>(left);
            buffer[i * 2 + 1] = static_cast<int16_t>(right);
        }
    }

    // Generate mono samples (for backwards compatibility)
    void generate(int16_t *buffer, int samples)
    {
        for (int i = 0; i < samples; i++) {
            ymfm::ymf262::output_data output;
            m_chip.generate(&output, 1);

            // Mix all 4 channels to mono
            int32_t sample = output.data[0] + output.data[1] + output.data[2] + output.data[3];
            sample /= 2;  // Scale down

            if (sample > 32767) sample = 32767;
            if (sample < -32768) sample = -32768;
            buffer[i] = static_cast<int16_t>(sample);
        }
    }

private:
    ymfm::ymf262 m_chip;
    int m_clock;
    int m_rate;
    OPL_TIMERHANDLER m_timer_handler;
    void *m_timer_param;
    bool m_timer_running[2];
};

// C interface implementation
extern "C" {

// FM_OPL is actually opl3_wrapper*
FM_OPL *OPLCreate(int clock, int rate)
{
    // Use OPL3 clock if caller passes OPL2 clock
    if (clock < 10000000)
        clock = OPL_CLOCK;
    opl3_wrapper *wrapper = new opl3_wrapper(clock, rate);
    return reinterpret_cast<FM_OPL *>(wrapper);
}

void OPLDestroy(FM_OPL *OPL)
{
    if (OPL) {
        opl3_wrapper *wrapper = reinterpret_cast<opl3_wrapper *>(OPL);
        delete wrapper;
    }
}

void OPLSetTimerHandler(FM_OPL *OPL, OPL_TIMERHANDLER TimerHandler, void *param)
{
    if (OPL) {
        opl3_wrapper *wrapper = reinterpret_cast<opl3_wrapper *>(OPL);
        wrapper->set_timer_handler(TimerHandler, param);
    }
}

int OPLWrite(FM_OPL *OPL, int a, int v)
{
    if (OPL) {
        opl3_wrapper *wrapper = reinterpret_cast<opl3_wrapper *>(OPL);
        wrapper->write(a, v);
    }
    return 0;
}

unsigned char OPLRead(FM_OPL *OPL, int a)
{
    if (OPL) {
        opl3_wrapper *wrapper = reinterpret_cast<opl3_wrapper *>(OPL);
        return wrapper->read(a);
    }
    return 0;
}

int OPLTimerOver(FM_OPL *OPL, int c)
{
    if (OPL) {
        opl3_wrapper *wrapper = reinterpret_cast<opl3_wrapper *>(OPL);
        wrapper->timer_over(c);
    }
    return 0;
}

// Mono output (backwards compatible name)
void YM3812UpdateOne(FM_OPL *OPL, int16_t *buffer, int length)
{
    if (OPL) {
        opl3_wrapper *wrapper = reinterpret_cast<opl3_wrapper *>(OPL);
        wrapper->generate(buffer, length);
    }
}

// Stereo output for OPL3
void YMF262UpdateOne(FM_OPL *OPL, int16_t *buffer, int length)
{
    if (OPL) {
        opl3_wrapper *wrapper = reinterpret_cast<opl3_wrapper *>(OPL);
        wrapper->generate_stereo(buffer, length);
    }
}

} // extern "C"
