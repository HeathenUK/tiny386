/*
 * ymfm wrapper - C interface for ymfm OPL2 emulation
 * Replaces fmopl.c with ymfm library
 */

#include <cstdint>

// ymfm headers
#include "thirdparty/ymfm/src/ymfm_opl.h"

// C interface header
extern "C" {
#include "fmopl.h"
}

// OPL2 clock frequency (standard AdLib clock)
static constexpr uint32_t OPL_CLOCK = 3579545;

// Wrapper class implementing ymfm_interface
class opl_wrapper : public ymfm::ymfm_interface
{
public:
    opl_wrapper(int clock, int rate) :
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
        if (addr & 1)
            m_chip.write_data(static_cast<uint8_t>(data));
        else
            m_chip.write_address(static_cast<uint8_t>(data));
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

    void generate(int16_t *buffer, int samples)
    {
        for (int i = 0; i < samples; i++) {
            ymfm::ym3812::output_data output;
            m_chip.generate(&output, 1);
            // ymfm output is 32-bit, clamp to 16-bit
            int32_t sample = output.data[0];
            if (sample > 32767) sample = 32767;
            if (sample < -32768) sample = -32768;
            buffer[i] = static_cast<int16_t>(sample);
        }
    }

private:
    ymfm::ym3812 m_chip;
    int m_clock;
    int m_rate;
    OPL_TIMERHANDLER m_timer_handler;
    void *m_timer_param;
    bool m_timer_running[2];
};

// C interface implementation
extern "C" {

// FM_OPL is actually opl_wrapper*
FM_OPL *OPLCreate(int clock, int rate)
{
    opl_wrapper *wrapper = new opl_wrapper(clock, rate);
    return reinterpret_cast<FM_OPL *>(wrapper);
}

void OPLDestroy(FM_OPL *OPL)
{
    if (OPL) {
        opl_wrapper *wrapper = reinterpret_cast<opl_wrapper *>(OPL);
        delete wrapper;
    }
}

void OPLSetTimerHandler(FM_OPL *OPL, OPL_TIMERHANDLER TimerHandler, void *param)
{
    if (OPL) {
        opl_wrapper *wrapper = reinterpret_cast<opl_wrapper *>(OPL);
        wrapper->set_timer_handler(TimerHandler, param);
    }
}

int OPLWrite(FM_OPL *OPL, int a, int v)
{
    if (OPL) {
        opl_wrapper *wrapper = reinterpret_cast<opl_wrapper *>(OPL);
        wrapper->write(a, v);
    }
    return 0;
}

unsigned char OPLRead(FM_OPL *OPL, int a)
{
    if (OPL) {
        opl_wrapper *wrapper = reinterpret_cast<opl_wrapper *>(OPL);
        return wrapper->read(a);
    }
    return 0;
}

int OPLTimerOver(FM_OPL *OPL, int c)
{
    if (OPL) {
        opl_wrapper *wrapper = reinterpret_cast<opl_wrapper *>(OPL);
        wrapper->timer_over(c);
    }
    return 0;
}

void YM3812UpdateOne(FM_OPL *OPL, int16_t *buffer, int length)
{
    if (OPL) {
        opl_wrapper *wrapper = reinterpret_cast<opl_wrapper *>(OPL);
        wrapper->generate(buffer, length);
    }
}

} // extern "C"
