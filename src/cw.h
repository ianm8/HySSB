#ifndef cw_h
#define cw_h

#include "Arduino.h"
#include "hardware/pwm.h"

#define GPIO_CWTONE 15u
#define PWM_MID_RAIL 512u

static const uint16_t keyclick_table[64] =
{
  0x0200u,
  0x0207u,
  0x020Bu,
  0x0209u,
  0x0200u,
  0x01F0u,
  0x01DEu,
  0x01CCu,
  0x01C0u,
  0x01BDu,
  0x01C7u,
  0x01DEu,
  0x0200u,
  0x0227u,
  0x024Eu,
  0x026Eu,
  0x027Fu,
  0x027Du,
  0x0265u,
  0x023Au,
  0x0200u,
  0x01BFu,
  0x0183u,
  0x0156u,
  0x0140u,
  0x0147u,
  0x016Du,
  0x01ADu,
  0x0200u,
  0x0258u,
  0x02A9u,
  0x02E4u,
  0x02FFu,
  0x02F3u,
  0x02BFu,
  0x026Bu,
  0x0200u,
  0x018Eu,
  0x0129u,
  0x00E0u,
  0x00C0u,
  0x00D1u,
  0x0113u,
  0x017Cu,
  0x0200u,
  0x0289u,
  0x0303u,
  0x035Au,
  0x037Fu,
  0x0369u,
  0x031Au,
  0x029Cu,
  0x0200u,
  0x015Du,
  0x00CFu,
  0x006Au,
  0x0040u,
  0x005Bu,
  0x00B8u,
  0x014Bu,
  0x0200u,
  0x02BAu,
  0x035Du,
  0x03D0u
};

static const uint16_t cw_table[16] =
{
  0x03FFu,
  0x03D8u,
  0x0369u,
  0x02C4u,
  0x0200u,
  0x013Cu,
  0x0097u,
  0x0028u,
  0x0001u,
  0x0028u,
  0x0097u,
  0x013Cu,
  0x0200u,
  0x02C4u,
  0x0369u,
  0x03D8u
};

volatile static bool tone_on = false;
volatile static uint32_t cw_pwm = 0;

static struct repeating_timer cw_timer;

static bool cw_callback(repeating_timer_t *rt) 
{
  enum cw_state_t {CW_IDLE, CW_START, CW_MIDDLE, CW_END};
  static cw_state_t cw_state = CW_IDLE;
  static uint32_t cw_sample = PWM_MID_RAIL;
  static uint32_t cw_index = 0;

  pwm_set_chan_level(cw_pwm,1,cw_sample);
  switch (cw_state)
  {
    case CW_IDLE:
    {
      cw_sample = PWM_MID_RAIL;
      cw_index = 0;
      if (tone_on)
      {
        cw_state = CW_START;
      }
      break;
    }
    case CW_START:
    {
      cw_sample = keyclick_table[cw_index++];
      cw_sample = (cw_sample-PWM_MID_RAIL)/8+PWM_MID_RAIL;
      if (cw_index>=64)
      {
        cw_index = 0;
        cw_state = CW_MIDDLE;
      }
      break;
    }
    case CW_MIDDLE:
    {
      cw_sample = cw_table[cw_index++];
      cw_sample = (cw_sample-PWM_MID_RAIL)/8+PWM_MID_RAIL;
      if (cw_index>=16)
      {
        cw_index = 0;
        cw_state = tone_on?CW_MIDDLE:CW_END;
      }
      break;
    }
    case CW_END:
    {
      const uint32_t i = 63-cw_index;
      cw_index++;
      cw_sample = keyclick_table[i];
      cw_sample = (cw_sample-PWM_MID_RAIL)/8+PWM_MID_RAIL;
      if (cw_index>=64)
      {
        cw_index = 0;
        cw_state = CW_IDLE;
      }
      break;
    }
  }
  return true;
}

class CW
{
  public:
    void init(void);
    void uninit(void);
    void toneOn(void);
    void toneOff(void);
    CW(void);
  private:
    bool _initialised = false;
};

CW::CW(void)
{
  _initialised = false;
}

void CW::init(void)
{
  if (_initialised)
  {
    return;
  }

  gpio_set_function(GPIO_CWTONE,GPIO_FUNC_PWM);

  // get PWM slice connected to the pin
  cw_pwm = pwm_gpio_to_slice_num(GPIO_CWTONE);

  // set period of 1024 cycles
  pwm_set_wrap(cw_pwm,1023);
  pwm_set_phase_correct(cw_pwm,true);

  // set to mid rail
  pwm_set_chan_level(cw_pwm,1,PWM_MID_RAIL);

  // start the PWM
  pwm_set_enabled(cw_pwm,true);

  // 1/(700*16) = 89us
  if (add_repeating_timer_us(-89LL, cw_callback, NULL, &cw_timer))
  {
    _initialised = true;
    return;
  }

  _initialised = false;
}

void CW::uninit(void)
{
  cancel_repeating_timer(&cw_timer);
  delay(1);
  pwm_set_enabled(cw_pwm,false);
  tone_on = false;
  _initialised = false;
}

void CW::toneOn(void)
{
  if (!_initialised)
  {
    return;
  }
  tone_on = true;
}

void CW::toneOff(void)
{
  if (!_initialised)
  {
    return;
  }
  tone_on = false;
}

#endif
