#include "Arduino.h"
#include "Radio.h"
#include "TCA9534A.h"
#include "cw.h"
#include <Rotary.h>

#define BAND_I2C_ADDRESS   0x20U
#define FILTER_I2C_ADDRESS 0x21U

// protect rotary encoder vars
static critical_section_t rotary_exclusive;
static Rotary tune = Rotary(Radio::PIN_ENC1A,Radio::PIN_ENC1B);
static Rotary func = Rotary(Radio::PIN_ENC2A,Radio::PIN_ENC2B);
static TCA9534 band_io;
static TCA9534 filter_io;
static CW cw;

Radio::Radio(
  const uint32_t _frequency,
  const uint32_t _tuning_step,
  const Radio::modes_t _mode,
  const Radio::bands_t _band)
{
  Radio::frequency = _frequency;
  Radio::tuning_step = _tuning_step;
  Radio::mode = _mode;
  Radio::band = _band;
  Radio::scope_speed = 1;
  Radio::scope_zoom = 0;
  Radio::_i2c_band_error = false;
  Radio::_i2c_filter_error = false;
  //Radio::_i2c_band_error = true;
  //Radio::_i2c_filter_error = true;
  critical_section_init(&rotary_exclusive);
  Radio::_tune_value = 0;
  Radio::_func_value = 0;
  Radio::_tx_enable = false;
  Radio::_locked = false;
  Radio::_att_enabled = false;
  Radio::_current_band = BANDXX;
  Radio::_current_filter = FILTER_XXX;
}

void Radio::init(void)
{
  pinMode(PIN_PTT,INPUT_PULLUP);      // Mic - PTT *
  pinMode(PIN_TP10,INPUT_PULLUP);     // spare pin - enable pull up
  pinMode(PIN_PADA,INPUT);            // CW paddle A
  pinMode(PIN_PADB,INPUT);            // CW paddle B
  pinMode(PIN_ENC1A,INPUT_PULLUP);    // tuning encoder
  pinMode(PIN_ENC1B,INPUT_PULLUP);    // tuning encoder
  pinMode(PIN_ENC2A,INPUT_PULLUP);    // multifunction encoder
  pinMode(PIN_ENC2B,INPUT_PULLUP);    // multifunction encoder
  pinMode(PIN_ENC1BUT,INPUT_PULLUP);  // tuning encoder button
  pinMode(PIN_ENC2BUT,INPUT_PULLUP);  // multifunction encoder button
  pinMode(PIN_DSENSE,INPUT_PULLUP);   // digital mode VOX sense line *
  pinMode(PIN_CWSIDETONE,OUTPUT);     // CW sidtone (PWM)
  pinMode(PIN_MUTE,OUTPUT);           // audio mute control
  pinMode(PIN_CWTONE,OUTPUT);         // CW tone (PWM)
  pinMode(PIN_MISO,INPUT);            // MISO (input unused, has external pullup)
  pinMode(PIN_CS,OUTPUT);             // LCD (SPI)
  pinMode(PIN_SCK,OUTPUT);            // LCD (SPI)
  pinMode(PIN_MOSI,OUTPUT);           // LCD (SPI)
  pinMode(PIN_DC,OUTPUT);             // LCD (SPI)
  pinMode(PIN_RST,OUTPUT);            // LCD (SPI)
  pinMode(PIN_TP11,INPUT_PULLUP);     // spare pin - enable pull up
  pinMode(PIN_PICOLED,OUTPUT);        // Pico on-board LED
  pinMode(PIN_QSDI,INPUT);            // QSD I channel
  pinMode(PIN_QSDQ,INPUT);            // QSD Q channel
  pinMode(PIN_AGC,INPUT);             // AGC level
  mute();
  LEDoff();
  digitalWrite(PIN_CWSIDETONE,LOW);
  digitalWrite(PIN_CWTONE,LOW);

  Wire.begin();
  Wire.beginTransmission(BAND_I2C_ADDRESS);
  volatile uint32_t i2c_status = Wire.endTransmission();
  if (i2c_status!=0)
  {
    Radio::_i2c_band_error = true;
    pinMode(LED_BUILTIN,OUTPUT);
    for (;;)
    {
      for (uint32_t i=0;i<4;i++)
      {
        digitalWrite(LED_BUILTIN,HIGH);
        delay(50);
        digitalWrite(LED_BUILTIN,LOW);
        delay(500);
      }
      delay(2000);
    }
  }
  Wire.beginTransmission(FILTER_I2C_ADDRESS);
  i2c_status = Wire.endTransmission();
  if (i2c_status!=0)
  {
    Radio::_i2c_filter_error = true;
    for (;;)
    {
      pinMode(LED_BUILTIN,OUTPUT);
      for (uint32_t i=0;i<5;i++)
      {
        digitalWrite(LED_BUILTIN,HIGH);
        delay(50);
        digitalWrite(LED_BUILTIN,LOW);
        delay(500);
      }
      delay(2000);
    }
  }
  if (!Radio::_i2c_band_error && !Radio::_i2c_filter_error)
  {
    band_io.attach(Wire);
    band_io.setDeviceAddress(BAND_I2C_ADDRESS);
    filter_io.attach(Wire);
    filter_io.setDeviceAddress(FILTER_I2C_ADDRESS);
    band_io.config(TCA9534::Config::OUT);
    band_io.polarity(TCA9534::Polarity::ORIGINAL);
    filter_io.config(TCA9534::Config::OUT);
    filter_io.polarity(TCA9534::Polarity::ORIGINAL);
    band_io.output(BAND_BIT_B80,TCA9534::Level::L);
    band_io.output(BAND_BIT_B40,TCA9534::Level::L);
    band_io.output(BAND_BIT_B20,TCA9534::Level::L);
    band_io.output(BAND_BIT_B15,TCA9534::Level::L);
    band_io.output(BAND_BIT_B10,TCA9534::Level::L);
    band_io.output(BAND_BIT_TX,TCA9534::Level::L);
    band_io.output(BAND_BIT_CWMIC,TCA9534::Level::L);
    filter_io.output(FILTER_BIT_SSB,TCA9534::Level::L);
    filter_io.output(FILTER_BIT_CW,TCA9534::Level::L);
    filter_io.output(FILTER_BIT_DIG,TCA9534::Level::L);
    filter_io.output(FILTER_BIT_ATT,TCA9534::Level::L);
    filter_io.output(FILTER_BIT_SP1,TCA9534::Level::L);
    filter_io.output(FILTER_BIT_SP2,TCA9534::Level::L);
    filter_io.output(FILTER_BIT_SP3,TCA9534::Level::L);
  }
  Radio::muteMic();
  Radio::setBand(Radio::band);
  Radio::setFilter(Radio::FILTER_SSB);
  Radio::attOff();
  LEDoff();
}

void Radio::process(void)
{
  // regularly call process() from within
  // an interrupt routine in order to
  // process the PWM CW tones and the
  // rotary encoders
  // if transmitting, don't process rotaries
  if (Radio::_tx_enable)
  {
    Radio::_tune_value = 0;
    Radio::_func_value = 0;
  }
  else
  {
    if (!Radio::_locked)
    {
      switch (tune.process())
      {
        case DIR_CW:  Radio::_tune_value++; break;
        case DIR_CCW: Radio::_tune_value--; break;
      }
    }
    switch (func.process())
    {
      case DIR_CW:  Radio::_func_value++; break;
      case DIR_CCW: Radio::_func_value--; break;
    }
  }
}

void Radio::mute(void)
{
  digitalWrite(PIN_MUTE,HIGH);
}

void Radio::unMute(void)
{
  digitalWrite(PIN_MUTE,LOW);
}

void Radio::muteMic(void)
{
  if (Radio::_i2c_band_error)
  {
    return;
  }
  band_io.output(Radio::BAND_BIT_CWMIC,TCA9534::Level::H);
}

void Radio::unmuteMic(void)
{
  if (Radio::_i2c_band_error)
  {
    return;
  }
  band_io.output(Radio::BAND_BIT_CWMIC,TCA9534::Level::L);
}

void Radio::attOn(void)
{
  Radio::_att_enabled = true;
  if (Radio::_i2c_filter_error)
  {
    return;
  }
  filter_io.output(Radio::FILTER_BIT_ATT,TCA9534::Level::H);
}

void Radio::attOff(void)
{
  Radio::_att_enabled = false;
  if (Radio::_i2c_filter_error)
  {
    return;
  }
  filter_io.output(Radio::FILTER_BIT_ATT,TCA9534::Level::L);
}

const boolean Radio::attEnabled(void)
{
  return Radio::_att_enabled;
}

void Radio::LEDon(void)
{
  digitalWrite(PIN_PICOLED,HIGH);
}

void Radio::LEDoff(void)
{
  digitalWrite(PIN_PICOLED,LOW);
}

void Radio::txEnable(void)
{
  // engage the TX relays
  LEDon();
  Radio::_tx_enable = true;
  if (Radio::_i2c_band_error)
  {
    return;
  }
  band_io.output(Radio::BAND_BIT_TX,TCA9534::Level::H);
}

const boolean Radio::txEnabled(void)
{
  return Radio::_tx_enable;
}

void Radio::rxEnable(void)
{
  // disengage the TX relays
  LEDoff();
  Radio::_tx_enable = false;  
  if (Radio::_i2c_band_error)
  {
    return;
  }
  band_io.output(Radio::BAND_BIT_TX,TCA9534::Level::L);
}

const boolean Radio::rxEnabled(void)
{
  return !Radio::_tx_enable;
}

const boolean Radio::PTT(void)
{
  return (digitalRead(Radio::PIN_PTT)==LOW);
}

const boolean Radio::DSENSE(void)
{
  return (digitalRead(Radio::PIN_DSENSE)==LOW);
}

const boolean Radio::paddleA(void)
{
  return (digitalRead(Radio::PIN_PADA)==LOW);
}

const boolean Radio::paddleB(void)
{
  return (digitalRead(Radio::PIN_PADB)==LOW);
}

const boolean Radio::tuneButton(void)
{
  return (digitalRead(Radio::PIN_ENC1BUT)==LOW);
}

const boolean Radio::multiButton(void)
{
  return (digitalRead(Radio::PIN_ENC2BUT)==LOW);
}

void Radio::lock(void)
{
  Radio::_locked = true;
}

void Radio::unlock(void)
{
  Radio::_locked = false;
}

const bool Radio::isLocked(void)
{
  return Radio::_locked;
}

const int32_t Radio::Tune(void)
{
  // return the number of counts the tune encoder has
  // turned and reset the count
  // if transmitting then don't process rotaries and
  // don't disable interrupts!
  if (Radio::_tx_enable||Radio::_locked)
  {
    return 0;
  }
  // disable interrupts for exclusive access to tune_value
  critical_section_enter_blocking(&rotary_exclusive);
  const int32_t tune_count = Radio::_tune_value;
  Radio::_tune_value = 0;
  critical_section_exit(&rotary_exclusive);
  return tune_count;
}

const int32_t Radio::Func(void)
{
  // return the number of counts the multifunc encoder
  // has turned and reset the count
  // if transmitting then don't process rotaries and
  // don't disable interrupts!
  if (Radio::_tx_enable)
  {
    return 0;
  }
  // disable interrupts for exclusive access to func_value
  critical_section_enter_blocking(&rotary_exclusive);
  const int32_t func_count = Radio::_func_value;
  Radio::_func_value = 0;
  critical_section_exit(&rotary_exclusive);
  return func_count;
}

void Radio::setFilter(const Radio::filter_t new_filter)
{
  if (Radio::_current_filter==new_filter)
  {
    return;
  }
  Radio::_current_filter = new_filter;
  if (Radio::_i2c_filter_error)
  {
    return;
  }
  filter_io.output(FILTER_BIT_SSB,TCA9534::Level::L);
  filter_io.output(FILTER_BIT_CW,TCA9534::Level::L);
  filter_io.output(FILTER_BIT_DIG,TCA9534::Level::L);
  switch (new_filter)
  {
    case Radio::FILTER_SSB:
    {
      break;
    }
    case Radio::FILTER_CW:
    {
      filter_io.output(FILTER_BIT_SSB,TCA9534::Level::H);
      filter_io.output(FILTER_BIT_CW,TCA9534::Level::H);
      break;
    }
    case Radio::FILTER_DIG:
    {
      filter_io.output(FILTER_BIT_SSB,TCA9534::Level::H);
      filter_io.output(FILTER_BIT_DIG,TCA9534::Level::H);
      break;
    }
  }
}

void Radio::setBand(const Radio::bands_t new_band)
{
  if (Radio::_current_band==new_band)
  {
    return;
  }
  Radio::_current_band = new_band;
  Radio::band = new_band;
  if (Radio::_i2c_band_error)
  {
    return;
  }
  band_io.output(BAND_BIT_B80,TCA9534::Level::L);
  band_io.output(BAND_BIT_B40,TCA9534::Level::L);
  band_io.output(BAND_BIT_B20,TCA9534::Level::L);
  band_io.output(BAND_BIT_B15,TCA9534::Level::L);
  band_io.output(BAND_BIT_B10,TCA9534::Level::L);
  switch (new_band)
  {
    case Radio::BAND80: band_io.output(BAND_BIT_B80,TCA9534::Level::H); break;
    case Radio::BAND40: band_io.output(BAND_BIT_B40,TCA9534::Level::H); break;
    case Radio::BAND20: band_io.output(BAND_BIT_B20,TCA9534::Level::H); break;
    case Radio::BAND15: band_io.output(BAND_BIT_B15,TCA9534::Level::H); break;
    case Radio::BAND10: band_io.output(BAND_BIT_B10,TCA9534::Level::H); break;
  }
}

const boolean Radio::band_io_error(void)
{
  return Radio::_i2c_band_error;
}

const boolean Radio::filter_io_error(void)
{
  return Radio::_i2c_filter_error;
}

const uint32_t Radio::band_index(const Radio::bands_t band)
{
  switch (band)
  {
    case Radio::BAND80: return 0u;
    case Radio::BAND40: return 1u;
    case Radio::BAND20: return 2u;
    case Radio::BAND15: return 3u;
    case Radio::BAND10: return 4u;
  }
  return 0;
}

void Radio::cwToneStart(void)
{
  cw.init();
  cw.toneOn();
}

void Radio::cwToneStop(void)
{
  cw.toneOff();
}

void Radio::cwStop(void)
{
  cw.uninit();
  pinMode(PIN_CWTONE,OUTPUT);
  digitalWrite(PIN_CWTONE,LOW);
}
