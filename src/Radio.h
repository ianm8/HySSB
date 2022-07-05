/*
  GP0 digital input (PTT)
  GP1 digital input, pullup enabled (TP10 unused)
  GP2 digital input, paddle A
  GP3 digital input, paddle B
  GP4 SDA (set up by Wire)
  GP5 SCL (set up by Wire)
  GP6 digital input, pullup enabled (rotary encoder one pin A – tuning)
  GP7 digital input, pullup enabled (rotary encoder one pin B – tuning)
  GP8 digital input, pullup enabled (rotary encoder two pin A – multi function)
  GP9 digital input, pullup enabled (rotary encoder two pin B – multi function)
  GP10 digital input, pullup enabled (rotary encoder A switch)
  GP11 digital input, pullup enabled (rotary encoder B switch)
  GP12 digital input (digital mode VOX sense)
  GP13 digital output, PWM (CW side tone)
  GP14 digital output, audio mute (default high until radio completes initialisation)
  GP15 digital output, PWM (CW tone in TX mode with key shaping)
  GP16 digital input, MISO (unused)
  GP17 digital output, CS (LCD)
  GP18 digital output, SCK (LCD)
  GP19 digital output, MOSI (LCD)
  GP20 digital output, DC (LCD)
  GP21 digital output, RST (LCD)
  GP22 digital input, pullup enabled (TP11 unused)
  GP23 (internal to Pico)
  GP24 (internal to Pico)
  GP25 digital output, low (Pico onboard LED – could be used for status info)
  GP26 analogue input (QSD I)
  GP27 analogue input (QSD Q)
  GP28 analogue input (AGC level)

 */
#ifndef Radio_h
#define Radio_h

#include "Arduino.h"
#include "si5351A.h"

class Radio
{
  public:
    enum modes_t {XXX, LSB, USB, CWL, CWU, DIGL, DIGU};
    enum bands_t {BANDXX, BAND80, BAND40, BAND20, BAND15, BAND10};
    enum filter_t {FILTER_XXX, FILTER_SSB, FILTER_CW, FILTER_DIG};
    uint32_t frequency = 0;
    uint32_t tuning_step = 0;
    uint32_t scope_speed = 1;
    uint32_t scope_zoom = 0;
    Radio::modes_t mode = Radio::XXX;
    Radio::bands_t band = Radio::BANDXX;
    void init(void);
    void process(void);
    void mute(void);
    void unMute(void);
    void LEDon(void);
    void LEDoff(void);
    void attOn(void);
    void attOff(void);
    void lock(void);
    void unlock(void);
    void txEnable(void);
    void rxEnable(void);
    void muteMic(void);
    void unmuteMic(void);
    void cwToneStart(void);
    void cwToneStop(void);
    void cwStop(void);
    const boolean band_io_error(void);
    const boolean filter_io_error(void);
    const boolean txEnabled(void);
    const boolean rxEnabled(void);
    const boolean isLocked(void);
    const boolean PTT(void);
    const boolean DSENSE(void);
    const boolean paddleA(void);
    const boolean paddleB(void);
    const boolean tuneButton(void);
    const boolean multiButton(void);
    const boolean attEnabled(void);
    const int32_t Tune(void);
    const int32_t Func(void);
    void setBand(const Radio::bands_t new_band);
    void setFilter(const Radio::filter_t new_filter);
    const uint32_t band_index(const Radio::bands_t band);

    static const uint32_t PIN_ENC1A = 7u;
    static const uint32_t PIN_ENC1B = 6u;
    static const uint32_t PIN_ENC2A = 9u;
    static const uint32_t PIN_ENC2B = 8u;
    static const uint32_t PIN_QSDI = 26u;
    static const uint32_t PIN_QSDQ = 27u;
    static const uint32_t PIN_AGC = 28u;
    static const uint32_t ADC_QSDI = 0u;
    static const uint32_t ADC_QSDQ = 1u;
    static const uint16_t NUM_BANDS = 5u;
    
    Radio(
      const uint32_t _frequency,
      const uint32_t _tuning_step,
      const Radio::modes_t _mode,
      const Radio::bands_t _band);
  private:
    static const uint32_t PIN_PTT = 0u;
    static const uint32_t PIN_TP10 = 1u;
    static const uint32_t PIN_PADA = 2u;
    static const uint32_t PIN_PADB = 3u;
    static const uint32_t PIN_ENC1BUT = 10u;
    static const uint32_t PIN_ENC2BUT = 11u;
    static const uint32_t PIN_DSENSE = 12u;
    static const uint32_t PIN_CWSIDETONE = 13u;
    static const uint32_t PIN_MUTE = 14u;
    static const uint32_t PIN_CWTONE = 15u;
    static const uint32_t PIN_MISO = 16u;
    static const uint32_t PIN_CS = 17u;
    static const uint32_t PIN_SCK = 18u;
    static const uint32_t PIN_MOSI = 19u;
    static const uint32_t PIN_DC = 20u;
    static const uint32_t PIN_RST = 21u;
    static const uint32_t PIN_TP11 = 22u;
    static const uint32_t PIN_PICOLED = 25u;
    static const uint8_t BAND_BIT_B80 = 6u; // B1
    static const uint8_t BAND_BIT_B40 = 3u; // B2
    static const uint8_t BAND_BIT_B20 = 2u; // B3
    static const uint8_t BAND_BIT_B15 = 1u; // B4
    static const uint8_t BAND_BIT_B10 = 0u; // B5
    static const uint8_t BAND_BIT_TX  = 5u; // TX enable
    static const uint8_t BAND_BIT_CWMIC  = 4u; // CW or Mic enable
    static const uint8_t FILTER_BIT_SSB = 0u; // ~SSB
    static const uint8_t FILTER_BIT_CW  = 1u; // CW
    static const uint8_t FILTER_BIT_DIG = 2u; // DIG
    static const uint8_t FILTER_BIT_ATT = 3u; // ATT
    static const uint8_t FILTER_BIT_SP1 = 6u; // Spare 1
    static const uint8_t FILTER_BIT_SP2 = 5u; // Spare 2
    static const uint8_t FILTER_BIT_SP3 = 4u; // Spare 3
    bool _i2c_band_error = false;
    bool _i2c_filter_error = false;
    int32_t _tune_value;
    int32_t _func_value;
    bool _tx_enable;
    bool _locked;
    bool _att_enabled;
    bands_t _current_band;
    filter_t _current_filter;
};

#endif
