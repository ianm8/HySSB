/*
 * HySSB Copyright 2022 Ian Mitchell VK7IAN Version 1.0
 *
 * HySSB006 - first working version SSB only
 * HySSB007 - mute mic on receive (mic feeds back via mixer on receive!)
 * HySSB008 - mode changes
 * HySSB009 - CW 10 bits, spectrum zooming
 * HySSB010 - CW Speed
 * HySSB011 - EEPROM save and digital mode BFO change by +/-100Hz
 * HySSB012 - Show WPM on screan
 * 
 * libraries used:
 *   https://github.com/etherkit/Si5351Arduino
 *   https://github.com/Bodmer/TFT_eSPI
 *   https://github.com/brianlow/Rotary
 *
 * NOTE: copy User_Setup.h to ..\Arduino\libraries\TFT_eSPI-master 
 * after first install or if the library is updated
 * 
 * Need to set in ..\TFT_Drivers\ST7789_2_Defines.h
 * #define TFT_MAD_BGR 0x00
 * ie, change 0x08 to 0x00
 *
 *
 * multi-functions:
 *   band BND: (80,40,20,15,10)
 *   mode MOD: (LSB,USB,CWL,CWU,...)
 *   lock LCK: (lock, unlock)
 *   attenuator: (ATT)
 *   TODO: spectrum options (width, speed (averaging)) SPE WAT SPM
 *   TODO: measure speed of FFT/ADC
 *   
*/

#include "Radio.h"
#include "Spectrum.h"
#include "si5351A.h"
#include <EEPROM.h>
#include <TFT_eSPI.h>                 

#define CALL_SIGN "VK7IAN"
#define VERSION "1.0"
#define CW_SPEED_DEFAULT 60ul;
#define CW_TIMEOUT 800UL
#define MULTIFUNCTION_TIMEOUT 4000UL
#define MESSAGE_TIMEOUT 2000UL


#define WATERFALL_ROWS 41

#define WIDTH  240
#define HEIGHT 135
#define POS_SPLASH_X       80
#define POS_SPLASH_Y       50
#define POS_FREQUENCY_X    80
#define POS_FREQUENCY_Y     0
#define POS_TX_X           10
#define POS_TX_Y            5
#define POS_RX_X           40
#define POS_RX_Y            5
#define POS_MODE_X         10
#define POS_MODE_Y         30
#define POS_ATT_X         190
#define POS_ATT_Y          30
#define POS_MULTI_X        55
#define POS_MULTI_Y        30
#define POS_MULTIVALUE_X   80
#define POS_MULTIVALUE_Y   65
#define POS_METER_X       100
#define POS_METER_Y        25
#define POS_TUNING_STEP_X 140
#define POS_TUNING_STEP_Y  40
#define POS_WATER_X         0
#define POS_WATER_Y        62
#define POS_CENTER_LEFT   119
#define POS_CENTER_RIGHT  120
#define BANDWIDTH_SHADE 0x0010

// radio state
enum state_t
{
  STATE_NO_STATE,
  STATE_LSB_RECEIVE_INIT,
  STATE_LSB_RECEIVE,
  STATE_LSB_TX_INIT,
  STATE_LSB_TX,
  STATE_USB_RECEIVE_INIT,
  STATE_USB_RECEIVE,
  STATE_USB_TX_INIT,
  STATE_USB_TX,
  STATE_CWL_RECEIVE_INIT,
  STATE_CWL_RECEIVE,
  STATE_CWL_TX_INIT,
  STATE_CWL_TX,
  STATE_CWU_RECEIVE_INIT,
  STATE_CWU_RECEIVE,
  STATE_CWU_TX_INIT,
  STATE_CWU_TX,
  STATE_DIGL_RECEIVE_INIT,
  STATE_DIGL_RECEIVE,
  STATE_DIGL_TX_INIT,
  STATE_DIGL_TX,
  STATE_DIGU_RECEIVE_INIT,
  STATE_DIGU_RECEIVE,
  STATE_DIGU_TX_INIT,
  STATE_DIGU_TX,
  STATE_STEP_CHANGE,
  STATE_STEP_WAIT
};

enum func_state_t
{
  FUNCTION_STATE_IDLE,
  FUNCTION_STATE_CHANGE_INIT,
  FUNCTION_STATE_VALUE_INIT,
  FUNCTION_STATE_VALUE_CHANGE,
  FUNCTION_STATE_WAIT_BUTTON_1,
  FUNCTION_STATE_WAIT_BUTTON_2,
  FUNCTION_STATE_CHANGE
};

enum functions_t
{
  FUNCTION_NONE,
  FUNCTION_BAND,
  FUNCTION_MODE,
  FUNCTION_LOCK,
  FUNCTION_ATTN,
  FUNCTION_BSCP,
  FUNCTION_CWSP
};

enum lock_t
{
  LOCKED,
  UNLOCKED
};

enum atten_t
{
  ATTN_ON,
  ATTN_OFF
};

enum wpm_t
{
  CW_WPM_10,
  CW_WPM_11,
  CW_WPM_12,
  CW_WPM_13,
  CW_WPM_14,
  CW_WPM_15,
  CW_WPM_16,
  CW_WPM_17,
  CW_WPM_18,
  CW_WPM_19,
  CW_WPM_20,
  CW_WPM_21,
  CW_WPM_22,
  CW_WPM_23,
  CW_WPM_24,
  CW_WPM_25,
  CW_WPM_26,
  CW_WPM_27,
  CW_WPM_28,
  CW_WPM_29,
  CW_WPM_30
};

enum scopeoption_t
{
  SCOPE_SPEED_1,
  SCOPE_SPEED_2,
  SCOPE_SPEED_3,
  SCOPE_SPEED_4,
  SCOPE_ZOOM_0,
  SCOPE_ZOOM_1,
  SCOPE_ZOOM_2
};

enum messages_t
{
  MESSAGE_NO_MESSAGE,
  MESSAGE_LOCKED
};

struct band_save_t
{
  uint32_t frequency;
  uint32_t tuning_step;
  Radio::modes_t mode;
  atten_t atten;
};

struct multifunc_t
{
  func_state_t state;
  functions_t current_function;
  functions_t new_function;
  functions_t value_change;
  Radio::bands_t current_value_band;
  Radio::bands_t new_value_band;
  Radio::modes_t current_value_mode;
  Radio::modes_t new_value_mode;
  lock_t current_value_lock;
  lock_t new_value_lock;
  atten_t current_value_atten;
  atten_t new_value_atten;
  wpm_t current_value_wpm;
  wpm_t new_value_wpm;
  scopeoption_t current_value_scopeoption;
  scopeoption_t new_value_scopeoption;
  boolean highlight;
  uint32_t timeout;
};

struct message_t
{
  messages_t message;
  uint32_t timeout;
};

// colours used in the waterfall
static const uint16_t color_map_16[16] =
{
  0x0000, // black
  0x0010, // blue
  0x0018,
  0x001f,
  0x4208,
  0x630c,
  0x8410,
  0xfff0,
  0xffe8,
  0xffe0,
  0xfc00,
  0xfdc8,
  0xfe73,
  0xfb2c,
  0xf986,
  0xf800  // red
};

static const uint16_t color_map_32[32] =
{
  0x0000, // black
  0x0004, // black
  0x0008, // blue
  0x000c, // blue
  0x0010,
  0x0014,
  0x0018,
  0x001c,
  0x4208,
  0x4210,
  0x630c,
  0x631c,
  0x8410,
  0x8410,
  0xffe0,
  0xfff0,
  0xffe8,
  0xffe8,
  0xffe0,
  0xffe0,
  0xfc00,
  0xfd00,
  0xfdc8,
  0xfdc8,
  0xfe73,
  0xfe73,
  0xfb2c,
  0xfb2c,
  0xf986,
  0xf986,
  0xf600, // red
  0xff00  // red
};

static uint32_t cw_dit = CW_SPEED_DEFAULT;
static state_t radio_state = STATE_LSB_RECEIVE_INIT;
static state_t saved_state = STATE_NO_STATE;
static state_t next_state = STATE_NO_STATE;
static struct repeating_timer radio_timer;
static uint8_t spectrum_data[N_WAVE];
static uint8_t spectrum_buffer[N_WAVE];
volatile static uint32_t wp = 0;
static uint8_t water[WATERFALL_ROWS][WIDTH] = {0};

static multifunc_t multifunc =
{
  FUNCTION_STATE_IDLE,
  FUNCTION_BAND,
  FUNCTION_BAND,
  FUNCTION_NONE,
  Radio::BAND40,
  Radio::BAND40,
  Radio::LSB,
  Radio::LSB,
  UNLOCKED,
  UNLOCKED,
  ATTN_OFF,
  ATTN_OFF,
  CW_WPM_20,
  CW_WPM_20,
  SCOPE_SPEED_1,
  SCOPE_SPEED_1,
  false,
  0
};

static message_t message =
{
  MESSAGE_NO_MESSAGE,
  0UL
};

static band_save_t band_save[Radio::NUM_BANDS] =
{
  {3600000UL,  1000UL, Radio::LSB, ATTN_ON},
  {7100000UL,  1000UL, Radio::LSB, ATTN_OFF},
  {14100000UL, 1000UL, Radio::USB, ATTN_OFF},
  {21100000UL, 1000UL, Radio::USB, ATTN_OFF},
  {28480000UL, 1000UL, Radio::USB, ATTN_OFF}
};

auto_init_mutex(spectrum_mutex);

const uint32_t FREQUENCY = 7105000UL;         // The starting frequency in Hz
const uint32_t STEP = 1000UL;                 // The starting tuning step in Hz
const Si5351A::modes_t MODE = Si5351A::LSB;   // The starting mode
const uint32_t CORRECTION = 0;                // Using TCXO

Radio radio(FREQUENCY,STEP,Radio::LSB,Radio::BAND40); // object to abstract radio hardware
Spectrum spectrum;                            // calculate the frequency spectrum (runs on core 1)
Si5351A si5351A;                              // Create a Si5351 object and set the frequency correction

// TFT control object
TFT_eSPI tft = TFT_eSPI();

// sprite object with pointer to TFT object,
// this will be the "display buffer"
TFT_eSprite spr = TFT_eSprite(&tft);

// if an error occurs during startup, flash
// the error number on the LED
static void error_stop(const uint32_t _errno)
{
  for (;;)
  {
    for (uint32_t i=0;i<_errno;i++)
    {
      radio.LEDon();
      delay(50);
      radio.LEDoff();
      delay(500);
    }
    delay(2000);
  }
}

static bool process_radio_callback(repeating_timer_t *rt)
{
  radio.process();
  return true;
}

static void save_settings(void)
{
  EEPROM.begin(256);
  EEPROM.write(0,(uint8_t)radio.scope_speed);
  EEPROM.write(1,(uint8_t)radio.scope_zoom);
  EEPROM.write(2,(uint8_t)cw_dit);
  EEPROM.commit();
  EEPROM.end();
}

static void restore_settings(void)
{
  EEPROM.begin(256);
  radio.scope_speed = EEPROM.read(0);
  radio.scope_zoom = EEPROM.read(1);
  cw_dit = EEPROM.read(2);
  EEPROM.end();
  if (radio.scope_speed<0 ||
    radio.scope_speed>8 ||
    radio.scope_zoom<0 ||
    radio.scope_zoom>3 ||
    cw_dit<40 ||
    cw_dit>120)
  {
    radio.scope_speed = 1;
    radio.scope_zoom = 0;
    cw_dit = CW_SPEED_DEFAULT;
  }
}

void setup(void)
{
  // set pico regulator to low noise
  pinMode(23,OUTPUT);
  digitalWrite(23,HIGH);
  
  pinMode(LED_BUILTIN,OUTPUT);
  for (uint32_t i=0;i<2;i++)
  {
    digitalWrite(LED_BUILTIN,HIGH);
    delay(10);
    digitalWrite(LED_BUILTIN,LOW);
    delay(100);
  }
  delay(500);

  radio.init();
  restore_settings();

  if (!add_repeating_timer_us(-1000LL, process_radio_callback, NULL, &radio_timer))
  {
    error_stop(5U);
  }
  if (!si5351A.begin(FREQUENCY, MODE, CORRECTION))
  {
    error_stop(6U);
  }

  tft.init();
  tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);

  // Optionally set colour depth to 8 or 16 bits, default is 16 if not specified
  // spr.setColorDepth(8);

  // Create a sprite of defined size
  spr.createSprite(WIDTH,HEIGHT);
  spr.fillSprite(TFT_BLACK);
  spr.pushSprite(0,0);
  delay(2000);
  spr.setTextSize(3);
  for (uint32_t i=0;i<16;i++)
  {
    spr.fillSprite(TFT_BLACK);
    spr.setTextColor(color_map_16[i],TFT_BLACK);
    spr.setCursor(POS_SPLASH_X,POS_SPLASH_Y);
    spr.print(CALL_SIGN);
    spr.pushSprite(0,0);
    delay(200);
  }
  spr.fillSprite(TFT_BLACK);
  spr.pushSprite(0,0);
}

static void show_frequency(void)
{
  spr.setTextSize(3);
  spr.setTextColor(radio.isLocked()?TFT_RED:TFT_WHITE,TFT_BLACK);
  spr.setCursor(POS_FREQUENCY_X,POS_FREQUENCY_Y);
  if (radio.frequency<10000000UL) spr.print(" ");
  if (radio.frequency<1000000UL) spr.print(" ");
  spr.print(radio.frequency);
}

static void show_tuning_step(void)
{
  spr.setTextSize(1);
  spr.setCursor(POS_TUNING_STEP_X-30,POS_TUNING_STEP_Y);
  spr.setTextColor(TFT_WHITE);
  spr.print("STEP");
  spr.setCursor(POS_TUNING_STEP_X,POS_TUNING_STEP_Y);
  spr.setTextColor(TFT_WHITE);
  spr.print(radio.tuning_step);
}

static void show_tx(void)
{
  spr.setTextSize(2);
  spr.setCursor(POS_TX_X,POS_TX_Y);
  spr.setTextColor(TFT_WHITE);
  spr.print("TX");
  spr.setCursor(POS_RX_X,POS_RX_Y);
  spr.setTextColor(TFT_RED);
  spr.print("RX");
}

static void show_rx(void)
{
  spr.setTextSize(2);
  spr.setCursor(POS_TX_X,POS_TX_Y);
  spr.setTextColor(TFT_RED);
  spr.print("TX");
  spr.setCursor(POS_RX_X,POS_RX_Y);
  spr.setTextColor(TFT_WHITE);
  spr.print("RX");
}

static void show_rx_tx(void)
{
  if (radio.txEnabled())
  {
    show_tx();
  }
  else
  {
    show_rx();
  }
}

static void show_mode()
{
  spr.fillRect(POS_MODE_X-5,POS_MODE_Y-5,45,25,TFT_WHITE);
  spr.setTextSize(2);
  spr.setTextColor(TFT_BLACK);
  spr.setCursor(POS_MODE_X,POS_MODE_Y);
  const char *sz_mode = "XXX";
  switch (radio.mode)
  {
    case Radio::LSB: sz_mode = "LSB"; break;
    case Radio::USB: sz_mode = "USB"; break;
    case Radio::CWL: sz_mode = "CWL"; break;
    case Radio::CWU: sz_mode = "CWU"; break;
    case Radio::DIGL: sz_mode = "DGL"; break;
    case Radio::DIGU: sz_mode = "DGU"; break;
  }
  spr.print(sz_mode);
}

static void show_meter_dial(void)
{
  uint32_t v = spectrum.AGC;
  if (radio.attEnabled() && v>0)
  {
    v += 3;
  }
  v = min(v,15);
  spr.setTextSize(1);
  spr.setCursor(POS_METER_X,POS_METER_Y);
  spr.setTextColor(TFT_WHITE);
  spr.print("1 3 5 7 9 +20");
  for (uint32_t i=0;i<v;i++)
  {
    spr.fillRect(POS_METER_X+i*5+0,POS_METER_Y+8,4,4,TFT_WHITE);
  }
}

static void show_attenuator(void)
{
  if (radio.attEnabled())
  {
    spr.fillRect(POS_ATT_X-5,POS_ATT_Y-5,45,25,TFT_GREEN);
    spr.setTextSize(2);
    spr.setTextColor(TFT_BLACK);
    spr.setCursor(POS_ATT_X,POS_ATT_Y);
    spr.print("ATT");
  }
  else if (radio.mode==Radio::CWL || radio.mode==Radio::CWU)
  {
    spr.fillRect(POS_ATT_X-5,POS_ATT_Y-5,45,25,TFT_PURPLE);
    spr.setTextSize(2);
    spr.setTextColor(TFT_WHITE);
    spr.setCursor(POS_ATT_X,POS_ATT_Y);
    switch (multifunc.current_value_wpm)
    {
      case CW_WPM_10: spr.print("10"); break;
      case CW_WPM_11: spr.print("11"); break;
      case CW_WPM_12: spr.print("12"); break;
      case CW_WPM_13: spr.print("13"); break;
      case CW_WPM_14: spr.print("14"); break;
      case CW_WPM_15: spr.print("15"); break;
      case CW_WPM_16: spr.print("16"); break;
      case CW_WPM_17: spr.print("17"); break;
      case CW_WPM_18: spr.print("18"); break;
      case CW_WPM_19: spr.print("19"); break;
      case CW_WPM_20: spr.print("20"); break;
      case CW_WPM_21: spr.print("21"); break;
      case CW_WPM_22: spr.print("22"); break;
      case CW_WPM_23: spr.print("23"); break;
      case CW_WPM_24: spr.print("24"); break;
      case CW_WPM_25: spr.print("25"); break;
      case CW_WPM_26: spr.print("26"); break;
      case CW_WPM_27: spr.print("27"); break;
      case CW_WPM_28: spr.print("28"); break;
      case CW_WPM_29: spr.print("29"); break;
      case CW_WPM_30: spr.print("30"); break;
    }
  }
  else
  {
    spr.fillRect(POS_ATT_X-5,POS_ATT_Y-5,45,25,TFT_PURPLE);
    spr.setTextSize(2);
    spr.setTextColor(TFT_WHITE);
    spr.setCursor(POS_ATT_X,POS_ATT_Y);
    spr.print(VERSION);
  }
}

static void show_bandwidth(void)
{
  spr.setTextSize(1);
  spr.setTextColor(TFT_WHITE);
  switch (radio.scope_zoom)
  {
    case 0:
    {
      // for 60KHz scope 250Hz / pixel
      spr.setCursor(0,POS_WATER_Y+4);
      spr.print("-30KHz");
      spr.setCursor(WIDTH-40,POS_WATER_Y+4);
      spr.print("+30KHz");
      break;
    }
    case 1:
    {
      // for 30KHz scope 125Hz / pixel
      spr.setCursor(0,POS_WATER_Y+4);
      spr.print("-15KHz");
      spr.setCursor(WIDTH-40,POS_WATER_Y+4);
      spr.print("+15KHz");
      break;
    }
    case 2:
    {
      // for 15KHz scope 63Hz / pixel
      spr.setCursor(0,POS_WATER_Y+4);
      spr.print("-8KHz");
      spr.setCursor(WIDTH-34,POS_WATER_Y+4);
      spr.print("+8KHz");
      break;
    }
  }
  if (radio.txEnabled())
  {
    switch (radio.scope_zoom)
    {
      case 0:
      {
        // for 60KHz scope 250Hz / pixel
        switch (radio.mode)
        {
          case Radio::LSB:
          case Radio::USB:
          {
            for (uint32_t x=0;x<5;x++)
            {
              spr.drawLine(POS_CENTER_LEFT-x,POS_WATER_Y,POS_CENTER_LEFT-x,POS_WATER_Y+31,BANDWIDTH_SHADE);
              spr.drawLine(POS_CENTER_RIGHT+x,POS_WATER_Y,POS_CENTER_RIGHT+x,POS_WATER_Y+31,BANDWIDTH_SHADE);
            }
            break;
          }
          case Radio::CWL:
          case Radio::CWU:
          {
            for (uint32_t x=0;x<3;x++)
            {
              spr.drawLine(POS_CENTER_LEFT-x,POS_WATER_Y,POS_CENTER_LEFT-x,POS_WATER_Y+31,BANDWIDTH_SHADE);
              spr.drawLine(POS_CENTER_RIGHT+x,POS_WATER_Y,POS_CENTER_RIGHT+x,POS_WATER_Y+31,BANDWIDTH_SHADE);
            }
            break;
          }
          case Radio::DIGL:
          case Radio::DIGU:
          {
            for (uint32_t x=0;x<7;x++)
            {
              spr.drawLine(POS_CENTER_LEFT-x,POS_WATER_Y,POS_CENTER_LEFT-x,POS_WATER_Y+31,BANDWIDTH_SHADE);
              spr.drawLine(POS_CENTER_RIGHT+x,POS_WATER_Y,POS_CENTER_RIGHT+x,POS_WATER_Y+31,BANDWIDTH_SHADE);
            }
            break;
          }
        }
        break;      
      }
      case 1:
      {
        // for 30KHz scope 125Hz / pixel
        switch (radio.mode)
        {
          case Radio::LSB:
          case Radio::USB:
          {
            for (uint32_t x=0;x<10;x++)
            {
              spr.drawLine(POS_CENTER_LEFT-x,POS_WATER_Y,POS_CENTER_LEFT-x,POS_WATER_Y+31,BANDWIDTH_SHADE);
              spr.drawLine(POS_CENTER_RIGHT+x,POS_WATER_Y,POS_CENTER_RIGHT+x,POS_WATER_Y+31,BANDWIDTH_SHADE);
            }
            break;
          }
          case Radio::CWL:
          case Radio::CWU:
          {
            for (uint32_t x=0;x<6;x++)
            {
              spr.drawLine(POS_CENTER_LEFT-x,POS_WATER_Y,POS_CENTER_LEFT-x,POS_WATER_Y+31,BANDWIDTH_SHADE);
              spr.drawLine(POS_CENTER_RIGHT+x,POS_WATER_Y,POS_CENTER_RIGHT+x,POS_WATER_Y+31,BANDWIDTH_SHADE);
            }
            break;
          }
          case Radio::DIGL:
          case Radio::DIGU:
          {
            for (uint32_t x=0;x<14;x++)
            {
              spr.drawLine(POS_CENTER_LEFT-x,POS_WATER_Y,POS_CENTER_LEFT-x,POS_WATER_Y+31,BANDWIDTH_SHADE);
              spr.drawLine(POS_CENTER_RIGHT+x,POS_WATER_Y,POS_CENTER_RIGHT+x,POS_WATER_Y+31,BANDWIDTH_SHADE);
            }
            break;
          }
        }
        break;      
      }
      case 2:
      {
        // for 15KHz scope 63Hz / pixel
        switch (radio.mode)
        {
          case Radio::LSB:
          case Radio::USB:
          {
            for (uint32_t x=0;x<20;x++)
            {
              spr.drawLine(POS_CENTER_LEFT-x,POS_WATER_Y,POS_CENTER_LEFT-x,POS_WATER_Y+31,BANDWIDTH_SHADE);
              spr.drawLine(POS_CENTER_RIGHT+x,POS_WATER_Y,POS_CENTER_RIGHT+x,POS_WATER_Y+31,BANDWIDTH_SHADE);
            }
            break;
          }
          case Radio::CWL:
          case Radio::CWU:
          {
            for (uint32_t x=0;x<10;x++)
            {
              spr.drawLine(POS_CENTER_LEFT-x,POS_WATER_Y,POS_CENTER_LEFT-x,POS_WATER_Y+31,BANDWIDTH_SHADE);
              spr.drawLine(POS_CENTER_RIGHT+x,POS_WATER_Y,POS_CENTER_RIGHT+x,POS_WATER_Y+31,BANDWIDTH_SHADE);
            }
            break;
          }
          case Radio::DIGL:
          case Radio::DIGU:
          {
            for (uint32_t x=0;x<25;x++)
            {
              spr.drawLine(POS_CENTER_LEFT-x,POS_WATER_Y,POS_CENTER_LEFT-x,POS_WATER_Y+31,BANDWIDTH_SHADE);
              spr.drawLine(POS_CENTER_RIGHT+x,POS_WATER_Y,POS_CENTER_RIGHT+x,POS_WATER_Y+31,BANDWIDTH_SHADE);
            }
            break;
          }
        }
        break;      
      }
    }
  }
  else
  {
    // receiving
    switch (radio.scope_zoom)
    {
      case 0:
      {
        // for 60KHz scope 250Hz / pixel
        switch (radio.mode)
        {
          case Radio::LSB:
          {
            for (uint32_t x=0;x<10;x++)
            {
              spr.drawLine(POS_CENTER_LEFT-x,POS_WATER_Y,POS_CENTER_LEFT-x,POS_WATER_Y+31,BANDWIDTH_SHADE);
            }
            break;
          }
          case Radio::USB:
          {
            for (uint32_t x=0;x<10;x++)
            {
              spr.drawLine(POS_CENTER_RIGHT+x,POS_WATER_Y,POS_CENTER_RIGHT+x,POS_WATER_Y+31,BANDWIDTH_SHADE);
            }
            break;
          }
          case Radio::CWL:
          {
            for (uint32_t x=0;x<6;x++)
            {
              spr.drawLine(POS_CENTER_LEFT-x,POS_WATER_Y,POS_CENTER_LEFT-x,POS_WATER_Y+31,BANDWIDTH_SHADE);
            }
            break;
          }
          case Radio::CWU:
          {
            for (uint32_t x=0;x<6;x++)
            {
              spr.drawLine(POS_CENTER_RIGHT+x,POS_WATER_Y,POS_CENTER_RIGHT+x,POS_WATER_Y+31,BANDWIDTH_SHADE);
            }
            break;
          }
          case Radio::DIGL:
          {
            for (uint32_t x=0;x<14;x++)
            {
              spr.drawLine(POS_CENTER_LEFT-x,POS_WATER_Y,POS_CENTER_LEFT-x,POS_WATER_Y+31,BANDWIDTH_SHADE);
            }
            break;
          }
          case Radio::DIGU:
          {
            for (uint32_t x=0;x<14;x++)
            {
              spr.drawLine(POS_CENTER_RIGHT+x,POS_WATER_Y,POS_CENTER_RIGHT+x,POS_WATER_Y+31,BANDWIDTH_SHADE);
            }
            break;
          }
        }
        break;      
      }
      case 1:
      {
        // for 30KHz scope 125Hz / pixel
        switch (radio.mode)
        {
          case Radio::LSB:
          {
            for (uint32_t x=0;x<20;x++)
            {
              spr.drawLine(POS_CENTER_LEFT-x,POS_WATER_Y,POS_CENTER_LEFT-x,POS_WATER_Y+31,BANDWIDTH_SHADE);
            }
            break;
          }
          case Radio::USB:
          {
            for (uint32_t x=0;x<20;x++)
            {
              spr.drawLine(POS_CENTER_RIGHT+x,POS_WATER_Y,POS_CENTER_RIGHT+x,POS_WATER_Y+31,BANDWIDTH_SHADE);
            }
            break;
          }
          case Radio::CWL:
          {
            for (uint32_t x=0;x<12;x++)
            {
              spr.drawLine(POS_CENTER_LEFT-x,POS_WATER_Y,POS_CENTER_LEFT-x,POS_WATER_Y+31,BANDWIDTH_SHADE);
            }
            break;
          }
          case Radio::CWU:
          {
            for (uint32_t x=0;x<12;x++)
            {
              spr.drawLine(POS_CENTER_RIGHT+x,POS_WATER_Y,POS_CENTER_RIGHT+x,POS_WATER_Y+31,BANDWIDTH_SHADE);
            }
            break;
          }
          case Radio::DIGL:
          {
            for (uint32_t x=0;x<28;x++)
            {
              spr.drawLine(POS_CENTER_LEFT-x,POS_WATER_Y,POS_CENTER_LEFT-x,POS_WATER_Y+31,BANDWIDTH_SHADE);
            }
            break;
          }
          case Radio::DIGU:
          {
            for (uint32_t x=0;x<28;x++)
            {
              spr.drawLine(POS_CENTER_RIGHT+x,POS_WATER_Y,POS_CENTER_RIGHT+x,POS_WATER_Y+31,BANDWIDTH_SHADE);
            }
            break;
          }
        }
        break;      
      }
      case 2:
      {
        // for 15KHz scope 63Hz / pixel
        switch (radio.mode)
        {
          case Radio::LSB:
          {
            for (uint32_t x=0;x<40;x++)
            {
              spr.drawLine(POS_CENTER_LEFT-x,POS_WATER_Y,POS_CENTER_LEFT-x,POS_WATER_Y+31,BANDWIDTH_SHADE);
            }
            break;
          }
          case Radio::USB:
          {
            for (uint32_t x=0;x<40;x++)
            {
              spr.drawLine(POS_CENTER_RIGHT+x,POS_WATER_Y,POS_CENTER_RIGHT+x,POS_WATER_Y+31,BANDWIDTH_SHADE);
            }
            break;
          }
          case Radio::CWL:
          {
            for (uint32_t x=0;x<20;x++)
            {
              spr.drawLine(POS_CENTER_LEFT-x,POS_WATER_Y,POS_CENTER_LEFT-x,POS_WATER_Y+31,BANDWIDTH_SHADE);
            }
            break;
          }
          case Radio::CWU:
          {
            for (uint32_t x=0;x<20;x++)
            {
              spr.drawLine(POS_CENTER_RIGHT+x,POS_WATER_Y,POS_CENTER_RIGHT+x,POS_WATER_Y+31,BANDWIDTH_SHADE);
            }
            break;
          }
          case Radio::DIGL:
          {
            for (uint32_t x=0;x<50;x++)
            {
              spr.drawLine(POS_CENTER_LEFT-x,POS_WATER_Y,POS_CENTER_LEFT-x,POS_WATER_Y+31,BANDWIDTH_SHADE);
            }
            break;
          }
          case Radio::DIGU:
          {
            for (uint32_t x=0;x<50;x++)
            {
              spr.drawLine(POS_CENTER_RIGHT+x,POS_WATER_Y,POS_CENTER_RIGHT+x,POS_WATER_Y+31,BANDWIDTH_SHADE);
            }
            break;
          }
        }
        break;      
      }
    }
  }
}

static void show_new_spectrum(void)
{
  switch (radio.scope_zoom)
  {
    case 0:
    {
      // this is about 60KHz wide
      for (uint32_t i=0;i<N_WAVE/4;i++)
      {
        uint8_t droplet = spectrum_buffer[i*4];
        droplet = max(droplet,spectrum_buffer[i*4+1]);
        droplet = max(droplet,spectrum_buffer[i*4+2]);
        droplet = max(droplet,spectrum_buffer[i*4+3]);
        spectrum_buffer[i] = droplet;
      }
      static const uint32_t buffer_start = (N_WAVE/4/2-1)-(WIDTH/2-1);
      for (uint32_t x=0;x<WIDTH;x++)
      {
        uint8_t droplet = spectrum_buffer[buffer_start+x];
        if (droplet>31) droplet = 31;
        water[wp][x] = droplet;
      }
      break;
    }
    case 1:
    {
      // this is about 30KHz wide
      for (uint32_t i=0;i<N_WAVE/4;i++) // 0 - 255 (<256)
      {
        uint8_t droplet = spectrum_buffer[256+i*2];        // 256, 258, 260,...,766
        droplet = max(droplet,spectrum_buffer[256+i*2+1]); // 257, 259, 262,...,767
        spectrum_buffer[i] = droplet;
      }
      static const uint32_t buffer_start = (N_WAVE/4/2-1)-(WIDTH/2-1);
      for (uint32_t x=0;x<WIDTH;x++)
      {
        uint8_t droplet = spectrum_buffer[buffer_start+x];
        if (droplet>31) droplet = 31;
        water[wp][x] = droplet;
      }
      break;
    }
    case 2:
    {
      // this is about 15KHz wide
      static const uint32_t buffer_start = (N_WAVE/2-1)-(WIDTH/2-1);
      for (uint32_t x=0;x<WIDTH;x++)
      {
        uint8_t droplet = spectrum_buffer[buffer_start+x];
        if (droplet>31) droplet = 31;
        water[wp][x] = droplet;
      }
      break;
    }
  }

  // draw the spectrum
  for (uint32_t x=0;x<WIDTH-1;x++)
  {
    const int32_t v0 = water[wp][x];
    const int32_t v1 = water[wp][x+1];
    const int32_t x0 = x;
    const int32_t y0 = POS_WATER_Y+31-v0;
    const int32_t x1 = x+1;
    const int32_t y1 = POS_WATER_Y+31-v1;
    spr.drawLine(x0,y0,x1,y1,TFT_WHITE);
  }

/*
  for (uint32_t x=0;x<WIDTH;x++)
  {
    const uint32_t droplet = water[wp][x];
    if (droplet==0) continue;
    spr.drawFastVLine(x,POS_WATER_Y+31-droplet,droplet,TFT_WHITE);
  }
*/
  // draw the waterfall
  int32_t r = wp;
  int32_t y = POS_WATER_Y+32;
  for (uint32_t i=0;i<WATERFALL_ROWS;i++,y++)
  {
    for (uint32_t x=0;x<WIDTH;x++)
    {
      const uint16_t c = color_map_32[water[r][x]];
      spr.drawPixel(x,y,c);
    }
    r--;
    if (r<0) r = WATERFALL_ROWS-1;
  }
  wp++;
  if (wp>=WATERFALL_ROWS) wp = 0;
}

static void show_old_spectrum(void)
{
  int32_t old_wp = wp-1;
  if (old_wp<0) old_wp = WATERFALL_ROWS-1;

  // draw the old spectrum
  for (uint32_t x=0;x<WIDTH-1;x++)
  {
    const int32_t v0 = water[old_wp][x];
    const int32_t v1 = water[old_wp][x+1];
    const int32_t x0 = x;
    const int32_t y0 = POS_WATER_Y+31-v0;
    const int32_t x1 = x+1;
    const int32_t y1 = POS_WATER_Y+31-v1;
    spr.drawLine(x0,y0,x1,y1,TFT_WHITE);
  }
/*
  for (uint32_t x=0;x<WIDTH;x++)
  {
    const uint32_t droplet = water[old_wp][x];
    if (droplet==0) continue;
    spr.drawFastVLine(x,POS_WATER_Y+31-droplet,droplet,TFT_WHITE);
  }
*/
  // draw the waterfall
  int32_t r = old_wp;
  int32_t y = POS_WATER_Y+32;
  for (uint32_t i=0;i<WATERFALL_ROWS;i++,y++)
  {
    for (uint32_t x=0;x<WIDTH;x++)
    {
      const uint16_t c = color_map_32[water[r][x]];
      spr.drawPixel(x,y,c);
    }
    r--;
    if (r<0) r = WATERFALL_ROWS-1;
  }
}

static void show_multifunc(void)
{
  // show the current multifunction function
  spr.fillRect(POS_MULTI_X-4,POS_MULTI_Y-5,45,25,multifunc.highlight?TFT_RED:TFT_PURPLE);
  spr.setTextSize(2);
  spr.setTextColor(TFT_WHITE);
  spr.setCursor(POS_MULTI_X,POS_MULTI_Y);
  const char *sz_func = "XXX";
  switch (multifunc.new_function)
  {
    case FUNCTION_BAND: sz_func = "BND"; break;
    case FUNCTION_MODE: sz_func = "MOD"; break;
    case FUNCTION_LOCK: sz_func = "LCK"; break;
    case FUNCTION_ATTN: sz_func = "ATT"; break;
    case FUNCTION_BSCP: sz_func = "SCP"; break;
    case FUNCTION_CWSP: sz_func = "WPM"; break;
  }
  spr.print(sz_func);
}

static void show_multifunc_value(void)
{
  if (multifunc.value_change==FUNCTION_NONE)
  {
    return;
  }
  // show the multifunction value
  static const uint32_t message_width = 9*14;
  static const uint32_t pos_message_x = WIDTH/2-message_width/2;
  spr.fillRect(pos_message_x-1,POS_MULTIVALUE_Y-1,message_width+2,26,TFT_WHITE);
  spr.fillRect(pos_message_x+1,POS_MULTIVALUE_Y+1,message_width-4,22,TFT_BLACK);
  spr.setTextSize(2);
  spr.setTextColor(TFT_WHITE);
  spr.setCursor(pos_message_x+8,POS_MULTIVALUE_Y+5);
  switch (multifunc.new_function)
  {
    case FUNCTION_BAND:
    {
      switch (multifunc.new_value_band)
      {
        case Radio::BAND80: spr.print("Band: 80M"); break;
        case Radio::BAND40: spr.print("Band: 40M"); break;
        case Radio::BAND20: spr.print("Band: 20M"); break;
        case Radio::BAND15: spr.print("Band: 15M"); break;
        case Radio::BAND10: spr.print("Band: 10M"); break;
      }
      break;
    }
    case FUNCTION_MODE:
    {
      switch (multifunc.new_value_mode)
      {
        case Radio::LSB:  spr.print("Mode: LSB"); break;
        case Radio::USB:  spr.print("Mode: USB"); break;
        case Radio::CWL:  spr.print("Mode: CWL"); break;
        case Radio::CWU:  spr.print("Mode: CWU"); break;
        case Radio::DIGL: spr.print("Mode: DGL"); break;
        case Radio::DIGU: spr.print("Mode: DGU"); break;
      }
      break;
    }
    case FUNCTION_LOCK:
    {
      switch (multifunc.new_value_lock)
      {                           
        case LOCKED:   spr.print("Lock: On");  break;
        case UNLOCKED: spr.print("Lock: Off"); break;
      }
      break;
    }
    case FUNCTION_ATTN: 
    {
      switch (multifunc.new_value_atten)
      {                           
        case ATTN_ON:  spr.print("Atten: On");  break;
        case ATTN_OFF: spr.print("Atten:Off"); break;
      }
      break;
    }
    case FUNCTION_BSCP:
    {
      switch (multifunc.new_value_scopeoption)
      {
        case SCOPE_SPEED_1: spr.print("Speed: 4"); break;
        case SCOPE_SPEED_2: spr.print("Speed: 3"); break;
        case SCOPE_SPEED_3: spr.print("Speed: 2"); break;
        case SCOPE_SPEED_4: spr.print("Speed: 1"); break;
        case SCOPE_ZOOM_0:  spr.print("Zoom: 0");  break;
        case SCOPE_ZOOM_1:  spr.print("Zoom: 1");  break;
        case SCOPE_ZOOM_2:  spr.print("Zoom: 2");  break;
      }
      break;
    }
    case FUNCTION_CWSP:
    {
      switch (multifunc.new_value_wpm)
      {
        case CW_WPM_10: spr.print("WPM: 10"); break;
        case CW_WPM_11: spr.print("WPM: 11"); break;
        case CW_WPM_12: spr.print("WPM: 12"); break;
        case CW_WPM_13: spr.print("WPM: 13"); break;
        case CW_WPM_14: spr.print("WPM: 14"); break;
        case CW_WPM_15: spr.print("WPM: 15"); break;
        case CW_WPM_16: spr.print("WPM: 16"); break;
        case CW_WPM_17: spr.print("WPM: 17"); break;
        case CW_WPM_18: spr.print("WPM: 18"); break;
        case CW_WPM_19: spr.print("WPM: 19"); break;
        case CW_WPM_20: spr.print("WPM: 20"); break;
        case CW_WPM_21: spr.print("WPM: 21"); break;
        case CW_WPM_22: spr.print("WPM: 22"); break;
        case CW_WPM_23: spr.print("WPM: 23"); break;
        case CW_WPM_24: spr.print("WPM: 24"); break;
        case CW_WPM_25: spr.print("WPM: 25"); break;
        case CW_WPM_26: spr.print("WPM: 26"); break;
        case CW_WPM_27: spr.print("WPM: 27"); break;
        case CW_WPM_28: spr.print("WPM: 28"); break;
        case CW_WPM_29: spr.print("WPM: 29"); break;
        case CW_WPM_30: spr.print("WPM: 30"); break;
      }
      break;
    }
  }
}

static void set_message(const messages_t msg)
{
  message.message = msg;
  message.timeout = millis()+MESSAGE_TIMEOUT;
}

static void show_message(void)
{
  if (message.message==MESSAGE_NO_MESSAGE)
  {
    return;
  }
  if (message.timeout>millis())
  {
    static const uint32_t message_width = 9*14;
    static const uint32_t pos_message_x = WIDTH/2-message_width/2;
    spr.fillRect(pos_message_x,POS_MULTIVALUE_Y,message_width,24,TFT_WHITE);
    spr.fillRect(pos_message_x+2,POS_MULTIVALUE_Y+2,message_width-4,20,TFT_BLACK);
    spr.setTextSize(2);
    spr.setTextColor(TFT_WHITE);
    spr.setCursor(pos_message_x+8,POS_MULTIVALUE_Y+5);
    const char *sz_message = "";
    switch (message.message)
    {
      case MESSAGE_LOCKED: sz_message = " LOCKED"; break;
    }
    spr.print(sz_message);
  }
  else
  {
    message.message = MESSAGE_NO_MESSAGE;
  }
}

static void display_clear(void)
{
  spr.fillSprite(TFT_BLACK);
}

static void display_refresh(void)
{
  spr.pushSprite(0,0);
}

void loop1(void)                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  
{
  spectrum.process(radio.scope_speed);

  // if the main loop is copying data,
  // just wait for it to complete
  mutex_enter_blocking(&spectrum_mutex);

  // copy spectrum data to spectrum data buffer
  for (uint32_t i=0;i<N_WAVE;i++)
  {
    spectrum_data[i] = spectrum.mag[i];
  }

  // indicate new data is available
  spectrum.dataReady();
  
  // main loop can now use the data
  mutex_exit(&spectrum_mutex);
}

void loop(void)
{
  static uint32_t cwtimeout = 0;
  
  if (multifunc.state==FUNCTION_STATE_IDLE)
  {
    switch (radio_state)
    {
      case STATE_LSB_RECEIVE_INIT:
      {
        if (!radio.rxEnabled())
        {
          radio.muteMic();
          radio.rxEnable();
        }
        if (radio.PTT())
        {
          // debounce until PTT is released
          break;
        }
        radio.setFilter(Radio::FILTER_SSB);
        si5351A.setFreq(radio.frequency,Si5351A::LSB);
        radio.mode = Radio::LSB;
        delay(100);
        radio.unMute();
        radio_state = STATE_LSB_RECEIVE;
        break;
      }
      case STATE_USB_RECEIVE_INIT:
      {
        if (!radio.rxEnabled())
        {
          radio.muteMic();
          radio.rxEnable();
        }
        if (radio.PTT())
        {
          // debounce until PTT is released
          break;
        }
        radio.setFilter(Radio::FILTER_SSB);
        si5351A.setFreq(radio.frequency,Si5351A::USB);
        radio.mode = Radio::USB;
        delay(100);
        radio.unMute();
        radio_state = STATE_USB_RECEIVE;
        break;
      }
      case STATE_CWL_RECEIVE_INIT:
      {
        if (!radio.rxEnabled())
        {
          radio.muteMic();
          radio.rxEnable();
        }
        if (radio.PTT())
        {
          // debounce until PTT is released
          break;
        }
        radio.setFilter(Radio::FILTER_CW);
        si5351A.setFreq(radio.frequency,Si5351A::CWL);
        radio.mode = Radio::CWL;
        delay(100);
        radio.unMute();
        radio_state = STATE_CWL_RECEIVE;
        break;
      }
      case STATE_CWU_RECEIVE_INIT:
      {
        if (!radio.rxEnabled())
        {
          radio.muteMic();
          radio.rxEnable();
        }
        if (radio.PTT())
        {
          // debounce until PTT is released
          break;
        }
        radio.setFilter(Radio::FILTER_CW);
        si5351A.setFreq(radio.frequency,Si5351A::CWU);
        radio.mode = Radio::CWU;
        delay(100);
        radio.unMute();
        radio_state = STATE_CWU_RECEIVE;
        break;
      }
      case STATE_DIGL_RECEIVE_INIT:
      {
        if (!radio.rxEnabled())
        {
          radio.muteMic();
          radio.rxEnable();
        }
        if (radio.PTT() || radio.DSENSE())
        {
          // debounce until PTT/DSENSE is released
          break;
        }
        radio.setFilter(Radio::FILTER_DIG);
        si5351A.setFreq(radio.frequency,Si5351A::DIGL);
        radio.mode = Radio::DIGL;
        delay(100);
        radio.unMute();
        radio_state = STATE_DIGL_RECEIVE;
        break;
      }
      case STATE_DIGU_RECEIVE_INIT:
      {
        if (!radio.rxEnabled())
        {
          radio.muteMic();
          radio.rxEnable();
        }
        if (radio.PTT() || radio.DSENSE())
        {
          // debounce until PTT/DSENSE is released
          break;
        }
        radio.setFilter(Radio::FILTER_DIG);
        si5351A.setFreq(radio.frequency,Si5351A::DIGU);
        radio.mode = Radio::DIGU;
        delay(100);
        radio.unMute();
        radio_state = STATE_DIGU_RECEIVE;
        break;
      }
      case STATE_LSB_RECEIVE:
      {
        // has tuning changed?
        const int32_t t = radio.Tune();
        if (t!=0)
        {
          if (radio.isLocked())
          {
            // if locked, can't change frequency
            set_message(MESSAGE_LOCKED);
            break;
          }
          uint32_t new_frequency = radio.frequency+radio.tuning_step*t;
          new_frequency -= new_frequency%radio.tuning_step;
          if (si5351A.setFreq(new_frequency))
          {
            radio.frequency = new_frequency;
          }
          break;
        }
        if (radio.tuneButton())
        {
          // pressed tuning step button
          saved_state = radio_state;
          radio_state = STATE_STEP_CHANGE;
          break;
        }
        if (radio.PTT())
        {
          // pressed PTT button
          radio_state = STATE_LSB_TX_INIT;
        }
        break;
      }
      case STATE_USB_RECEIVE:
      {
        // has tuning changed?
        const int32_t t = radio.Tune();
        if (t!=0)
        {
          if (radio.isLocked())
          {
            // if locked, can't change frequency
            set_message(MESSAGE_LOCKED);
            break;
          }
          uint32_t new_frequency = radio.frequency+radio.tuning_step*t;
          new_frequency -= new_frequency%radio.tuning_step;
          if (si5351A.setFreq(new_frequency))
          {
            radio.frequency = new_frequency;
          }
          break;
        }
        if (radio.tuneButton())
        {
          // pressed tuning step button
          saved_state = radio_state;
          radio_state = STATE_STEP_CHANGE;
          break;
        }
        if (radio.PTT())
        {
          // pressed PTT button
          radio_state = STATE_USB_TX_INIT;
        }
        break;
      }
      case STATE_CWL_RECEIVE:
      {
        // has tuning changed?
        const int32_t t = radio.Tune();
        if (t!=0)
        {
          if (radio.isLocked())
          {
            // if locked, can't change frequency
            set_message(MESSAGE_LOCKED);
            break;
          }
          uint32_t new_frequency = radio.frequency+radio.tuning_step*t;
          new_frequency -= new_frequency%radio.tuning_step;
          if (si5351A.setFreq(new_frequency))
          {
            radio.frequency = new_frequency;
          }
          break;
        }
        if (radio.tuneButton())
        {
          // pressed tuning step button
          saved_state = radio_state;
          radio_state = STATE_STEP_CHANGE;
          break;
        }
        if (radio.PTT() || radio.paddleA() || radio.paddleB())
        {
          // pressed PTT or paddle
          radio_state = STATE_CWL_TX_INIT;
        }
        break;
      }
      case STATE_CWU_RECEIVE:
      {
        // has tuning changed?
        const int32_t t = radio.Tune();
        if (t!=0)
        {
          if (radio.isLocked())
          {
            // if locked, can't change frequency
            set_message(MESSAGE_LOCKED);
            break;
          }
          uint32_t new_frequency = radio.frequency+radio.tuning_step*t;
          new_frequency -= new_frequency%radio.tuning_step;
          if (si5351A.setFreq(new_frequency))
          {
            radio.frequency = new_frequency;
          }
          break;
        }
        if (radio.tuneButton())
        {
          // pressed tuning step button
          saved_state = radio_state;
          radio_state = STATE_STEP_CHANGE;
          break;
        }
        if (radio.PTT() || radio.paddleA() || radio.paddleB())
        {
          // pressed PTT or paddle
          radio_state = STATE_CWU_TX_INIT;
        }
        break;
      }
      case STATE_DIGL_RECEIVE:
      {
        // has tuning changed?
        const int32_t t = radio.Tune();
        if (t!=0)
        {
          if (radio.isLocked())
          {
            // if locked, can't change frequency
            set_message(MESSAGE_LOCKED);
            break;
          }
          uint32_t new_frequency = radio.frequency+radio.tuning_step*t;
          new_frequency -= new_frequency%radio.tuning_step;
          if (si5351A.setFreq(new_frequency))
          {
            radio.frequency = new_frequency;
          }
          break;
        }
        if (radio.tuneButton())
        {
          // pressed tuning step button
          saved_state = radio_state;
          radio_state = STATE_STEP_CHANGE;
          break;
        }
        if (radio.PTT() || radio.DSENSE())
        {
          // pressed PTT button or DSENSE
          radio_state = STATE_DIGL_TX_INIT;
        }
        break;
      }
      case STATE_DIGU_RECEIVE:
      {
        // has tuning changed?
        const int32_t t = radio.Tune();
        if (t!=0)
        {
          if (radio.isLocked())
          {
            // if locked, can't change frequency
            set_message(MESSAGE_LOCKED);
            break;
          }
          uint32_t new_frequency = radio.frequency+radio.tuning_step*t;
          new_frequency -= new_frequency%radio.tuning_step;
          if (si5351A.setFreq(new_frequency))
          {
            radio.frequency = new_frequency;
          }
          break;
        }
        if (radio.tuneButton())
        {
          // pressed tuning step button
          saved_state = radio_state;
          radio_state = STATE_STEP_CHANGE;
          break;
        }
        if (radio.PTT() || radio.DSENSE())
        {
          // pressed PTT button or DSENSE
          radio_state = STATE_DIGU_TX_INIT;
        }
        break;
      }
      case STATE_LSB_TX_INIT:
      {
        radio.mute();
        radio.txEnable();
        radio.unmuteMic();
        radio_state = STATE_LSB_TX;
        break;
      }
      case STATE_USB_TX_INIT:
      {
        radio.mute();
        radio.txEnable();
        radio.unmuteMic();
        radio_state = STATE_USB_TX;
        break;
      }
      case STATE_CWL_TX_INIT:
      {
        radio.muteMic();
        radio.txEnable();
        if (radio.PTT()) delay(30);
        radio_state = STATE_CWL_TX;
        cwtimeout = millis()+CW_TIMEOUT;
        break;
      }
      case STATE_CWU_TX_INIT:
      {
        radio.muteMic();
        radio.txEnable();
        if (radio.PTT()) delay(30);
        cwtimeout = millis()+CW_TIMEOUT;
        radio_state = STATE_CWU_TX;
        break;
      }
      case STATE_DIGL_TX_INIT:
      {
        radio.mute();
        radio.txEnable();
        radio.unmuteMic();
        radio_state = STATE_DIGL_TX;
        break;
      }
      case STATE_DIGU_TX_INIT:
      {
        radio.mute();
        radio.txEnable();
        radio.unmuteMic();
        radio_state = STATE_DIGU_TX;
        break;
      }
      case STATE_LSB_TX:
      {
        // wait for PTT to release
        if (radio.PTT())
        {
          // PTT still in effect
          break;
        }
        // go back to receive
        radio_state = STATE_LSB_RECEIVE_INIT;
        break;
      }
      case STATE_USB_TX:
      {
        // wait for PTT to release
        if (radio.PTT())
        {
          // PTT still in effect
          break;
        }
        // go back to receive
        radio_state = STATE_USB_RECEIVE_INIT;
        break;
      }
      case STATE_CWL_TX:
      {
        // wait for PTT to release
        if (radio.PTT())
        {
          // PTT still in effect
          radio.cwToneStart();
          cwtimeout = millis()+CW_TIMEOUT;
          break;
        }
        if (radio.paddleA())
        {
          // dit
          delay(cw_dit);
          radio.cwToneStart();
          delay(cw_dit);
          radio.cwToneStop();
          cwtimeout = millis()+CW_TIMEOUT;
        }
        if (radio.paddleB())
        {
          // dah
          delay(cw_dit);
          radio.cwToneStart();
          delay(cw_dit * 3);
          radio.cwToneStop();
          cwtimeout = millis()+CW_TIMEOUT;
        }
        if (radio.paddleA() && radio.paddleB())
        {
          // dit
          delay(cw_dit);
          radio.cwToneStart();
          delay(cw_dit);
          radio.cwToneStop();
          // dah
          delay(cw_dit);
          radio.cwToneStart();
          delay(cw_dit * 3);
          radio.cwToneStop();
          cwtimeout = millis()+CW_TIMEOUT;
        }
        radio.cwToneStop();
        if (cwtimeout>millis())
        {
           // stay in transmit until timeout
           break;
        }
        // go back to receive
        radio.cwStop();
        radio_state = STATE_CWL_RECEIVE_INIT;
        break;
      }
      case STATE_CWU_TX:
      {
        // wait for PTT to release
        if (radio.PTT())
        {
          // PTT still in effect
          radio.cwToneStart();
          cwtimeout = millis()+CW_TIMEOUT;
          break;
        }
        if (radio.paddleA())
        {
          // dit
          delay(cw_dit);
          radio.cwToneStart();
          delay(cw_dit);
          radio.cwToneStop();
          cwtimeout = millis()+CW_TIMEOUT;
        }
        if (radio.paddleB())
        {
          // dah
          delay(cw_dit);
          radio.cwToneStart();
          delay(cw_dit * 3);
          radio.cwToneStop();
          cwtimeout = millis()+CW_TIMEOUT;
        }
        if (radio.paddleA() && radio.paddleB())
        {
          // dit
          delay(cw_dit);
          radio.cwToneStart();
          delay(cw_dit);
          radio.cwToneStop();
          // dah
          delay(cw_dit);
          radio.cwToneStart();
          delay(cw_dit * 3);
          radio.cwToneStop();
          cwtimeout = millis()+CW_TIMEOUT;
        }
        radio.cwToneStop();
        if (cwtimeout>millis())
        {
           // stay in transmit until timeout
           break;
        }
        // go back to receive
        radio.cwStop();
        radio_state = STATE_CWU_RECEIVE_INIT;
        break;
      }
      case STATE_DIGL_TX:
      {
        // wait for PTT/DSENSE to release
        if (radio.PTT() || radio.DSENSE())
        {
          // PTT still in effect
          break;
        }
        // go back to receive
        radio_state = STATE_DIGL_RECEIVE_INIT;
        break;
      }
      case STATE_DIGU_TX:
      {
        // wait for PTT/DSENSE to release
        if (radio.PTT() || radio.DSENSE())
        {
          // PTT still in effect
          break;
        }
        // go back to receive
        radio_state = STATE_DIGU_RECEIVE_INIT;
        break;
      }
      case STATE_STEP_CHANGE:
      {
        if (radio.isLocked())
        {
          // if locked, can't change frequency
          set_message(MESSAGE_LOCKED);
        }
        else
        {
          radio.tuning_step *= 10;
          if (radio.tuning_step>100000UL)
          {
            radio.tuning_step = 10UL;
          }
        }
        radio_state = STATE_STEP_WAIT;
        break;
      }
      case STATE_STEP_WAIT:
      {
        // wait for tuning step button release
        if (radio.tuneButton())
        {
          // if tuning button is held down while
          // rotating then change step
          if (!radio.isLocked())
          {
            const int32_t t = radio.Tune();
            if (t>=0)
            {
              for (uint8_t p=0;p<t;p++)
              {
                radio.tuning_step *= 10;
                if (radio.tuning_step>100000UL)
                {
                  radio.tuning_step = 100000UL;
                  break;
                }
              }
            }
            else
            {
              for (uint8_t p=0;p<-t;p++)
              {
                radio.tuning_step /= 10;
                if (radio.tuning_step==0)
                {
                  radio.tuning_step = 1;
                  break;
                }
              }
            }
          }
          break;
        }
        // clear the encoder value
        radio.Tune();
        radio_state = saved_state;
        break;
      }
    }
  }

  // process multifunctions
  if (radio.rxEnabled())
  {
    // only process multifunction in receive mode
    switch (multifunc.state)
    {
      case FUNCTION_STATE_IDLE:
      {
        // waiting for multifunction button or rotation
        if (radio.multiButton())
        {
          // init to change the current function
          multifunc.state = FUNCTION_STATE_CHANGE_INIT;
          break;
        }
        if (radio.Func()==0)
        {
          break;
        }
        // init to change the value of the current function
        multifunc.state = FUNCTION_STATE_VALUE_INIT;
        break;
      }
      case FUNCTION_STATE_VALUE_INIT:
      {
        // indicate which function value to change
        multifunc.value_change = multifunc.current_function;
        multifunc.new_value_band = multifunc.current_value_band;
        multifunc.new_value_mode = multifunc.current_value_mode;
        multifunc.new_value_lock = multifunc.current_value_lock;
        multifunc.new_value_wpm = multifunc.current_value_wpm;
        multifunc.state = FUNCTION_STATE_VALUE_CHANGE;
        multifunc.timeout = millis()+MULTIFUNCTION_TIMEOUT;
        break;
      }
      case FUNCTION_STATE_VALUE_CHANGE:
      {
        if (radio.multiButton())
        {
          // button is pressed,
          // activate the selected function
          // save the selected function
          // band
          if (multifunc.new_value_band!=multifunc.current_value_band)
          {
            if (radio.isLocked())
            {
              // if locked, can't change band
              set_message(MESSAGE_LOCKED);
              multifunc.value_change = FUNCTION_NONE;
              multifunc.new_value_band = multifunc.current_value_band;
              multifunc.new_value_mode = multifunc.current_value_mode;
              multifunc.new_value_lock = multifunc.current_value_lock;
              multifunc.new_value_atten = multifunc.current_value_atten;
              multifunc.new_value_scopeoption = multifunc.current_value_scopeoption;
              multifunc.new_function = multifunc.current_function;
            }
            else
            {
              radio.setBand(multifunc.new_value_band);
            }
            const uint32_t old_band = radio.band_index(multifunc.current_value_band);
            const uint32_t new_band = radio.band_index(multifunc.new_value_band);
            band_save[old_band].frequency = radio.frequency;
            band_save[old_band].tuning_step = radio.tuning_step;
            band_save[old_band].mode = radio.mode;
            band_save[old_band].atten = radio.attEnabled()?ATTN_ON:ATTN_OFF;
            radio.frequency = band_save[new_band].frequency;
            radio.tuning_step = band_save[new_band].tuning_step;
            radio.mode = band_save[new_band].mode;
            multifunc.new_value_mode = radio.mode;
            multifunc.current_value_mode = radio.mode;
            multifunc.new_value_atten = band_save[new_band].atten;
            switch (radio.mode)
            {
              case Radio::LSB:  radio_state = STATE_LSB_RECEIVE_INIT;  break;
              case Radio::USB:  radio_state = STATE_USB_RECEIVE_INIT;  break;
              case Radio::CWL:  radio_state = STATE_CWL_RECEIVE_INIT;  break;
              case Radio::CWU:  radio_state = STATE_CWU_RECEIVE_INIT;  break;
              case Radio::DIGL: radio_state = STATE_DIGL_RECEIVE_INIT; break;
              case Radio::DIGU: radio_state = STATE_DIGU_RECEIVE_INIT; break;
            }
          }
          // mode
          if (multifunc.new_value_mode!=multifunc.current_value_mode)
          {
            if (radio.isLocked())
            {
              // if locked, can't change mode
              set_message(MESSAGE_LOCKED);
              multifunc.value_change = FUNCTION_NONE;
              multifunc.new_value_band = multifunc.current_value_band;
              multifunc.new_value_mode = multifunc.current_value_mode;
              multifunc.new_value_lock = multifunc.current_value_lock;
              multifunc.new_value_atten = multifunc.current_value_atten;
              multifunc.new_value_scopeoption = multifunc.current_value_scopeoption;
              multifunc.new_function = multifunc.current_function;
              multifunc.highlight = false;
              multifunc.state = FUNCTION_STATE_IDLE;
              break;
            }
            const uint32_t current_band = radio.band_index(multifunc.current_value_band);
            band_save[current_band].frequency = radio.frequency;
            band_save[current_band].tuning_step = radio.tuning_step;
            band_save[current_band].mode = radio.mode;
            band_save[current_band].atten = radio.attEnabled()?ATTN_ON:ATTN_OFF;
            radio.mode = multifunc.new_value_mode;
            switch (radio.mode)
            {
              case Radio::LSB:  radio_state = STATE_LSB_RECEIVE_INIT;  break;
              case Radio::USB:  radio_state = STATE_USB_RECEIVE_INIT;  break;
              case Radio::CWL:  radio_state = STATE_CWL_RECEIVE_INIT;  break;
              case Radio::CWU:  radio_state = STATE_CWU_RECEIVE_INIT;  break;
              case Radio::DIGL: radio_state = STATE_DIGL_RECEIVE_INIT; break;
              case Radio::DIGU: radio_state = STATE_DIGU_RECEIVE_INIT; break;
            }
          }
          // attenuator
          if (multifunc.new_value_atten!=multifunc.current_value_atten)
          {
            multifunc.current_value_atten = multifunc.new_value_atten;
            switch (multifunc.new_value_atten)
            {
              case ATTN_ON: radio.attOn(); break;
              case ATTN_OFF: radio.attOff(); break;
            }
          }
          // lock, unlock
          if (multifunc.new_value_lock!=multifunc.current_value_lock)
          {
            const uint32_t current_band = radio.band_index(multifunc.current_value_band);
            band_save[current_band].frequency = radio.frequency;
            band_save[current_band].tuning_step = radio.tuning_step;
            band_save[current_band].mode = radio.mode;
            band_save[current_band].atten = radio.attEnabled()?ATTN_ON:ATTN_OFF;
            switch (multifunc.new_value_lock)
            {
              case LOCKED: radio.lock(); break;
              case UNLOCKED: radio.unlock(); break;
            }
          }
          // spectrum scope speed
          if (multifunc.new_value_scopeoption!=multifunc.current_value_scopeoption)
          {
            switch (multifunc.new_value_scopeoption)
            {
              case SCOPE_SPEED_1: radio.scope_speed = 1u; break;
              case SCOPE_SPEED_2: radio.scope_speed = 2u; break;
              case SCOPE_SPEED_3: radio.scope_speed = 4u; break;
              case SCOPE_SPEED_4: radio.scope_speed = 8u; break;
              case SCOPE_ZOOM_0:  radio.scope_zoom  = 0u; break;
              case SCOPE_ZOOM_1:  radio.scope_zoom  = 1u; break;
              case SCOPE_ZOOM_2:  radio.scope_zoom  = 2u; break;
            }
            save_settings();
          }
          // CW speed
          if (multifunc.new_value_wpm!=multifunc.current_value_wpm)
          {
            switch (multifunc.new_value_wpm)
            {
              case CW_WPM_10: cw_dit = 1000*60/(50*10); break;
              case CW_WPM_11: cw_dit = 1000*60/(50*11); break;
              case CW_WPM_12: cw_dit = 1000*60/(50*12); break;
              case CW_WPM_13: cw_dit = 1000*60/(50*13); break;
              case CW_WPM_14: cw_dit = 1000*60/(50*14); break;
              case CW_WPM_15: cw_dit = 1000*60/(50*15); break;
              case CW_WPM_16: cw_dit = 1000*60/(50*16); break;
              case CW_WPM_17: cw_dit = 1000*60/(50*17); break;
              case CW_WPM_18: cw_dit = 1000*60/(50*18); break;
              case CW_WPM_19: cw_dit = 1000*60/(50*19); break;
              case CW_WPM_20: cw_dit = 1000*60/(50*20); break;
              case CW_WPM_21: cw_dit = 1000*60/(50*21); break;
              case CW_WPM_22: cw_dit = 1000*60/(50*22); break;
              case CW_WPM_23: cw_dit = 1000*60/(50*23); break;
              case CW_WPM_24: cw_dit = 1000*60/(50*24); break;
              case CW_WPM_25: cw_dit = 1000*60/(50*25); break;
              case CW_WPM_26: cw_dit = 1000*60/(50*26); break;
              case CW_WPM_27: cw_dit = 1000*60/(50*27); break;
              case CW_WPM_28: cw_dit = 1000*60/(50*28); break;
              case CW_WPM_29: cw_dit = 1000*60/(50*29); break;
              case CW_WPM_30: cw_dit = 1000*60/(50*30); break;
            }
            save_settings();
          }
          multifunc.value_change = FUNCTION_NONE;
          // current value becomes new value
          multifunc.current_value_band = multifunc.new_value_band;
          multifunc.current_value_mode = multifunc.new_value_mode;
          multifunc.current_value_lock = multifunc.new_value_lock;
          multifunc.current_value_atten = multifunc.new_value_atten;
          multifunc.current_value_wpm = multifunc.new_value_wpm;
          multifunc.current_value_scopeoption = multifunc.new_value_scopeoption;
          multifunc.new_function = multifunc.current_function;
          multifunc.highlight = false;
          multifunc.state = FUNCTION_STATE_WAIT_BUTTON_2;
          break;
        }
        const int32_t f = radio.Func();
        if (f==0)
        {
          if (millis()>multifunc.timeout)
          {
            // times up, exit without change
            multifunc.value_change = FUNCTION_NONE;
            multifunc.new_value_band = multifunc.current_value_band;
            multifunc.new_value_mode = multifunc.current_value_mode;
            multifunc.new_value_lock = multifunc.current_value_lock;
            multifunc.new_value_scopeoption = multifunc.current_value_scopeoption;
            multifunc.new_function = multifunc.current_function;
            multifunc.highlight = false;
            multifunc.state = FUNCTION_STATE_IDLE;
          }
          break;
        }
        // knob has moved, reset the timeout
        multifunc.timeout = millis()+MULTIFUNCTION_TIMEOUT;
        if (f>0)
        {
          // clockwise
          switch (multifunc.value_change)
          {
            case FUNCTION_BAND:
            {
              switch (multifunc.new_value_band)
              {
                case Radio::BAND80: multifunc.new_value_band = Radio::BAND40; break;
                case Radio::BAND40: multifunc.new_value_band = Radio::BAND20; break;
                case Radio::BAND20: multifunc.new_value_band = Radio::BAND15; break;
                case Radio::BAND15: multifunc.new_value_band = Radio::BAND10; break;
                case Radio::BAND10: multifunc.new_value_band = Radio::BAND80; break;
              }
              break;
            }
            case FUNCTION_MODE:
            {
              // LSB, USB, CWL, CWU, DIGL, DIGU
              switch (multifunc.new_value_mode)
              {
                case Radio::LSB:  multifunc.new_value_mode = Radio::USB;  break;
                case Radio::USB:  multifunc.new_value_mode = Radio::CWL;  break;
                case Radio::CWL:  multifunc.new_value_mode = Radio::CWU;  break;
                case Radio::CWU:  multifunc.new_value_mode = Radio::DIGL; break;
                case Radio::DIGL: multifunc.new_value_mode = Radio::DIGU; break;
                case Radio::DIGU: multifunc.new_value_mode = Radio::LSB;  break;
              }
              break;
            }
            case FUNCTION_LOCK:
            {
              // locked, unlocked
              switch (multifunc.new_value_lock)
              {
                case LOCKED: multifunc.new_value_lock = UNLOCKED; break;
                case UNLOCKED: multifunc.new_value_lock = LOCKED; break;
              }
              break;
            }
            case FUNCTION_ATTN:
            {
              // on or off
              switch (multifunc.new_value_atten)
              {
                case ATTN_ON: multifunc.new_value_atten = ATTN_OFF; break;
                case ATTN_OFF: multifunc.new_value_atten = ATTN_ON; break;
              }
              break;
            }
            case FUNCTION_BSCP:
            {
              // band scope, speed
              switch (multifunc.new_value_scopeoption)
              {
                case SCOPE_SPEED_1: multifunc.new_value_scopeoption = SCOPE_ZOOM_0;  break;
                case SCOPE_SPEED_2: multifunc.new_value_scopeoption = SCOPE_SPEED_1; break;
                case SCOPE_SPEED_3: multifunc.new_value_scopeoption = SCOPE_SPEED_2; break;
                case SCOPE_SPEED_4: multifunc.new_value_scopeoption = SCOPE_SPEED_3; break;
                case SCOPE_ZOOM_0:  multifunc.new_value_scopeoption = SCOPE_ZOOM_1;  break;
                case SCOPE_ZOOM_1: multifunc.new_value_scopeoption  = SCOPE_ZOOM_2;  break;
                case SCOPE_ZOOM_2: multifunc.new_value_scopeoption  = SCOPE_SPEED_4; break;
              }
              break;
            }
            case FUNCTION_CWSP:
            {
              // CW WPM
              switch (multifunc.new_value_wpm)
              {
                case CW_WPM_10: multifunc.new_value_wpm = CW_WPM_11;  break;
                case CW_WPM_11: multifunc.new_value_wpm = CW_WPM_12;  break;
                case CW_WPM_12: multifunc.new_value_wpm = CW_WPM_13;  break;
                case CW_WPM_13: multifunc.new_value_wpm = CW_WPM_14;  break;
                case CW_WPM_14: multifunc.new_value_wpm = CW_WPM_15;  break;
                case CW_WPM_15: multifunc.new_value_wpm = CW_WPM_16;  break;
                case CW_WPM_16: multifunc.new_value_wpm = CW_WPM_17;  break;
                case CW_WPM_17: multifunc.new_value_wpm = CW_WPM_18;  break;
                case CW_WPM_18: multifunc.new_value_wpm = CW_WPM_19;  break;
                case CW_WPM_19: multifunc.new_value_wpm = CW_WPM_20;  break;
                case CW_WPM_20: multifunc.new_value_wpm = CW_WPM_21;  break;
                case CW_WPM_21: multifunc.new_value_wpm = CW_WPM_22;  break;
                case CW_WPM_22: multifunc.new_value_wpm = CW_WPM_23;  break;
                case CW_WPM_23: multifunc.new_value_wpm = CW_WPM_24;  break;
                case CW_WPM_24: multifunc.new_value_wpm = CW_WPM_25;  break;
                case CW_WPM_25: multifunc.new_value_wpm = CW_WPM_26;  break;
                case CW_WPM_26: multifunc.new_value_wpm = CW_WPM_27;  break;
                case CW_WPM_27: multifunc.new_value_wpm = CW_WPM_28;  break;
                case CW_WPM_28: multifunc.new_value_wpm = CW_WPM_29;  break;
                case CW_WPM_29: multifunc.new_value_wpm = CW_WPM_30;  break;
                case CW_WPM_30: multifunc.new_value_wpm = CW_WPM_10;  break;
              }
              break;
            }
          }
        }
        else
        {
          // f < 0 (aniclockwise)
          switch (multifunc.value_change)
          {
            case FUNCTION_BAND:
            {
              switch (multifunc.new_value_band)
              {
                case Radio::BAND80: multifunc.new_value_band = Radio::BAND10; break;
                case Radio::BAND40: multifunc.new_value_band = Radio::BAND80; break;
                case Radio::BAND20: multifunc.new_value_band = Radio::BAND40; break;
                case Radio::BAND15: multifunc.new_value_band = Radio::BAND20; break;
                case Radio::BAND10: multifunc.new_value_band = Radio::BAND15; break;
              }
              break;
            }
            case FUNCTION_MODE:
            {
              // LSB, USB, CWL, CWU, DIGL, DIGU
              switch (multifunc.new_value_mode)
              {
                case Radio::LSB:  multifunc.new_value_mode = Radio::DIGU; break;
                case Radio::USB:  multifunc.new_value_mode = Radio::LSB;  break;
                case Radio::CWL:  multifunc.new_value_mode = Radio::USB;  break;
                case Radio::CWU:  multifunc.new_value_mode = Radio::CWL;  break;
                case Radio::DIGL: multifunc.new_value_mode = Radio::CWU;  break;
                case Radio::DIGU: multifunc.new_value_mode = Radio::DIGL; break;
              }
              break;
            }
            case FUNCTION_LOCK:
            {
              // locked, unlocked
              switch (multifunc.new_value_lock)
              {
                case LOCKED: multifunc.new_value_lock = UNLOCKED; break;
                case UNLOCKED: multifunc.new_value_lock = LOCKED; break;
              }
              break;
            }
            case FUNCTION_ATTN:
            {
              // on or off
              switch (multifunc.new_value_atten)
              {
                case ATTN_ON: multifunc.new_value_atten = ATTN_OFF; break;
                case ATTN_OFF: multifunc.new_value_atten = ATTN_ON; break;
              }
              break;
            }
            case FUNCTION_BSCP:
            {
              // band scope, speed, zoom
              switch (multifunc.new_value_scopeoption)
              {
                case SCOPE_SPEED_1: multifunc.new_value_scopeoption = SCOPE_SPEED_2; break;
                case SCOPE_SPEED_2: multifunc.new_value_scopeoption = SCOPE_SPEED_3; break;
                case SCOPE_SPEED_3: multifunc.new_value_scopeoption = SCOPE_SPEED_4; break;
                case SCOPE_SPEED_4: multifunc.new_value_scopeoption = SCOPE_ZOOM_2;  break;
                case SCOPE_ZOOM_2:  multifunc.new_value_scopeoption = SCOPE_ZOOM_1;  break;
                case SCOPE_ZOOM_1:  multifunc.new_value_scopeoption = SCOPE_ZOOM_0;  break;
                case SCOPE_ZOOM_0:  multifunc.new_value_scopeoption = SCOPE_SPEED_1; break;
              }
            }
            case FUNCTION_CWSP:
            {
              // CW WPM
              switch (multifunc.new_value_wpm)
              {
                case CW_WPM_10: multifunc.new_value_wpm = CW_WPM_30;  break;
                case CW_WPM_11: multifunc.new_value_wpm = CW_WPM_10;  break;
                case CW_WPM_12: multifunc.new_value_wpm = CW_WPM_11;  break;
                case CW_WPM_13: multifunc.new_value_wpm = CW_WPM_12;  break;
                case CW_WPM_14: multifunc.new_value_wpm = CW_WPM_13;  break;
                case CW_WPM_15: multifunc.new_value_wpm = CW_WPM_14;  break;
                case CW_WPM_16: multifunc.new_value_wpm = CW_WPM_15;  break;
                case CW_WPM_17: multifunc.new_value_wpm = CW_WPM_16;  break;
                case CW_WPM_18: multifunc.new_value_wpm = CW_WPM_17;  break;
                case CW_WPM_19: multifunc.new_value_wpm = CW_WPM_18;  break;
                case CW_WPM_20: multifunc.new_value_wpm = CW_WPM_19;  break;
                case CW_WPM_21: multifunc.new_value_wpm = CW_WPM_20;  break;
                case CW_WPM_22: multifunc.new_value_wpm = CW_WPM_21;  break;
                case CW_WPM_23: multifunc.new_value_wpm = CW_WPM_22;  break;
                case CW_WPM_24: multifunc.new_value_wpm = CW_WPM_23;  break;
                case CW_WPM_25: multifunc.new_value_wpm = CW_WPM_24;  break;
                case CW_WPM_26: multifunc.new_value_wpm = CW_WPM_25;  break;
                case CW_WPM_27: multifunc.new_value_wpm = CW_WPM_26;  break;
                case CW_WPM_28: multifunc.new_value_wpm = CW_WPM_27;  break;
                case CW_WPM_29: multifunc.new_value_wpm = CW_WPM_28;  break;
                case CW_WPM_30: multifunc.new_value_wpm = CW_WPM_29;  break;
              }
              break;
            }
          }
        }
        break;
      }
      case FUNCTION_STATE_CHANGE_INIT:
      {
        // change the current function
        multifunc.highlight = true;
        multifunc.state = FUNCTION_STATE_WAIT_BUTTON_1;
        break;
      }
      case FUNCTION_STATE_WAIT_BUTTON_1:
      {
        // wait for the button to be released
        if (radio.multiButton())
        {
          break;
        }
        multifunc.timeout = millis()+MULTIFUNCTION_TIMEOUT;
        multifunc.new_function = multifunc.current_function;
        multifunc.state = FUNCTION_STATE_CHANGE;
        break;
      }
      case FUNCTION_STATE_CHANGE:
      {
        if (radio.multiButton())
        {
          // save the selected function
          multifunc.current_function = multifunc.new_function;
          multifunc.highlight = false;
          multifunc.state = FUNCTION_STATE_WAIT_BUTTON_2;
          break;
        }
        const int32_t f = radio.Func();
        if (f==0)
        {
          if (millis()>multifunc.timeout)
          {
            // times up, exit without change
            multifunc.new_function = multifunc.current_function;
            multifunc.highlight = false;
            multifunc.state = FUNCTION_STATE_WAIT_BUTTON_2;
          }
          break;
        }
        multifunc.timeout = millis()+MULTIFUNCTION_TIMEOUT;
        if (f>0)
        {
          // move to next function
          switch (multifunc.new_function)
          {
            case FUNCTION_BAND: multifunc.new_function = FUNCTION_MODE; break;
            case FUNCTION_MODE: multifunc.new_function = FUNCTION_LOCK; break;
            case FUNCTION_LOCK: multifunc.new_function = FUNCTION_ATTN; break;
            case FUNCTION_ATTN: multifunc.new_function = FUNCTION_BSCP; break;
            case FUNCTION_BSCP: multifunc.new_function = FUNCTION_CWSP; break;
            case FUNCTION_CWSP: multifunc.new_function = FUNCTION_BAND; break;
          }
          break;
        }
        else
        {
          // move to previous function
          switch (multifunc.new_function)
          {
            case FUNCTION_BAND: multifunc.new_function = FUNCTION_CWSP; break;
            case FUNCTION_MODE: multifunc.new_function = FUNCTION_BAND; break;
            case FUNCTION_LOCK: multifunc.new_function = FUNCTION_MODE; break;
            case FUNCTION_ATTN: multifunc.new_function = FUNCTION_LOCK; break;
            case FUNCTION_BSCP: multifunc.new_function = FUNCTION_ATTN; break;
            case FUNCTION_CWSP: multifunc.new_function = FUNCTION_BSCP; break;
          }
          break;
        }
      }
      case FUNCTION_STATE_WAIT_BUTTON_2:
      {
        // wait for the button to be released
        if (radio.multiButton())
        {
          break;
        }
        // back to start
        multifunc.state = FUNCTION_STATE_IDLE;
        break;
      }
    }
  }

  display_clear();
  show_rx_tx();
  show_mode();
  show_frequency();
  show_tuning_step();
  show_meter_dial();
  show_multifunc();
  show_attenuator();
  show_bandwidth();

  // if we can access the spectrum data then
  // make a copy of it for display, otherwise
  // just use the last data set
  boolean update_spectrum_display = false;
  static uint32_t mutex_owner;
  if (mutex_try_enter(&spectrum_mutex,&mutex_owner))
  {
    if (spectrum.isDataReady())
    {
      // copy spectrum_data to spectrum display buffer
      for (uint32_t i=0;i<N_WAVE;i++)
      {
        spectrum_buffer[i] = spectrum_data[i];
      }
      update_spectrum_display = true;
    }
    mutex_exit(&spectrum_mutex);
  }

  if (update_spectrum_display)
  {
    // new spectrum display data is avaliable so
    // update the spectrum display
    show_new_spectrum();
  }
  else
  {
    show_old_spectrum();
  }

  // stuff that can display over the spectrum
  
  // this is a message or update of the multifunction
  // value that will overlay the waterfall
  show_multifunc_value();
  show_message();

  // send the display buffer to the display
  static uint32_t next_update = 0;
  if (next_update<=millis())
  {
    next_update = millis()+20;
    display_refresh();
  }
}
