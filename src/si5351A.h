//si5351A.h - si5351A library. Version 1.0 21 November 2021
//Written by Julie VK3FOWL and Joe VK3YSP
//For the School Amateur Radio Club Network VK3SRC at www.sarcnet.org
//Provides a calibrated VFO and BFO with adjustable frequencies and modes
#ifndef SI5351A_H
#define SI5351A_H
#include "si5351.h"
#include "Wire.h"

class Si5351A: public Si5351 {                                                            // si5351A class
  public:
    static const uint32_t XTAL = 26000000;                                                // Uncomment for 26MHz clock frequecny
    //static const uint32_t XTAL = 25000000;                                              // Uncomment for 25MHz clock frequecny
    static const uint8_t LOAD = SI5351_CRYSTAL_LOAD_0PF;                                  // No load, driven by TCXO
    //static const uint8_t LOAD = SI5351_CRYSTAL_LOAD_8PF;                                // Crystal load capacitance
    static const si5351_clock_source CLKSRC = SI5351_CLK_SRC_XTAL;                        // Uncomment for crystal clock source 
    //static const si5351_clock_source CLKSRC = SI5351_CLK_SRC_CLKIN;                     // Uncomment for external clock source
    static const uint32_t CW_FILTER_CENTRE = 11057200UL;
    static const uint16_t NUM_MODES = 6;                                                  // Number of modes
    static const uint16_t NUM_BANDS = 5;                                                  // Number of bands
    enum modes_t {LSB, USB, CWL, CWU, DIGL, DIGU};                                        // Modes
  //const uint32_t bfos[NUM_MODES] = {11056700, 11059300, 11056500, 11057900, 11056900, 11060300}; // BFO frequencies for each mode
    const uint32_t bfos[NUM_MODES] = {11056600, 11059300, 11056500, 11057900, 11056700, 11060500}; // BFO frequencies for each mode
    const uint8_t wavelengths[NUM_BANDS] = {80, 40, 20, 15, 10};                                // Wavelength for each band
    const uint32_t bandMin[NUM_BANDS] = {3500000, 7000000, 14000000, 21000000, 28000000};       // Band min frequency
    const uint32_t bandMax[NUM_BANDS] = {3800000, 7300000, 14350000, 21450000, 29700000};       // Band max frequency
    Si5351A();                                                                            // Constructor.
    bool begin(uint32_t freq, modes_t mode,uint32_t corr);                                // Initializer. Specify the starting frequency, mode and frequency correction.
    void setMode(modes_t mode);                                                           // Set the mode. Will automatically set the BFO frequency.
    bool setFreq(uint32_t freq);                                                          // Set the VFO frequency in Hz and set the Mode
    bool setFreq(uint32_t freq, modes_t mode);                                            // Set the VFO frequency in Hz and set the Mode
    bool setRevFreq(uint32_t freq);                                                       // Set the VFO frequency in Hz and set the Mode (reverse sideband)
    bool setRevFreq(uint32_t freq, modes_t mode);                                         // Set the VFO frequency in Hz and set the Mode (reverse sideband)
    uint8_t band;                                                                         // Current band index
    uint8_t wavelength;                                                                   // Current band wavelength
    modes_t mode;                                                                         // Current mode
    uint32_t vfo;                                                                         // Current VFO frequency in Hz
    uint32_t bfo;                                                                         // Current BFO frequency in Hz
  private:
    bool _getBand(uint32_t freq);                                                         // Get the band associated with the frequency, returning true if valid
};

#endif
