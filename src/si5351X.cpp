// si5351A.cpp - si5351A library. Version 1.0 21 November 2021
// Written by Julie VK3FOWL and Joe VK3YSP
// For the School Amateur Radio Club Network VK3SRC at www.sarcnet.org
// Provides a calibrated VFO and BFO with adjustable frequencies and modes
#include "si5351A.h" // Our si5351 library
#include "arduino.h"

Si5351A::Si5351A() {  // Constructor.
}

bool Si5351A::begin(uint32_t freq, modes_t mode, uint32_t corr) { // Initializer. Specify the starting frequency and mode
  set_clock_source(SI5351_CLK0, CLKSRC);                          // Set the clock source for CLK0
  set_clock_source(SI5351_CLK2, CLKSRC);                          // Set the clock source for CLK2
  if (!init(LOAD, XTAL, corr))                                    // Initialize the si5351 load capacitance, crystal frequency and frequency correction
  {
    return false;
  }                                         
  setMode(mode);                                                  // Set the mode
  setFreq(freq);                                                  // Set the frequency
  drive_strength(SI5351_CLK2, SI5351_DRIVE_8MA);
  return true;
}

bool Si5351A::_getBand(uint32_t freq) { // Get the band associated with the frequency, returning true if valid
  uint16_t i = 0;                       // Band index
  while (i < NUM_BANDS) {               // Iterate through each band
    if (freq > bandMax[i]) {            // The frequency of this band is too low
      i++;                              // Go to the next band
    } else {                            // The frequency of this band is not too low
      if (freq >= bandMin[i]) {         // The frequency of this band is just right
        band = i;                       // Save the current band index
        wavelength = wavelengths[i];    // Save the current wavelength of this band
        return true;                    // The frequency is within this band
      }
      i++;
    }
  }
  return false;                         // The frequency is out of band
}

void Si5351A::setMode(modes_t amode) {                // Set the mode. Will automatically set the BFO frequency.
  mode = amode;                                       // Save the current mode
  bfo = bfos[mode];                                   // Save the current BFO frequency
  uint64_t si5351bfo = bfo * 4ULL * SI5351_FREQ_MULT; // Get the si5351 CLK0 frequency, which is 4 times the BFO frequency
  set_freq(si5351bfo, SI5351_CLK0);                   // Set the si5351 CLK0 frequency
}

bool Si5351A::setFreq(uint32_t freq) {                // Set the VFO frequency in Hz
  if (_getBand(freq)) {                               // The frequency is in band
    if (mode==CWL || mode==CWU)
    {
      vfo = freq + CW_FILTER_CENTRE;                  //  for CW, put the signal in the centre of the passband
    }
    else
    {
      vfo = freq + bfo;                               // Get the VFO frequency
    }
    uint64_t si5351vfo = vfo * SI5351_FREQ_MULT;      // Get the si5351 CLK2 frequency
    set_freq(si5351vfo, SI5351_CLK2);                 // Set the si5351 CLK2 frequency
    return true;                                      // The frequency is in band
  } else {
    return false;                                     // The frequency is out of band
  }
}

bool Si5351A::setFreq(uint32_t freq, modes_t mode) {  // Set the VFO frequency in Hz and set the Mode
  setMode(mode);                                      // Set the mode, getting the BFO frequency
  return setFreq(freq);                               // Set the VFO frequency if it is in band. Return true if it is in band. 
}
