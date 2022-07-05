#ifndef Spectrum_h
#define Spectrum_h

#define N_WAVE      1024    /* full length of sinewave[] */
#define LOG2_N_WAVE 10      /* log2(N_WAVE) */

#include "Arduino.h"

class Spectrum
{
  public:
    Spectrum(void);
    void __attribute__((noinline,long_call,section(".time_critical"))) process(uint32_t speed = 4);
    const boolean isDataReady(void);
    void dataReady(void);
    uint8_t mag[N_WAVE];
    uint8_t AGC;
  private:
    void FFT(int16_t fr[], int16_t fi[], int16_t m);
    uint32_t _new_refcount;
    uint32_t _old_refcount;
};

#endif
