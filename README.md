# HySSB
Hybrid Single Sideband Transceiver

This is the firmware and design files for a multiband hybrid single sideband (SSB) transceiver for the traditional amateur bands. Bands covered are 80M, 40M, 20M, 15M, and 10M.

73, VK7IAN

# Build Environment

Built using the Pi Pico arduino environment by Earle F. Philhower, III. See install details at:

https://github.com/earlephilhower/arduino-pico


Here is the "Additional Boards Manager URL" for quick start:

https://github.com/earlephilhower/arduino-pico/releases/download/global/package_rp2040_index.json

# Libraries Used
The following libraries are needed to work with the MS5351M and SPI colour LCD:

# Circuit Description

The HySSB is a single conversion superhet with an IF of ~11.06MHz. There are 3 bandpass filters at 3KHz, 2.4KHz and 400Hz. The spectrum display is implemented using a quadrature sampling detector (QSD) after the first mixer and before the crystal filter. A 6db resistive splitter routes the signal to the crystal filter and the QSD. The QSD operates at 4 times the BFO frequency. The I and Q outputs of the QSD are sampled by the ADC in the Pi Pico microcontroller at 250Khz (interleaved). The uC performs a 1024 point complex FFT from which the magnitude of the signal is used to generate the sprectrum and waterfall display.

# Some Pics

![Populated PCB](/HySSB_Populated_PCB.jpg?raw=true "PCB")

# Short video of spectrum display

[![IMAGE ALT TEXT HERE](https://img.youtube.com/vi/Xhs8i59NhmU/0.jpg)](https://www.youtube.com/watch?v=Xhs8i59NhmU)
