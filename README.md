# ESP32 FFT Spectrum Analyzer

A real-time, 8-band signal spectrum analyzer built with **ESP32**, **ESP-IDF**, and a **custom radix-2 FFT implementation in C**. The spectrum is displayed on an 8×8 LED matrix over I²C.

The project was developed as a hands-on embedded systems and digital signal processing exercise. The FFT is implemented from scratch rather than delegated to a DSP library.

## Features

- 12-bit sampling from ESP32 **ADC1, channel 6 (GPIO34)**
- Nominal **4 kHz** sampling rate, with **256 samples per FFT block**
- Custom in-place, radix-2 Cooley–Tukey FFT with bit-reversal permutation
- DC-offset removal before spectral analysis
- Peak frequency estimation and a separate zero-crossing frequency estimate for debugging
- Eight non-uniform frequency bands, mapped to eight LED-matrix columns
- I²C display output using an **IS31FL3728** LED driver (Amperka/Troyka 8×8 matrix with 788BS display)
- Built-in LEDC square-wave test generator on GPIO25; no external function generator required for the basic demo

## Hardware

- ESP32 DOIT DevKit V1 (classic ESP32)
- Amperka/Troyka 8×8 LED matrix module (788BS display, IS31FL3728 driver)
- One **1 kΩ** series resistor for the internal signal-generator loopback
- Jumper wires; an oscilloscope is useful but optional

### Wiring

| Signal | Connection |
| --- | --- |
| Test waveform | **GPIO25 → 1 kΩ → GPIO34** |
| Matrix SDA (`D`) | ESP32 **GPIO21** |
| Matrix SCL (`C`) | ESP32 **GPIO22** |
| Matrix power (`V`) | ESP32 **3V3** (for the low-brightness test configuration) |
| Matrix ground (`G`) | ESP32 **GND** |

The matrix's `S` connection is not used in this project. Check the pin markings on your particular module before applying power.

**Electrical safety:** The ESP32 ADC must not receive negative voltage or voltage above its permitted input range. A 1 kΩ series resistor alone is **not** an adequate protection circuit for an arbitrary external signal generator. The included internal PWM loopback stays between the ESP32's logic levels; near-rail ADC readings can saturate and are not calibrated voltage measurements.

## Signal processing pipeline

```text
LEDC PWM (GPIO25)
       |
     1 kΩ
       |
ADC1 (GPIO34) --> 256 raw samples
                         |
                  Remove DC offset
                         |
                  Bit-reversal order
                         |
                 Radix-2 FFT (C)
                         |
                  Spectral magnitudes
                         |
               Eight frequency bands
                         |
               Normalize to 0..8 LEDs
                         |
               I²C / IS31FL3728
                         |
                  8×8 LED matrix
```

With the current sampling configuration:

| Parameter | Value |
| --- | ---: |
| Sampling interval | 250 µs (nominal) |
| Sampling frequency | 4,000 Hz (nominal) |
| FFT size | 256 |
| FFT bin spacing | 15.625 Hz |
| Nyquist frequency | 2,000 Hz |
| Acquisition time per block | ~64 ms |

The first FFT bin represents the DC component and is excluded from the eight display bands. For each band, the firmware uses the maximum bin power; band heights are normalized relative to the strongest band in the current frame.

## Build and flash

This is a **PlatformIO + ESP-IDF** project, written in C (not Arduino).

1. Install [PlatformIO Core](https://docs.platformio.org/en/latest/core/installation/index.html) and an ESP32-compatible USB serial driver if needed.
2. Connect the ESP32 via USB.
3. In the repository root, run:

```sh
pio run
pio run -t upload
pio device monitor -b 115200
```

The target environment in `platformio.ini` is `esp32doit-devkit-v1`. PlatformIO downloads the required toolchain/framework packages during the first build.

## Demo and verification

The square-wave generator uses **270 Hz**, **50% duty cycle** on GPIO25. Its spectrum contains a strong fundamental plus weaker odd harmonics (3rd, 5th, 7th, etc.). A representative eight-column terminal rendering from the hardware test was:

```text
....#...
....#...
....#...
....#...
....#...
....#.#.
....#.#.
#..#####
```

In an earlier sine-wave test, a signal measured at approximately **271 Hz** produced an FFT peak at **bin 17 = 265.625 Hz**, consistent with the available frequency resolution.

The firmware also prints diagnostic ADC and FFT information to the serial monitor. Printing extensive debug output may reduce display update responsiveness.

## Project structure

```text
.
├── CMakeLists.txt
├── platformio.ini
├── src/
│   ├── CMakeLists.txt
│   └── main.c
├── .gitignore
└── README.md
```

## Limitations and possible improvements

- Sampling currently uses an `esp_timer_get_time()`-based busy-wait loop rather than ADC continuous mode with DMA. Timing jitter can affect spectral measurements.
- No anti-aliasing input filter is fitted. Frequencies above 2 kHz may alias into the visible spectrum.
- The implementation currently uses no window function; a Hann window would reduce spectral leakage for non-bin-centered tones.
- The ESP32 ADC is not a precision, linear, calibrated instrument; near-rail input levels may clip.
- Each frame is normalized independently, so even a weak signal can produce a full-height bar. A noise floor and smoothing would improve the visualization.
- This is a **low-frequency signal spectrum analyzer**, **not** a 2.4 GHz Wi-Fi spectrum analyzer. Scanning Wi-Fi channels/RSSI would be a separate feature, not an extension of this ADC FFT input.

## Background

The project explores the complete path from hardware waveform generation and sampling through Fourier analysis to display output. The custom FFT code is intended to remain readable and inspectable for studying the Cooley–Tukey algorithm, complex butterfly operations, and embedded DSP trade-offs.
