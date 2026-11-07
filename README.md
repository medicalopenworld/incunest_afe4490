# incunest_afe4490

AFE4490 PPG/SpO2/PI driver for ESP32-S3 (Arduino-ESP32 or ESP-IDF, FreeRTOS) — [Medical Open World](http://medicalopenworld.org)

Part of the [IncuNest](https://github.com/medicalopenworld/IncuNest) neonatal incubator project.

> **Not validated for clinical use.** This library is research and development software. It is not a
> certified medical device and has not been clinically validated: do not use its SpO2, heart-rate or
> perfusion-index outputs to diagnose, monitor or treat patients.

## Features

- Non-blocking FreeRTOS task: DRDY ISR → SPI read → signal processing → queue
- SpO2 (R-ratio method, calibratable coefficients)
- HR1: peak detection
- HR2: autocorrelation
- HR3: FFT + Harmonic Product Spectrum
- Signal Quality Index (SQI) for SpO2 and each HR algorithm
- Display waveform `ppg_disp` in three modes: as measured, vertically scaled, or scaled with the
  perfusion index (see below)
- Runtime-configurable gain, LED current, filter cutoffs and sample rate

## PPG display waveform (`ppg_disp`)

`AFE4490Data::ppg_disp` is the waveform meant for a screen: the chosen channel's optical
transmission, band-passed (0.5–20 Hz by default) and inverted so that each heartbeat rises. No
algorithm reads it. Since v0.116 it has three modes, `afe.setPPGDispMode(...)` (spec §5.9):

| Mode | What the screen gets | Use |
|---|---|---|
| `NO_NORM` (default) | the waveform as measured, in transmission units (~1e-6..1e-5): its size follows the perfusion | plots that auto-range; what every version before v0.116 produced |
| `NORM` | the waveform vertically scaled to a constant pulse amplitude, in [−1, +1] | laboratory comparison; the "normalized" display of ISO 80601-2-61 201.3.241, which the standard warns can mask an unreliable signal on its own |
| `NORM_PI` | as `NORM`, but the amplitude follows the perfusion index: full size at PI ≥ 2 %, half size at PI ≤ 0.5 %, logarithmic in between (`setPPGDispNormPI()`, its own thresholds, not the SpO2 quality gate's); still clamped to [−1, +1] | the product mode: a shrinking trace tells the operator the perfusion is falling |

How the scaling works (the "SLEW" tracker): the library keeps a frame around the waveform from the
highest and lowest values of the last second. When a beat is larger than the frame, the frame grows
towards it at a limited rate — at most half its own size per second — so a brief artefact saturates
the trace for a moment instead of re-scaling it; when beats get smaller, the frame shrinks
exponentially with a 1 s time constant. The frame's own extremes map to ±1/1.15, leaving 15 % of
headroom for the next beat. While the probe is not applied, or during the first second after it is,
the scaled modes output 0: an auto-gain never turns noise into a live-looking trace. The four
numbers are runtime parameters (`setPPGDispNormTracker()`); the reasons behind them are in the
design rationale, §21.

## Requirements

- ESP32-S3
- **Arduino-ESP32** (PlatformIO) **or ESP-IDF ≥ 5** as a CMake component — same sources, no patches
  (hardware abstraction layer, spec §1.4)
- AFE4490 connected via SPI; the application owns the bus (`SPI.begin()` / `spi_bus_initialize()`)

## Usage

See [`examples/basic/main.cpp`](examples/basic/main.cpp) for a minimal working example.

```cpp
#include "incunest_afe4490.h"

INCUNEST_AFE4490 afe;

void setup() {
    SPI.begin(SCK, MISO, MOSI, -1);
    afe.begin(CS_PIN, DRDY_PIN);
}
```

In your `platformio.ini` (pin to a tag or commit — never `#master` — for a firmware you ship; each tag's
[release notes](https://github.com/medicalopenworld/incunest_afe4490/releases) say what changes and what
to touch when upgrading):

```ini
lib_deps =
    https://github.com/medicalopenworld/incunest_afe4490.git#v0.111
```

### ESP-IDF (no Arduino)

Add the repository as a component (submodule under `components/`, or point `EXTRA_COMPONENT_DIRS` at it)
and require it from your main component:

```cmake
# project CMakeLists.txt
set(EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_LIST_DIR}/components/incunest_afe4490")
# main/CMakeLists.txt
idf_component_register(SRCS "main.cpp" REQUIRES incunest_afe4490 esp_driver_spi)
```

```cpp
#include "driver/spi_master.h"
#include "incunest_afe4490.h"

static INCUNEST_AFE4490 afe;

extern "C" void app_main(void) {
    spi_bus_config_t bus = {};              // the application owns the bus, as with SPI.begin()
    bus.mosi_io_num = MOSI; bus.miso_io_num = MISO; bus.sclk_io_num = SCK;
    bus.quadwp_io_num = -1; bus.quadhd_io_num = -1; bus.max_transfer_sz = 64;
    spi_bus_initialize(SPI2_HOST, &bus, SPI_DMA_DISABLED);
    afe.begin(CS_PIN, DRDY_PIN);            // the library adds its device on SPI2_HOST
}
```

`-DINCUNEST_AFE4490_HAL_SPI_HOST=SPI3_HOST` if the AFE is not on SPI2. The library builds
under `-Wall -Wextra -Werror` on both frameworks and on the host.

## Tests

Host unit tests (Unity) in [`test/`](test/README), run on the PC with cmake, ninja and g++:

```sh
python test/run_host_tests.py
```

GitHub Actions runs them on every push and every tag, after `scripts/check_version.py` (every version
string in the repository names the same version, and the tag's on a tag).

## Specification

See [`incunest_afe4490_spec.md`](incunest_afe4490_spec.md) for the full design specification.
The spec and the library are versioned together (same semantic version).

## References

- [AFE4490 Datasheet (Texas Instruments)](https://www.ti.com/lit/ds/symlink/afe4490.pdf) — primary hardware reference for register map, timing formulas, and electrical characteristics.

## License

[MIT](LICENSE) — Copyright (c) 2026 medicalopenworld. Provided "as is", without warranty of any kind;
see the clinical-use notice above.
