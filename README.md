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
- Runtime-configurable gain, LED current, filter cutoffs and sample rate

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

In your `platformio.ini` (pin to a tag or commit — never `#master` — for a firmware you ship):

```ini
lib_deps =
    https://github.com/medicalopenworld/incunest_afe4490.git#v0.91
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
under `-Wall -Wextra -Werror` on both frameworks and on the host (`pio test -e native`).

## Specification

See [`incunest_afe4490_spec.md`](incunest_afe4490_spec.md) for the full design specification.
The spec and the library are versioned together (same semantic version).

## References

- [AFE4490 Datasheet (Texas Instruments)](https://www.ti.com/lit/ds/symlink/afe4490.pdf) — primary hardware reference for register map, timing formulas, and electrical characteristics.

## License

[MIT](LICENSE) — Copyright (c) 2026 medicalopenworld. Provided "as is", without warranty of any kind;
see the clinical-use notice above.
