// incunest_afe4490_hal.h — platform contract of the incunest_afe4490 library
// Library version: v0.93 — ESP32-S3, Arduino-ESP32 or ESP-IDF, FreeRTOS
// Spec: incunest_afe4490_spec.md §1.4 "Hardware abstraction layer"
// Author: Medical Open World — http://medicalopenworld.org — <contact@medicalopenworld.org>
//
// WHAT THIS ABSTRACTS, AND WHAT IT DELIBERATELY DOES NOT
//
// The library runs on three build targets:
//   * Arduino-ESP32   (PulseNest bench today; IncuNest motherBoard until its ESP-IDF port merges)
//   * ESP-IDF, no Arduino (IncuNest motherBoard after the port; PulseNest after phase 1)
//   * host (PC)       (unit tests under `pio test -e native`, the offline capture runner)
//
// Measured on v0.90b: the ONLY symbols that differ between the two embedded targets come from
// <Arduino.h> and <SPI.h> — 18 call sites in the .cpp (GPIO, the DRDY interrupt, two 4-byte SPI
// register transactions, three console prints) plus a handful of constants. Everything else the
// library touches — FreeRTOS (~270 calls), ESP_LOGx, esp_timer_get_time(), IRAM_ATTR — is the
// SAME API on Arduino-ESP32 and on ESP-IDF, because Arduino-ESP32 runs on top of ESP-IDF. Those
// are therefore NOT wrapped here: wrapping ~270 FreeRTOS calls would rewrite every line of
// synchronisation in a patient-monitoring library for zero portability gain. What they need is
// only to EXIST on the host, and that is the job of incunest_afe4490_hal_host.h (fake FreeRTOS /
// esp_log / esp_timer / esp_attr with the real names), which replaces both the old
// incunest_afe4490_platform_stub.h and PulseNest's test/stubs/.
//
// This header declares the functions the library calls instead of Arduino. Exactly one of the
// three implementation files is active per build, selected by DETECTION, never by a hand-set macro:
//   incunest_afe4490_hal_arduino.cpp   #if defined(ARDUINO)                        forwards to Arduino
//   incunest_afe4490_hal_idf.cpp       #if defined(ESP_PLATFORM) && !defined(ARDUINO) driver/gpio + spi_master
//   incunest_afe4490_hal_host.cpp      #if !defined(ESP_PLATFORM)                  no-ops (see each note)
// `ARDUINO` is defined by the Arduino core; `ESP_PLATFORM` by ESP-IDF (and hence also under Arduino).
// PlatformIO compiles all three files and two of them are empty; the ESP-IDF CMakeLists.txt lists
// only the _idf one. No build has to define anything to pick its HAL.
//
// ACCEPTANCE CRITERION FOR THE ARDUINO PATH (phase 0 is a pure refactor for the incubator firmware):
// incunest_afe4490_hal_arduino.cpp must execute the same Arduino calls, in the same order, with
// the same arguments as v0.90b did inline — byte for byte on the SPI bus. The xtensa object code
// before/after is compared (disassembly, ignoring __LINE__ in log strings) to prove it.

#pragma once

#include <stddef.h>
#include <stdint.h>

// ── GPIO ─────────────────────────────────────────────────────────────────────────────────────

// Configure `pin` as a push-pull output and drive it to `level` in the same call.
// Used once, for the AFE4490 chip-select: v0.90b did pinMode(cs, OUTPUT); digitalWrite(cs, HIGH).
//   arduino: pinMode(OUTPUT) + digitalWrite      idf: gpio_config(GPIO_MODE_OUTPUT) + gpio_set_level
//   host: no-op
void incunest_afe4490_hal_gpio_init_output(int pin, bool level);

// Configure `pin` as an input with the internal pull-up. Used once, for DRDY (INPUT_PULLUP).
//   arduino: pinMode(INPUT_PULLUP)      idf: gpio_config(GPIO_MODE_INPUT, pull_up_en)      host: no-op
void incunest_afe4490_hal_gpio_init_input_pullup(int pin);

// Drive an output pin. Used for the manual chip-select around every SPI transaction.
//   arduino: digitalWrite(pin, level ? HIGH : LOW)      idf: gpio_set_level      host: no-op
void incunest_afe4490_hal_gpio_write(int pin, bool level);

// ── DRDY interrupt ───────────────────────────────────────────────────────────────────────────

// Handler type: same shape Arduino's attachInterrupt() takes — no argument, no return. The
// library's handler is `static void IRAM_ATTR _drdy_isr_static()`; it runs in interrupt context
// and only does xSemaphoreGiveFromISR() + portYIELD_FROM_ISR(), as required.
typedef void (*incunest_afe4490_hal_isr_t)(void);

// Attach `isr` to the RISING edge of `pin` and enable it. Returns false if the platform could not
// install the handler (ESP-IDF: gpio_install_isr_service / gpio_isr_handler_add failed).
//   arduino: attachInterrupt(digitalPinToInterrupt(pin), isr, RISING) — digitalPinToInterrupt() is
//            the identity on ESP32; kept in the Arduino implementation so the call is literally
//            what v0.90b did.
//   idf:     gpio_install_isr_service(0) once — flag 0 is what Arduino-ESP32 2.0.x uses, so the
//            handler is NOT guaranteed to run while flash is being written (OTA, NVS): unchanged
//            from today, recorded so nobody "improves" it without measuring —, then
//            gpio_set_intr_type(GPIO_INTR_POSEDGE) + gpio_isr_handler_add(trampoline) + gpio_intr_enable.
//   host:    stores nothing, returns false — begin() must not pretend a DRDY exists.
bool incunest_afe4490_hal_gpio_attach_isr_rising(int pin, incunest_afe4490_hal_isr_t isr);

// Detach and disable the interrupt on `pin`. Safe to call when nothing is attached.
//   arduino: detachInterrupt(digitalPinToInterrupt(pin))      idf: gpio_intr_disable + gpio_isr_handler_remove
//   host: no-op
void incunest_afe4490_hal_gpio_detach_isr(int pin);

// ── SPI ──────────────────────────────────────────────────────────────────────────────────────
//
// The AFE4490 is addressed with 4-byte frames: 1 address byte + 24 data bits, chip-select held
// low for the whole frame, 2 MHz, MSB first, mode 0 (CPOL=0, CPHA=0). Those three bus parameters
// are the library's, not the application's, so they are fixed here rather than passed around.
static const uint32_t INCUNEST_AFE4490_HAL_SPI_CLOCK_HZ = 2000000;

// The SPI BUS (pins, host peripheral) is owned by the application, exactly as in v0.90b, where the
// application called SPI.begin() and the library only ran transactions on it ("This library does
// not call SPI.begin() internally" — see begin() in the main header). The HAL keeps that split:
//
// Called once from begin(). Prepares whatever the platform needs to run transactions on the bus
// the application already initialised. Returns false if that is impossible.
//   arduino: nothing to do (the global SPI object is ready once the app called SPI.begin()) → true
//   idf:     spi_bus_add_device(INCUNEST_AFE4490_HAL_SPI_HOST, {clock, mode 0, spics_io_num = -1,
//            queue_size 1}) → a device handle kept inside the HAL. spics_io_num = -1 because the
//            library drives CS itself, as it always did. The bus must have been initialised by
//            the app with spi_bus_initialize() (as SPO2.cpp does through its SPI shim today).
//   host:    → true (transactions become no-ops that return 0)
bool incunest_afe4490_hal_spi_attach(void);

// Called once from stop(). Releases what _attach() acquired.
//   arduino: no-op      idf: spi_bus_remove_device      host: no-op
void incunest_afe4490_hal_spi_detach(void);

// One complete register transaction, atomic on the bus:
//   acquire bus → CS low → clock `n` bytes out of `tx` while reading `n` bytes into `rx` → CS high
//   → release bus.
// `rx` may be nullptr when the reply is not needed (register writes). `n` is 4 for the AFE4490.
//   arduino: SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE0)); digitalWrite(cs, LOW);
//            then `n` calls to SPI.transfer(uint8_t) — one per byte, NOT the buffer overload — so
//            the bus sees the very same byte-by-byte sequence v0.90b produced; digitalWrite(cs, HIGH);
//            SPI.endTransaction().
//   idf:     spi_device_acquire_bus(); CS low; ONE spi_device_polling_transmit() carrying all `n`
//            bytes (SPI_TRANS_USE_TXDATA/RXDATA, no DMA); CS high; spi_device_release_bus().
//            One driver call per register instead of the four the Arduino-shaped shim needs — the
//            per-byte overhead of spi_device_polling_transmit() is what makes SPI cost 5–10× more
//            under IDF, so this is the lever that keeps the 500 Hz acquisition task cheap.
//            Timing is still to be measured on the bench ($TIMING spi_us per sample).
//   host:    fills `rx` with zeros if given; returns.
void incunest_afe4490_hal_spi_transfer(int cs_pin, const uint8_t* tx, uint8_t* rx, size_t n);

// ── Console ──────────────────────────────────────────────────────────────────────────────────

// Raw printf to the diagnostic console. Used ONLY by the INCUNEST_TIMING_STATS frames ($TIMING,
// $TASKS, $TASKS_END), which are a checksummed line protocol parsed by pulsenest_lab.py: the line
// must start with '$' and end with "*XX\r\n" and nothing else — so this must NOT go through
// ESP_LOGx, which prepends "I (ms) tag:" and appends its own newline (the mistake the vendored
// copy in IncuNest made).
//   arduino: vsnprintf + Serial.write      idf: vsnprintf + fwrite(stdout) (UART0 through the VFS)
//   host: vsnprintf + fwrite(stdout)       all three: the formatted line is also handed to the sink below
void incunest_afe4490_hal_console_printf(const char* fmt, ...) __attribute__((format(printf, 1, 2)));

// Optional tee for those console lines (v0.92). Once a sink is set, every line console_printf()
// writes is ALSO passed to it, complete and NUL-terminated (len excludes the NUL), on the calling
// task — the library's 500 Hz acquisition task. So the sink must only queue, never block or send:
// PulseNest drops the line into its UDP data queue so the ESP32 TIMING window works with no UART
// attached; motherBoard sets none. nullptr clears it. Not thread-safe against a concurrent
// console_printf(): set it before begin().
typedef void (*incunest_afe4490_hal_console_sink_t)(const char* line, size_t len);
void incunest_afe4490_hal_console_set_sink(incunest_afe4490_hal_console_sink_t sink);

// ── ESP-IDF SPI host selection (idf implementation only) ─────────────────────────────────────
// Which SPI peripheral the AFE4490 hangs on. Only the _idf implementation reads it; Arduino
// resolves this inside the global SPI object the application configured. Override from the
// build (-DINCUNEST_AFE4490_HAL_SPI_HOST=SPI3_HOST) if a board wires the AFE elsewhere.
#if defined(ESP_PLATFORM) && !defined(ARDUINO)
  #ifndef INCUNEST_AFE4490_HAL_SPI_HOST
    #define INCUNEST_AFE4490_HAL_SPI_HOST SPI2_HOST
  #endif
#endif

// ── Not in this contract, on purpose ─────────────────────────────────────────────────────────
// * FreeRTOS (xSemaphore*, xQueue*, xTaskCreatePinnedToCore, vTaskDelay, pdMS_TO_TICKS, ...),
//   esp_timer_get_time(), IRAM_ATTR: identical on both embedded targets; provided to the host
//   by incunest_afe4490_hal_host.h.
// * ESP_LOGx: same NAMES on both targets but not the same implementation — Arduino-ESP32
//   redefines them in esp32-hal-log.h. The main header includes that file under ARDUINO so the
//   Arduino logging path (format, CORE_DEBUG_LEVEL) is the one the library always had.
// * constrain(): an Arduino macro. Replaced in the library by its own clamp helper (the build is
//   gnu++11 under Arduino, so std::clamp is not available).
// * SPI.begin() / spi_bus_initialize(): the application's, as before.
