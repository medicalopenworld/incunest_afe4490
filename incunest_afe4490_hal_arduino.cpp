// incunest_afe4490_hal_arduino.cpp — HAL implementation on Arduino-ESP32
// Library version: v0.91 — ESP32-S3, Arduino + FreeRTOS
// Spec: incunest_afe4490_spec.md §1.4 "Hardware abstraction layer"
// Author: Medical Open World — http://medicalopenworld.org — <contact@medicalopenworld.org>
//
// Pure forwarding. Every function below performs the same Arduino calls, in the same order and
// with the same arguments, that incunest_afe4490.cpp v0.90b made inline — so for a firmware that
// stays on Arduino (IncuNest motherBoard until its ESP-IDF port merges) the HAL is a refactor and
// nothing else. That is the reason this file exists at all; when no Arduino consumer is left it
// can simply be deleted.
//
// Active only under Arduino: ARDUINO is defined by the Arduino core. PlatformIO compiles every
// .cpp of the library, so this file is an empty translation unit on ESP-IDF and on the host.

#if defined(ARDUINO)

#include "incunest_afe4490_hal.h"

#include <Arduino.h>
#include <SPI.h>
#include <cstdarg>
#include <cstdio>

// ── GPIO ─────────────────────────────────────────────────────────────────────────────────────

void incunest_afe4490_hal_gpio_init_output(int pin, bool level) {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, level ? HIGH : LOW);
}

void incunest_afe4490_hal_gpio_init_input_pullup(int pin) {
    pinMode(pin, INPUT_PULLUP);
}

void incunest_afe4490_hal_gpio_write(int pin, bool level) {
    digitalWrite(pin, level ? HIGH : LOW);
}

// ── DRDY interrupt ───────────────────────────────────────────────────────────────────────────

bool incunest_afe4490_hal_gpio_attach_isr_rising(int pin, incunest_afe4490_hal_isr_t isr) {
    // digitalPinToInterrupt() is the identity on ESP32; kept so the call is literally v0.90b's.
    attachInterrupt(digitalPinToInterrupt(pin), isr, RISING);
    return true;    // Arduino's attachInterrupt() has no failure path to report
}

void incunest_afe4490_hal_gpio_detach_isr(int pin) {
    detachInterrupt(digitalPinToInterrupt(pin));
}

// ── SPI ──────────────────────────────────────────────────────────────────────────────────────

bool incunest_afe4490_hal_spi_attach(void) {
    // The application already called SPI.begin(); the global SPI object is the bus.
    return true;
}

void incunest_afe4490_hal_spi_detach(void) {}

void incunest_afe4490_hal_spi_transfer(int cs_pin, const uint8_t* tx, uint8_t* rx, size_t n) {
    // One SPI.transfer(uint8_t) per byte — NOT the buffer overload — so the bus sees the exact
    // byte-by-byte sequence v0.90b produced from _write_reg() / _read_spi_raw().
    SPI.beginTransaction(SPISettings(INCUNEST_AFE4490_HAL_SPI_CLOCK_HZ, MSBFIRST, SPI_MODE0));
    digitalWrite(cs_pin, LOW);
    for (size_t i = 0; i < n; i++) {
        const uint8_t r = SPI.transfer(tx[i]);
        if (rx) rx[i] = r;
    }
    digitalWrite(cs_pin, HIGH);
    SPI.endTransaction();
}

// ── Console ──────────────────────────────────────────────────────────────────────────────────

void incunest_afe4490_hal_console_printf(const char* fmt, ...) {
    // Serial.printf() cannot take a va_list; format here and write the bytes. The only caller
    // emits one $TIMING/$TASKS line (≤ 256-byte payload + "$", "*XX\r\n"), well inside the buffer.
    char buf[320];
    va_list ap;
    va_start(ap, fmt);
    const int n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (n <= 0) return;
    const size_t len = (size_t)n < sizeof(buf) ? (size_t)n : sizeof(buf) - 1;
    Serial.write(reinterpret_cast<const uint8_t*>(buf), len);
}

#endif  // defined(ARDUINO)
