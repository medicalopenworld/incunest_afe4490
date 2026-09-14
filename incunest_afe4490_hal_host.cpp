// incunest_afe4490_hal_host.cpp — HAL implementation on the host (PC): no hardware
// Library version: v0.91 — host (no hardware)
// Spec: incunest_afe4490_spec.md §1.4 "Hardware abstraction layer", §9 "Offline runner"
// Author: Medical Open World — http://medicalopenworld.org — <contact@medicalopenworld.org>
//
// Everything that would touch a pin or a bus does nothing, and says so where it can: attaching
// the DRDY interrupt returns false, so begin() cannot pretend the acquisition is running. The
// console is the process's stdout. Selected when ESP_PLATFORM is not defined — i.e. by any build
// that is not ESP-IDF or Arduino-ESP32 — with no macro to set.

#if !defined(ESP_PLATFORM)

#include "incunest_afe4490_hal.h"

#include <cstdarg>
#include <cstdio>
#include <cstring>

void incunest_afe4490_hal_gpio_init_output(int, bool) {}
void incunest_afe4490_hal_gpio_init_input_pullup(int) {}
void incunest_afe4490_hal_gpio_write(int, bool) {}

bool incunest_afe4490_hal_gpio_attach_isr_rising(int, incunest_afe4490_hal_isr_t) { return false; }
void incunest_afe4490_hal_gpio_detach_isr(int) {}

bool incunest_afe4490_hal_spi_attach(void) { return true; }
void incunest_afe4490_hal_spi_detach(void) {}

void incunest_afe4490_hal_spi_transfer(int, const uint8_t*, uint8_t* rx, size_t n) {
    if (rx != nullptr) memset(rx, 0, n);
}

void incunest_afe4490_hal_console_printf(const char* fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    vprintf(fmt, ap);
    va_end(ap);
}

#endif  // !defined(ESP_PLATFORM)
