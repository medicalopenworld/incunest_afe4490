// incunest_afe4490_hal_idf.cpp — HAL implementation on ESP-IDF without Arduino
// Library version: v0.91 — ESP32-S3, ESP-IDF + FreeRTOS
// Spec: incunest_afe4490_spec.md §1.4 "Hardware abstraction layer"
// Author: Medical Open World — http://medicalopenworld.org — <contact@medicalopenworld.org>
//
// Talks to driver/gpio and driver/spi_master directly. It does NOT depend on any application
// platform layer (e.g. IncuNest's components/incunest_platform): a library must not depend on
// its consumer. Compiled only by the ESP-IDF CMake build (see CMakeLists.txt); the guard makes
// it an empty translation unit everywhere else.

#if defined(ESP_PLATFORM) && !defined(ARDUINO)

#include "incunest_afe4490_hal.h"

#include <cstdarg>
#include <cstdio>
#include <cstring>

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_attr.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"

static const char* TAG = "afe4490_hal";

// ── GPIO ─────────────────────────────────────────────────────────────────────────────────────

static void configure_pin(int pin, gpio_mode_t mode, gpio_pullup_t pull_up) {
    gpio_config_t cfg = {};
    cfg.pin_bit_mask  = 1ULL << pin;
    cfg.mode          = mode;
    cfg.pull_up_en    = pull_up;
    cfg.pull_down_en  = GPIO_PULLDOWN_DISABLE;
    cfg.intr_type     = GPIO_INTR_DISABLE;
    const esp_err_t err = gpio_config(&cfg);
    if (err != ESP_OK) ESP_LOGE(TAG, "gpio_config(%d) -> %s", pin, esp_err_to_name(err));
}

void incunest_afe4490_hal_gpio_init_output(int pin, bool level) {
    // Same order as Arduino's pinMode(OUTPUT) then digitalWrite(): configure, then drive.
    configure_pin(pin, GPIO_MODE_OUTPUT, GPIO_PULLUP_DISABLE);
    gpio_set_level(static_cast<gpio_num_t>(pin), level ? 1 : 0);
}

void incunest_afe4490_hal_gpio_init_input_pullup(int pin) {
    configure_pin(pin, GPIO_MODE_INPUT, GPIO_PULLUP_ENABLE);
}

void incunest_afe4490_hal_gpio_write(int pin, bool level) {
    gpio_set_level(static_cast<gpio_num_t>(pin), level ? 1 : 0);
}

// ── DRDY interrupt ───────────────────────────────────────────────────────────────────────────
// ESP-IDF hands the handler a void* argument; the HAL contract (like Arduino) takes none. A
// per-GPIO table plus a trampoline bridges the two — indexed by pin, so the ISR does no search.

static incunest_afe4490_hal_isr_t s_isr[GPIO_NUM_MAX] = {};
static bool s_isr_service_installed = false;

static void IRAM_ATTR isr_trampoline(void* arg) {
    const uint32_t pin = static_cast<uint32_t>(reinterpret_cast<uintptr_t>(arg));
    if (pin < GPIO_NUM_MAX && s_isr[pin] != nullptr) s_isr[pin]();
}

bool incunest_afe4490_hal_gpio_attach_isr_rising(int pin, incunest_afe4490_hal_isr_t isr) {
    if (pin < 0 || pin >= GPIO_NUM_MAX || isr == nullptr) {
        ESP_LOGE(TAG, "attach_isr(%d): invalid pin or handler", pin);
        return false;
    }
    if (!s_isr_service_installed) {
        // Flag 0 = what Arduino-ESP32 2.0.x uses: the handler is NOT placed in IRAM-safe mode and
        // will not run while flash is being written (OTA, NVS). Unchanged from today on purpose;
        // switching to ESP_INTR_FLAG_IRAM is a behavioural change to be measured, not assumed.
        const esp_err_t err = gpio_install_isr_service(0);
        if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {   // INVALID_STATE: already installed by the app
            ESP_LOGE(TAG, "gpio_install_isr_service -> %s", esp_err_to_name(err));
            return false;
        }
        s_isr_service_installed = true;
    }
    const gpio_num_t gpio = static_cast<gpio_num_t>(pin);
    s_isr[pin] = isr;
    gpio_set_intr_type(gpio, GPIO_INTR_POSEDGE);
    const esp_err_t err = gpio_isr_handler_add(gpio, isr_trampoline,
                                               reinterpret_cast<void*>(static_cast<uintptr_t>(pin)));
    if (err != ESP_OK) {
        s_isr[pin] = nullptr;
        ESP_LOGE(TAG, "gpio_isr_handler_add(%d) -> %s", pin, esp_err_to_name(err));
        return false;
    }
    gpio_intr_enable(gpio);
    return true;
}

void incunest_afe4490_hal_gpio_detach_isr(int pin) {
    if (pin < 0 || pin >= GPIO_NUM_MAX) return;
    const gpio_num_t gpio = static_cast<gpio_num_t>(pin);
    gpio_intr_disable(gpio);
    gpio_isr_handler_remove(gpio);
    s_isr[pin] = nullptr;
}

// ── SPI ──────────────────────────────────────────────────────────────────────────────────────
// The bus (spi_bus_initialize) belongs to the application, as SPI.begin() did under Arduino.
// The HAL only adds a device on it: 2 MHz, mode 0, CS driven by the library over GPIO
// (spics_io_num = -1), one transaction in flight.

static spi_device_handle_t s_spi_dev = nullptr;

bool incunest_afe4490_hal_spi_attach(void) {
    if (s_spi_dev != nullptr) return true;
    spi_device_interface_config_t cfg = {};
    cfg.clock_speed_hz = static_cast<int>(INCUNEST_AFE4490_HAL_SPI_CLOCK_HZ);
    cfg.mode           = 0;        // CPOL = 0, CPHA = 0 (SPI_MODE0)
    cfg.spics_io_num   = -1;       // chip-select is the library's, over GPIO, as always
    cfg.queue_size     = 1;
    const esp_err_t err = spi_bus_add_device(INCUNEST_AFE4490_HAL_SPI_HOST, &cfg, &s_spi_dev);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "spi_bus_add_device -> %s (was spi_bus_initialize() called by the app?)",
                 esp_err_to_name(err));
        s_spi_dev = nullptr;
        return false;
    }
    return true;
}

void incunest_afe4490_hal_spi_detach(void) {
    if (s_spi_dev == nullptr) return;
    spi_bus_remove_device(s_spi_dev);
    s_spi_dev = nullptr;
}

void incunest_afe4490_hal_spi_transfer(int cs_pin, const uint8_t* tx, uint8_t* rx, size_t n) {
    if (s_spi_dev == nullptr || tx == nullptr || n == 0) return;

    spi_transaction_t t = {};
    t.length = n * 8;
    if (n <= 4) {
        // Whole AFE4490 frame in the transaction's own 4-byte fields: no DMA-capable buffers
        // needed, and ONE driver call per register instead of one per byte.
        t.flags = SPI_TRANS_USE_TXDATA | (rx != nullptr ? SPI_TRANS_USE_RXDATA : 0);
        memcpy(t.tx_data, tx, n);
    } else {
        // Not used by the library (frames are 4 bytes). Buffers must be DMA-capable if the
        // application initialised the bus with DMA.
        t.tx_buffer = tx;
        t.rx_buffer = rx;
    }

    // acquire → CS low → frame → CS high → release: what SPI.beginTransaction()/endTransaction()
    // guaranteed under Arduino, so nothing else can slip a transfer inside an AFE register.
    if (spi_device_acquire_bus(s_spi_dev, portMAX_DELAY) != ESP_OK) return;
    gpio_set_level(static_cast<gpio_num_t>(cs_pin), 0);
    const esp_err_t err = spi_device_polling_transmit(s_spi_dev, &t);
    gpio_set_level(static_cast<gpio_num_t>(cs_pin), 1);
    spi_device_release_bus(s_spi_dev);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "spi_device_polling_transmit -> %s", esp_err_to_name(err));
        return;
    }
    if (rx != nullptr && n <= 4) memcpy(rx, t.rx_data, n);
}

// ── Console ──────────────────────────────────────────────────────────────────────────────────

static incunest_afe4490_hal_console_sink_t s_console_sink = nullptr;

void incunest_afe4490_hal_console_set_sink(incunest_afe4490_hal_console_sink_t sink) {
    s_console_sink = sink;
}

void incunest_afe4490_hal_console_printf(const char* fmt, ...) {
    // Raw stdout (UART0 through the VFS): the $TIMING/$TASKS lines are a checksummed protocol
    // and must not pick up ESP_LOG's prefix or trailing newline.
    // Formatted into a buffer first so the optional sink gets the same complete line as the console.
    char buf[320];
    va_list ap;
    va_start(ap, fmt);
    const int n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (n <= 0) return;
    const size_t len = (size_t)n < sizeof(buf) ? (size_t)n : sizeof(buf) - 1;
    fwrite(buf, 1, len, stdout);
    if (s_console_sink) s_console_sink(buf, len);   // tee (v0.92): PulseNest forwards it to UDP
}

#endif  // defined(ESP_PLATFORM) && !defined(ARDUINO)
