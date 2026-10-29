// incunest_afe4490_hal_host.h — host (PC) stand-ins for the ESP-IDF / FreeRTOS API the library uses
// Library version: v0.91 — host (no hardware)
// Spec: incunest_afe4490_spec.md §1.4 "Hardware abstraction layer", §9 "Offline runner"
// Author: Medical Open World — http://medicalopenworld.org — <contact@medicalopenworld.org>
//
// Included by incunest_afe4490.h when ESP_PLATFORM is NOT defined, i.e. for the unit tests in
// test/ and for tools/offline_runner. It provides, with their REAL names, the handful of FreeRTOS,
// esp_log and esp_timer symbols the library calls, so that the library source compiles unchanged
// on a PC. Nothing here is Arduino: the Arduino-only calls went behind incunest_afe4490_hal.h,
// whose host implementation is incunest_afe4490_hal_host.cpp.
//
// This file replaces two things that used to coexist (and drift apart): the library's own
// incunest_afe4490_platform_stub.h (INCUNEST_OFFLINE path) and PulseNest's test/stubs/ (UNIT_TEST
// path). One host path, one file, in the library's repo.
//
// Semantics on the host, chosen so that nothing PRETENDS to work:
//   * xSemaphoreCreate*/xQueueCreate return nullptr → begin() fails with its own
//     "FreeRTOS object creation failed" and never claims to have started. Algorithms are fed
//     through the UNIT_TEST accessors (test_feed_*) instead, exactly as before.
//   * xSemaphoreTake/Give succeed (the setters take the mutex only when _initialized, which is
//     never true on the host, but the calls still compile and are harmless).
//   * xQueueSend/xQueueReceive report failure; xTaskCreatePinnedToCore reports pdFAIL.
//   * ESP_LOGx print to stderr (so the offline runner's stdout stays clean data).
//   * esp_timer_get_time() is a real monotonic microsecond clock (INCUNEST_TIMING_STATS on host).

#pragma once

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <chrono>

// ── FreeRTOS types and constants ─────────────────────────────────────────────────────────────
typedef void*         SemaphoreHandle_t;
typedef void*         QueueHandle_t;
typedef void*         TaskHandle_t;
typedef uint32_t      TickType_t;
typedef int           BaseType_t;
typedef unsigned int  UBaseType_t;
typedef void (*TaskFunction_t)(void*);

#define pdTRUE        ((BaseType_t)1)
#define pdFALSE       ((BaseType_t)0)
#define pdPASS        pdTRUE
#define pdFAIL        pdFALSE
#define portMAX_DELAY ((TickType_t)0xFFFFFFFFUL)
#define pdMS_TO_TICKS(ms) ((TickType_t)(ms))
#define portYIELD_FROM_ISR(x) ((void)(x))

// ── FreeRTOS functions ───────────────────────────────────────────────────────────────────────
inline SemaphoreHandle_t xSemaphoreCreateMutex()                                          { return nullptr; }
inline SemaphoreHandle_t xSemaphoreCreateBinary()                                         { return nullptr; }
inline BaseType_t        xSemaphoreTake(SemaphoreHandle_t, TickType_t)                    { return pdTRUE; }
inline BaseType_t        xSemaphoreGive(SemaphoreHandle_t)                                { return pdTRUE; }
inline BaseType_t        xSemaphoreGiveFromISR(SemaphoreHandle_t, BaseType_t* woken)      { if (woken) *woken = pdFALSE; return pdTRUE; }
inline void              vSemaphoreDelete(SemaphoreHandle_t)                              {}

inline QueueHandle_t     xQueueCreate(UBaseType_t, UBaseType_t)                           { return nullptr; }
inline BaseType_t        xQueueSend(QueueHandle_t, const void*, TickType_t)               { return pdFALSE; }
inline BaseType_t        xQueueReceive(QueueHandle_t, void*, TickType_t)                  { return pdFALSE; }
inline void              vQueueDelete(QueueHandle_t)                                      {}

inline BaseType_t xTaskCreatePinnedToCore(TaskFunction_t, const char*, uint32_t, void*,
                                          UBaseType_t, TaskHandle_t* out, int)             { if (out) *out = nullptr; return pdFAIL; }
inline void              vTaskDelete(TaskHandle_t)                                        {}
inline void              vTaskDelay(TickType_t)                                           {}
inline UBaseType_t       uxTaskGetStackHighWaterMark(TaskHandle_t)                        { return 0; }

// ── esp_attr.h ───────────────────────────────────────────────────────────────────────────────
#define IRAM_ATTR

// ── esp_log.h ────────────────────────────────────────────────────────────────────────────────
// stderr on purpose: tools/offline_runner writes its results to stdout.
#define ESP_LOGE(tag, fmt, ...) std::fprintf(stderr, "[E][%s] " fmt "\n", tag, ##__VA_ARGS__)
#define ESP_LOGW(tag, fmt, ...) std::fprintf(stderr, "[W][%s] " fmt "\n", tag, ##__VA_ARGS__)
#define ESP_LOGI(tag, fmt, ...) std::fprintf(stderr, "[I][%s] " fmt "\n", tag, ##__VA_ARGS__)
#define ESP_LOGD(tag, fmt, ...) ((void)0)
#define ESP_LOGV(tag, fmt, ...) ((void)0)

// ── esp_timer.h ──────────────────────────────────────────────────────────────────────────────
// Microseconds since an arbitrary origin, monotonic — the contract esp_timer_get_time() gives.
inline int64_t esp_timer_get_time() {
    using namespace std::chrono;
    static const steady_clock::time_point t0 = steady_clock::now();
    return duration_cast<microseconds>(steady_clock::now() - t0).count();
}
