#pragma once

// incunest_afe4490 — Medical Open World AFE4490 driver + PPG algorithms (HR, SpO2)
// Library version: v0.19 — ESP32-S3, Arduino + FreeRTOS
// Spec: incunest_afe4490_spec.md
// Author: Medical Open World — http://medicalopenworld.org — <contact@medicalopenworld.org>

#ifdef INCUNEST_OFFLINE
  #ifndef UNIT_TEST
    #define UNIT_TEST
  #endif
  #include "incunest_afe4490_platform_stub.h"
#else
  #include <Arduino.h>
  #include <SPI.h>
  #include <freertos/FreeRTOS.h>
  #include <freertos/task.h>
  #include <freertos/semphr.h>
  #include <freertos/queue.h>
  #include <stdint.h>
#endif

// ── Compile-time configuration (override before including this header) ────────
#ifndef INCUNEST_AFE4490_QUEUE_SIZE
#define INCUNEST_AFE4490_QUEUE_SIZE      10
#endif

#ifndef INCUNEST_AFE4490_TASK_PRIORITY
#define INCUNEST_AFE4490_TASK_PRIORITY   5
#endif

#ifndef INCUNEST_AFE4490_TASK_STACK
#define INCUNEST_AFE4490_TASK_STACK      8192  // increased from 4096: HR3 FFT calls cosf/sinf which needs extra stack
#endif

#ifndef INCUNEST_AFE4490_HR2_TASK_STACK
#define INCUNEST_AFE4490_HR2_TASK_STACK  3072  // acorr_buf[138] on stack + overhead
#endif

#ifndef INCUNEST_AFE4490_HR3_TASK_STACK
#define INCUNEST_AFE4490_HR3_TASK_STACK  2048  // FFT data lives in _hr3_fft member, minimal stack
#endif

#ifndef INCUNEST_AFE4490_HR23_TASK_PRIORITY
#define INCUNEST_AFE4490_HR23_TASK_PRIORITY  (INCUNEST_AFE4490_TASK_PRIORITY - 1)
#endif

#ifndef INCUNEST_TIMING_STATS
#define INCUNEST_TIMING_STATS 0
#endif

// ── Public data struct ────────────────────────────────────────────────────────
struct AFE4490Data {
    // Field order mirrors the $M1/$P1 serial frame: raw signals first, then processed outputs
    // Raw ADC outputs (6 signals from AFE4490)
    int32_t led2;        // LED2VAL  — RED raw          (frame: RED)
    int32_t led1;        // LED1VAL  — IR raw           (frame: IR)
    int32_t aled2;       // ALED2VAL — ambient after LED2 (frame: RED_Amb)
    int32_t aled1;       // ALED1VAL — ambient after LED1 (frame: IR_Amb)
    int32_t led2_aled2;  // LED2-ALED2 — RED ambient-corrected (frame: RED_Sub)
    int32_t led1_aled1;  // LED1-ALED1 — IR ambient-corrected  (frame: IR_Sub)
    // Processed outputs
    int32_t ppg;         // filtered PPG of selected channel
    float   spo2;        // SpO2 in %
    float   spo2_sqi;    // SpO2 Signal Quality Index [0–1]: PI-based; 0=invalid/no finger, 1=full quality (PI ≥ 2%)
    float   spo2_r;      // R ratio used for SpO2 calculation: (AC_red/DC_red)/(AC_ir/DC_ir)
    float   pi;          // Perfusion Index: (AC_ir / DC_ir) * 100 [%]
    float   hr1;         // HR1 (peak detection) in bpm
    float   hr1_sqi;     // HR1 Signal Quality Index [0–1]: 1 − CV/0.15; 0=arrhythmia/artefact/invalid, 1=perfectly regular
    float   hr2;         // HR2 (autocorrelation) in bpm
    float   hr2_sqi;     // HR2 Signal Quality Index [0–1]: normalised autocorrelation at dominant lag; 0=no periodicity, 1=perfect
    float   hr3;         // HR3 (FFT + HPS) in bpm
    float   hr3_sqi;     // HR3 Signal Quality Index [0–1]: HPS peak prominence in search range; 0=diffuse HPS, 1=dominant peak
};

// ── Enumerations ──────────────────────────────────────────────────────────────
enum class AFE4490Channel {
    LED1,       // IR raw
    LED2,       // RED raw
    ALED1,      // ambient after LED1
    ALED2,      // ambient after LED2
    LED1_ALED1, // IR ambient-corrected (default)
    LED2_ALED2  // RED ambient-corrected
};

enum class AFE4490Filter {
    NONE,
    MOVING_AVERAGE,
    BUTTERWORTH  // default: 0.5–20 Hz Butterworth bandpass
};

enum class AFE4490TIAGain {
    RF_10K,
    RF_25K,
    RF_50K,
    RF_100K,
    RF_250K,
    RF_500K,  // default
    RF_1M
};

enum class AFE4490TIACF {
    CF_5P,    // 5 pF (default)
    CF_10P,
    CF_20P,
    CF_30P,
    CF_55P,
    CF_155P
};

enum class AFE4490Stage2Gain {
    GAIN_0DB,    // Stage 2 disabled (default)
    GAIN_3_5DB,
    GAIN_6DB,
    GAIN_9_5DB,
    GAIN_12DB
};

// ── Chip configuration snapshot ───────────────────────────────────────────────
// Returned by getConfig(). Groups all runtime-configurable parameters.
struct AFE4490Config {
    uint16_t          sample_rate_hz;    // ADC sample rate [Hz]
    uint8_t           num_averages;      // ADC hardware averages (1 = no averaging)
    float             led1_current_mA;   // IR LED drive current [mA]
    float             led2_current_mA;   // RED LED drive current [mA]
    uint8_t           led_range_mA;      // LED full-scale range: 75 or 150 mA
    AFE4490TIAGain    tia_gain;          // TIA transimpedance gain (RF)
    AFE4490TIACF      tia_cf;            // TIA feedback capacitance
    AFE4490Stage2Gain stage2_gain;       // Stage 2 amplifier gain
    AFE4490Channel    ppg_channel;       // PPG display channel selection
    AFE4490Filter     filter_type;       // PPG display filter type
    float             filter_f_low_hz;   // PPG display bandpass lower cutoff [Hz]
    float             filter_f_high_hz;  // PPG display bandpass upper cutoff [Hz]
    float             hr2_f_low_hz;      // HR2 bandpass lower cutoff [Hz]
    float             hr2_f_high_hz;     // HR2 bandpass upper cutoff [Hz]
    float             hr3_f_high_hz;     // HR3 anti-aliasing LP cutoff [Hz]
    float             spo2_a;            // SpO2 calibration: SpO2 = a - b*R
    float             spo2_b;
};

// ── INCUNEST_AFE4490 class ─────────────────────────────────────────────────────────
class INCUNEST_AFE4490 {
public:
    INCUNEST_AFE4490();
    ~INCUNEST_AFE4490();

    // Initialization — configures chip with defaults, attaches DRDY ISR, starts task.
    // Requires SPI.begin() to have been called beforehand by the application.
    // This library does not call SPI.begin() internally to avoid interfering with
    // other SPI devices sharing the same bus.
#ifndef INCUNEST_OFFLINE
    void begin(int pin_cs, int pin_drdy);
#endif

    // Chip configuration setters (callable before or after begin())
    void setSampleRate(uint16_t hz);        // 63–5000 Hz; recalculates NUMAV_max
    void setNumAverages(uint8_t num);       // 1=no averaging; clamped to floor(5000/PRF)
    void setLED1Current(float mA);
    void setLED2Current(float mA);
    void setLEDRange(uint8_t mA);           // 75 or 150 mA
    void setTIAGain(AFE4490TIAGain gain);
    void setTIACF(AFE4490TIACF cf);
    void setStage2Gain(AFE4490Stage2Gain gain);

    // Signal and filter configuration
    void setPPGChannel(AFE4490Channel channel);
    void setFilter(AFE4490Filter type, float f_low_hz = 0.5f, float f_high_hz = 20.0f);

    // HR2 bandpass filter cutoffs (default 0.5–5 Hz); callable before or after begin()
    void setHR2Filter(float f_low_hz = 0.5f, float f_high_hz = 5.0f);

    // HR3 low-pass filter cutoff (default 10 Hz, anti-aliasing before decimation)
    void setHR3Filter(float f_high_hz = 10.0f);

    // Data retrieval — non-blocking; returns true if data was available
#ifndef INCUNEST_OFFLINE
    bool getData(AFE4490Data& data);
#endif

    // Shutdown — detaches ISR, deletes internal task and FreeRTOS objects, resets state.
    // After stop(), begin() can be called again to restart.
#ifndef INCUNEST_OFFLINE
    void stop();
#endif

    // SpO2 calibration coefficients (SpO2 = a - b*R).
    // Defaults are experimentally calibrated for UpnMed U401-D(01AS-F), Nellcor Non-Oximax type.
    void setSpO2Coefficients(float a, float b);

    // Returns a snapshot of all current chip and algorithm configuration parameters.
    // Thread-safe: takes the relevant mutexes if the library is initialized.
    // Callable before or after begin().
    AFE4490Config getConfig();

    // ISR entry point (must be public for static trampoline)
#ifndef INCUNEST_OFFLINE
    void _drdy_isr();
#endif

private:
    // ── Private types ─────────────────────────────────────────────────────────

    struct BiquadState { float v1, v2; };

    // BiquadFilter groups coefficients, state and cutoff frequencies for one filter instance.
    // _recalc_biquad() writes b0/b1/b2/a1/a2 from f_low/f_high and _sample_rate_hz.
    // _biquad_step() and _biquad_precharge() operate on state in-place.
    struct BiquadFilter {
        float f_low, f_high;          // cutoff frequencies (Hz) — parameterisable at runtime
        float b0, b1, b2, a1, a2;    // DF-II transposed coefficients
        BiquadState state;
        bool  needs_precharge;        // true after reset; consumed on first sample
    };

    // ── SPI primitives ────────────────────────────────────────────────────────
#ifndef INCUNEST_OFFLINE
    void     _write_reg(uint8_t addr, uint32_t data);
    uint32_t _read_spi_raw(uint8_t addr);   // assumes SPI_READ already enabled
    uint32_t _read_reg(uint8_t addr);       // handles SPI_READ enable/disable
#endif

    // Sign-extend 22-bit two's complement ADC output
    static int32_t _sign_extend_22(uint32_t raw);

    // Recomputes rate-dependent algorithm parameters from _sample_rate_hz
    void _recalc_rate_params();
    // Recomputes Butterworth bandpass biquad coefficients into filt from _sample_rate_hz and filt.f_low/f_high
    void _recalc_biquad(BiquadFilter& filt);
    // Recomputes Butterworth low-pass biquad coefficients into filt (uses filt.f_high as cutoff)
    void _recalc_biquad_lp(BiquadFilter& filt);

    // Chip init
#ifndef INCUNEST_OFFLINE
    void _chip_init();
    void _apply_timing_regs();
    void _apply_analog_regs();
    void _apply_control_regs();
    uint32_t _build_tiagain();
#endif

    // FreeRTOS task
#ifndef INCUNEST_OFFLINE
    static void _task_trampoline(void* pv);
    void _task_body();
#endif

    // Signal processing
    float _biquad_process(float x, BiquadFilter& filt);  // precharge on first call, then step
    void  _process_sample(int32_t led1, int32_t led2, int32_t aled1, int32_t aled2,
                          int32_t led1_aled1, int32_t led2_aled2);

    // Algorithms — synchronous (used by unit tests and SpO2/HR1)
    void _update_spo2(int32_t ir_corr, int32_t red_corr);
    void _update_hr1(int32_t led1_aled1);
    // HR2/HR3 synchronous entry points (unit-test only; production uses split paths below)
    void _update_hr2(int32_t led1_aled1);
    void _update_hr3(int32_t led1_aled1);
    void _reset_algorithms();

    // HR2 async split: fast per-sample path + linearise + compute
    bool _update_hr2_sample(int32_t led1_aled1); // filter+decimate+buffer; returns true when interval fires
    void _linearize_hr2();                        // copy _hr2_buf → _hr2_seg (call under _state_mutex)
    void _compute_hr2();                          // autocorr on _hr2_seg → _hr2_result/_hr2_sqi_result

    // HR3 async split
    bool _update_hr3_sample(int32_t led1_aled1); // filter+decimate+buffer; returns true when interval fires
    void _linearize_hr3();                        // DC+Hann into _hr3_fft (call under _state_mutex)
    void _compute_hr3();                          // FFT+HPS on _hr3_fft → _hr3_result/_hr3_sqi_result

    // HR2/HR3 async FreeRTOS tasks
#ifndef INCUNEST_OFFLINE
    static void _hr2_task_trampoline(void* pv);
    void _hr2_task_body();
    static void _hr3_task_trampoline(void* pv);
    void _hr3_task_body();
#endif

    // ── Hardware ──
    int _pin_cs;
    int _pin_drdy;

    // ── FreeRTOS ──
    SemaphoreHandle_t _drdy_sem;
    SemaphoreHandle_t _spi_mutex;    // protects SPI bus access (_write_reg / _read_spi_raw)
    SemaphoreHandle_t _state_mutex;  // protects internal processing state (_ppg_channel, filter
                                     // buffers, SpO2/HR accumulators) shared between _process_sample()
                                     // and the config setters that do not access the SPI bus
    QueueHandle_t     _data_queue;
    TaskHandle_t      _task_handle;
    bool              _initialized;

    // ── HR2/HR3 async computation tasks ──────────────────────────────────────
    // Task A (_task_body) runs the fast per-sample path and signals Task B/C when
    // the computation window fires. Task B/C run the slow autocorr / FFT+HPS
    // outside the real-time loop, then write results under _state_mutex.
    SemaphoreHandle_t _hr2_calc_sem;    // given by Task A, taken by Task B
    SemaphoreHandle_t _hr3_calc_sem;    // given by Task A, taken by Task C
    TaskHandle_t      _hr2_task_handle;
    TaskHandle_t      _hr3_task_handle;
    volatile bool     _hr2_computing;   // true while Task B holds _hr2_seg; prevents Task A from overwriting
    volatile bool     _hr3_computing;   // true while Task C uses _hr3_fft; prevents Task A from overwriting
    float             _hr2_result;      // written by Task B, copied to _current_data under _state_mutex
    float             _hr2_sqi_result;
    float             _hr3_result;      // written by Task C
    float             _hr3_sqi_result;

#if INCUNEST_TIMING_STATS
    // ── Timing instrumentation ─────────────────────────────────────────────────
    struct TimingStat {
        uint64_t max_us = 0;
        uint64_t sum_us = 0;
        uint32_t count  = 0;
        void update(uint64_t dt) { if (dt > max_us) max_us = dt; sum_us += dt; count++; }
        uint64_t mean_us() const { return count ? sum_us / count : 0; }
        void reset() { max_us = sum_us = count = 0; }
    };
    TimingStat _ts_spo2, _ts_hr1, _ts_hr2, _ts_hr3, _ts_cycle;  // Task A fast-path timings
    TimingStat _ts_hr2_compute, _ts_hr3_compute;                 // Task B/C slow-path timings
    // Note: uxTaskGetStackHighWaterMark() returns bytes on ESP32 (portSTACK_TYPE = uint8_t)
    uint32_t   _ts_emit_counter = 0;
    static constexpr uint32_t ts_emit_interval = 2500;  // emit every 5 s at 500 Hz
    void _emit_timing();
    void _emit_tasks();   // emits $TASK frame per FreeRTOS task + $TASKS_END
#endif

    // ── Chip configuration ──
    uint16_t          _sample_rate_hz;
    uint8_t           _num_averages;     // user-visible count (1 = no averaging)
    float             _led1_current_mA;
    float             _led2_current_mA;
    uint8_t           _led_range_mA;     // 75 or 150
    AFE4490TIAGain    _tia_gain;
    AFE4490TIACF      _tia_cf;
    AFE4490Stage2Gain _stage2_gain;

    // ── Signal processing configuration ──
    AFE4490Channel    _ppg_channel;
    AFE4490Filter     _filter_type;

    // ── PPG display filter (Butterworth bandpass or MA, configurable via setFilter()) ──
    BiquadFilter      _ppg_bpf;          // default: 0.5–20 Hz

    // ── Moving average state (PPG display filter — used when _filter_type == MOVING_AVERAGE) ──
    static constexpr int ma_len = 8;
    float    _ma_buf[ma_len];
    int      _ma_idx;
    float    _ma_sum;

    // ── HR1 moving average state (independent of PPG display filter) ──
    static constexpr int hr1_ma_max_len = 64;  // supports up to 640 Hz @ 5 Hz cutoff
    float    _hr1_ma_buf[hr1_ma_max_len];
    uint32_t _hr1_ma_len;   // computed from sample_rate in _recalc_rate_params()
    int      _hr1_ma_idx;
    float    _hr1_ma_sum;

    // ── Rate-dependent algorithm parameters (derived from _sample_rate_hz) ──
    uint32_t          _spo2_warmup_samples;
    uint32_t          _hr1_refractory_samples;
    float             _dc_iir_alpha;
    float             _ac_ema_beta;
    float             _hr1_dc_alpha;

    // ── SpO2 state ──
    float    _dc_ir;
    float    _dc_red;
    float    _ac2_ir;
    float    _ac2_red;
    uint32_t _spo2_sample_count;
    float    _spo2_a;
    float    _spo2_b;

    // ── HR1 state ──
    float    _hr1_dc;
    float    _hr1_running_max;
    bool     _hr1_ppg_above_thresh;
    uint32_t _hr1_last_peak_idx;
    uint32_t _hr1_sample_idx;
    int32_t  _hr1_intervals[5];
    uint8_t  _hr1_interval_count;

    // ── HR2 — autocorrelation-based HR algorithm ──────────────────────────────
    // Bandpass-filters led1_aled1 (0.5–5 Hz), decimates by hr2_decim_factor,
    // accumulates a circular buffer of hr2_buf_len samples, then periodically
    // computes normalised autocorrelation to find the fundamental RR period.
    static constexpr int hr2_buf_len         = 400;  // 8 s at 50 Hz (fs/hr2_decim_factor)
    static constexpr int hr2_acorr_max_lag   = 137;  // guard band lower bound 22 BPM at 50 Hz: 50*60/22 = 137 samples
    static constexpr int hr2_decim_factor    = 10;   // 500 Hz → 50 Hz
    static constexpr int hr2_update_interval = 25;   // recompute every 0.5 s (25 decimated samples)

    BiquadFilter _hr2_bpf;                   // bandpass filter (default 0.5–5 Hz)
    float    _hr2_buf[hr2_buf_len];          // circular buffer of decimated filtered samples
    float    _hr2_seg[hr2_buf_len];          // linearized copy for autocorrelation (avoids stack pressure)
    int      _hr2_buf_idx;                   // next write position in _hr2_buf
    uint32_t _hr2_buf_count;                 // samples written (capped at hr2_buf_len)
    uint32_t _hr2_decim_counter;             // decimation phase counter
    uint32_t _hr2_update_counter;            // decimated samples since last autocorr computation

    // ── HR3 — FFT + Harmonic Product Spectrum HR algorithm ───────────────────
    // Low-pass-filters led1_aled1 (10 Hz anti-aliasing), decimates by hr3_decim_factor,
    // accumulates 512 samples, then every hr3_update_interval decimated samples applies
    // a Hann window, computes the real FFT, and finds the dominant peak via the
    // Harmonic Product Spectrum (fundamental × 2nd harmonic × 3rd harmonic).
    static constexpr int hr3_buf_len         = 512;  // 10.24 s at 50 Hz → freq resolution 0.098 Hz ≈ 5.9 BPM/bin
    static constexpr int hr3_decim_factor    = 10;   // 500 Hz → 50 Hz
    static constexpr int hr3_update_interval = 25;   // recompute every 0.5 s (25 decimated samples)

    BiquadFilter _hr3_lpf;                     // low-pass anti-aliasing filter (default 10 Hz cutoff)
    float    _hr3_buf[hr3_buf_len];            // circular buffer of decimated LP-filtered samples
    float    _hr3_hann[hr3_buf_len];           // precomputed Hann window coefficients (computed once in begin())
    float    _hr3_fft[hr3_buf_len * 2];        // complex FFT buffer (interleaved re/im), also scratch for windowed input
    int      _hr3_buf_idx;                     // next write position in _hr3_buf
    uint32_t _hr3_buf_count;                   // samples written (capped at hr3_buf_len)
    uint32_t _hr3_decim_counter;               // decimation phase counter
    uint32_t _hr3_update_counter;              // decimated samples since last FFT computation

    // ── Output snapshot (written by task, pushed to queue) ──
    AFE4490Data _current_data;

    // ── Static ISR trampoline ──
    // _g_instance holds a pointer to the single active INCUNEST_AFE4490 object so that
    // _drdy_isr_static (a plain C-compatible function required by attachInterrupt)
    // can forward the interrupt to the correct instance.
    //
    // LIMITATION: only one INCUNEST_AFE4490 instance is supported at a time. A second
    // instance would overwrite _g_instance and its DRDY interrupts would be routed
    // to the wrong object. To support two AFE4490 chips, either:
    //   - add a second static ISR + pointer pair, or
    //   - switch to ESP-IDF gpio_isr_handler_add(), which passes a void* argument
    //     per handler, eliminating the need for a singleton pointer altogether.
#ifndef INCUNEST_OFFLINE
    static INCUNEST_AFE4490* _g_instance;
    static void IRAM_ATTR _drdy_isr_static();
#endif

#ifdef UNIT_TEST
public:
    // Expose internals for unit testing only — not part of the public API

    // Biquad filter
    using TestBiquadFilter = BiquadFilter;
    void  test_recalc_biquad(BiquadFilter& f)           { _recalc_biquad(f); }
    float test_biquad_process(float x, BiquadFilter& f) { return _biquad_process(x, f); }

    // HR1
    void  test_feed_hr1(int32_t led1_aled1) { _update_hr1(led1_aled1); }
    float test_hr1()                        { return _current_data.hr1; }
    float test_hr1_sqi()                    { return _current_data.hr1_sqi; }

    // HR2
    void  test_feed_hr2(int32_t led1_aled1) { _update_hr2(led1_aled1); }
    float test_hr2()                        { return _current_data.hr2; }
    float test_hr2_sqi()                    { return _current_data.hr2_sqi; }

    // HR3
    void  test_feed_hr3(int32_t led1_aled1) { _update_hr3(led1_aled1); }
    float test_hr3()                        { return _current_data.hr3; }
    float test_hr3_sqi()                    { return _current_data.hr3_sqi; }

    // SpO2
    void  test_feed_spo2(int32_t ir_corr, int32_t red_corr) { _update_spo2(ir_corr, red_corr); }
    float test_spo2()                       { return _current_data.spo2; }
    float test_spo2_r()                     { return _current_data.spo2_r; }
    float test_spo2_sqi()                   { return _current_data.spo2_sqi; }
#endif
};
