# incunest_afe4490 — Specification v0.98

Medical Open World proprietary library for the AFE4490 chip (PPG/SpO2 pulse oximeter).
Designed for ESP32-S3 with Arduino + FreeRTOS. Phase 2 of the AFE4490 test project.
Coexists with `protocentral-afe4490-arduino` for behavioral comparison.

**Primary hardware reference:** [AFE4490 Datasheet (Texas Instruments)](https://www.ti.com/lit/ds/symlink/afe4490.pdf)

---

## 1. Internal architecture

### 1.1 DRDY management
The AFE4490 DRDY pin is managed **internally** by the library via ISR + FreeRTOS semaphore. The user does not need to handle interrupts.

```
AFE4490 chip (DRDY pin)
    │
    ▼
ISR → xSemaphoreGiveFromISR()
    │
    ▼
afe4490_task (internal)
    ├── reads SPI → 6 raw signals
    ├── processes PPG signal (bandpass filter)
    ├── computes HR1 (peak detection)
    ├── computes HR2 (autocorrelation, runs in parallel with HR1)
    ├── computes HR3 (FFT + HPS, runs in parallel with HR1 and HR2)
    ├── computes SpO2 (AC/DC ratio)
    └── xQueueSend() → FreeRTOS queue
    │
    ▼
sensors_Task (user)
    └── afe.getData(data)  ← reads from queue
```

### 1.2 Internal signals
The AFE4490 produces 6 signals per sample. All are read internally:

| Signal       | Description                              |
|--------------|------------------------------------------|
| LED1VAL      | IR raw                                   |
| LED2VAL      | RED raw                                  |
| ALED1VAL     | Ambient after LED1                       |
| ALED2VAL     | Ambient after LED2                       |
| LED1-ALED1   | IR ambient-corrected ← default           |
| LED2-ALED2   | RED ambient-corrected                    |

### 1.3 Processing chain
```
4 raw ADC codes (led1, led2, aled1, aled2)  +  2 SW-subtracted (led1_sub, led2_sub)
    │
    └─→ _compute_analog_state()  →  AFE4490AnalogState (V_ADC, V_TIA, I_PD, OT)  — see §3c
            ├─→ _rsqm_update()      (probe state, DiagCode, RSQI)
            ├─→ _hgac_update()      (RF guard/leveling, two EMA per domain, §5.8)
            │
            └─→ ot_led1 / ot_led2 (gain-invariant, §5.1) →
                    ├─→ selected channel (setPPGChannel: LED1/LED2) → bandpass filter
                    │       (setPPGDispFilter) → negate → ppg_disp (display only, v0.69, §2.1)
                    ├─→ HR1 (peak detection)
                    ├─→ bandpass 0.5–5 Hz → decimate ×10 → HR2 (autocorrelation)
                    ├─→ LP 10 Hz → decimate ×10 → 512-sample Hann → FFT → HPS → HR3
                    └─→ AC/DC → SpO2
```
**v0.69: `ppg_disp` joined the OT-domain migration** that HR1/HR2/HR3/SpO2 already went through
(§5.1/§5.2) — it was the last consumer of the raw ambient-corrected `led1_sub`/`led2_sub`, and the
only one that still showed a scaling discontinuity whenever HGAC changed RF. See §2.1.

---

### 1.4 Hardware abstraction layer (HAL) — v0.91

The library builds unchanged on three targets: **Arduino-ESP32** (PlatformIO — PulseNest, and
IncuNest motherBoard until its ESP-IDF port merges), **ESP-IDF without Arduino** (CMake component —
motherBoard after the port) and the **host** (PC — `pio test -e native`, `tools/offline_runner`).

**Only what came from `<Arduino.h>` and `<SPI.h>` differs between the two embedded targets.**
Measured on v0.90b: 18 call sites (GPIO ×7, DRDY interrupt ×2, two 4-byte SPI register
transactions, `Serial.printf` ×3, `constrain` ×6) plus seven constants. FreeRTOS (~270 calls),
`esp_timer_get_time()` and `IRAM_ATTR` are the same API on both — Arduino-ESP32 runs on top of
ESP-IDF — so they are used directly and only have to *exist* on the host. `ESP_LOGx` has the same
names on both but not the same implementation (see below).

**Contract — `incunest_afe4490_hal.h`.** Free functions, prefix `incunest_afe4490_hal_`:

| Function | Arduino (`_hal_arduino.cpp`) | ESP-IDF (`_hal_idf.cpp`) | Host (`_hal_host.cpp`) |
|---|---|---|---|
| `gpio_init_output(pin, level)` | `pinMode(OUTPUT)` + `digitalWrite` | `gpio_config` + `gpio_set_level` | no-op |
| `gpio_init_input_pullup(pin)` | `pinMode(INPUT_PULLUP)` | `gpio_config` (pull-up) | no-op |
| `gpio_write(pin, level)` | `digitalWrite` | `gpio_set_level` | no-op |
| `gpio_attach_isr_rising(pin, isr) → bool` | `attachInterrupt(digitalPinToInterrupt(pin), isr, RISING)`, always `true` | `gpio_install_isr_service(0)` once, per-GPIO trampoline table, `gpio_isr_handler_add` | **`false`** |
| `gpio_detach_isr(pin)` | `detachInterrupt` | `gpio_intr_disable` + `gpio_isr_handler_remove` | no-op |
| `spi_attach() → bool` / `spi_detach()` | no-op — the bus is the app's `SPI` | `spi_bus_add_device` (2 MHz, mode 0, `spics_io_num = -1`, queue 1) / remove | `true` / no-op |
| `spi_transfer(cs, tx, rx, n)` | `beginTransaction(2 MHz, MSB, MODE0)`, CS low, **n × `SPI.transfer(uint8_t)`**, CS high, `endTransaction` | `acquire_bus`, CS low, **one `spi_device_polling_transmit`** (TXDATA/RXDATA for n ≤ 4), CS high, `release_bus` | zero-fills `rx` |
| `console_printf(fmt, …)` | `vsnprintf` + `Serial.write` | `vsnprintf` + `fwrite(stdout)` (UART0 via VFS) | `vsnprintf` + `fwrite(stdout)` |
| `console_set_sink(sink)` (v0.92) | stores the pointer; every `console_printf` line is also handed to it, NUL-terminated, on the calling task (the 500 Hz task) — the sink must only queue. PulseNest forwards to UDP; motherBoard sets none | same | same |

Rules the contract encodes:

- **The SPI bus belongs to the application** (`SPI.begin()` under Arduino, `spi_bus_initialize()`
  under ESP-IDF), as it always did; the library only attaches a device and runs its 4-byte frames.
  Clock 2 MHz, MSB first, mode 0 are the AFE4490's and are fixed in the HAL.
  `INCUNEST_AFE4490_HAL_SPI_HOST` (default `SPI2_HOST`) selects the peripheral on ESP-IDF.
- **Arduino path = what v0.90b did, byte for byte.** `spi_transfer` uses the per-byte
  `SPI.transfer()` so the bus sequence is identical; `attach_isr` keeps `digitalPinToInterrupt()`.
- **ESP-IDF path = one driver call per register, not four.** The per-call cost of
  `spi_device_polling_transmit()` is what makes a byte-wise shim 5–10× more expensive than the
  Arduino register-level `SPI.transfer()`; the bench measurement of the acquisition task under
  ESP-IDF is still pending.
- **ISR flags:** `gpio_install_isr_service(0)`, the same as Arduino-ESP32 2.0.x — the handler is
  not IRAM-safe during flash writes (OTA, NVS). Unchanged on purpose; changing it is a measured
  decision, not a port side effect.
- **`console_printf` exists because `$TIMING`/`$TASKS` (§8) are a checksummed line protocol**: the
  line must start with `$` and end with `*XX\r\n`, so it must not go through `ESP_LOGx` (prefix and
  extra newline).
- **The host never pretends.** `attach_isr` returns `false` and the host FreeRTOS creates no
  objects, so `begin()` logs "FreeRTOS object creation failed" and returns; algorithms are
  exercised through the `UNIT_TEST` accessors (§9).

**Selection is by detection, never by a hand-set macro.** Each implementation file is guarded —
`#if defined(ARDUINO)` / `#if defined(ESP_PLATFORM) && !defined(ARDUINO)` / `#if !defined(ESP_PLATFORM)`.
PlatformIO compiles all three (two are empty translation units); the root `CMakeLists.txt`
registers `incunest_afe4490.cpp` + `incunest_afe4490_hal_idf.cpp` with
`REQUIRES esp_driver_gpio esp_driver_spi esp_timer`. `INCUNEST_OFFLINE` no longer exists;
`UNIT_TEST` remains, only to open the test accessors.

**Host stand-ins — `incunest_afe4490_hal_host.h`**, included by the main header when
`ESP_PLATFORM` is not defined: FreeRTOS types and functions, `IRAM_ATTR`, `ESP_LOGx` (to stderr),
`esp_timer_get_time()` (steady clock), all under their real names (§9.4). It replaces the former
`incunest_afe4490_platform_stub.h` and PulseNest's `test/stubs/`, two host paths that had drifted
apart.

**`ESP_LOGx` on Arduino.** Arduino-ESP32 redefines `ESP_LOGx` in `esp32-hal-log.h` (`log_printf`,
`[E][file:line] func():` format, level from `CORE_DEBUG_LEVEL`). `Arduino.h` used to pull it in;
the main header now includes it explicitly under `ARDUINO`, so the logging path is the one the
library always had. Without that line IDF's `esp_log.h` silently takes over on Arduino (other
format, other level control) — found by comparing objects, see below.

**Acceptance evidence, v0.90b → v0.91 (xtensa, PlatformIO's exact command line).** The compiler
output (`-S`, logging disabled so `__LINE__` immediates do not vary) is identical for **102 of 113
functions**; the 11 that differ are exactly the ones touched: `_write_reg`, `_read_spi_raw`,
`_emit_tasks`, `_emit_timing`, `begin`, `stop`, and the five former `constrain()` users
(`setLED1Current`, `setLED2Current`, `setHgacVTiaHigh2`, `_apply_analog_regs`,
`_quantize_led_mA` — same comparisons, the 0/255 constant materialised as float instead of int).
External symbols: the 10 Arduino ones gone, the 8 HAL ones in, nothing else. The assembled
objects differ in more places, all of them assembler decisions (branch relaxation, alignment
padding, literal pool) — which is why the comparison is made on compiler output. Builds verified:
`pio run -e incunest_V17` (Arduino, `-Werror`), `pio test -e native` (76/77 — the failing suite
predates v0.91, §9), ESP-IDF v6.0.1 component for esp32s3 (`-Wall -Wextra -Werror`, GCC 15.2),
host link and run.

## 2. Public API

### 2.1 Data struct
```cpp
struct AFE4490Data {
    // Field order mirrors the $M1/$P1 serial frame: raw signals first, then processed outputs
    // Raw ADC outputs (6 signals from AFE4490)
    int32_t led2;       // LED2VAL  — RED raw           (frame: LED2)
    int32_t led1;       // LED1VAL  — IR raw            (frame: LED1)
    int32_t aled2;      // ALED2VAL — ambient after LED2 (frame: ALED2)
    int32_t aled1;      // ALED1VAL — ambient after LED1 (frame: ALED1)
    int32_t led2_sub;   // LED2-ALED2 — RED ambient-corrected (frame: LED2_SUB)
    int32_t led1_sub;   // LED1-ALED1 — IR ambient-corrected  (frame: LED1_SUB)
    // Processed outputs
    float   ppg_disp;   // PPG display signal, OT domain (v0.69): BPF(ot_led1/ot_led2) + negated.
                        // A/A units, tiny magnitude (~1e-5..1e-6) — NOT normalized (candidate
                        // ppg_disp_norm, PI-weighted [0..1], is backlog). Gain-invariant like the
                        // algorithms below — no discontinuity when HGAC changes RF, unlike the
                        // raw-ADC-code led1_sub/led2_sub it replaced. Display only, no algorithm
                        // consumes it (there is no separate `ppg` field — removed pre-v0.69, HR1
                        // reads ot_led1 directly, see §1.3).
    float   spo2;       // SpO2 in %
    float   spo2_sqi;   // SpO2 Signal Quality Index [0–1]: PI-based; 0=no finger, 1=PI ≥ 2%
    float   spo2_r;     // R ratio: (AC_red/DC_red)/(AC_ir/DC_ir) — for calibration
    float   pi;         // Perfusion Index: (AC_ir / DC_ir) × 100 [%]
    float   hr1;        // HR1 (peak detection) in bpm
    float   hr1_sqi;    // HR1 Signal Quality Index [0–1]: RR interval regularity; 0=arrhythmia/invalid, 1=perfectly regular
    float   hr2;        // HR2 (autocorrelation) in bpm
    float   hr2_sqi;    // HR2 Signal Quality Index [0–1]: normalised autocorrelation at dominant lag; 0=no periodicity, 1=perfect
    float   hr3;        // HR3 (FFT + HPS) in bpm
    float   hr3_sqi;    // HR3 Signal Quality Index [0–1]: spectral concentration at peak; 0=diffuse spectrum, 1=dominant tone
    // RSQM outputs (filled every sample by _rsqm_update)
    uint8_t    rsqi;         // Raw Signal Quality Index: 0=invalid, 1=valid
    uint32_t   diag_code;    // DiagCode bitmask: RSQM_DIAG_AMB_SAT | RSQM_DIAG_SWITCHED_RC_SETTLING | ...
    ProbeState probe_state;  // enum ProbeState (0-4); isProbeAbsent() answers "no patient"
};
```

### 2.2 Initialization
```cpp
void begin(int pin_cs, int pin_drdy, bool debug = false);
```
Initializes SPI, configures the chip with default values, attaches ISR to DRDY, and starts the internal FreeRTOS task.

**`debug` parameter:** when `true`, the internal queue stores `DebugQueueItem {AFE4490Data + AFE4490DebugData}` per slot instead of `AFE4490Data` alone. This enables `getData(data, &dbg)` to return both atomically from the same queue item (same sample, no skew). When `false` (default), queue items are `sizeof(AFE4490Data)` — zero overhead for production use (IncuNest).

**Default values after begin():**

| Parameter      | Default value                  |
|----------------|-------------------------------|
| Sample rate    | 500 Hz                        |
| LED1 current   | ~49.8 mA (code 85/256 × 150 mA) — preliminary dev-phase default (v0.47) |
| LED2 current   | ~49.8 mA (code 85/256 × 150 mA) — preliminary dev-phase default (v0.47) |
| LED range      | 150 mA (TX_REF = 0.75V)       |
| TIA gain (RF)  | 100 kΩ                        |
| TIA CF         | auto (see §7.2): 100 pF @ 500 Hz / RF_100K |
| Stage 2 gain   | disabled                      |
| ENSEPGAIN      | 0 (single gain)               |
| NUMAV          | 7 (8 averages, max 9 at 500Hz)|
| PPG channel    | LED1 (ot_led1)                |
| Filter         | BUTTERWORTH, 0.5–20 Hz        |

### 2.3 Chip configuration
```cpp
void setSampleRate(uint16_t hz);        // valid range: 63–5000 Hz. Updates PRF and recalculates NUMAV_max.
void setAdcAverages(uint8_t num);       // num = number of samples to average: 1 (no averaging) .. NUMAV_max+1
                                        // NUMAV_max = floor(5000 / PRF) - 1  (e.g.: PRF=500Hz → max=9 → up to 10 samples)
                                        // Absolute hardware limit: 16 samples (NUMAV=15)
                                        // If num > NUMAV_max for current PRF, clamped and logE emitted
void setLED1Current(float mA);          // stored as actual DAC-quantised value; depends on setLEDRange()
void setLED2Current(float mA);          // stored as actual DAC-quantised value; depends on setLEDRange()
void setLEDRange(uint8_t mA);           // 75 or 150 mA (with TX_REF=0.75V default); re-quantises both LED currents
void setTIAGain(AFE4490RF gain);   // auto-recalculates CF (see §7.2); setTIACF() can override
void setTIACF(float pF);                // manual override, quantised DOWN to the largest CF ≤ pF;
                                        // overridden again if setTIAGain()/setSampleRate() is called
void setStage2Gain(AFE4490RG gain);      // sets gain for both channels; does NOT touch STAGE2EN1/2
void setStage2GainLED1(AFE4490RG gain);  // STG2GAIN1 only; does NOT touch STAGE2EN1
void setStage2GainLED2(AFE4490RG gain);  // STG2GAIN2 only; does NOT touch STAGE2EN2
void setStage2En1(bool en);                      // STAGE2EN1 (D14 of TIAGAIN) — independent of gain
void setStage2En2(bool en);                      // STAGE2EN2 (D14 of TIA_AMB_GAIN) — independent of gain; also auto-set when AMBDAC > 0
void setAmbDac(uint8_t uA);             // Ambient cancellation DAC current [0–10 µA]; AMBDAC[3:0] in TIA_AMB_GAIN D19:D16
```

### 2.3b Configuration readback

`AFE4490Config getConfig()` returns a snapshot of all current configuration parameters.

```cpp
struct AFE4490Config {
    uint16_t          afe_sample_rate_hz;    // ADC sample rate [Hz]
    uint8_t           afe_adc_averages;      // ADC hardware averages (1 = no averaging)
    float             afe_led1_current_mA;   // IR LED drive current [mA] — actual DAC-quantised value
    float             afe_led2_current_mA;   // RED LED drive current [mA] — actual DAC-quantised value
    uint8_t           afe_led_range_mA;      // LED full-scale range: 75 or 150 mA
    AFE4490RF    afe_tia_rf_led1;      // TIA RF for LED1 (IR) — TIAGAIN; active when ENSEPGAIN=1
    AFE4490CFCode     afe_tia_cf_led1_code;  // CF_LED1[4:0] code (0–31) — TIAGAIN; active when ENSEPGAIN=1
    float             afe_tia_cf_led1_pF;    // same setting in pF (= kAFE_CF_PF[code])
    AFE4490RG afe_stg2_rg_led1;  // STG2GAIN1 for LED1 — gain bits only, no enable
    bool              afe_stg2_en_led1;        // STAGE2EN1 (D14 of TIAGAIN) — explicit enable
    AFE4490RF    afe_tia_rf_led2;     // TIA RF for LED2 (RED) — TIA_AMB_GAIN; always active
    AFE4490CFCode     afe_tia_cf_led2_code;  // CF_LED2[4:0] code (0–31) — TIA_AMB_GAIN; always active
    float             afe_tia_cf_led2_pF;    // same setting in pF (= kAFE_CF_PF[code])
    AFE4490RG afe_stg2_rg_led2;  // STG2GAIN2 for LED2 — gain bits only, no enable
    bool              afe_stg2_en_led2;        // STAGE2EN2 (D14 of TIA_AMB_GAIN) — explicit enable; also forced ON when AMBDAC > 0
    uint8_t           afe_ambdac_uA;         // Ambient cancellation DAC current [0–10 µA]; AMBDAC[3:0] in TIA_AMB_GAIN D19:D16
    AFE4490Channel    ppgdisp_channel;       // PPG display channel selection
    float             ppgdisp_f_low_hz;      // PPG display bandpass lower cutoff [Hz]
    float             ppgdisp_f_high_hz;     // PPG display bandpass upper cutoff [Hz]
    float             hr2_f_low_hz;          // HR2 bandpass lower cutoff [Hz]
    float             hr2_f_high_hz;         // HR2 bandpass upper cutoff [Hz]
    float             hr3_f_high_hz;         // HR3 bandpass upper cutoff [Hz]
    // R → SpO2 curve: SpO2 = spo2_r_curve_a − spo2_r_curve_b · R (v0.98; was spo2_a / spo2_b)
    char              spo2_r_method_id[SPO2_R_ID_LEN];          // how this library computes R (SPO2_R_METHOD_ID)
    char              spo2_r_curve_id[SPO2_R_ID_LEN];           // the R curve in use
    char              spo2_r_curve_r_method_id[SPO2_R_ID_LEN];  // the R method that curve was fitted with
    bool              spo2_r_curve_method_match;               // false: curve fitted for another R method (warning only)
    float             spo2_r_curve_a;
    float             spo2_r_curve_b;
    // SpO2 algorithm parameters
    float             spo2_warmup_s;             // warmup before reporting SpO2 [s]
    float             spo2_ema_mean_tau_s;         // DC removal IIR time constant [s]
    float             spo2_ema_var_tau_s;         // AC² EMA time constant [s]
    float             spo2_min;                  // valid output lower bound [%]
    float             spo2_max;                  // valid output upper bound [%]
    float             spo2_min_dc;               // no-finger DC threshold [adc_code]
    float             spo2_pi_sqi_lo;            // PI below this → SQI=0 [%]
    float             spo2_pi_sqi_hi;            // PI above this → SQI=1 [%]
    // HR1 algorithm parameters
    float             hr1_dc_tau_s;              // DC removal IIR time constant [s]
    float             hr1_ma_cutoff_hz;          // peak detection LP MA cutoff [Hz]
    float             hr1_sqi_cv_max;            // CV above this → SQI=0 [dimensionless]
    // HR2 algorithm parameters
    float             hr2_min_corr;              // min normalised autocorrelation for SQI=1
    uint32_t          hr2_update_interval;       // autocorrelation recompute period [decimated samples]
    // HR3 algorithm parameters
    uint32_t          hr3_update_interval;       // FFT+HPS recompute period [decimated samples]
    // HR valid range (HR1 + HR2 + HR3)
    float             hr_min_bpm;                // valid output lower bound [bpm]
    float             hr_max_bpm;                // valid output upper bound [bpm]
    // RSQM algorithm parameters (runtime-configurable via setRsqm*() setters)
    float             rsqm_ot_thr;               // OT threshold: NOT_APPLIED vs APPLIED [A/A]
    float             rsqm_disconn_led_sub_thr;  // |led_sub| min for DISCONNECTED detection [adc_code]
    float             rsqm_disconn_i_pd_thr;     // |i_pd| min for DISCONNECTED detection [A]
    float             rsqm_probe_state_min_s;    // debounce time to change probe state [s]
    float             rsqm_ema_mean_tau_s;        // RSQM EMA mean time constant [s]
    float             rsqm_ema_var_tau_s;         // RSQM EMA variance time constant [s]
};
AFE4490Config getConfig();
```

**Thread safety:** takes `_spi_mutex` (for hardware fields) and `_state_mutex` (for signal/filter fields) if `_initialized == true`. Safe before and after `begin()`.

**Use case:** allows the host application (e.g. `pulsenest_lab.py`) to read back the active chip configuration for display, logging, or verification.

### 2.4 Signal and filter configuration
```cpp
void setPPGChannel(AFE4490Channel channel);
void setPPGDispFilter(float f_low_hz = 0.5f, float f_high_hz = 20.0f);
// HR2 bandpass filter cutoffs (default 0.5–5 Hz); callable before or after begin()
void setHR2Filter(float f_low_hz = 0.5f, float f_high_hz = 5.0f);
// HR3 bandpass filter cutoffs (default 0.4–15 Hz); callable before or after begin()
void setHR3Filter(float f_high_hz = 15.0f);   // pure low-pass by design — no lower corner
```

### 2.5 Algorithm parameter setters

All setters callable before or after `begin()`. Setters marked *recalc* internally call `_recalc_rate_params()` when invoked after `begin()`.

```cpp
// SpO2
void setSpO2WarmupS(float s);                              // warmup before reporting SpO2 [s] — recalc
void setSpO2EmaMeanTauS(float tau_s);                        // DC IIR time constant [s] — recalc
void setSpO2EmaVarTauS(float tau_s);                        // AC² EMA time constant [s] — recalc
void setSpO2Range(float min_pct, float max_pct);           // valid SpO2 output range [%]
void setSpO2MinDC(float counts);                           // no-finger DC threshold [adc_code]
void setSpO2PiSqiThresholds(float lo_pct, float hi_pct);  // PI-based SQI thresholds [%]
bool setSpO2RCurve(const char* id, const char* r_method_id, float a, float b);  // R curve + the R method it was fitted with (v0.98; replaces setSpO2Coefficients)

// HR1
void setHR1DcTauS(float tau_s);                            // DC removal IIR time constant [s] — recalc
void setHR1MaCutoffHz(float hz);                           // peak detection LP MA cutoff [Hz] — recalc
void setHR1SqiCvMax(float cv);                             // SQI CV threshold [dimensionless]

// HR2
void setHR2MinCorr(float corr);                            // min autocorrelation for SQI=1
void setHR2UpdateInterval(uint32_t decimated_samples);     // autocorrelation recompute period

// HR3
void setHR3UpdateInterval(uint32_t decimated_samples);     // FFT+HPS recompute period

// HR1 + HR2 + HR3
void setHRValidRange(float min_bpm, float max_bpm);        // valid HR output range [bpm]

// RSQM — all callable before or after begin()
void setRsqmOtThr(float v);                     // OT threshold: NOT_APPLIED vs APPLIED [A/A]
void setRsqmDisconnLedSubThr(float v);          // |led_sub| min for DISCONNECTED detection [adc_code]
void setRsqmDisconnIPdThr(float v);             // |i_pd| min for DISCONNECTED detection [A]
void setRsqmProbeStateMinS(float s);            // debounce time [s] — recalc
void setRsqmEmaTau(float mean_tau_s, float var_tau_s); // RSQM EMA time constants [s] — recalc
```

### 2.6 Data retrieval
```cpp
bool getData(AFE4490Data& data, AFE4490DebugData* dbg = nullptr);
// Returns true and fills data if an item is available in the queue.
// Returns false immediately if the queue is empty (non-blocking).
// dbg: if non-null and begin(debug=true) was used, filled atomically with the debug
//      snapshot for the same sample. Ignored when debug mode is inactive.
```

The library samples internally at the configured rate (default 500 Hz) regardless of how often `getData()` is called.

The internal queue holds up to 10 samples (`INCUNEST_AFE4490_QUEUE_SIZE`). When full, the oldest sample is automatically discarded to make room for the newest.

| Consumption rate | Behaviour |
|---|---|
| > 500 Hz | Most calls return `false`. No data available yet. |
| ≈ 500 Hz | Ideal. Queue stays empty or holds 1 item. Samples consumed in order, no discards, negligible latency. |
| Occasional jitter | Queue buffers up to 10 samples without discards. Consumer drains them in subsequent calls. |
| Consistently < 500 Hz | Queue stays full in steady state. Samples returned in FIFO order with a **fixed latency of 10 × T_sample = 20 ms**. Intermediate samples are discarded: e.g. at 100 Hz, 1 in every 5 samples is received. |

> The internal algorithms (HR1, HR2, HR3, SpO2) process every sample at 500 Hz regardless of the consumption rate. The call frequency of `getData()` does not affect calculation quality — only how many result samples the user receives and with what latency.

### 2.6b Hardware diagnostics
```cpp
uint32_t runAfeDiagnostics(uint32_t diag_holdoff_ms = kAFE_DIAG_HOLDOFF_MS);
```
Runs the AFE4490 hardware self-test sequence (datasheet section 8.4.3.3, Table 3).

**Parameters:**
- `diag_holdoff_ms` — duration (ms) of the post-diagnostic holdoff window. Default: `kAFE_DIAG_HOLDOFF_MS`, derived from `kAFE_FLTRCNRSEL`: 500 Hz corner → 28 ms; 1000 Hz corner → 16 ms (datasheet p.11, filter settling time after diagnostics mode). Set to 0 to observe raw artefacts from the re-settling analog front-end.

**Sequence:**
1. Sets `_diag_active = true` — signals `_task_body()` to skip ADC reads and repeat the last valid output.
2. Acquires `_spi_mutex` briefly; writes `CONTROL0 = 0x000004` (DIAG_EN=1, SPI_READ=0); releases `_spi_mutex`.
3. Waits 10 ms (chip requires 8 ms in fast mode; +2 ms margin). `_task_body()` consumes each DRDY sem, skips the SPI read, and pushes the last valid `_current_data` to the queue to preserve temporal continuity.
4. Acquires `_spi_mutex`; sets `CONTROL0 = ctrl0_spi_read` (SPI_READ=1); reads `REG_DIAG` (0x30); restores `CONTROL0 = 0x000000`; releases `_spi_mutex`.
5. Sets `_diag_holdoff_samples = diag_holdoff_ms × sample_rate / 1000`, then sets `_diag_active = false`. The holdoff counter is written before the flag is cleared to prevent a race where `_task_body()` could slip through with a real SPI read between the two assignments.

**Post-diagnostic holdoff (`_diag_holdoff_samples > 0`):**
After DIAG_EN is cleared the analog front-end (TIA, stage 2) needs a few PRP cycles to re-settle. During holdoff, `_task_body()` skips the SPI read and instead calls `_process_sample()` with the frozen last-valid raw ADC values (`_last_valid_led1/2`, `_last_valid_aled1/2`, `_last_valid_led1/2_sub`). A constant input is absorbed by all IIR/BPF/LP filters with near-zero AC contribution, making it far less disruptive to HR1/HR2/HR3/SpO2 than a transient outlier. The raw values are saved on every normal `_task_body()` cycle and are only accessed from that same task — no mutex is needed.

**Same mechanism now also covers HGAC RF-change settling (v0.72, found by Alex from a visible
spike in `ppg_disp`):** `_task_body()`'s freeze condition is `_diag_active || _diag_holdoff_samples
> 0 || _switched_rc_settling_countdown > 0` — the third term reuses the identical frozen-input path for
the settling window that opens after `_hgac_change_rf()` (§5.8.4), the "same physical phenomenon,
two different triggers" as the diagnostic holdoff. Before this, `RSQM_DIAG_SWITCHED_RC_SETTLING` was a label
with no enforcement: it was set in `diag_code`, but nothing actually stopped SpO2/HR1/HR2/HR3/
`ppg_disp` from processing the real (transient, not-yet-settled) values during that window — only
HGAC's own gate checked the bit, to pause its own actuation. `_switched_rc_settling_countdown` is
decremented inside `_process_sample()` (`_rsqm_update()`) unconditionally, whether fed frozen or
fresh inputs, so no extra bookkeeping was needed in `_task_body()` beyond the OR term itself.

**`_diag_active` flag — design rationale:**
`_task_body()` writes `CONTROL0=0` after every ADC burst read. Without the flag, it would clear DIAG_EN mid-sequence and abort the diagnostic. The flag avoids holding `_spi_mutex` for the full 10 ms sleep (which is bad FreeRTOS practice and causes priority inversion). Instead, `_task_body()` checks `_diag_active` after `xSemaphoreTake(_drdy_sem)` and, when set, repeats the last valid `_current_data` into the queue (under `_state_mutex`) before looping. Repeating the last sample rather than dropping it preserves temporal equi-spacing for the autocorrelation (HR2) and FFT (HR3) algorithms and reduces IIR filter discontinuity at resumption. The chip physically cannot produce valid ADC data during diagnostics; the ~10 ms window (≈5 samples at 500 Hz) is unavoidable but transparent to the consumer.

**Return value:** raw 24-bit `DIAG` register. Returns `0` if not initialized.

| Bit | Name            | Meaning when set (1)            |
|-----|-----------------|----------------------------------|
| 0   | PD_ALM          | Photodiode out of range          |
| 1   | LED_ALM         | LED out of range                 |
| 2   | DIAG_OUT        | General diagnostic fault         |
| 3   | LED2_ALM        | LED2 driver fault                |
| 4   | LED3_ALM        | LED3 driver fault (if used)      |
| 5   | LED1_ALM        | LED1 driver fault                |
| 6   | PDOC_ALM        | Photodiode open-circuit alarm    |
| 7   | PDSC_ALM        | Photodiode short-circuit alarm   |
| 8   | LED2OC_ALM      | LED2 open-circuit alarm          |
| 9   | LED2SC_ALM      | LED2 short-circuit alarm         |
| 10  | LED1OC_ALM      | LED1 open-circuit alarm          |
| 11  | LED1SC_ALM      | LED1 short-circuit alarm         |
| 12  | COMMON_MODE_ALM | Common-mode voltage out of range |

**Callers:**
- On demand from the host via `$DIAG?` serial command (one-shot).
- Internally by `_diag_task_body()` — see §5.7 below. Both callers are independent; `_spi_mutex` serialises concurrent access.

---

### 2.7 Shutdown
```cpp
void stop();
```
Cleanly shuts down the library:
1. Detaches the DRDY interrupt.
2. Waits for any in-progress SPI transaction to finish (acquires `_cfg_mutex`).
3. Deletes the internal FreeRTOS task.
4. Deletes the FreeRTOS queue, semaphore, and mutex.
5. Resets all algorithm state (SpO2, HR, filter).
6. Sets `_initialized = false`.

After `stop()`, `begin()` can be called again to restart from a clean state. Configuration (sample rate, LED current, filter, etc.) is preserved across `stop()`/`begin()` cycles.

**Typical use:** hot-swap between incunest_afe4490 and protocentral at runtime without recompiling.

---

## 3. Enumerations

```cpp
// PPG display source channel (setPPGChannel() / ppg_disp, §2.1). v0.69: OT domain — only IR/RED
// remain selectable. The four single-ended raw options this enum used to have (bare LED1/LED2/
// ALED1/ALED2, without ambient subtraction — and, before that, LED1_ALED1/LED2_ALED2 for the
// ambient-corrected pair) had no OT equivalent: OT is structurally an ambient-corrected ratio
// (AFE4490AnalogState has no ot_aled1/ot_aled2), so they were dropped rather than kept
// meaningless. Named LED1/LED2 (not IR/RED) for consistency with ot_led1/ot_led2.
enum class AFE4490Channel {
    LED1,  // ot_led1 (IR)  — default
    LED2   // ot_led2 (RED)
};


enum class AFE4490RF {
    RF_10K,
    RF_25K,
    RF_50K,
    RF_100K,   // default
    RF_250K,
    RF_500K,
    RF_1M
};

// TIA feedback capacitance — NOT an enum: the CF_LED[4:0] register field selects five
// switchable capacitors in parallel over a fixed 5 pF base (datasheet Figure 114, p.36),
// so all 32 combinations are legal and the register code IS the value.
//   bit weights: b0=5, b1=15, b2=25, b3=50, b4=150 pF
//   CF(code) = 5 pF + Σ selected weights  → 5, 10, 20, 25, … 245, 250 pF
//   datasheet worked example: code 01111 = 50+25+15+5 +5 = 100 pF
// The weights are superincreasing, so capacitance is strictly increasing in the code
// (static_assert via afeCFTableIsMonotonic()).
using AFE4490CFCode = uint8_t;                      // valid range 0…31
constexpr AFE4490CFCode kAFE_CF_CODE_MAX     = 31;
constexpr AFE4490CFCode kAFE_CF_CODE_DEFAULT = 0;   // 5 pF — chip reset value

enum class AFE4490RG {
    RG_100K,    // 0 dB   — ×1   — RG=100 kΩ  (Ri=100 kΩ fixed, datasheet Table 1)
    RG_150K,  // 3.5 dB — ×1.5 — RG=150 kΩ
    RG_200K,    // 6 dB   — ×2   — RG=200 kΩ
    RG_300K,  // 9.5 dB — ×3   — RG=300 kΩ
    RG_400K    // 12 dB  — ×4   — RG=400 kΩ
};

// Wire encoding ((int)probe_state in $M2/$M3/$M4 frames and CSVs) — values are explicit and
// stable. v0.90 split PROBE_OT_HIGH / PROBE_ONLY_LED_SATURATING into its two physical causes: PROBE_OT_HIGH keeps value 1
// (it was the usual cause in historical captures, so old CSVs still read correctly) and the
// LED-only saturation case gets 4. PROBE_AMB_SATURATING was renamed PROBE_AMB_SATURATING (value 3
// unchanged): the old name suggested "any saturation", but LED-only saturation never came here.
// When adding a state: classify it in isProbeAbsent() below, and update the name tables in
// pulsenest_lab.py (OT MONITOR, SIGNAL STATS, AFE SWEEP TEST).
enum class ProbeState {
    PROBE_DISCONNECTED        = 0,  // cable or probe connector not plugged in
    PROBE_OT_HIGH             = 1,  // not saturated, and BOTH channels' OT > rsqm_ot_thr (v0.89 AND):
                                    // too much light reaches the PD for tissue to be in the path.
    PROBE_APPLIED             = 2,  // probe on patient, normal measurement
    PROBE_AMB_SATURATING      = 3,  // positive saturation where the AMBIENT (ALED) phase also clips —
                                    // and therefore the LED phase too (LED phase = ambient + LED
                                    // light). The saturating light is EXTERNAL (lamp/phototherapy).
                                    // Presence UNKNOWN. SpO2/HR1/HR2/HR3/RSQI: non-APPLIED (reset).
                                    // HGAC: acts, like APPLIED, to clear it (§5.8.2).
    PROBE_ONLY_LED_SATURATING = 4   // positive saturation where ONLY the LED phase clips, ambient
                                    // clean: the LED's own light reaches the PD with nothing
                                    // attenuating it. Read as probe-in-air (finger removed) since
                                    // v0.61 — but thin tissue at high RF gives the same signature,
                                    // and OT is not consulted under saturation, so that case lands
                                    // here too (§5.6.2, open check). Treated as ABSENT everywhere
                                    // via isProbeAbsent() — no behavioural change from v0.89.
};

// The one place that defines "no patient on the probe". Consumers that must separate the idle
// states from APPLIED / AMB_SATURATING — the HGAC gate, the diagnostics task — call this instead
// of listing states, so a new ProbeState is classified here, once, deliberately. A state missing
// from this list counts as PRESENT: HGAC would act on it and diagnostics would not run.
inline bool isProbeAbsent(ProbeState s) {
    return s == ProbeState::PROBE_DISCONNECTED
        || s == ProbeState::PROBE_OT_HIGH
        || s == ProbeState::PROBE_ONLY_LED_SATURATING;
}

// DiagCode bitmask flags (returned in AFE4490Data::diag_code — uint32_t, 32 bits)
//
// Bits 0–12: AFE4490 hardware DIAG register — set by runAfeDiagnostics()
constexpr uint32_t AFE_DIAG_PD_ALM          = 0x00000001UL;  // bit  0 — photodiode out of range
constexpr uint32_t AFE_DIAG_LED_ALM         = 0x00000002UL;  // bit  1 — LED out of range
constexpr uint32_t AFE_DIAG_OUT             = 0x00000004UL;  // bit  2 — general diagnostic fault
constexpr uint32_t AFE_DIAG_LED2_ALM        = 0x00000008UL;  // bit  3 — LED2 driver fault
constexpr uint32_t AFE_DIAG_LED3_ALM        = 0x00000010UL;  // bit  4 — LED3 driver fault
constexpr uint32_t AFE_DIAG_LED1_ALM        = 0x00000020UL;  // bit  5 — LED1 driver fault
constexpr uint32_t AFE_DIAG_PDOC_ALM        = 0x00000040UL;  // bit  6 — photodiode open-circuit
constexpr uint32_t AFE_DIAG_PDSC_ALM        = 0x00000080UL;  // bit  7 — photodiode short-circuit
constexpr uint32_t AFE_DIAG_LED2OC_ALM      = 0x00000100UL;  // bit  8 — LED2 open-circuit
constexpr uint32_t AFE_DIAG_LED2SC_ALM      = 0x00000200UL;  // bit  9 — LED2 short-circuit
constexpr uint32_t AFE_DIAG_LED1OC_ALM      = 0x00000400UL;  // bit 10 — LED1 open-circuit
constexpr uint32_t AFE_DIAG_LED1SC_ALM      = 0x00000800UL;  // bit 11 — LED1 short-circuit
constexpr uint32_t AFE_DIAG_COMMON_MODE_ALM = 0x00001000UL;  // bit 12 — common-mode voltage out of range
constexpr uint32_t AFE_DIAG_MASK            = 0x00001FFFUL;  // mask for all 13 AFE DIAG bits
//
// Bits 13+: RSQM/HGAC software checks — updated continuously at 500 Hz
// Bit 14 = RSQM_DIAG_AMBIENT_HIGH (HGAC ambient alarm, v0.59; previously SIGNAL_WEAK, removed v0.56).
constexpr uint32_t RSQM_DIAG_AMB_SAT      = 0x00002000UL;  // bit 13 — ambient channel near positive rail
constexpr uint32_t RSQM_DIAG_AMBIENT_HIGH = 0x00004000UL;  // bit 14 — ambient saturates at RF floor (HGAC)
constexpr uint32_t RSQM_DIAG_SWITCHED_RC_SETTLING = 0x00008000UL;  // bit 15 — hardware settling after config change.
// v0.72: enforced, not just labelled — _task_body() freezes inputs (last-valid raw values) for
// the whole window this bit is set, same mechanism as the diagnostic holdoff (§2.6b/§5.6.3).
// bits 16–31 reserved
```

---

## 3b. Physical LUT arrays and string codecs

Declared `public` in `incunest_afe4490.h`. These are the **single source of truth** for all physical values; neither application code nor external tools should maintain local copies.

### Physical LUT arrays (constexpr)

Indexed by casting the enum to `int`. Example: `float rf = kAFE_RF_OHM[(int)cfg.afe_tia_rf_led1];`

```cpp
// TIA feedback resistor — indexed by AFE4490RF (0=RF_10K … 6=RF_1M)
constexpr float kAFE_RF_OHM[7] = { 10e3f, 25e3f, 50e3f, 100e3f, 250e3f, 500e3f, 1e6f };

// TIA feedback capacitor — indexed by the raw CF_LED[4:0] code (0…31), value = 5 + Σ weights.
// Strictly increasing (superincreasing bit weights), which afeCFPFToCode() and auto-CF rely on.
constexpr float kAFE_CF_PF[32] = {
      5,  10,  20,  25,  30,  35,  45,  50,   // 00000…00111
     55,  60,  70,  75,  80,  85,  95, 100,   // 01000…01111  (0x0F = 100 pF, datasheet example)
    155, 160, 170, 175, 180, 185, 195, 200,   // 10000…10111
    205, 210, 220, 225, 230, 235, 245, 250    // 11000…11111
};
// Same index → display string, used by the $CFG/$SET protocol.
constexpr const char* kAFE_CF_STR[32] = { "5p", "10p", …, "245p", "250p" };

// Relative tolerance for matching a requested CF against the grid. The auto-CF budget
// (settle_s / 5 / RF) × 1e12 lands a few ULP BELOW the exact grid value in float — at
// 500 Hz / RF_100K it yields 99.9999924 instead of 100.0, which would drop a whole step.
// 0.01% is far below the on-chip capacitor tolerance, so it cannot cause a real violation.
constexpr float kAFE_CF_MATCH_TOL = 1e-4f;

// Stage 2 feedback resistor RG — indexed by AFE4490RG (0=RG_100K … 4=RG_400K)
constexpr float kAFE_RG_OHM[5] = { 100e3f, 150e3f, 200e3f, 300e3f, 400e3f };

// Stage 2 linear gain (= RG/Ri) — same indexing as above
constexpr float kAFE_RG_GAIN[5] = { 1.0f, 1.5f, 2.0f, 3.0f, 4.0f };

// Stage 2 input resistor Ri — fixed by chip design (datasheet p.28)
constexpr float kAFE_RI_OHM = 100e3f;

// ── ADC domain (namespace adc) — SINGLE SOURCE OF TRUTH ─────────────────────
// Signed 22-bit twos-complement ADC (datasheet p.11: ±1.2 V FS; p.43 §8.4.1).
// NOMINAL = datasheet; ACTUAL = measured on 16.A (rails ~230 codes short of FS).
namespace adc {
    constexpr float   FSR     = 1.2f;            // V — full-scale range (±1.2 V differential)
    constexpr float   FS_CODE = 2097152.0f;      // 2^21 — full-scale divisor
    constexpr float   SCALE   = FSR / FS_CODE;   // V per code (1 LSB): V_ADC = code × SCALE
    constexpr int32_t POS_RAIL_NOMINAL =  2097151;   // +2^21−1 (max positive code)
    constexpr int32_t NEG_RAIL_NOMINAL = -2097152;   // −2^21   (min negative code)
    constexpr int32_t POS_RAIL_ACTUAL  =  2096921;   // measured (constant, all 4 channels)
    constexpr int32_t NEG_RAIL_ACTUAL  = -2096919;   // measured (LED1/2; ALED averaged → approx)
    constexpr int32_t SAT_GUARD = 221;                          // codes (≈2× worst ALED-to-rail distance)
    constexpr int32_t SAT_POS   = POS_RAIL_ACTUAL - SAT_GUARD;  // 2096700
    constexpr int32_t SAT_NEG   = -SAT_POS;                     // −2096700 (symmetric by choice; NEG_RAIL_ACTUAL doc-only)
    static_assert(POS_RAIL_ACTUAL < POS_RAIL_NOMINAL && NEG_RAIL_ACTUAL > NEG_RAIL_NOMINAL &&
                  SAT_POS < POS_RAIL_ACTUAL && SAT_NEG > NEG_RAIL_ACTUAL,
                  "ADC actual rails inside nominal FS; guarded sat inside the actual rail");
}

// Low-pass filter corner frequency (FLTRCNRSEL bit D15 of TIA_AMB_GAIN, datasheet p.75).
// 0 = 500 Hz (reset default); 1 = 1000 Hz.
// Written unconditionally to TIA_AMB_GAIN in _build_tia_amb_gain_led2().
constexpr uint8_t  kAFE_FLTRCNRSEL      = 0;
// Diagnostic holdoff: filter settling time after diagnostics mode (datasheet p.11).
// 500 Hz corner → 28 ms; 1000 Hz corner → 16 ms.
constexpr uint32_t kAFE_DIAG_HOLDOFF_MS = kAFE_FLTRCNRSEL ? 16U : 28U;
```

Register-code arrays (`rf_code[]`, `stg2_gain_code[]`) remain in an anonymous namespace in `incunest_afe4490.cpp` — internal only, never exposed. CF has no such array: the code is the value, written as `(code & 0x1F) << 3` into bits D[7:3].

### CF conversion helpers (inline)

```cpp
inline float         afeCFCodeToPF(AFE4490CFCode code);  // → kAFE_CF_PF[code & 31]
inline AFE4490CFCode afeCFPFToCode(float pF);            // largest achievable CF ≤ pF
```

`afeCFPFToCode()` quantises **DOWN**, never up (within `kAFE_CF_MATCH_TOL`): overshooting CF would stretch the TIA time constant past the settling budget reserved before the ADC sample window (5τ ≤ settle_time, §7.2). Requests below 5 pF clamp to code 0 — the base capacitor cannot be switched out. This is the same rule the public setters and auto-CF both use, so manual and automatic selection can never disagree.

### String codecs (inline)

Convert between enum values and the label strings used in the `$CFG`/`$SET` protocol.

```cpp
// TIA feedback resistor RF
inline const char* afeRFToStr(AFE4490RF g);          // → "10K"/"25K"/"50K"/"100K"/"250K"/"500K"/"1M"
inline bool        afeStrToRF(const char* s, AFE4490RF& out);  // returns false if unrecognised

// TIA feedback capacitor CF
inline const char* afeCFToStr(AFE4490CFCode code);         // → "5p" … "250p" (32 values), "?" if out of range
inline bool        afeStrToCF(const char* s, AFE4490CFCode& out);
// The six strings used before v0.62 ("5p","10p","20p","30p","55p","155p") remain valid
// and map to the same hardware setting — no protocol break.

// Stage 2 gain
inline const char* afeRGToStr(AFE4490RG g);      // → "0dB"/"3.5dB"/"6dB"/"9.5dB"/"12dB"
inline bool        afeStrToRG(const char* s, AFE4490RG& out);
```

---

## 3c. AFE4490AnalogState — analog reconstruction struct

Declared `public` in `incunest_afe4490.h`. Computed once per sample by `_compute_analog_state()` at the top of `_process_sample()`, before RSQM and HGAC. **Not stored in `AFE4490Data`** — ephemeral, passed by `const&`.

### Formulas (datasheet Eq.2, p.30)

Datasheet forward equation: `V_DIFF = 2 × (I_PD × RF/Ri − I_CANCEL) × RG`

Inverted step by step:

```
V_ADC      = code × adc::SCALE                             [V]   (= code / adc::FS_CODE × adc::FSR; datasheet p.11: ±1.2 V FS; p.43: 22-bit twos complement)
V_TIA = 2 × (V_ADC / (2 × RG) + I_CANCEL) × Ri             [V]   (= 2 × I_PD × RF, differential TIA output)
I_PD       = V_TIA / (2 × RF)                              [A]   (photodiode current)
OT         = (I_PD_LED − I_PD_ALED) / I_LED                     [—]   (electro-optical system ratio: LED efficiency × tissue transmittance × PD responsivity)
```

**`adc::FS_CODE = 2^21` (2097152), not 2^21−1 (corrected v0.51, 2026-08-03):** the ADC is a
22-bit bipolar twos-complement converter, so 1 LSB = `adc::FSR / 2^21` and the code range is
`[−2^21, +2^21−1]` — **asymmetric**. Table 7 (p.88) confirms the LSB via rows `code=+1 ↔ +1.2/2^21 V`,
`code=−1 ↔ −1.2/2^21 V`, `code=0 ↔ 0 V`, and the negative rail `−1.2 V ↔ −2^21` (exact). The table's
last row `+1.2 V ↔ +2^21−1` is mutually **incompatible** with those (it implies LSB = 1.2/(2^21−1)):
it is a nominal label — the max positive code +2^21−1 actually represents `+1.2 V − 1 LSB`. The scale
is anchored on the consistent four rows (denominator 2^21), not on that last row. Numerical impact of
the former 2^21−1: ~0.5 ppm — negligible, but the value now matches the datasheet transfer function.

**V_TIA convention (v0.51, 2026-08-03):** the library stores the **differential** TIA output voltage, matching the datasheet numbers literally (§9.2.2 + Fig. 135: "Ideal Operating Point" = 0.6 V, "TIA max (Differential)" = 1.0 V — the datasheet has no symbol named "V_OD"; a prior session's citation of "V_OD(fs)" was corrected 2026-07-21 after rendering the actual PDF). All identifiers use the bare `v_tia` / `V_TIA` name with no `_diff` suffix: the code has no single-ended or per-branch value (each branch would be `v_tia / 2`), so there is nothing to disambiguate from. The earlier "naming rule 4b" that forbade bare `v_tia` and mandated a `_diff`/`_branch` suffix was removed in v0.51. Empirical limits (IncuNest 16.A, `tia_linearity_sweep.py` 2026-07-08): linear up to ~1.8 V diff, hard clip ~1.94 V diff (TIA output node hits the rail, pre-ambient-subtraction).

### Circuit parameters per channel

| Symbol | Source | Notes |
|---|---|---|
| `RF` | `kAFE_RF_OHM[(int)_afe_tia_rf_led*]` | TIA feedback resistor (TIAGAIN reg, datasheet p.27) |
| `RG` | `kAFE_RG_OHM[(int)_afe_stg2_rg_led*]` | Stage 2 feedback resistor (TIAGAIN reg, datasheet p.30) |
| `Ri` | `kAFE_RI_OHM` = 100 kΩ | Fixed by chip design (datasheet p.28) |
| `I_CANCEL` | `_afe_ambdac_uA × 1e-6` | Ambient DAC (TIA_AMB_GAIN bits[19:16], 1 µA/LSB, datasheet p.27) |
| `I_LED` | `_afe_led*_current_mA × 1e-3` | LED drive current (LEDCNTRL reg, datasheet p.28) |

When `STAGE2EN = 0` (bits[14] of TIAGAIN / TIA_AMB_GAIN): Stage 2 bypassed → unity gain → `RG = Ri = 100 kΩ`.

ALED1/ALED2 channels share the same RF and RG as LED1/LED2 respectively (same TIA path, only timing differs).

### Struct definition (v0.35)

```cpp
enum class AFE4490Ch : uint8_t { LED1 = 0, ALED1 = 1, LED2 = 2, ALED2 = 3 };  // value = mask bit index

enum class AFE4490ChState : uint8_t {
    CH_VALID_RANGE    = 0,  // trustworthy measurement
    CH_EXTENDED_RANGE = 1,  // v_tia in (tia_axis::FS_V, tia_axis::LIN_V] = (1.0, 1.8] V: TIA past its ±1 V TI
                      // guarantee but empirically linear. TIA property — observability ADC-gated (see below)
    CH_CLIPPED_RANGE  = 2,  // ADC saturated or TIA beyond empirical linearity: value is a BOUND, not a measurement
};

struct AFE4490AnalogState {
    // Detection thresholds live in namespaces, not here: v_tia in tia_axis (FS_V/LIN_V),
    // ADC saturation rails in adc (SAT_POS/SAT_NEG). Both single sources of truth, no aliases.

    // Values — ALWAYS computed (value-always policy), even under saturation
    float v_adc_led1,      v_adc_led2,      v_adc_aled1,      v_adc_aled2;       // V
    float v_tia_led1, v_tia_led2, v_tia_aled1, v_tia_aled2;  // V  (= 2 × I_PD × RF)
    float i_pd_led1,       i_pd_led2,       i_pd_aled1,       i_pd_aled2;        // A
    float ot_led1,         ot_led2;                                              // A/A — (i_pd_led − i_pd_aled)/ILEDx_A

    // Validity masks — bit = channel (1 << (uint8_t)AFE4490Ch)
    uint8_t adc_sat_pos  = 0;  // code ≥ adc::SAT_POS
    uint8_t adc_sat_neg  = 0;  // code ≤ adc::SAT_NEG
    uint8_t tia_over_fs  = 0;  // v_tia > tia_axis::FS_V (no lower bound: photocurrent unipolar)
    uint8_t tia_over_lin = 0;  // v_tia > tia_axis::LIN_V

    // Helpers — mandatory access path (mask packing may change without touching consumers)
    bool adcSatPos/adcSatNeg/adcSat(AFE4490Ch) const;
    bool tiaOverFs/tiaOverLin(AFE4490Ch) const;
    AFE4490ChState chState(AFE4490Ch) const;   // precedence: CLIPPED > OFF_SPEC > CLEAN
    bool chValidRange(AFE4490Ch) const;             // == CH_VALID_RANGE
    bool otLed1Valid/otLed2Valid() const;      // strict: both source channels CH_VALID_RANGE
    bool anyAdcSat() const;                    // HGAC O1
    bool anyTiaOverFs() const;                 // HGAC O2
};
```

**Invalidity policy (value-always, v0.35):** every field is always computed from the raw codes — no NaN, no sentinels. The validity masks tell the consumer whether each value is a *measurement* (`CH_VALID_RANGE`), a probably-correct value outside TI guarantee (`CH_EXTENDED_RANGE`, empirically linear band 1.0–1.8 V diff), or a *bound* (`CH_CLIPPED_RANGE`: ADC sat pos → real ≥ stored; sat neg → real ≤ stored; TIA over lin → TIA compresses). Rationale: an OFF_SPEC value is still the best estimate; a clipped value is an actionable bound for HGAC; and traces remain plottable. Consumers must use the helpers, never the raw masks — packing is per-stage today (option A) but may become per-channel (option B).

**Thresholds — single source of truth (`namespace tia_axis` for v_tia, `namespace adc` for ADC; v0.48, aliases removed v0.51, ADC constants consolidated into `adc`):** the four ordered `v_tia` thresholds live once in `tia_axis` — `IDEAL_V` 0.6 / `GUARD_V` 0.9 / `FS_V` 1.0 / `LIN_V` 1.8 — with a `static_assert` enforcing strict ascending order, including `adc::FSR` (1.2 V, the ADC full-scale mapped onto the v_tia axis at unity gain) between `FS_V` and `LIN_V`. All consumers reference `tia_axis::` / `adc::` directly (no per-struct aliases): the `tia_over_fs`/`tia_over_lin` masks compare against `tia_axis::FS_V`/`LIN_V`, the `adc_sat_pos`/`adc_sat_neg` masks against `adc::SAT_POS`/`SAT_NEG`, and HGAC policy (`hgac_v_tia_thr`/`hgac_v_tia_ideal`, runtime-tunable `AFE4490Config` fields) takes its defaults from `tia_axis`, with `setHgacVTiaThr()` clamping to ≤ `FS_V`. The ADC full-scale (`adc::FSR`) is nominal — the operative saturation flag is `adc::SAT_POS`, an empirical rail in adc_code (`POS_RAIL_ACTUAL − SAT_GUARD`), not derived from the voltage axis.

**Plausible physical derivation of `LIN_V` = 1.8 V (Alex, 2026-08-21) — previously purely empirical.**
`LIN_V` was found on the bench (`tia_linearity_sweep.py`, 2026-07-08: linear to ~1.8 V diff, hard
clip ~1.94 V diff) with no theoretical basis recorded anywhere in this spec. Two datasheet facts,
each independently documented but never connected before, predict it:

1. **§8.3.4 (Fig. 65, "Receiver Subsystem Power Routing"):** `RX_ANA_SUP` is regulated internally
   to **1.8 V** before feeding the receive analog blocks (TIA, Stage 2, ambient DAC, filter) — the
   real analog rail for this whole chain is 1.8 V, not the external supply (2.0–3.6 V).
2. **§8.3.1.1 + Electrical Characteristics (p.11):** the TIA's common-mode is **0.9 V, "set
   internally"** — exactly half of that 1.8 V rail, the natural bias point for a fully-differential
   stage to maximise symmetric headroom. §8.3.1.1 states the ambient DAC's purpose is precisely to
   *recentre* the DC-biased signal to this ±0.9 V mid-point — i.e. common-mode positioning doesn't
   appear in the differential reading directly, but it sets how much room is left before either
   single-ended node hits its own rail.

For a symmetric fully-differential output biased at 0.9 V on a 0–1.8 V rail, the two single-ended
nodes move in opposite directions from that centre; the differential swing reaches its maximum
exactly when one node hits 1.8 V while the other hits 0 V — a differential of **1.8 − 0 = 1.8 V**.
That matches `LIN_V` exactly. The TI-*guaranteed* `FS_V` = 1.0 V (`V_OD(fs)`, p.11) sits well inside
this 1.8 V rail-limited maximum, consistent with TI specifying guaranteed performance with
guardband below the physical ceiling. The small gap between this 1.8 V prediction and the
*measured* hard clip (~1.94 V) is expected, not a contradiction: real output stages rarely
saturate at the exact rail (dropout in the output transistors, regulator tolerance), so a little
headroom past the theoretical 1.8 V before true hard clipping is normal.

Not verified against a schematic or TI application engineer — recorded as the most plausible
explanation found for a number that was, until now, unexplained.

**`CH_EXTENDED_RANGE` observability is ADC-gated (not the full (1.0, 1.8] band):** `tia_axis::FS_V` (1.0 V) and `tia_axis::LIN_V` (1.8 V) are *TIA* properties — the silicon's extended-linearity band — independent of the downstream chain. What is actually *observable* as `CH_EXTENDED_RANGE` depends on Stage 2 gain and AMBDAC, because `chState()` gives `adcSat` precedence (`CLIPPED > OFF_SPEC`) and the ADC full-scale is only ±1.2 V:

| Config | Relation | ADC saturates at `v_tia` = | `CH_EXTENDED_RANGE` observable band |
|---|---|---|---|
| **Default** (RG = Ri, AMBDAC = 0) | `V_ADC = V_TIA` | 1.2 V | (1.0, **1.2**] V |
| Stage 2 ×2 (6 dB) | `V_ADC = 2·V_TIA` | 0.6 V | **∅** (ADC saturates below 1.0 V) |
| AMBDAC = *k* µA (RG = Ri) | `V_ADC = V_TIA − 0.2k` | 1.2 + 0.2k V | extends up; **k ≥ 3 µA** reaches 1.8 V |

With the default configuration `tia_over_lin` (`v_tia > 1.8 V`) can never fire — the reconstructed value is capped at ~1.2 V by ADC saturation, which triggers `CH_CLIPPED_RANGE` via `adcSat` first. The (1.2, 1.8] tail is only reachable by using AMBDAC to shift the ADC window upward (operational use of AMBDAC in the 1.2–1.8 V TIA zone is the deferred "AMBDAC off-spec-zone" task, layer v2).

**ADC rail characterization (IncuNest 16.A, 2026-07-10, via AMBDAC over-cancellation):** the empirical rails are NOT the datasheet code limits (±2^21). Positive rail = **+2096921**, constant and identical on all 4 channels. Negative rail = **−2096919**, constant on LED1/LED2; ALED1/ALED2 do not pin — their published codes oscillate down to ~−2096810 (approximate interval, limited observation time), because ADC averaging mixes clipped and non-clipped conversions within the window. Detection thresholds therefore carry a **guard band of ~220 counts** (2× the worst observed ALED-to-rail distance of 109 counts) inside the rail, applied to both polarities since the averaging mechanism is sign-independent. A legitimate code within 220 counts of the rail (0.01% of range) is functionally saturated anyway, so false positives are not a practical concern. Note: an exact comparison against the assumed symmetric rail −2096921 would never have triggered (`adc_sat_neg` dead flag) — found empirically.

**OT (optical transmittance)** is stored in the struct since v0.35 (was previously computed inside `_rsqm_update()`). Formula: `ot_ledx = (i_pd_ledx − i_pd_aledx) / ILEDx_A` [A/A]. Value-always; validity via `otLedxValid()` (both source channels must be `CH_VALID_RANGE` — OT is a ratio of two measurements). The former `rsqm_ot_saturated = 100.0` sentinel and `rsqm_adc_positive_sat` constant are removed.

### Consumers

| Consumer | Fields used |
|---|---|
| `_rsqm_update()` | `ot_led1/ot_led2` + `otLedxValid()` (probe state), `adcSatPos(ALEDx)` (RSQM_DIAG_AMB_SAT), `i_pd_*` (disconnect) |
| `_hgac_update()` (§5.8, Phase 1) | `v_tia_led1/led2/aled1/aled2` (guard trip, per gain domain) |

---

## 3d. AFE4490DebugData — debug/test snapshot struct

Declared `public` in `incunest_afe4490.h`. Aggregates internal signals not published via `AFE4490Data`. Not in the production data path — not in the queue unless `begin(debug=true)` was called.

```cpp
struct AFE4490DebugData {
    AFE4490AnalogState analog;  // V_ADC, V_TIA, I_PD for all 4 channels
    AFE4490RF rf_led1;          // current RF, IR domain — set via $SET or moved by HGAC (§5.8)
    AFE4490RF rf_led2;          // current RF, RED domain
};
```

**Naming (v0.81):** `rf_led1`/`rf_led2` — no `hgac_` prefix. It's the live config value
(mirrors `_afe_tia_rf_led1`/`_afe_tia_rf_led2` exactly), regardless of who last changed it —
a manual `$SET` or HGAC's own descent. The field was originally named `hgac_rf_led1/led2`
(v0.37), which wrongly implied HGAC always computed it.

**Access:** `getData(data, &dbg)` — only valid when `begin(debug=true)`. When debug is inactive, `dbg` is not filled.

**Queue mechanism:** when `debug=true`, the internal queue stores `DebugQueueItem {AFE4490Data + AFE4490DebugData}` per slot. A single `xQueueReceive` fills both `data` and `dbg` atomically — they always correspond to the same sample. When `debug=false`, queue slots are `sizeof(AFE4490Data)` only — zero RAM/CPU overhead.

**Scalability:** future debug signals (e.g. RSQM intermediate values) are added as fields of `AFE4490DebugData`. The `getData()` API does not change.

---

## 4. Compile-time parameters

Overridable in `incunest_afe4490.h` before compilation:

```cpp
#define INCUNEST_AFE4490_QUEUE_SIZE      10   // number of samples in the FreeRTOS queue
#define INCUNEST_AFE4490_TASK_PRIORITY   5    // internal task priority
```

**Full queue behaviour:** oldest item is discarded to always keep the most recent sample.

---

## 5. Algorithms

### 5.1 SpO2

**EXPERIMENT (branch `experiment/ot-domain-inputs`, since v0.38): input domain changed from
raw ambient-corrected counts to OT.** `_spo2_update()` now consumes `as.ot_led1`/`as.ot_led2`
(optical transmittance, §3c) instead of `led1_sub`/`led2_sub`. Rationale: `OT = (i_pd_led −
i_pd_aled) / I_LED` is a linear rescaling of `led_sub` by `1/(2·RF·I_LED)` — since the real
photocurrent does not change instantaneously when RF/ILED change, OT (and `i_pd`) is
**gain-invariant by construction**. An RF change (HGAC, §5.8) no longer creates a
discontinuity in SpO2's input, so an RF change no longer needs to rescale the SpO2
`EmaChannel` by k (`EmaChannel::rescale()` deleted as dead code) — R is
never corrupted by a gain step, without any reactive rescale mechanism.
This also decouples HGAC's actuation rate from any future change to how PI/SpO2 are
estimated (see "future alternative" notes below) — a rescale-by-k trick only works for a
linear EMA; a more sophisticated estimator might not have one, which would otherwise force
HGAC to wait out a settling window before actuating again.

**Presence detection: delegated entirely to RSQM (`ProbeState`), not computed by SpO2.**
This migration tried two self-contained "no-finger"/"no-signal" mechanisms inside
`_spo2_update()` in turn — `spo2_min_i_pd_a` (Amps, on absolute `i_pd_ir`/`i_pd_red`
magnitude, v0.38) and then `spo2_min_ot_dc` (A/A, on `dc_ir`/`dc_red`, v0.40) — both removed.
For a transmittance probe, "no finger" typically shows **high** OT (light passes largely
unobstructed), not low `i_pd`/DC — so both gates rarely caught an actual no-finger condition;
what they did catch overlapped with RSQM's own `PROBE_DISCONNECTED`/`PROBE_OT_HIGH`
classification, just with a narrower/uncalibrated criterion and no debounce. Having SpO2
(and, in principle, HR1/HR2/HR3) each maintain their own copy of "is there a probe/finger"
detection duplicates RSQM's one job across N places with N different, uncoordinated
thresholds — architecturally wrong even before considering whether any single one of them
works correctly.

**v0.41: `_spo2_update(float ot_ir, float ot_red, ProbeState probe_state)`** — a third
parameter, RSQM's classification, consumed but never computed here. While
`probe_state != PROBE_APPLIED` (covers `PROBE_OT_HIGH`, `PROBE_ONLY_LED_SATURATING`,
`PROBE_DISCONNECTED` and `PROBE_AMB_SATURATING` identically — same treatment RSQI already gives them): both EMAs are reset **every sample**
(idempotent, not just once on the transition edge — needs no stored "previous probe_state",
keeping the function's only internal memory as `_spo2_ch_ir_ema`/`_spo2_ch_red_ema`, nothing
else reads or writes them), and `pi`/`spo2`/`spo2_r` are forced to `NaN`. Warmup restarts
naturally once `PROBE_APPLIED` resumes, via `ir_ema.count` (see below) starting from 0.

What remains inside `_spo2_update()` is **only** a division-safety guard,
`spo2_div_eps = 1e-9` — purely numerical (not physiological), applied to the actual divisors:
`dc_ir` (divisor in the PI calc) and `dc_red`/`ac_ir` (divisors in R; `ac_red` is a
numerator-only multiplicand, doesn't need guarding). A single shared epsilon is safe here
specifically *because* presence detection moved to `probe_state` — the earlier attempt to
also use this same kind of threshold as a physiological floor (`spo2_min_ot_dc`, `1e-6`) broke
4/7 native `test_spo2` cases once tried alongside AC (AC legitimately gets very small at low
PI, e.g. PI=0.5% on typical DC gives AC≈7e-8 — far below a physiologically-motivated floor,
but nowhere near the purely numerical `1e-9`).

**Output contract, now uniform:** `sqi==0.0f` always implies `pi==spo2==spo2_r==NaN` — across
warmup, `probe_state != PROBE_APPLIED`, and the division guard alike. `NaN` replaces the
earlier `-1.0f` sentinel (used only for `pi`'s near-zero-mean case, and only for `spo2` at the
wire-formatting layer in `main.cpp` — `spo2_r`/`pi` were never masked there at all, so a stale
value would previously have traveled over `$M3`/`$M4` unmasked). `NaN` fails safe: any
consumer that forgets to check `sqi` and instead compares or averages the value gets a
comparison that's always false (e.g. a naive `spo2 < 70` alarm never fires on `NaN`) or a
propagated `NaN` — not a plausible-looking, alarm-triggering number the way `-1.0f` could be.

`_spo2_sample_count` removed — `EmaChannel::count` (already incremented every `.update()`
call, in lockstep with the old separate counter) is used directly for the warmup check.

**Precision note (analyzed, not a concern):** OT magnitudes (~1e-5) are nowhere near
`float`'s precision floor — `float`'s relative precision (~1.19e-7) is ~100× finer than the
ADC's own resolution reflected in OT units (~1e-5 relative, i.e. the same information content
as the raw ADC codes, since OT is a linear transform). The `i_pd_led − i_pd_aled` subtraction
does not suffer meaningful catastrophic cancellation at the magnitudes involved in this
system (error many orders of magnitude below the ~2% physiological AC modulation).

**Known limitation (not fixed by OT):** ILED changes still require gating SpO2 invalid during
settling — ambient current does not scale with ILED (asymmetric residual, tracked separately,
see the gain-influence analysis referenced from §5.8) — OT's gain-invariance covers RF
changes, not the ILED/ambient-residual problem.

**pulsenest_lab.py impact — migrated (2026-07-15, lab v1.19; updated for v0.41 in lab
v1.23):** the four algorithm-mirror classes (`SpO2TestCalc`, `HR1TestCalc`, `HR2TestCalc`,
`HR3TestCalc`) now consume OT too, requiring frame mode `$M4` (`OT_LED1`/`OT_LED2` are not
transmitted in `$M1`/`$M2`/`$M3`) — tracked in `pulsenest_lab_spec.md` (v1.19/v1.23
changelog), not reconstructed locally from `led1_sub` + RF/ILED config. `SpO2TestCalc` mirrors
the v0.41 design: `update(ot_ir, ot_red, probe_state, fs)`, `ProbeState` consumed from the
already-parsed `ProbeState` column, `spo2_div_eps` (single numerical guard), `NaN` sentinel.

---

Based on the ratio R of the AC and DC components of the ambient-corrected signals:

```
R = (AC_RED / DC_RED) / (AC_IR / DC_IR)

SpO2 = a - b × R
```

`a` and `b` are the empirical **R curve**, set with `setSpO2RCurve()` together with the curve's id and
the R method it was fitted with — see *R methods and R curves* at the end of this section.

**DC and AC² estimation via EmaChannel:**

Both DC and AC power are extracted from a single `EmaChannel` per signal (IR and RED), using two independent time constants:

- **`spo2_ema_mean_tau_s` = 2.0 s** — tracks the DC baseline (slow IIR low-pass, f_c ≈ 0.08 Hz). Must satisfy f_c << 0.5 Hz (minimum physiological HR at 30 bpm) so the mean does not follow the pulsatile component. Same value as `rsqm_ema_mean_tau_s` — both track the DC of the same signal with the same physical requirement.
- **`spo2_ema_var_tau_s` = 6.0 s** — tracks the AC² power (EMA of `(x − mean)²`). Lower end of the 6–10 s averaging window that **ISO 80601-2-61:2026 Annex JJ.2 d)** (informative) sets for an SpO2 *transfer standard*; the standard prescribes no window for the device itself, only its disclosure (201.7.9.2.1.101 c). It sits between the 2–4 s used in neonatal resuscitation and intermittent-hypoxaemia research and the 8 s NICU default (Masimo). A shorter τ gives a noisier AC estimate; a longer one under-reads short neonatal desaturations (Ahmed 2010, Vagedes 2013/2014). Full evidence base in the design rationale §4.5.
- **`spo2_warmup_s` = 18.0 s** — SpO2 output suppressed until 3 × τ_var = 3 × 6 s, time for the EMA variance estimator to converge.

The two τ values are intentionally different because DC and AC² have distinct temporal requirements: DC must track slow baseline drift (τ = 2 s), while AC² must be stable enough for a reliable SpO2 estimate (τ = 6 s, see above).

**First-sample seeding (v0.55):** `EmaChannel::update()` seeds on the first sample after
construction or `reset()` — `count == 0` → `mean = x`, `var = 0` — instead of ramping `mean` up
from 0. Ramping from 0 leaves `d = x − mean ≈ x` for the first ~3·τ, which spuriously **inflates
`var`** (hence PI) during warm-up. Seeding removes that
start-up bias. It does **not** shorten `spo2_warmup_s` (= 3·τ_var): `var`
still needs several pulse cycles to become representative — seeding only fixes the *bias*, not the
*convergence time*. Steady-state behaviour is unchanged (same recursion, same α).

**Optional warmup / `valid()` (v0.58):** `init()` takes an optional `warmup_s` (default 0). When
> 0, `valid()` returns false until `warmup_s · fs` samples have been seen since the last `reset()`;
`mean`/`var` remain plain public fields (always real values — no NaN), so the filter keeps
accumulating during warmup and consumers that don't care (SpO2, `warmup_s = 0` → always valid)
are unaffected. HGAC uses it (3·τ per EMA) to skip decisions until an estimator has matured after
a gain-change reset — the check `ema.valid() && ema.mean >= thr` short-circuits with no action.

**Perfusion Index (PI):**

```
PI = sqrt(EmaChannel.var) / EmaChannel.mean × 100  [%]
   = RMS_AC / DC × 100
```

`spo2_sqi` is a continuous [0–1] quality metric based on PI:

```
sqi = clamp((PI − 0.5) / (2.0 − 0.5), 0, 1)
```

PI < 0.5 % → SQI = 0 (absent or very weak signal, no finger contact). PI ≥ 2.0 % → SQI = 1 (full quality). Thresholds from Nellcor/Masimo clinical reference. If SpO2 is outside the valid range [70–100 %] or the DC level is below the no-finger threshold, SQI is forced to 0.

#### R methods and R curves (v0.98)

Two different things decide the SpO2 number, and each has its own identifier:

| | What it names | Format | Where it lives |
|---|---|---|---|
| **R method** | *How* R is computed from the signals: input domain, DC and AC estimators and their τ, filtering before them | `R-METHOD-<n>` | `SPO2_R_METHOD_ID` (header constant), `AFE4490Config::spo2_r_method_id` |
| **R curve** | The curve that turns R into SpO2 (SpO2 = a − b·R), with its provenance | `R-CURVE-<probe>-<source>-<date>` | `setSpO2RCurve()`, `AFE4490Config::spo2_r_curve_*` |

**Rules.**
1. `SPO2_R_METHOD_ID` changes **if and only if**, for the same input, R comes out as a different
   number. A change that leaves R alone (HR3's filter in v0.97, for instance) does not change it;
   the library version is therefore not a substitute for it.
2. An R curve is only valid for the R method it was fitted with, so the coefficients never travel
   alone: `setSpO2RCurve(id, r_method_id, a, b)` takes both identifiers, and rejects (keeping the
   current curve whole) an empty or oversized id (`SPO2_R_ID_LEN`, terminator included) or a
   non-finite coefficient.
3. **Mismatch = warning, not invalidation (v0.98).** A curve fitted for another R method is still
   applied; `getConfig().spo2_r_curve_method_match` is `false` and a warning is logged at `begin()`
   and at every `setSpO2RCurve()`. It is deliberately **not** a `diag_code` bit: `rsqi` is 1 only
   while `diag_code == 0`, so a static configuration warning there would zero `rsqi` for good and
   invalidate the signal — the opposite of a warning. To be made strict once a curve fitted with
   the current R method exists.

**R methods.**

| Id | Definition | In force |
|---|---|---|
| `R-METHOD-0` | Inputs: ambient-corrected `adc_code` (`led1_sub` / `led2_sub`). DC = IIR low-pass, τ = 1.6 s. AC = √(EMA of (x − DC)²), τ = 1.0 s. R = (AC_red/DC_red)/(AC_ir/DC_ir). | The library of 2026-03-23, when the default curve was fitted. Reconstructed from the conversation log (session 26 and the SpO2LocalCalc mirror of that day), not from a tagged version. |
| `R-METHOD-1` | Inputs: OT (`ot_led1` / `ot_led2`, v0.38). DC = `EmaChannel` mean, τ = `spo2_ema_mean_tau_s` = 2.0 s. AC = √(`EmaChannel` var of x − DC), τ = `spo2_ema_var_tau_s` = 6.0 s (both since 2026-06-11). Same ratio. | Current (named in v0.98). |

The OT input domain alone would not move R (each channel's AC/DC is invariant to its own scale),
but the τ changes do, slightly; with no way to rebuild R-METHOD-0 exactly, the two are kept apart.
An `$SET`-style runtime change of `spo2_ema_mean_tau_s` / `spo2_ema_var_tau_s` makes the running
method "R-METHOD-1 with modified parameters"; those parameters travel in the capture's `alg:`
snapshot. First offline comparison of candidate methods (band-pass + RMS, regression on samples and
on derivatives): PulseNest `captures/sessions/20260923_HOSPNAV_SUBJ08_ANALYSIS/r_regression_test.py`.

**R curves known to the library.**

| Id | a | b | Fitted with | Provenance |
|---|---|---|---|---|
| `R-CURVE-U401D-MS100-20260323` (**default**) | 114.9208 | 30.5547 | `R-METHOD-0` | CONTEC MS100 simulator, setting "R-Curve Nellcor, 100 bpm", probe UpnMed U401-D(01AS-F) (Nellcor Non-Oximax type, IR 905 nm). A simulator does not validate accuracy (ISO 80601-2-61 EE.4). |

Because the default was fitted with `R-METHOD-0`, **the default configuration raises the
mismatch warning by itself**: that is the honest state of the library today, not a fault. The
registry of all curves, with their data and fitting method, lives in PulseNest
(`docs/spo2_r_curves.md`, planned).

On the wire (PulseNest fw ≥ 0.16), `$CFG` carries `spo2_r_method_id`, `spo2_r_curve_id`,
`spo2_r_curve_r_method_id`, `spo2_r_curve_method_match`, `spo2_r_curve_a`, `spo2_r_curve_b`
(fw ≤ 0.15 printed only `spo2a`/`spo2b`).

### 5.2 HR1 — Peak detection

**EXPERIMENT (branch `experiment/ot-domain-inputs`, since v0.38): input changed to OT.**
`_hr1_update()` now takes `as.ot_led1` (float) instead of `led1_sub` (int32_t raw
ambient-corrected counts) — same rationale as SpO2 (§5.1): gain-invariant by construction.
Unlike SpO2, HR1 is peak-**timing** based, not an amplitude ratio — `_hr1_running_max` and
the `0.6×running_max` threshold are self-relative, so a uniform gain scale does not change
peak timing and no threshold recalibration was needed (pure domain/type change).

**v0.42: `_hr1_update(float ir, ProbeState probe_state)`** — mirrors the SpO2 v0.41 design
(§5.1): presence detection stays RSQM's exclusive job, `probe_state` is consumed but never
computed here. While `probe_state != PROBE_APPLIED`, all internal state (DC filter, running
max, threshold-crossing flag, peak/interval indices, RR interval buffer, moving-average
buffer) resets **every sample** (idempotent, no stored "previous probe_state" needed), and
`hr1`/`hr1_sqi` are forced to `NaN`/`0`. Once `PROBE_APPLIED` resumes, HR1 naturally needs 5
fresh RR intervals before reporting again, same as any cold start.

**Processing chain:**
1. **DC removal:** IIR low-pass filter (τ = 1.6 s) estimates DC; subtracted from raw signal. Signal is negated for conventional PPG polarity (peaks up).
2. **Low-pass filter:** moving average with cutoff ~5 Hz (`len = fs / (2 × 5)`; at 500 Hz → 50 samples). Capped at 64 samples max.
3. **Running maximum:** amplitude reference, decayed exponentially with time constant
   `hr1_max_decay_tau_s` (**1.5 s** since v0.93; per-sample factor `exp(-1/(tau × fs))`, derived
   from the rate so it means the same at every PRF). It is the RECOVERY parameter of the detector:
   after a step drop in amplitude to a fraction *g* of the previous peak, detection resumes only
   after `tau × ln(hr1_threshold_fraction / g)` — 1.0 s at tau = 1.5 s for a halved beat, against
   13.9 s at the former 20 s. Too long blinds the detector after any amplitude change; too short
   lets the threshold collapse between beats and the dicrotic notch is counted. See §10.4 for the
   measurements behind 1.5 s.
4. **Threshold crossing:** rising edge detected when signal crosses `0.6 × running_max`. A
   **refractory period** — a blind window after each accepted beat, during which further
   crossings are ignored, named after the cardiac refractory period — prevents double-detection,
   i.e. counting the dicrotic notch as a second beat and reporting twice the true rate.
   0.185 s allows up to ~324 BPM.
5. **RR interval buffer:** last 5 consecutive intervals stored. HR1 computed as average:

```
HR1 (bpm) = fs × 60 / mean(last 5 RR intervals in samples)
```

`hr1_sqi` is a continuous [0–1] quality metric based on the coefficient of variation (CV) of the 5 most recent RR intervals:

```
CV  = std(RR[0..4]) / mean(RR[0..4])
SQI = clamp(1 − CV / 0.15, 0, 1)
```

CV = 0 (perfectly regular rhythm) → SQI = 1. CV ≥ 15 % → SQI = 0 (arrhythmia, artefact, or loss of signal). The 15 % threshold is the standard clinical criterion for rhythm regularity. If fewer than 5 intervals have been detected, or if HR1 is outside [40, 260] BPM, SQI is forced to 0. The refractory period of 185 ms naturally supports detection up to ~324 BPM, providing a guard band above the 260 BPM limit at no cost.

> **Implementation note:** peak location is currently approximated by the rising threshold crossing (first sample above `0.6 × running_max`). This introduces a timing error that depends on signal slope and amplitude — if either changes (low perfusion, motion), the detected instant shifts relative to the true peak, introducing jitter in the RR interval. Planned improvement: apply derivative to the filtered signal and detect the rising edge as the maximum of the derivative (steepest ascending slope), which gives a more stable and precise timing reference independent of signal amplitude.

**Diagnostic field `hr1_ppg`:** after each detected peak, `hr1_ppg` is forced to 0 for 10 samples. This produces a visible marker in the serial stream / plotter that survives serial downsampling.

**Key constants:**

| Constant | Value | Meaning |
|---|---|---|
| `hr1_dc_tau_s` | 1.6 s | IIR DC removal time constant |
| `hr1_ma_cutoff_hz` | 5 Hz | Moving average low-pass cutoff |
| `hr1_ma_max_len` | 64 samples | Max moving average length |
| `hr1_max_decay_tau_s` | 1.5 s | Running-max decay time constant. **Measured, v0.93** (was 20 s, the inherited `0.9999` per-sample literal). Governs recovery after an amplitude change, not detection margin — see §5.2 step 3 and §10.4 |
| `hr1_refractory_s` | 0.185 s | Refractory period: blind window after a beat, so the dicrotic notch is not counted twice (~324 BPM practical limit). A fixed value cannot protect bradycardia — see the rationale §5.3 |
| `hr1_threshold_fraction` | 0.6 | Detection threshold as a fraction of the decaying running max. **The single parameter that most determines whether a beat is counted.** Lower → more sensitive, risks counting the dicrotic notch; higher → misses weak beats. A bare literal until v0.88 |
| `hr1_peak_marker_samples` | 10 samples | Duration of peak marker (hr1_ppg=0) |
| `hr_min_bpm` | 40 BPM | Reported valid HR range minimum. Engineering choice, no normative source; WHO-UNICEF 2019 §6.5 asks for 30 (lowering pending) — see the note below |
| `hr_max_bpm` | 260 BPM | Reported valid HR range maximum. Engineering choice; covers WHO-UNICEF's 240 with room for neonatal SVT (source pending) |
| `hr_search_min_bpm` | 37 BPM | Internal search lower bound (guard band: hr_min − 3) |
| `hr_search_max_bpm` | 263 BPM | Internal search upper bound (guard band: hr_max + 3) |

> **Guard band design:** all HR algorithms search internally in [37, 263] BPM and force `hr*_sqi = 0.0` when the result falls outside [40, 260] BPM. This prevents boundary-clipping errors where a signal at exactly 40 BPM could be missed due to small algorithmic offsets. The ±3 BPM margin matches the ±3 bpm pulse-rate accuracy of WHO-UNICEF 2019 §6.5.

> **Where [40, 260] comes from:** nowhere normative. The history is 40–240 → 30–250 (v0.9) → 25–300 (v0.11) →
> 40–260 (2026-05-02 for the top, v0.25 for the bottom), each change a project decision with no recorded source.
> ISO 80601-2-61 §201.12 does not set a pulse-rate range: it requires the manufacturer to disclose "the displayed
> ranges of pulse rate". The citable requirement is WHO-UNICEF 2019, *Technical specifications and guidance for
> oxygen therapy devices*, §6.5 (tabletop, continuous NICU monitoring) and Annex 1 Table A1.11: pulse-rate detection
> range to include **30–240 bpm**, resolution ≤ 1 bpm, accuracy ±3 bpm. 260 meets the top; 40 does not meet the
> bottom, and lowering it to 30 is pending (HR2 band-pass corner sits at exactly 30 BPM, HR3 nears the drift bins,
> HR1 needs 10 s for 5 intervals, and neonatal breathing at 30–60 rpm overlaps the band).

### 5.3 HR2 — Autocorrelation
Independent second HR algorithm running in parallel with HR1 on the same `led1_aled1` signal.

**EXPERIMENT (branch `experiment/ot-domain-inputs`, since v0.38): input changed to OT** —
same rationale as §5.1/§5.2. Autocorrelation is normalised (divides by its own `acorr0`), so
a uniform gain scale does not change the result. The near-zero-energy guard (`acorr0 < 1.0f`
in raw-count units) was recalibrated to `acorr0 < hr2_ot_energy_eps` (1e-16, OT-scale sum of
~400 squared samples) — a pure division-by-near-zero guard, not a physiological threshold.

**v0.42: `_hr2_update_sample(float ir, ProbeState probe_state)`** — mirrors the SpO2 v0.41 /
HR1 v0.42 design (§5.1/§5.2). The `probe_state` gate lives in the **fast path** (shared by
both the production call site and the synchronous test wrapper `_hr2_update_for_test()`): while
`probe_state != PROBE_APPLIED`, the bandpass filter and circular buffer reset **every
sample**, `hr2`/`hr2_sqi` are forced to `NaN`/`0`, and the slow autocorrelation path (Task B)
is never even signalled. Once `PROBE_APPLIED` resumes, the buffer must fill fresh (8 s at
50 Hz) before HR2 reports again.

**Processing chain:**
1. Biquad bandpass filter 0.5–5 Hz at the AFE rate (eliminates DC, high-frequency noise, and acts as anti-aliasing)
2. Decimate via `_hr2_decim` → 50 Hz (see §5.3.1)
3. Accumulate in circular buffer of 400 samples (8 s at 50 Hz)
4. Every 25 decimated samples (0.5 s), compute normalised autocorrelation over lags corresponding to 30–272 BPM

**Peak selection:** first local maximum above `hr2_min_corr = 0.5`. Sub-sample resolution via parabolic interpolation.

**Key constants:**

| Constant | Value | Meaning |
|---|---|---|
| `hr2_buf_len` | 400 | Buffer length (8 s at 50 Hz) |
| `hr2_acorr_lag_cap` | 137 | **Storage cap** of the lag scratch array, not the search policy. The swept range is derived at runtime from `hr_min_bpm`/`hr_max_bpm` — see below |
| `_hr2_decim.factor` | 10 at 500 Hz | **Derived** from `hr2_decim_target_rate_hz`, not configured — see §5.3.1 |
| `hr2_update_interval` | 25 | Recompute every 0.5 s |
| `hr2_min_lag_s` | 0.22 s | Min RR lag (~272 BPM max) |
| `hr2_min_corr` | 0.5 | Normalised correlation threshold for valid peak |

**Lag sweep bounds (derived, not hardcoded):** both ends of the autocorrelation search come
from the configured HR range plus a ±3 BPM guard band, converted to lag at the *current*
decimated rate of HR2's chain, `_hr2_decim.rate_hz`:

```
min_lag = 60 / (hr_max_bpm + 3) * hr2_decim.rate_hz
max_lag = 60 / (hr_min_bpm - 3) * hr2_decim.rate_hz   capped at hr2_acorr_lag_cap and at hr2_buf_len/2 - 1
```

At the operating point (`_hr2_decim.rate_hz` = 50 Hz, HR range [40, 260] BPM) this gives
`min_lag = 11`, `max_lag = 81`. Neither cap is active: 81 is well under both 137 and 199.

The `hr2_buf_len/2 - 1` cap exists because the unbiased normaliser `N/(N−τ)` grows without
bound as τ → N; beyond τ = N/2 the factor exceeds 2 and the estimate is too noisy to trust.
A `static_assert` keeps `hr2_acorr_lag_cap` below that bound at compile time.

**Behaviour at non-default sample rates (resolved in v0.83).** Because the chain's rate
is now the invariant and each chain's factor derives from it (§5.3.1), raising the AFE sample rate no
longer moves the analysis window, the lag bounds or the caps: at 1000 Hz the factor becomes 20 and
the decimated rate stays at 50 Hz. The previous design — a fixed factor of 10 — made
`hr2_acorr_lag_cap` binding for 40 BPM above ~845 Hz and shrank the window below what 40 BPM needs
above ~1200 Hz, both silently. Pinned by `test_hr2_decimated_rate_invariant_to_sample_rate`, which
checks 40 BPM at 1000 Hz: the discriminating case, since a 60 BPM check passes either way.

`hr2_sqi` is a continuous [0–1] quality metric equal to the **unbiased** normalised autocorrelation value at the dominant lag:

```
acorr_unbiased[τ] = Σ_{i=0}^{N-τ-1} x[i]·x[i+τ]  /  (acorr0 · (N−τ)/N)
acorr0            = Σ_{i=0}^{N-1} x[i]²
SQI               = acorr_unbiased[peak_lag]
```

The unbiased correction factor `N/(N−τ)` compensates for the finite-window effect: the biased estimator (dividing by `acorr0` alone) systematically underestimates because its numerator sums only `N−τ` terms while `acorr0` reflects `N` terms. Without this correction a perfectly periodic signal at 60 BPM (τ=50, N=400) would yield SQI = 350/400 = 0.875 even with zero noise. With the correction, a clean periodic signal yields SQI ≈ 1.0 regardless of the HR.

A value close to 1 means the signal is strongly periodic at the detected RR period. `hr2_min_corr = 0.5` acts as the minimum threshold below which no peak is reported and SQI is forced to 0. If the buffer is not yet full or no valid peak is found, SQI = 0.

#### 5.3.1 The decimation chains — one per filtered path, and why both target 50 Hz

HR1 runs on the full AFE rate, sample by sample. The window-based estimators run on a decimated
copy of the signal — but there is **one decimation chain per filtered path**, not one shared
decimation:

| Chain | Anti-alias filter | Nyquist needed | Consumers | Target rate |
|---|---|---|---|---|
| `_hr2_decim` | bandpass 0.5–5 Hz | > 5 Hz | HR2_ACF, and HR2_MPM / HR2_YIN when added (§5.5) — all three share this chain and its buffer | 50 Hz |
| `_hr3_decim` | low-pass 15 Hz | > 15 Hz | HR3 | 50 Hz |

**On the names (v0.84b):** the chains are named after their consumer, which is accurate for all
three lag estimators — `HR2_ACF`, `HR2_MPM` and `HR2_YIN` share HR2's chain and its buffer. Accepted
risk: the name goes stale the day a non-`HR2_*` estimator feeds off it, at which point the honest
rename is after what each chain *preserves* — HR2's guarantees the fundamental across [30, 300] BPM,
HR3's guarantees up to the 3rd harmonic of 260 BPM because its HPS multiplies it. Naming them after
the band was tried and rejected: a "pulse band" that reaches the 5th harmonic of a 60 BPM pulse and
a "harmonic band" that includes the fundamental describe a split that does not exist.

The two chains share **nothing**: not the filter, not the circular buffer, not the phase counter.
Nothing therefore forces a common rate, and running them at one rate buys no saving — no shared
convolution, no shared buffer, not one CPU cycle. Each is a `Decimator` deriving its own factor
from its own target (v0.84). Before v0.84 a single `_decim_factor` served both, which implied a
coupling that does not exist.

**In both chains the rate is the contract and the factor is derived:**

```
factor  = round(afe_sample_rate_hz / target_rate_hz)     // >= 1
rate_hz = afe_sample_rate_hz / factor                    // rate ACTUALLY achieved
```

Constants calibrated against a chain's rate — `hr2_buf_len` (400 = 8 s), `hr3_buf_len` (512 =
10.24 s, 0.098 Hz per bin), the `hrX_update_interval` defaults (25 = 0.5 s) and
`hr2_acorr_lag_cap` (137 = 22 BPM) — therefore keep meaning the same number of *seconds* whatever
the AFE rate does. Holding the factor fixed instead, as before v0.83, moved all of them at once
and silently. Consumers read `chain.rate_hz`, never the nominal target: for every PRF in the
validated grid (§7.4) the factor is exact and the two coincide, but a rate outside it must degrade
honestly rather than lie.

**Why both target 50 Hz — and why that is a coincidence of requirements, not a constraint.**
This had never been justified; it was inherited from dividing 500 Hz by a round 10.

*HR3's chain needs it.* Four independent reasons, all pointing at ~50 Hz:

1. **50 Hz is the GCD of the validated PRF grid** {500, 800, 1000, 1250, 1600} — the highest
   decimated rate with an *integer* factor at every admissible PRF (10, 16, 20, 25, 32). The only
   alternatives are its divisors (25, 10, 5, 2, 1 Hz).
2. **Below it the anti-aliasing collapses.** HR3 low-passes at 15 Hz, so at a 25 Hz decimated
   rate the Nyquist limit (12.5 Hz) falls *below the filter's own corner*: −1.7 dB of attenuation
   with the 2nd order of v0.96, −0.9 dB with the 4th order of v0.97 — no protection for the
   decimation at all, whatever the order. That is why the order cannot buy a lower rate on its own:
   the corner would have to come down too.
3. **Below it the window collides with physiology.** At 25 Hz it grows to 20.5 s, where a
   transient neonatal bradycardia — seconds long — changes the rate *inside* the window and
   smears the peak instead of sharpening it.
4. **Above it the fixed-length buffer grows too short in time**, and the bin resolution degrades
   (0.24 Hz, 14.6 BPM per bin, at 125 Hz).

| Decimated rate | Nyquist | 15 Hz LP at Nyquist, 2nd order (≤ v0.96) | 4th order (v0.97) | Bin (N=512) | HR3 window | HR2 window |
|---|---|---|---|---|---|---|
| 25 Hz | 12.5 Hz | **−1.7 dB** | **−0.9 dB** | 0.049 Hz | 20.5 s | 16.0 s |
| 40 Hz | 20 Hz | −6.2 dB | −10.5 dB | 0.078 Hz | 12.8 s | 10.0 s |
| **50 Hz** | 25 Hz | −9.4 dB | **−18.0 dB** | 0.098 Hz | 10.24 s | 8.0 s |
| 100 Hz | 50 Hz | −21.0 dB | −42.9 dB | 0.195 Hz | 5.1 s | 4.0 s |

*(Digital responses at a 500 Hz PRF, bilinear transform prewarped at 15 Hz.)*

*HR2's chain does not.* Only reason 1 applies to it. Its bandpass cuts at 5 Hz, so 25 Hz would
still leave Nyquist at 12.5 Hz — 2.5× of margin, the same HR3's chain has today over its
15 Hz. Lowering it to 25 Hz (factor 20 at 500 Hz, integer across the whole grid since 25 divides
50) would give, keeping the window at 8 s: 200 samples instead of 400, ~36 lags instead of 71,
`O(N·lags)` down from 28 400 to 7 200 — a **3.9× cut**, *tripled* once MPM and YIN sweep the same
buffer — and half the RAM in `_hr2_buf` + `_hr2_snapshot`.

**What holds that back, and how to settle it.** Decimating further costs lag resolution: at 25 Hz
a 120 BPM beat lands on lag 12.5, so ±0.5 sample is 4.8 BPM before interpolation versus 2.4 BPM at
50 Hz. The ACF tolerates it (parabolic interpolation over a broad peak recovers most of it), but
MPM (NSDF) and YIN (CMND) locate a sharp extremum whose definition *is* their advantage over the
ACF — halving the points per period attacks exactly that. It is also a confound: a campaign
comparing three lag estimators must not change the chain rate at the same time, or it measures the
chain instead of the estimators. **Decide it from the offline experiment** (`tools/offline_runner`
replays recorded captures at both rates — no hardware needed), not by reasoning.

**HR3's anti-alias low-pass is 4th order since v0.97** (it was the chain's known weak point):
the 2nd-order low-pass left −9.4 dB at the 25 Hz Nyquist, thin for a ×10 decimation. Harmless for
the HR estimate (there is almost no PPG content above 25 Hz), but the folded noise raises the
spectral floor, and that is paid in `hr3_sqi`, not in `hr3`. The lever was the filter order before
decimating — **not** a different rate, which reasons 1–4 had already fixed. A 4th-order Butterworth
gives −18.0 dB there for two biquads instead of one (`LowPass4`: sections Q = 0.5412 and 1.3066,
lower Q first). Its extra group delay (≈ 28 ms at DC vs ≈ 15 ms) is irrelevant to an FFT window
of 10.24 s. And it is *flatter* over the pulse band the HPS multiplies — at 260 BPM, 2nd harmonic
(8.7 Hz) −0.05 dB vs −0.46 dB, 3rd harmonic (13 Hz) −1.2 dB vs −1.9 dB — so the harmonics reach the
HPS less attenuated than before, not more.

### 5.4 HR3 — FFT + Harmonic Product Spectrum

**EXPERIMENT (branch `experiment/ot-domain-inputs`, since v0.38): input changed to OT** —
same rationale as §5.1-5.3. SQI is a ratio of HPS values (same power units, cancels scale),
so a uniform gain scale does not change the result — pure domain/type change, no threshold
recalibration needed.

**v0.42: `_hr3_update_sample(float ir, ProbeState probe_state)`** — mirrors the SpO2 v0.41 /
HR1/HR2 v0.42 design (§5.1-5.3). Same fast-path gate placement as HR2: while
`probe_state != PROBE_APPLIED`, the low-pass filter and circular buffer reset **every
sample**, `hr3`/`hr3_sqi` are forced to `NaN`/`0`, and the slow FFT/HPS path (Task C) is
never signalled. Once `PROBE_APPLIED` resumes, the buffer must fill fresh (10.24 s at 50 Hz)
before HR3 reports again.

Independent third HR algorithm running in parallel with HR1 and HR2 on `led1_aled1`.

**Processing chain:**
1. 4th-order Butterworth low-pass filter at 15 Hz (`_hr3_lp`, two cascaded biquads; 2nd order until v0.96 — anti-aliasing before decimation, see §5.3.1 for the order and the note below on why it is a pure low-pass)
2. Decimate via `_hr3_decim` → 50 Hz (see §5.3.1)
3. Accumulate in circular buffer of 512 samples (10.24 s at 50 Hz)
4. Every 25 decimated samples (0.5 s): mean subtraction (DC removal) → Hann window → in-place radix-2 DIT FFT → HPS → parabolic interpolation

**Why a pure low-pass and not a bandpass (v0.83).** HR2 bandpasses because autocorrelation only
needs the periodicity; HR3 feeds an FFT, which needs the harmonics preserved in amplitude and
phase — the HPS multiplies the fundamental by its 2nd and 3rd harmonics, so a high-pass corner
near the fundamental would distort exactly what the HPS relies on. The DC that a high-pass would
have removed is removed instead by **subtracting the window mean** in `_hr3_prepare_fft_input()`, which is
both exact for the window analysed and free of phase distortion.

Until v0.83 `setHR3Filter()` took an `f_low_hz` argument and `AFE4490Config` exposed
`hr3_f_low_hz`, both inert: `init_lp()` never reads the lower corner. Worse than useless, the
reported value was a plausible 0.4 Hz left over from the constructor, so a caller passing a
different value got that stale 0.4 back and could believe a bandpass was in effect. Both were
removed; `f_low` in `BiquadFilter` stays 0 and unused for low-pass instances.

**Harmonic Product Spectrum (HPS):** multiplies the power spectrum by its 2nd and 3rd harmonic downsampled versions: `HPS[k] = P[k] · P[2k] · P[3k]`. Reinforces the fundamental frequency; suppresses dominant harmonics that can mislead simpler peak-finding.

**Self-contained FFT:** radix-2 DIT FFT (Cooley-Tukey) implemented in the anonymous namespace to avoid external dependencies. Works in both `env:in3ator_V15` and `env:native` test environments.

**Key constants:**

| Constant | Value | Meaning |
|---|---|---|
| `hr3_buf_len` | 512 | Buffer length (10.24 s at 50 Hz) |
| `_hr3_decim.factor` | 10 at 500 Hz | Own chain, independent of HR2's — **derived**, see §5.3.1 |
| `hr3_update_interval` | 25 | Recompute every 0.5 s |
| LP cutoff | 10 Hz | Anti-aliasing before decimation |
| HPS harmonics | 2, 3 | P[k]·P[2k]·P[3k] |
| HPS search cap | `nyquist/3` | Ensures 3rd harmonic stays within Nyquist band |

**Stack note:** `_hr3_fft[1024]` (4096 bytes complex buffer) is stored as a class member (heap). `INCUNEST_AFE4490_TASK_STACK` default increased to 8192 to accommodate `cosf`/`sinf` stack usage during butterfly stages.

**True-peak validation** — `peak_bin` must be the strict maximum in a window of ±`hr3_peak_valid_w` bins (`hr3_peak_valid_w = 5`, i.e. ±0.49 Hz). Bins outside `[1, nyquist)` are skipped; bins outside the search range are **included** intentionally. If any bin in the window has HPS ≥ `peak_hps`, the candidate is rejected as a boundary maximum caused by an out-of-range energy hump (e.g. low-frequency movement artifact leaking into the search range) — HR and SQI are both forced to 0 and the function returns early.

`hr3_sqi` is a continuous [0–1] metric based on the **two-bin HPS fraction** in a local window. This design is bin-position-independent: a signal at a non-integer bin position (`peak_bin + delta`) splits its HPS energy across b1 = floor(`peak_bin + delta`) and b2 = b1 + 1. Because b1 < b2 always, the window [b1−W, b2+W] is unambiguously defined.

```
b1        = floor(peak_bin + delta)         lower bin; b1 ≥ 1
b2        = b1 + 1                          upper bin; b2 ≤ nyquist/3
hps_num   = hps_at(b1) + hps_at(b2)
b_lo      = b1 − hr3_sqi_local_w
b_hi      = b2 + hr3_sqi_local_w
hps_win   = Σ hps_at(k) for k in [b_lo, b_hi] ∩ [1, nyquist/3]
n_win     = number of bins summed
fraction  = hps_num / hps_win
baseline  = 2 / n_win               (uniform-noise reference: 2 signal bins out of n_win)
SQI       = clamp((fraction − baseline) / (1 − baseline), 0, 1)
```

Note: `peak_bin` (the HPS search maximum) coincides with either b1 or b2; `hps_at(b1/b2)` reuses `peak_hps` in that case to avoid a redundant computation.

Parameters: `hr3_sqi_local_w = 5` bins (window extends ±5 bins beyond b1 and b2, ~±0.49 Hz). If `hps_win = 0` or `n_win ≤ 2`, SQI = 0 (conservative).

**Bin-independence:** P[k] and HPS metrics based on a single bin suffer from a half-bin misalignment effect (energy splits across two bins → denominator changes but numerator is halved). By summing two adjacent bins in both numerator and denominator, the effect cancels and SQI is approximately constant regardless of bin alignment.

**Why HPS and not P[k]:** HPS suppresses harmonics of the fundamental's subharmonics (movement artifacts at 0.5×, 0.33× the PPG frequency). A true PPG signal concentrates HPS at the fundamental; a noisy signal spreads it. The cubic amplification of HPS is controlled by the local-window denominator (not a peak-vs-mean ratio), so it does not saturate to 1.

**Excluded harmonics:** the denominator sums only a local window around the two peak bins, not the full search range. High-HR signals have their 2nd and 3rd harmonics inside the search range but far from the peak window; these do not inflate the denominator, keeping SQI bin-rate-independent.

A clean PPG signal → SQI approaches 1. A weak or noisy signal → intermediate SQI. A movement artifact or absent signal → true-peak validation fails → SQI = 0.

### 5.5 HR algorithms roadmap

| Algorithm | Method | Status |
|---|---|---|
| HR1 | Threshold crossing (rising edge, 0.6 × running_max) | Implemented |
| HR2_ACF | Autocorrelation on decimated signal (50 Hz, 8 s window) | Implemented |
| HR3 | FFT + HPS on decimated signal (50 Hz, 10.24 s window) | Implemented |
| HR2_MPM | McLeod Pitch Method — NSDF + clarity, same buffer as HR2_ACF | Planned |
| HR2_YIN | CMND minimum, same buffer as HR2_ACF | Planned |
| HR4 | True peak detection via derivative (max of derivative = steepest ascending slope) | Planned |

**Acronyms.** `ACF` = *autocorrelation function* (maximises normalised correlation). `NSDF` =
*normalised square difference function*, the basis of the `MPM` = *McLeod Pitch Method*, which
reports its own *clarity* metric. `CMND` = *cumulative mean normalized difference*, minimised by
`YIN` — not an acronym: its authors (de Cheveigné and Kawahara, 2002) named it after yin-yang, for
the duality between correlation and difference. All three estimate the same period: one maximises a
correlation, the other two minimise a normalised difference, over the same decimated buffer.

HR1–HR4 all operate on `led1_aled1` (IR ambient-corrected) and run in parallel. `AFE4490Data` will expose `hr4`/`hr4_valid` when implemented.

> All HR algorithms developed from scratch. Protocentral code is not used as a base.

**HR1's filter and detector are under review.** The moving-average low-pass is inherited from a
constraint that disappeared in v0.6 (the biquad could not compute its own coefficients then), and
measurement shows no argument left for keeping it: under phototherapy interference, with a true
rate of 60 BPM, the MA reads 75.5 BPM against the biquad's 66.1. Those same figures show the
**detector** is the weaker link — a +10 % error is still a failing monitor. The full analysis, the
strategy catalogue (TERMA, SSF, derivative variants, template matching, envelope, plus waveform
validation, adaptive refractory and cross-checking against HR2) and the recommendation live in
[`incunest_afe4490_design_rationale.md`](incunest_afe4490_design_rationale.md) §5, each claim with its source.

---

### 5.6 RSQM — Raw Signal Quality Monitor

Runs in `_process_sample()` after `_compute_analog_state()`, once per sample at 500 Hz. Fills `AFE4490Data::rsqi`, `diag_code`, and `probe_state`. Receives `AFE4490AnalogState` by `const&` (pre-computed OT values).

#### 5.6.1 Estimators

RSQM keeps **no EMA estimators of its own**. Until v0.56 it maintained two `EmaChannel`s over the
differential signal (LED_Sub = LED − ALED), but both of their consumers are gone: the pulsatile-amplitude
flag `RSQM_DIAG_SIGNAL_WEAK` (removed v0.56 — redundant with the PI-based `spo2_sqi`, and not fixable
by gain) and the `ready`/`rsqm_ready_count` warm-up gate on `rsqi` (removed v0.57 — a vestige of
SIGNAL_WEAK with no remaining statistic to mature). Probe-state detection uses **instantaneous**
quantities (`i_pd`, OT, saturation masks), so it needs no warm-up. The generic `EmaChannel` (§5.1) is
now used only by SpO2. If a pulsatile-amplitude check is ever needed again, reintroduce it as
`RSQM_DIAG_AC_LOW`.

#### 5.6.2 Probe state machine

**Classifier (v0.61).** Presence ("is there tissue?") is answered by OT, which is **invariant to
ambient light** because it subtracts the ALED phase — so it separates tissue from no-tissue even
under a lamp. DISCONNECTED first; then, if not saturated, OT is a clean measurement; if saturated,
OT is clipped so the *clipping phase pattern* decides:

```
if DISCONNECTED                          → PROBE_DISCONNECTED
elif NOT anyPositiveSaturation:                              # OT is a clean measurement
        OT_led1 > thr AND OT_led2 > thr  → PROBE_OT_HIGH              # no tissue (v0.89: AND)
        else                             → PROBE_APPLIED
else (saturated: ADC at rail or v_tia > FS — OT clipped):   # discriminate by phase
        ALED1/ALED2 phase also clips     → PROBE_AMB_SATURATING       # external light (lamp/phototherapy)
        else (only LED phase clips)      → PROBE_ONLY_LED_SATURATING  # LED's own light reaches the PD

isProbeAbsent(s) = DISCONNECTED || OT_HIGH || ONLY_LED_SATURATING     # the one definition of "no patient"
                                                                       # (HGAC gate, diagnostics task)
```

**PROBE_DISCONNECTED** — instantaneous criterion, ALL of:
- `LED1_raw < 0` AND `ALED1_raw < 0` AND `LED2_raw < 0` AND `ALED2_raw < 0`
- `|LED1_Sub| < 5000` AND `|LED2_Sub| < 5000`

Physical cause: when cable is disconnected, TIA input is floating; input-offset current drives all four channels (LED1, ALED1, LED2, ALED2) to the same large negative value, so `LED_Sub = LED − ALED ≈ 0`. No EMA warmup required; responds in one sample.

Validated with afe_sweep_test.csv: DISCONNECTED → all channels negative, LED_Sub_mean ≈ −592 (range −3117..+65).

**Presence by OT (not saturated).** `OT = (I_PD_LED − I_PD_ALED) / I_LED` [A/A] is computed in
`_compute_analog_state()` (`ot_led1`/`ot_led2`, §3c); `_rsqm_update()` consumes it. It is the full
electro-optical system ratio (LED efficiency × tissue × PD responsivity), not tissue transmittance
alone. Threshold (**v0.89**): `ot_led1 > rsqm_ot_thr` **AND** `ot_led2 > rsqm_ot_thr` →
PROBE_OT_HIGH; otherwise PROBE_APPLIED.

*Why AND and not OR.* With OR, whichever channel happened to read higher decided **alone**: one
channel drifting over the threshold declared the probe absent while the other still saw tissue. On
thin tissue (preterm foot, CONTEC MS100 simulator) that made a correctly applied probe read as
absent, which resets SpO2/HR1/HR2/HR3 on every sample — the failure mode with no downstream safety
net. AND requires two independent channels to agree. In air neither channel is attenuated by tissue,
so both are high and OT_HIGH still holds: margin is gained on thin tissue without giving any
back on the air case.

> **The channel ordering is not predictable from tissue spectra, and an earlier version of this
> section claimed otherwise.** It argued that RED (660 nm) is absorbed more than IR (940 nm), so
> `ot_led1 > ot_led2` "always". Measured on the MS100 with the probe applied (2026-09-07):
> **`ot_led1` (IR) = 91 ppm, `ot_led2` (RED) = 112 ppm** — the opposite ordering. OT is the whole
> electro-optical ratio (LED efficiency × tissue × PD responsivity × geometry), not tissue
> transmittance, and the simulator's plastic reproduces the ratio R that yields a given SpO2, not
> per-wavelength transmittance. The AND does not depend on which channel is larger, so the change
> stands; its justification does not rest on the ordering.

> **How little margin this leaves.** Both channels sit within 12 % of `rsqm_ot_thr` (then 100 ppm).
> The AND holds on the MS100 only because IR happens to be 9 ppm under the threshold. This is a
> working configuration, not a robust one, and it is direct evidence that `rsqm_ot_thr` needs the
> empirical calibration it has never had.
>
> **Update (v0.95, 2026-09-24).** That calibration data arrived with the first real-patient
> campaign (HOSPNAV, 2026-09-23): SUBJ08, probe applied, read OT_LED1 mean ≈ 120 ppm / max
> ≈ 145 ppm and spent long stretches in PROBE_OT_HIGH — resetting SpO2/HR each time — on an
> applied probe. `rsqm_ot_thr` widened 1.0e-4 → 1.5e-4. Margin above the observed maximum is
> only ×1.03 (n=1 patient — revisit as campaigns accumulate); margin below the theoretical
> probe-in-air OT (~8e-4) remains ×5.3, and ONLY_LED_SATURATING detects probe-in-air
> independently of OT.

*The cost, stated plainly.* AND introduces a failure mode OR did not have: a channel reading
persistently low — weak or failing LED, partially obstructed photodiode — now forces APPLIED
regardless of the other. Two existing guards limit the exposure without removing it: the
LED-only-saturation branch below detects probe-in-air independently of OT, and PROBE_DISCONNECTED
still checks all four `i_pd` channels. A channel that is degraded but not dead is not covered. Default `rsqm_ot_thr = 1.5e-4` (v0.95), runtime-configurable via `setRsqmOtThr()`. Single
threshold (no hysteresis) — oscillation is prevented by the `rsqm_probe_state_min_s` debounce.

**Saturated — discriminate by phase (v0.61).** When `anyPositiveSaturation()` (ADC positive rail or
v_tia > FS on any channel), OT is clipped/unreliable, so RSQM looks at *which* phase clips:
- An **ambient** phase (ALED1 or ALED2) also clips (`adcSatPos || tiaOverFs`) → the saturating light
  is present in the LED-off phase too → **external light** (lamp/phototherapy) → **PROBE_AMB_SATURATING**.
  Presence unknown, but HGAC should try to correct it (and the ambient alarm §5.8.3 may fire). Its
  own state (not folded into NOT_APPLIED) is what lets HGAC's gate act on real saturation.
- Only **LED** phases clip, ambient phases clean → the LED's own light reaches the PD directly with
  nothing attenuating it → **probe in air** (finger removed) → **PROBE_ONLY_LED_SATURATING** (absent per `isProbeAbsent()`), so HGAC does
  NOT chase RF to the floor on finger removal (which would ruin the operating point for reattach).
  *Limitation:* LED-only clipping can't distinguish air from tissue so thin that it saturates
  without ambient — assumed air. **v0.90 gives that case its own state** so the assumption is
  visible in the data instead of buried in a comment. **Open check:** thin tissue at high RF →
  `PROBE_ONLY_LED_SATURATING` → absent → HGAC gate closed → nobody lowers RF → saturation
  persists → stays absent. A stable false negative that nobody corrects, symmetric to the bug
  v0.61 fixed. Verification protocol and tooling: `tools/probe_state_saturation_check.py`
  (PulseNest). If confirmed, the fix needs a discriminant in this branch — OT is degraded under
  saturation but an OT far below `rsqm_ot_thr` still says "tissue" — not a change of gate.

**Consumers:** SpO2/HR1/HR2/HR3 and RSQI treat PROBE_AMB_SATURATING, PROBE_ONLY_LED_SATURATING and PROBE_OT_HIGH like any
non-`PROBE_APPLIED` state (invalid/reset — they gate on `!= PROBE_APPLIED`). `_diag_task_body()` and
HGAC (§5.8) treat PROBE_AMB_SATURATING like PROBE_APPLIED (HGAC acts; the ambient alarm may then fire).

**History:** ≤2026-07-24 saturation was folded into NOT_APPLIED; 2026-07-24…v0.60 *any* positive
saturation → PROBE_SATURATING (which drove RF to the floor on finger removal — the bug that
motivated v0.61); v0.61 splits saturation by phase as above. **v0.90** renames PROBE_SATURATING → PROBE_AMB_SATURATING (the old
name suggested "any saturation"; LED-only saturation never came here) and splits PROBE_NOT_APPLIED
into PROBE_OT_HIGH (value 1) and PROBE_ONLY_LED_SATURATING (value 4), with `isProbeAbsent()` as the
single definition of "no patient". No behavioural change.

Estimated values (theoretical, need empirical calibration with HW): APPLIED ≈ 1.4e-5, NOT_APPLIED ≈ 8e-4 (factor ~×54). First real-HW test (2026-07-19, CONTEC MS100 simulator, probe applied) showed the theoretical APPLIED estimate does not hold in practice: the simulator is highly sensitive to probe placement and OT frequently reads above 8.5e-5 with the probe applied, so `probe_state` never reached APPLIED at that threshold. `rsqm_ot_thr` was widened to 1.0e-4 as a practical mitigation — still not a calibrated value from a real neonatal probe/patient. The first real-patient data (HOSPNAV campaign, 2026-09-23, SUBJ08: probe applied, OT_LED1 mean ≈ 1.2e-4, max ≈ 1.45e-4) widened it again to 1.5e-4 (v0.95); n=1 patient, to be recalibrated as campaigns accumulate.

> Previous formula was heuristic: `LED_Sub / (LED_mA × RF_Ω × RG_linear)` — dimensionally not A/A, scale ~×3495 larger. Values measured in earlier sessions (APPLIED ≈ 0.05, NOT_APPLIED ≈ 2.8) correspond to the old scale.

> Valid for clip-type reusable probes only. Disposable adhesive probes not validated.

**PROBE_APPLIED** — neither of the above.

**State transition debounce (`_rsqm_request_probe_state`):** any state change requires `_rsqm_probe_state_min_samples` consecutive samples (= `rsqm_probe_state_min_s × fs`, default 0.2 s = 100 samples @ 500 Hz) with the same requested state before being committed. A single different sample resets the counter. This eliminates false transient states during cable connect/disconnect events. Runtime-configurable via `setRsqmProbeStateMinS()`.

#### 5.6.3 DiagCode

**Bits 0–12 — AFE hardware (set by `runAfeDiagnostics()`, persist until next call):**

| Bit | Constant | Condition |
|---|---|---|
| 0x0001 | `AFE_DIAG_PD_ALM` | Photodiode out of range |
| 0x0002 | `AFE_DIAG_LED_ALM` | LED out of range |
| 0x0004 | `AFE_DIAG_OUT` | General diagnostic fault |
| 0x0008 | `AFE_DIAG_LED2_ALM` | LED2 driver fault |
| 0x0010 | `AFE_DIAG_LED3_ALM` | LED3 driver fault |
| 0x0020 | `AFE_DIAG_LED1_ALM` | LED1 driver fault |
| 0x0040 | `AFE_DIAG_PDOC_ALM` | Photodiode open-circuit |
| 0x0080 | `AFE_DIAG_PDSC_ALM` | Photodiode short-circuit |
| 0x0100 | `AFE_DIAG_LED2OC_ALM` | LED2 open-circuit |
| 0x0200 | `AFE_DIAG_LED2SC_ALM` | LED2 short-circuit |
| 0x0400 | `AFE_DIAG_LED1OC_ALM` | LED1 open-circuit |
| 0x0800 | `AFE_DIAG_LED1SC_ALM` | LED1 short-circuit |
| 0x1000 | `AFE_DIAG_COMMON_MODE_ALM` | Common-mode voltage out of range |

**Bits 13+ — RSQM software (updated continuously at 500 Hz by `_rsqm_update()`):**

| Bit | Constant | Condition |
|---|---|---|
| 0x2000 | `RSQM_DIAG_AMB_SAT` | `as.adcSatPos(ALED1)` OR `as.adcSatPos(ALED2)` (code ≥ `adc::SAT_POS` = 2 096 700) |
| 0x4000 | `RSQM_DIAG_AMBIENT_HIGH` | HGAC alarm: RF at the floor (`RF_10K`) AND the ambient (ALED) EMA ≥ HIGH2 on either domain → ambient light alone saturates ("TOO MUCH AMBIENT LIGHT"). Preventive (EMA @ HIGH2), distinct from `AMB_SAT`'s per-sample hard rail. (Was `SIGNAL_WEAK` until v0.56.) |
| 0x8000 | `RSQM_DIAG_SWITCHED_RC_SETTLING` | Countdown > 0 (`_switched_rc_settling_countdown`). **Enforced since v0.72**: while set, `_task_body()` feeds frozen last-valid raw values instead of the real transient ones (§2.6b) — not just a label on data that still flows through. |

HGAC publishes no DiagCode bit (the former bit 16 `HGAC_DIAG_GAIN_FLOOR` was removed in v0.50 — no consumer decoded it; the "at RF floor and still saturating" condition is derivable at the point of use as `rf == RF_10K` while TIAGUARD persists, and its actuator, ILED, is Phase 4).

Multiple bits may be set simultaneously.

**Update (v0.37):** `RSQM_DIAG_SWITCHED_RC_SETTLING` is no longer unreachable — HGAC (§5.8) is its first real producer, arming `_switched_rc_settling_countdown` after every RF step-down. `RSQM_DIAG_AMB_SAT` still has no programmatic consumer beyond the aggregate `rsqi` gate and human inspection.

#### 5.6.4 RSQI

```
rsqi = 1  iff  (probe_state == PROBE_APPLIED) AND (diag_code == 0)
rsqi = 0  otherwise
```

---

### 5.7 Autonomous diagnostics task (`_diag_task_body`)

A dedicated internal FreeRTOS task (`incunest_diag`, low priority) calls `runAfeDiagnostics()` autonomously on a periodic schedule:

- **Periodic** — the task wakes every `rsqm_diag_period_ms` (default 5000 ms) via `vTaskDelay`. Diagnostics run only in states with no patient confirmed in the optical path: `probe_state == PROBE_DISCONNECTED` OR `probe_state == PROBE_OT_HIGH / PROBE_ONLY_LED_SATURATING`. `PROBE_APPLIED` and `PROBE_AMB_SATURATING` are excluded to avoid disrupting HR/SpO2 acquisition with the 10 ms blackout window — and, for `PROBE_AMB_SATURATING` specifically, to avoid disrupting HGAC's attempt to clear the saturation while a real patient may be connected. This is a whitelist by design (since v0.50): any future `ProbeState` defaults to NOT diagnosing, so a new state can never silently interrupt acquisition — it must be added here deliberately. Equivalent to the former `≠ APPLIED AND ≠ SATURATING` blacklist for the four states that exist today.

There is no on-state-change trigger: it only advanced the first post-transition diagnostic by ≤ 5 s while no patient was present (no clinical value) and woke the task pointlessly on transitions into APPLIED/SATURATING. Removed in v0.50 (see §14).

The task is created in `begin()` and deleted in `stop()`. It runs on core 1 at `INCUNEST_AFE4490_DIAG_TASK_PRIORITY` (= main task priority − 2).

`runAfeDiagnostics()` remains a public API callable independently from external code (e.g. via `$DIAG?`). Both callers serialise through `_spi_mutex`.

---

### 5.8 HGAC — Hardware Gain Control (RF-only, two EMA per gain domain)

**Status (v0.58): RF-only, two EMA per gain domain — a fast EMA drives an urgent saturation
guard (HIGH2) and a slow EMA drives a bidirectional leveling dead-band (HIGH1/LOW1). Disabled by
default (`hgac_enable = false`).** Runs in `_process_sample()` immediately after `_rsqm_update()`,
once per sample at 500 Hz. Two independent loops, one per gain domain (IR = LED1+ALED1, RED =
LED2+ALED2 — ENSEPGAIN=1; see §5.6/§11). ILED is fixed at its thermal-envelope maximum (RF is the
only actuator); alarms (RF floor→ambient, RF ceiling→weak signal), the ALED cause-discriminator
and feed-forward jumps are deferred. Full design rationale in the project's HGAC design notes.

> **Why HGAC is RF-only — ILED is a weak de-saturation actuator at the gain floor (analysis
> 2026-08-04).** In the LED phase, `v_tia_led = 2·RF·(i_amb + i_led)` with `i_led = OT·ILED`; only
> `i_led` scales with ILED (ambient current does not — see the ILED-settling note above). The
> maximum voltage that lowering ILED can shed at RF = 10K (the floor) is
> `Δv = 2·RF·OT·(ILED_max − ILED_min) = 800·OT` V. For an APPLIED neonatal probe (OT < `rsqm_ot_thr`
> ≈ 1e-4) that is only **~0.01–0.08 V** — negligible: at RF = 10K the LED-driven part of `v_tia`
> is itself tiny (~0.1 V), so if a channel is *saturating* there (~1.2 V ADC rail), the bulk is
> **ambient**, which ILED cannot touch.

#### 5.8.1 Tracked signal and thresholds

Only the **LED phase** of each domain is tracked (`v_tia_led1`/`v_tia_led2`); the ALED phase is
not. Rationale (v1: unipolar photocurrent ≥ 0, AMBDAC=0): `v_tia_led ≥ v_tia_aled` always, so the
LED phase is the upper envelope — the active constraint both for descending (it saturates first)
and ascending (raise RF until LED enters the band → ALED stays below). ALED is reserved as the
cause-discriminator of the (deferred) excessive-ambient-light alarm.

Three thresholds on `v_tia_led` (per domain), all runtime-tunable:

| Threshold | Default | Source | Acts on | Action |
|---|---|---|---|---|
| `hgac_v_tia_high2` | 0.90 V | `tia_axis::HIGH2_V` (clamped ≤ FS_V) | fast EMA | guard: `RF--` urgently |
| `hgac_v_tia_high1` | 0.75 V | policy literal | slow EMA | leveling: `RF--` |
| `hgac_v_tia_low1`  | 0.20 V | policy literal | slow EMA | leveling: `RF++` |

`[LOW1, HIGH1]` is a dead-band (no action). Stability invariant: `HIGH1/LOW1 = 3.75 > 2.5×` (the
largest RF LUT step), so a single step never crosses from one band edge to the other (no hunting).
LOW1 was chosen lower than HIGH1's symmetric counterpart (0.30 would match the 3.0 ratio) rather
than raising HIGH1 to widen the band: HIGH1 is capped by its 0.15 V margin to HIGH2/GUARD (0.90 V,
a safety margin against saturation), while LOW1 has no symmetric lower guard (`v_tia_diff ≥ 0`
always) — widening costs only SNR, never safety.

#### 5.8.2 Two EMA per domain (fast guard / slow leveling)

Each domain runs two `EmaChannel`s (§5.1) over `v_tia_led`:

| EMA | τ default | warmup (3·τ) | Role |
|---|---|---|---|
| fast (`hgac_ema_fast_tau_s`) | 0.1 s | 0.3 s | HIGH2 guard — integrates **severity × time** (how far/how long the level exceeds the guard). An EMA is chosen over the raw sample precisely for this: the raw sample can't tell a brief graze from a sustained excursion. |
| slow (`hgac_ema_slow_tau_s`) | 2.0 s | 6 s | HIGH1/LOW1 leveling — clean DC (f_c ≈ 0.08 Hz ≪ HR_min 0.5 Hz, so it does not follow the pulse). |
| ambient (`hgac_ema_ambient_tau_s`) | 2.0 s | 6 s | AMBIENT_HIGH alarm on the **ALED** phase (`v_tia_aled`). Sustained (long τ) so a brief ambient flash doesn't flicker the alarm. |

There is **no separate debounce**: the temporal weighting is the EMA's τ itself (an EMA already
rejects transients shorter than its scale). `valid()` (§5.1) short-circuits each check, so nothing
acts during an EMA's warmup after a reset — a fail-safe with no extra branch.

#### 5.8.3 Response algorithm

**Actuation — `_hgac_track()`, per domain (RF only, no diagnostic output):**
```
guard (fast): if fast.valid() && fast.mean >= HIGH2 → if RF>RF_10K: RF-- ; return  (priority)
level (slow): if !slow.valid() → return
              if slow.mean >= HIGH1 → if RF>RF_10K: RF--
              else if slow.mean < LOW1 → if RF<RF_1M: RF++
```
RF steps one `kAFE_RF_OHM` LUT level per action. At the floor (`RF_10K`) a further `RF--` is a
no-op; at the ceiling (`RF_1M`) a further `RF++` is a no-op.

**Ambient alarm — `_hgac_ambient_high()`, diagnostic, evaluated separately in `_hgac_update()`:**
```
ambient_high(domain) = (RF == RF_10K) && ambient.valid() && ambient.mean >= HIGH2
AMBIENT_HIGH bit = ambient_high(IR) || ambient_high(RED)      (global OR)
```
Kept **out of `_hgac_track()`** so actuation and diagnostic stay separate. Meaning: RF is at the
floor (HGAC can do no more) AND the **ALED** phase alone is ≥ HIGH2 → ambient light alone saturates,
so lowering RF/ILED can't help (`v_tia_led ≥ v_tia_aled`, LED adds only ~0.1 V at the floor) →
**AMBIENT_HIGH** (bit 14, §5.6.3). Deliberately **independent of the LED guard** (`fast`): ALED is
the direct ambient indicator, so the alarm doesn't require the LED guard to have tripped. The
LED-dominated case (ALED low) raises no alarm (ILED actuation deferred). Produced by HGAC (not RSQM)
because it depends on RF state; evaluated after `_hgac_track()` (a step to the floor resets the
ambient EMA → `valid()==false` holds the alarm off until it re-warms). HGAC sets/clears the bit each
sample and refreshes `_current_data.diag_code` (it runs after `_rsqm_update()`, which wrote the
mirror without it).

#### 5.8.4 Reset on RF change (`_hgac_change_rf`) — no rescale; settling armed on any signal-chain change

**Naming (v0.75, Alex):** renamed `_rsqm_settling_countdown`/`RSQM_DIAG_HW_SETTLING`/
`_arm_settling()`/`_compute_afe_settle_samples()` to their `*_switched_rc_settling_*` equivalents.
"Settling" alone was ambiguous: this library has **three** unrelated settling phenomena that could
all claim the word — the TIA's own RC pole (`τ=RF·CF`, settles within one sample, drives auto-CF),
the LED/cable turn-on transient (`_compute_led_on_to_sample_margin_counts()`, also within one
sample), and this one, which needs **several PRP cycles** (the datasheet's 3 ms minimum, not
microseconds). "Switched RC" is the datasheet's own phrase for the affected circuitry (§7.7 t5
footnote (1): "four switched RC filters") — chosen over alternatives like `gain_settling` (names
the *trigger*, not the mechanism, and would misdescribe a future ILED-triggered case — ILED is a
drive current, not a gain) or `sample_hold_settling` (avoids "RC" but isn't the datasheet's term).

A gain change scales `v_tia` by `k = RF_new/RF_old`, making prior measurements meaningless, so
`_hgac_change_rf()` **resets** the domain's two EMAs (rather than rescaling them analytically).
SpO2's own EMAs are **not** touched: SpO2 consumes gain-invariant OT (§5.1), so an RF change is not
a discontinuity in its input.

**Settling is armed by the setter itself, not by the caller (v0.74) — a manual RF change is
protected exactly like an HGAC one.** Reviewed 2026-08-22 (Alex): `_switched_rc_settling_countdown` used
to be armed only inside `_hgac_change_rf()`, so calling `setTIAGainLED1/2()`/`setTIAGain()`
directly — e.g. a manual `$SET,tiagain1,...` from the host — changed RF without arming
`RSQM_DIAG_SWITCHED_RC_SETTLING` at all: the same physical front-end transient occurred, with **no**
input-freeze protection (§2.6b). `_arm_switched_rc_settling()` now lives inside `setTIAGain()`/
`setTIAGainLED1/2()` themselves (guarded so re-applying the *same* value is a no-op, matching
`_hgac_change_rf()`'s own `new_rf == old_rf` early return) — `_hgac_change_rf()` gets it "for
free" through the setter call it already makes, so its own separate arm was removed.

**Why `_switched_rc_settling_countdown` is `volatile` (no mutex):** with the arm moved into the public
setters, it can now be written from a *different* thread than the one that reads/decrements it —
e.g. `Cmd_Task` processing a manual `$SET` (PulseNest `main.cpp`) races against `_task_body()`'s
own thread, which reads it in `_should_freeze_input()` and decrements it inside `_process_sample()`
(`_rsqm_update()`). A full mutex was considered and rejected: both sides only ever perform a plain
32-bit store or decrement, never a cross-thread read-modify-write, so the worst-case race outcome
is benign — at most ±1 sample of settle duration (a decrement overwritten by a fresh arm, or an
arm racing an in-flight decrement), never a corrupted or out-of-range value. `volatile` is enough
to prevent the compiler from caching a stale value across the loop iteration boundary — the same
pattern already used for `_diag_active`/`_diag_holdoff_samples` (§2.6b) for the identical reason
(`runAfeDiagnostics()` is likewise callable from a different task than `_task_body()`).

**Extended to ILED, AMBDAC, and RG (v0.76, Alex) — datasheet §7.7 t5 footnote (1) names all of
them.** The footnote reads *"applicable whenever there is a change to any of the signal chain
controls (for example, LED current setting, TIA gain, and so forth)"* — RF was the only one wired
up through v0.75. `setLED1Current()`/`setLED2Current()`, `setAmbDac()`, and
`setStage2Gain()`/`setStage2GainLED1/2()` now call `_arm_switched_rc_settling()` themselves too
(same no-op guard as RF: re-applying the same value doesn't re-arm). AMBDAC and RG (Stage 2) sit
in the identical feedback path as RF (Eq. 2, §3c) — a step in either produces the same kind of
front-end transient. `setTIACF*()` is **not** included: CF only shapes transient response, not the
DC operating point (Eq. 2 has no CF term — the steady-state output depends on `I_PD`, `RF`,
`I_CANCEL`, `RG`, none of which CF touches), so a CF-only change doesn't step the DC level the way
RF/ILED/AMBDAC/RG do. `setLEDRange()` (which shifts `TX_REF`) is a different, much longer (~1 s)
settling window (§7.7 t6 footnote (2)) that this mechanism does not cover at all — see the
settling catalog memory for that still-open item.

**`setStage2En1/2()` included too (v0.77, Alex).** Bypassing Stage 2 (`STAGE2EN=0` → unity gain)
vs. enabling it (`STAGE2EN=1` → gain `RG`) steps the effective gain exactly like `setStage2Gain()`
itself does — noticed in v0.76 and deliberately left for a follow-up decision rather than folded
in silently; now wired up the same way (guard: toggling to the *same* enable state is a no-op).

**Bug found and fixed (v0.79, Alex, from a real capture): freezing the raw codes wasn't enough —
the reconstruction was still combining stale codes with live parameters.** `_task_body()` freezes
the *raw ADC codes* (`_last_valid_led1/2/aled1/2`), but `_compute_analog_state()` reads the
*circuit parameters* (`_afe_tia_rf_led1/2`, `_afe_stg2_rg_led1/2`, `_afe_stg2_en_led1/2`,
`_afe_ambdac_uA`, `_afe_led1/2_current_mA`) as **live members** — which, for every frozen sample
after the first, are already the **new** values (the setter that armed the freeze updates them
immediately). Recomputing `V_TIA`/`I_PD`/`OT` from an OLD code with a NEW parameter produces an
**artificially stepped** value — e.g. `OT_led1` scaled by roughly `RF_old/RF_new` — not the flat
line the freeze was meant to produce. Symptom: `ppg_disp` showed a sharp pulse (two abrupt steps
bracketing the settle window) instead of the intended smooth, low-AC decay — and RSQM/SpO2/HR1-3
were silently exposed to the same wrongly-scaled `OT` for the whole window, not just the display.

**Two designs were considered:** (A) freeze the circuit parameters too (mirror
`_last_valid_led1/2/aled1/2` with `_last_valid_tia_rf_led1/2` etc., ~9 new members, thread them
through `_compute_analog_state()`'s signature); (B) freeze the **entire computed**
`AFE4490AnalogState`, and skip recomputation altogether while frozen. **B was chosen**: it fixes
the mismatch by construction (no partial-freeze state where some parameters are stale and others
live is even possible), needs one new member instead of nine, and leaves `_compute_analog_state()`
untouched. `_process_sample()` now branches on `_should_freeze_input()` itself: reuse
`_last_valid_analog_state` verbatim while frozen, or compute fresh and save it otherwise. As a side
effect, `ppg_disp`'s bandpass filter now receives a genuinely constant input during the freeze (not
a stepped one), which is what the original v0.72 design rationale ("absorbed with near-zero AC
contribution") always assumed but didn't actually deliver until now.

Proven with `test_analog_state_stays_flat_during_settle_window` (PulseNest `test_hgac.cpp`) the
strong way: feeds a **deliberately bogus** raw code while frozen and asserts `OT_led1` doesn't
move — if the bug were still present, recomputing that bogus code with the new RF would visibly
shift it.

Arms `_switched_rc_settling_countdown` via `_compute_switched_rc_settling_samples()` — the producer of
`RSQM_DIAG_SWITCHED_RC_SETTLING` (§5.6.3).

**Second bug found and fixed (v0.80, Alex, suspected from a real capture "a veces parece que
FW_OT_LED1 se congela con la RF nueva"): a cross-thread race, distinct from the v0.79 bug above.**
v0.79 fixed *recomputation* while frozen (reusing `_last_valid_analog_state` verbatim). It did not
cover how that snapshot gets *saved* in the first place. Every setter that arms the settling window
(`setTIAGainLED1/2()`, `setLED1/2Current()`, `setAmbDac()`, `setStage2Gain()`/`setStage2GainLED1/2()`,
`setStage2En1/2()`) ran, until v0.80: mutate the live circuit-parameter member, *then* call
`_arm_switched_rc_settling()`. That two-step sequence is protected only by `_spi_mutex` — never
`_state_mutex`, the lock `_process_sample()` runs under. A manual `$SET` reaches these setters from
`Cmd_Task`, a FreeRTOS task distinct from `_task_body()`'s. If `_task_body()`'s thread executes
`_process_sample()` in the gap between the parameter write and the arm call, `_should_freeze_input()`
still sees the countdown at 0 → takes the "compute fresh" branch → combines the *old* raw ADC code
(read moments earlier, still under the old analog state) with the circuit parameter, which is
*already new* → produces exactly the same kind of wrongly-scaled `AFE4490AnalogState` the v0.79 fix
eliminated for the *recompute-while-frozen* case — except here it gets written straight into
`_last_valid_analog_state` as the "good" value, and *that* is what the settling window then holds
constant for its whole duration. The HGAC-driven path was never exposed: `_hgac_change_rf()` mutates
and arms sequentially inside the very `_process_sample()` call that also computes/saves `as`, all
under `_state_mutex`, on a single thread — no second thread can interleave.

**Fix: reorder to arm-before-mutate in all six setters above.** Verified safe: the countdown's
*value* doesn't depend on which parameter changed (`_compute_switched_rc_settling_samples()` reads
only `_afe_sample_rate_hz`, confirmed in its own comment), only on *whether* it changed and *when*
it's armed relative to the mutation. With arm-first, `_should_freeze_input()` can only ever observe
"old parameter + not yet armed" or "new parameter + already armed" — the unsafe combination ("new
parameter + not yet armed") can no longer occur, because arming strictly precedes the write on every
code path, cross-thread or not. No new locks, no lock-order/deadlock risk (the alternative — taking
`_state_mutex` inside the setters — was rejected: `_hgac_change_rf()` already nests a setter call
under a caller-held `_state_mutex`, and `Cmd_Task` independently taking `_spi_mutex` then
`_state_mutex` would invert that order → real deadlock). See
`test_should_freeze_input`/`test_analog_state_stays_flat_during_settle_window` (PulseNest
`test_hgac.cpp`) for the existing single-thread coverage; the cross-thread race itself is not
exercised by the native test suite (single-threaded), so this fix relies on the ordering argument
above, not on a new failing-then-passing test.

**Settling window (v0.65): datasheet §7.7 t5, replacing the untraceable `afe_settle_time_s`.**
The datasheet's own row for "time between an SPI command and valid data" (§7.7, p.17) requires
**> 3 ms of cumulative sampling time in each [of the four switched-RC] phase[s]**, and footnote (1)
generalises it explicitly: *"The same time is applicable whenever there is a change to any of the
signal chain controls (for example, LED current setting, **TIA gain**, and so forth)."* This is the
only datasheet-traceable figure for this wait — unlike the three auto-CF constants (§7.2), it did
not need to be reverse-engineered, TI states it directly.

`_compute_switched_rc_settling_samples()` turns that into a sample count: since each phase delivers exactly
one sample per PRP cycle, `ceil(3 ms / phase_window_s)` cycles must elapse. The four phase windows
are pairwise symmetric (LED1≡LED2, ALED1≡ALED2) and depend only on `_afe_sample_rate_hz` — not on
RF — so a single number covers an RF step on either color domain. The ambient window is normally
the shorter of the two (its 400-count margin, §7.2, exceeds the LED window's `tia_margin`), so it
is normally the limiting phase; the function takes the min of both rather than assuming which.

At the default 500 Hz / 25% duty config: LED window ≈ 449.5 µs, ambient window ≈ 399.5 µs →
`ceil(3 ms / 399.5 µs) = 8` samples ≈ **16 ms** — about **9× shorter** than the old fixed 0.15 s,
and traceable to the datasheet instead of a guess (see §14 v0.65, and the `afe_settle_time_s`
provenance note in §7.2).

**CF/RF atomicity (reviewed 2026-08-20, no bug found):** a natural worry when RF changes at
runtime is whether `C_F` could transiently lag behind the new `R_F`, breaking the 5τ criterion.
It can't: `RF_LEDx` and `CF_LEDx` for one color share the **same 24-bit register** (`TIAGAIN` for
LED1, `TIA_AMB_GAIN` for LED2, datasheet p.57), and `setTIAGainLED1/2()` (which `_hgac_change_rf()`
calls) recomputes CF for the new RF (`_recalc_afe_tia_cf_led1/2()`) *before* `_apply_analog_regs()`
writes both fields in a single SPI transaction. The chip only ever sees (old RF, old CF) or
(new RF, new CF) — never a mix — and both writes happen under `_spi_mutex`, which `getConfig()`
also takes for these fields, so no torn read either. Locked in by
`test_hgac_change_rf_recalculates_cf` (PulseNest `test_hgac.cpp`).

**500 Hz post-stage2 filter pole vs. the settle window (reviewed 2026-08-21):** the `FLTRCNRSEL`
low-pass filter (Fig. 55, between Stage 2 and the sampler; corner 500 Hz always in this library,
`kAFE_FLTRCNRSEL`=0, §7.2) has its own settling time — `5τ = 5/(2π·500) ≈ 1.59 ms` — independent
of RF/CF. Computed numerically across the full documented `setSampleRate()` range (63–5000 Hz):

| fs (Hz) | `_compute_switched_rc_settling_samples()` window | vs. 5τ (1.59 ms) |
|---|---|---|
| 63–2000 (covers the 500 Hz default and everything the ambient-margin note below already scopes as valid) | 15–61.5 ms | **10×–39× margin** |
| ≥ ~2487 | collapses to 1 sample (≈ 0.2–0.4 ms) | **fails — see caveat below** |

For every sample rate this library is actually configured at (500 Hz, the only value used in this
project), the settle window covers the filter pole with roughly an order of magnitude to spare —
**closes this open question, no fix needed at the rates in use.**

⚠️ **Found while checking this: the settle-window calculation itself collapses above ~2487 Hz**
(within the documented valid range of `setSampleRate()`), which is a **more severe re-statement**
of the pre-existing "Valid for PRF ≤ ~2 kHz" caveat already on `ambient_margin` in
`_apply_timing_regs()` (§7.2) — not a new problem, but a newly-quantified one. Above that
threshold, `_compute_ambient_sample_window_counts()` returns 0 (the fixed 400-count ambient margin
exceeds the available per-phase window), which (a) makes `_compute_switched_rc_settling_samples()` hit its
`worst_window == 0` guard and return the floor of 1 sample — far below even the raw 3 ms datasheet
requirement it exists to satisfy, not just the filter's 5τ — and (b) at the register level,
`_apply_timing_regs()` would write `ALEDxSTC` (=400) *after* `ALEDxENDC` (=`q−2`, <400 in this
regime): an inverted timer-compare pair on real hardware, not just a software miscalculation.
Not fixed here — out of scope (this library only runs at 500 Hz in this project) — tracked in
memory (`project_high_prf_ambient_window_collapse_task`).

**Settling and EMA warmup are SEQUENTIAL, not overlapped (corrected 2026-08-16 — see §14):** the HGAC
gate (below) closes off whenever `RSQM_DIAG_SWITCHED_RC_SETTLING` is active, and while gated off it `reset()`s
the four EMAs **every sample** (not just once at the RF change) — so `count` stays pinned at 0 for
the entire settling window, and no `EmaChannel::update()` is ever called during it. The EMA warmup
counter (`count` vs. `3·τ·fs` in `valid()`) therefore does not start advancing until the settling
countdown reaches 0 and the gate re-opens. The total delay before a domain can act again after an
RF change is the **sum**, not the max, of the two windows (figures below at the default 500 Hz
config — recompute via `_compute_switched_rc_settling_samples()` at other sample rates):
- Guard (fast, τ=0.1s) operative again: settling (~16 ms) + 3×0.1 s = **~0.32 s** (was 0.45 s)
- Leveling/ambient (slow, τ=2.0s) operative again: ~16 ms + 3×2.0 s = **~6.02 s** (was 6.15 s)

**HGAC gate** (the design's *Gate G0*, from the RSQM/HGAC separation axiom A6 — the only gate; the numbering never grew): HGAC acts only while `probe_state` is `PROBE_APPLIED` or `PROBE_AMB_SATURATING` and no
`RSQM_DIAG_SWITCHED_RC_SETTLING` is active. The absent states are defined once, in
`isProbeAbsent()` (`PROBE_DISCONNECTED`, `PROBE_OT_HIGH`, `PROBE_ONLY_LED_SATURATING`), shared with
the diagnostics task (§5.7); a ProbeState missing from that list is NOT gated off. While gated off,
the EMAs are `reset()` so they re-warm cleanly when the probe is re-applied (otherwise stale
probe-in-air values would leak into the first decisions). `PROBE_AMB_SATURATING` (§5.6.2) is a
distinct state so a guard event can't flip to an absent state and deadlock the step-down.
**Open check (v0.90):** `PROBE_ONLY_LED_SATURATING` is absent by inheritance from NOT_APPLIED — with
thin tissue at high RF that closes the HGAC gate exactly when it should lower RF (§5.6.2).

**Concurrency:** `_hgac_update()` and everything it calls take no lock of their own — they run from
`_process_sample()`, already under `_state_mutex`. `_hgac_change_rf()` calls `setTIAGainLED1/2()`,
which take `_spi_mutex` internally; nesting `_spi_mutex` inside the caller's `_state_mutex` is safe
(no path takes `_spi_mutex` first then blocks on `_state_mutex`). **Contract:** the `_hgac_*`
actuation primitives are only ever called from `_hgac_update()`.

**Announcing the move (v0.94).** HGAC is the only thing that changes an AFE register without
the host asking, so until v0.94 a host recording the stream had no way to know RF had moved
except by reading `RF1,RF2` off every `$M4` row. `_hgac_change_rf()` now stamps
`_hgac_rf_change_ts_us = esp_timer_get_time()` and then increments `_hgac_rf_change_count`;
`hgacRfChangeCount(uint64_t* last_ts_us)` returns both. The library deliberately does **not**
emit anything itself: it runs in the measurement task, and PulseNest's rule since fw 0.13 is
that no measurement task calls the network — the consumer's command task polls the count and
sends whatever frame it likes (PulseNest fw 0.15: a `$CFG` with `cause=hgac`). The getter is
lock-free: the timestamp is written before the count, and read between two reads of the count
that must agree, so a reader never gets a torn 64-bit timestamp or a count ahead of its
timestamp. Manual `setTIAGainLED1/2()` calls are not counted — a `$SET` already answers with its
own `$CFG`, and counting it twice would tell the host a move happened that it itself made.

#### 5.8.5 Parameters (see §10.9) and telemetry

`AFE4490Config` fields: `hgac_enable`, `hgac_v_tia_high2`, `hgac_v_tia_high1`, `hgac_v_tia_low1`,
`hgac_ema_fast_tau_s`, `hgac_ema_slow_tau_s`, `hgac_ema_ambient_tau_s`. `$SET`/`$LCFG` wiring in PulseNest's `src/main.cpp`.
`$M4` carries `RF1,RF2` (current RF per domain, string), from `AFE4490DebugData`
`rf_led1`/`rf_led2` (§3d, renamed from `hgac_rf_led1/led2` in v0.81), populated alongside
`analog` in `_process_sample()`.

---

## 6. Hardware interface (SPI)

| Parameter   | Value                           | Source      |
|-------------|----------------------------------|-------------|
| Speed       | 2 MHz                           | Protocentral|
| Mode        | SPI_MODE0                       | Protocentral|
| Bit order   | MSBFIRST                        | Protocentral|
| Frame       | 1 address byte + 3 data bytes   | Datasheet   |

---

## 7. Constraints and rules

- **Never use `delay()`** — use `vTaskDelay()` with `pdMS_TO_TICKS()`
- **Thread-safe** — shared resources protected with mutex
- **SPI errors** — always check communication results
- **Medical device** — reliability is priority 1
- **Timing registers** — calculated automatically by the library from `setSampleRate()`. Not exposed to the user.
- **This spec must be updated** with any design change to the library, so that it can always regenerate the corresponding version

### 7.1 ADC Averaging — formula and constraints

Source: [AFE4490 datasheet](https://www.ti.com/lit/ds/symlink/afe4490.pdf) section 8.4.1, Equation 5.

Each ADC conversion phase has 25% of the repetition period (PRP). Each individual conversion takes 50µs. The maximum number of averages is limited by how many conversions fit in that window:

```
NUMAV_max = floor( (0.25 × PRP) / 50µs ) − 1
           = floor( 5000 / PRF ) − 1
```

| PRF (Hz) | PRP (µs) | 25% window (µs) | Max averages | Max NUMAV |
|---|---|---|---|---|
| 500 | 2000 | 500 | **10** | **9** |
| 625 | 1600 | 400 | 8 | 7 |
| 1000 | 1000 | 250 | 5 | 4 |
| 1250 | 800 | 200 | 4 | 3 |
| 2000 | 500 | 125 | 2 | 1 |
| 2500 | 400 | 100 | 2 | 1 |
| 5000 | 200 | 50 | 1 | 0 |

**Absolute hardware limit:** NUMAV ≤ 15. Any value ≥ 15 behaves as 16 averages. This limit is only relevant at very low PRF (< 313 Hz).

**`setAdcAverages(uint8_t num)` behaviour:**
- `num` is the number of samples to average (1 = no averaging, equivalent to NUMAV=0).
- The library computes internally `NUMAV = num − 1`.
- If `num > NUMAV_max + 1` for the current PRF: clamped to the maximum and `logE` emitted with the actual applied value.
- If `num == 0`: treated as `num = 1` (no averaging).
- `setSampleRate()` recalculates NUMAV_max; if the configured NUMAV exceeds the new maximum, it is automatically clamped and `logE` is emitted.

**Effect on SNR:**
```
SNR_improvement = sqrt(num)    →  num=8: ×2.83,  num=10: ×3.16
```

**Default:** `num = 8` (NUMAV=7). Balance between SNR and latency at 500Hz.

### 7.2 Auto-CF — TIA feedback capacitance selection

The TIA RC time constant `τ = RF × CF` determines how long the output takes to settle after the LED turns on. The ADC sampling window opens `tia_margin` counts after LED turn-on. For reliable readings the TIA must be settled to < 0.7% error, which requires ≥ 5τ:

```
5 × RF × CF ≤ settle_time_s
CF ≤ settle_time_s / (5 × RF)
```

The settle margin is computed as:
```
q            = afeclk / (4 × Fs)          [LED-on window in counts]
tia_margin   = max(50, floor(q × 0.10))   [10% of LED window, floor 12.5 µs]
settle_time_s = tia_margin / afeclk
```

This margin is used both by `_apply_timing_regs()` (to set LED2STC/LED1STC) and by `_recalc_afe_tia_cf_led1/2()` (to select CF). Both functions always use the same value — they are in sync.

**`_recalc_afe_tia_cf_led1/2()`** converts the budget to picofarads and delegates to `afeCFPFToCode()`, which returns the largest of the **32** achievable capacitances that satisfies the constraint:

```cpp
float cf_max_pF  = (settle_s / tia_n_tau / RF) * 1e12f;
_afe_tia_cf_led1 = afeCFPFToCode(cf_max_pF);     // quantise DOWN
```

Selection and the manual setter therefore share one rule and cannot disagree. Called automatically from `_recalc_rate_params()` (triggered by `setSampleRate()` and the constructor) and from `setTIAGain*()`. `setTIACF*()` can override afterwards.

> **Float note (v0.62):** `cf_max_pF` lands a few ULP below the exact grid value — at 500 Hz / RF_100K it computes 99.9999924, not 100.0. Without `kAFE_CF_MATCH_TOL` the selector would drop a whole step (95 pF) and silently negate the benefit of exposing the full grid. The tolerance is applied inside `afeCFPFToCode()`, so it protects both paths.

**CF selection table** (auto-CF output, generated from the library):

| RF | 125 Hz | 250 Hz | 500 Hz *(default)* | 1000 Hz |
|---|---|---|---|---|
| RF_10K  | 250 pF | 250 pF | 250 pF | 250 pF |
| RF_25K  | 250 pF | 250 pF | 250 pF | 200 pF |
| RF_50K  | 250 pF | 250 pF | 200 pF | 100 pF |
| RF_100K | 250 pF | 200 pF | **100 pF** | 50 pF |
| RF_250K | 160 pF | 80 pF  | 35 pF  | 20 pF |
| RF_500K | 80 pF  | 35 pF  | 20 pF  | 10 pF |
| RF_1M   | 35 pF  | 20 pF  | 10 pF  | 5 pF  |

Lower Fs → wider LED window → larger settle margin → larger CF (more noise filtering).

**Change vs. v0.61** (which could only pick from six single-bit settings): at the default operating point (500 Hz / RF_100K) auto-CF now selects **100 pF instead of 55 pF** — an exactly reachable, legal register setting that was previously unrepresentable. Every entry in the table above either improves or is unchanged; none violates the 5τ budget.

**TIA pole frequency** (for reference, not used in the calculation):
```
f_pole = 1 / (2π × RF × CF)
```
At 500 Hz / RF_100K / 100 pF: f_pole ≈ 15.9 kHz — well above the PPG signal band (0.5–20 Hz).

#### Measured behaviour on IncuNest 16.A (2026-08-17) — CF has no observable effect

Bench measurements at RF_100K / 500 Hz, sweeping CF across its **full range (5 pF → 250 pF, a factor of 50)**, with HGAC disabled so RF stayed fixed:

| Quantity | CF = 5 pF | CF = 250 pF | Verdict |
|---|---|---|---|
| V_TIA (all four channels) | ~0.5 V | ~0.5 V (±1 mV, i.e. setup drift) | no effect |
| `LED1_SUB` % SD/Mean | 0.01 | 0.01 | no effect |

Write path verified by a control: changing RF 100K → 50K halved V_TIA as expected, and RF lives in the same two registers (TIAGAIN / TIA_AMB_GAIN) in adjacent bit fields — so the CF bits are reaching the chip and the null result is physical.

**Why:** the AFE's internal low-pass filter (`FLTRCNRSEL` = 500 Hz) sits between Stage 2 and the sampler and dominates the noise bandwidth. The TIA pole ranges from 318 kHz (5 pF) to 6.4 kHz (250 pF) — always far above the 500 Hz corner — so CF cannot narrow a bandwidth that the internal filter has already set. (An earlier version of this paragraph added "and because that filter precedes the sample-and-hold, there is no out-of-band noise left to alias either". That coda was wrong: a first-order pole leaves out-of-band noise to fold. The residual aliasing is small but not zero, and it is quantified in §7.3 — which does not change this CF conclusion.)

**Consequences:**
- The claim that a larger CF buys meaningful noise filtering **is not supported by measurement** in this configuration. v0.62's value is that it fixes an incomplete register grid, not that it improves SNR.
- Settling is not the binding constraint either: the ADC sample phase is a ~449.5 µs window and the held value reflects its end, so the TIA has margin + window ≈ 500 µs — not the ~50 µs pre-window margin the auto-CF criterion assumes. At RF_100K / 250 pF the settling residual is ≈ e⁻²⁰ (~1 nV), which is why 250 pF reads identically to 5 pF.
- The library's criterion (`5τ ≤ tia_margin`) is **≈4.5× stricter than the datasheet's own Equation 1** (`RF × CF ≤ Rx Sample Time / 10`):

  ```
  library:  5τ ≤ 0.10·q      → τ ≤ 0.02·q = 40 counts = 10 µs
  Eq. 1:      τ ≤ window/10    window = q−2−margin = 1798 counts → τ ≤ 44.95 µs
  ratio ≈ 4.5×   (uniform across RF — both bounds scale as 1/RF)
  ```

  At RF_100K / 500 Hz Eq. 1 would permit up to ~449 pF, i.e. the whole grid.

#### Two constraints, not one — do not conflate them

The `/10` in Equation 1 and the `0.10` in `tia_settle_fraction` are numerically similar and govern **different decisions**. Substituting one for the other is a category error:

| Decision | Correct basis | Depends on |
|---|---|---|
| **When the sample window opens** (`LEDxSTC = q + margin`) | LED and cable turn-on transient — datasheet §8.3.1.3: *"To avoid settling effects resulting from the LED or cable, program S_LED2 to start after the LED turns on."* | the LED and the wiring — **not** on RF or CF |
| **How large CF may be** | Equation 1: `τ ≤ Rx Sample Time / 10` | RF and the sample window |

Note the datasheet attributes the sample-start delay to **LED/cable** settling, not TIA settling. The library sizes that margin from a TIA-settling rationale (5τ) and then reuses it as the auto-CF budget — which is why the two constraints became entangled. Eq. 1 cannot replace `tia_settle_fraction`: it says nothing about when the window should open, so substituting it would leave the margin with no criterion at all.

**Decision: both numbers are kept as they are.** The margin is conservative but free (sampling here is not window-limited — see the measurements above), and shrinking it risks sampling before the LED has stabilised, which *would* bias the reading. Re-deriving it properly would require measuring the LED/cable turn-on transient, which has not been done.

#### Equation 1 is checked, never enforced

`_recalc_afe_tia_cf_led1/2()` applies only the 5τ rule, which is stricter, so **auto-CF can never violate Eq. 1** (locked by `test_autocf_always_within_eq1` across every RF and rate). A **manual `setTIACF*()` can**: it bypasses auto-CF and is deliberately not clamped. Example at 500 Hz: `RF_250K` + 250 pF gives RF×CF = 62.5 µs against the 44.95 µs limit.

`getCFMaxEq1PF(rf)` returns that limit so callers can detect it. PulseNest's firmware calls it after every manual `$SET tiacf*` and emits

```
# WARN cf1=250 pF exceeds datasheet Eq.1 limit 180 pF (RF=250K) - applied anyway
```

as a comment line, not `$ERR` — the setting *is* applied. Rationale for warning rather than clamping: manual override exists precisely for bench experiments that need to leave the documented range, and silently clamping would make such an experiment lie about what the chip was doing.

#### Provenance of the three settle constants — none is from the datasheet

All three entered together in v0.20 (`incunest_afe4490.cpp`, anonymous namespace). None has a basis in the AFE4490 datasheet; this is recorded so nobody re-derives them from a source that does not exist, or "aligns them with TI" in the wrong direction.

| Constant | Value | Actual provenance |
|---|---|---|
| `tia_settle_min` | 50 counts (12.5 µs) | Matches the example value in datasheet Table 2 (p.36: `t1 = 6050` = `t3` + 50 at PRF 500 Hz / 25% duty), adopted as a floor after comparing the TI EVM (80 counts, judged excessive) and Protocentral (0, no guard). An example value, not a stated requirement. |
| `tia_settle_fraction` | 0.10 | **Probably Equation 1's `/10` applied to the wrong quantity** — turned from *"τ ≤ 10% of the sample window"* into *"reserve 10% of the LED window as dead time"*. The two are nearly the same size (0.10·q = 200 counts vs 0.10·window = 179.8), which is why the substitution is easy to make and hard to notice. Added in v0.20 to make the fixed 50-count margin scale with PRF. |
| `tia_n_tau` | 5.0 | The generic **five-time-constants rule of thumb**. The code comment *"5τ → < 0.7% settling error"* is not a derivation: 0.7% **is** e⁻⁵ (0.674%) restated, so it states the consequence as though it were the requirement. The datasheet never mentions 5τ. |

**The datasheet's own implied time-constant count is 10, not 5** — Equation 1 (`τ ≤ Rx Sample Time / 10`) demands the window be at least 10τ, i.e. settling to e⁻¹⁰ ≈ 45 ppm. The library is therefore *laxer* in τ-count (5 vs 10) but applies it against a far shorter interval (the 50 µs margin instead of the 449.5 µs window). Net: 4.5× stricter.

⚠️ **The three constants only make sense together.** Changing one in isolation moves the operating point, sometimes opposite to the intent:

| Criterion | τ max | CF max at RF_100K |
|---|---|---|
| Current (`5τ ≤ margin` = 50 µs) | 10 µs | **100 pF** |
| Raise `tia_n_tau` 5 → 10 "to match TI", keeping the margin basis | 5 µs | 50 pF — *twice as strict* |
| Datasheet Eq. 1 (`10τ ≤ window`) | 44.95 µs | 449 pF |

None of this is wrong or unsafe — the combination errs conservative, which is the right side to err on — but the justification is weaker than the code comments suggest. See the decision above: all three are kept unchanged.

#### `tia_settle_min` does not depend on PRF — and that is almost certainly correct

Reviewed 2026-08-21 (Alex): `tia_settle_min` is a flat `constexpr` (50 AFECLK counts), not a
function of `_afe_sample_rate_hz`. Since AFECLK is fixed at 4 MHz regardless of PRF, 50 counts is
also a fixed **12.5 µs in wall-clock time** at every sample rate — it only ever "wins" the
`max(tia_settle_min, tia_settle_fraction × q)` at high PRF, where `q` (which does scale with PRF)
shrinks below it.

**Table 2's own footnote (1), p.36, sharpens the provenance above:** *"Values are based off a
pulse repetition frequency (PRF) = 500 Hz and duty cycle = 25%."* Every number in that table —
including the `t1 = t3 + 50` example `tia_settle_min` was lifted from — is TI's worked example at
one specific PRF, not a stated rule for any other rate. TI never says "always reserve 50 counts";
that is the library's own extrapolation from a single data point.

**Despite that weak origin for the exact number, keeping the *shape* PRF-independent (a fixed
count, not a fixed fraction of the window) is the physically sound choice.** The stated purpose of
this margin is LED/cable turn-on settling (§8.3.1.3) — a property of the LED and the wiring's own
electrical time constants, which does not care how often the AFE pulses them. A fixed wall-clock
floor models that correctly; a floor that shrank at high PRF (e.g. if it were expressed as a
fraction of `q` instead of a count) would not. So: the formula's structure (fixed floor +
PRF-proportional term, combined via `max()`) is well-designed; only the specific magnitude "50" is
inherited from one PRF-500 Hz data point TI never validated elsewhere — the same class of weak
justification already recorded above for `tia_settle_fraction`/`tia_n_tau`, now with the precise
textual reason (the table's own scope footnote) why it is single-PRF and not a general rule.

#### `afe_settle_time_s` (v0.65: replaced, not just documented)

Unlike the three constants above, `afe_settle_time_s` (0.15 s, arming `RSQM_DIAG_SWITCHED_RC_SETTLING`
after an HGAC RF change, §5.8.4) had **no derivation at all**, traceable or not: it entered in
v0.37 (HGAC's first commit) as a bare guess, and its own code comment said so explicitly
("approximate... NOT derived from `_compute_settle_margin()`"). Rather than record its provenance
as "none" and keep it, v0.65 replaces it outright with `_compute_switched_rc_settling_samples()`, built from
datasheet §7.7 t5 — see §5.8.4 for the full derivation. The two are unrelated in origin and in
magnitude (0.15 s fixed vs. ~16 ms computed at 500 Hz): the old value was conservative by
coincidence, not by design.

**Decision: keep the conservative criterion.** Relaxing it would buy nothing measurable while spending settling margin, and the margin is free. Revisit only if a configuration is found where CF demonstrably affects noise — e.g. with `FLTRCNRSEL` raised to 1000 Hz, a much higher RF, or a probe/cable whose capacitance makes TIA loop stability the limiting factor (the one CF-related risk these measurements did *not* probe, since no ringing was observed even at 5 pF).

### 7.3 Anti-aliasing — the effective Rx bandwidth is the duty-cycled corner, not 500 Hz

Analysis 2026-09-04, from the datasheet only (no bench measurement yet). **No code change:**
`kAFE_FLTRCNRSEL = 0` (500 Hz corner) is confirmed correct at PRF = 500 Hz.

**The chain.** TIA → ambient-cancellation DAC + Stage 2 → low-pass filter (500 Hz or 1 kHz, set by
`FLTRCNRSEL`) → sample-and-hold onto CLEDx → 22-bit SAR ADC (datasheet §8.3.1.1 and §8.3.1.3:
*"the gained-up pleth signal is then low-pass filtered (500-Hz bandwidth) and buffered before
driving a 22-bit ADC"*). That filter does precede the sampler and is the **only** anti-aliasing
filter in the chain — there is nothing else between Stage 2 and the S/H.

**But 500 Hz is not the corner that acts.** Each phase tracks the filter output only during its own
Rx sample window and holds for the rest of the PRP. Duty-cycling a single-pole RC stretches its
effective time constant by 1/D, so the corner of the *sampled* channel is
`f_c,eff ≈ f_c,nominal × D`. The datasheet documents this behaviour in two places:

- **Figure 54, "Filter Response vs Duty Cycle" (p.26)** — −3 dB at ≈ **80 Hz** for 25 % duty and
  ≈ **16 Hz** for 5 % duty. The 5:1 ratio matches the 5:1 duty ratio exactly, confirming
  `f_c,eff ∝ D`; the absolute value is ~0.65× the naive `500 × 0.25 = 125 Hz`. Roll-off is first
  order (−20 dB/dec).
- **§7.5 Electrical Characteristics, low-pass filter pass-band attenuation (2–10 Hz):** 0.004 dB at
  25 % duty vs 0.041 dB at 10 % duty — the same duty dependence, and proof that the PPG band itself
  is untouched.

**Margin at the operating point.** At PRF = 500 Hz the library's LED sample window is 449.5 µs of
the 2000 µs PRP (D = 22.5 %) and the ambient window is 399.5 µs (D = 20 %) — see §11.1. Hence:

| Quantity | Value |
|---|---|
| `f_c,eff`, LED phases (D = 22.5 %) | ≈ 70–110 Hz |
| `f_c,eff`, ambient phases (D = 20 %) | ≈ 65–100 Hz |
| Nyquist (`fs` = PRF = 500 Hz) | 250 Hz |
| Anti-aliasing margin | **≈ 2.3–3.6×** |

(The range spans the Figure-54 reading and the naive `f_c × D`; the two bracket the true value.)

**Consequences:**

1. **Signal aliasing: none.** PPG content lives below ~20 Hz, where attenuation is < 0.1 dB in both
   phases. The small LED-vs-ambient bandwidth difference (22.5 % vs 20 % duty) is irrelevant there,
   so the digital `LEDx − ALEDx` subtraction is not distorted by it.
2. **Broadband noise aliasing: present but negligible.** A first-order pole at ~100 Hz leaves ~24 %
   of the total noise power above Nyquist, which folds back white across 0–250 Hz. After the
   downstream 0.5–5 Hz digital band-pass, that adds ≈ 15 % to the in-band noise density
   (≈ +0.6 dB in power, ≈ +7 % in RMS). Not worth acting on.
3. **Deterministic out-of-band interference: the real residual risk.** Optical or electrical
   interference near `k × 500 Hz` folds into the pulse band attenuated only by a first-order pole
   (≈ −14 dB at 500 Hz, ≈ −20 dB at 1 kHz). **Ambient subtraction does not cancel it:** the LED and
   ambient phases are sampled a quarter period apart (500 µs = 90° at 500 Hz), so a tone at that
   frequency arrives with different phase in each. Relevant in an incubator — LED phototherapy PWM
   drivers and switched heaters are exactly this kind of source. Not yet measured.
4. **Do not raise `FLTRCNRSEL` to 1 kHz.** It would double `f_c,eff` to ~140–220 Hz, i.e. essentially
   Nyquist at PRF 500 Hz, converting a 2.3–3.6× margin into ~1×. Its only benefit is a shorter
   post-diagnostics settling (16 ms vs 28 ms, §7.5) — see `kAFE_DIAG_HOLDOFF_MS`.
5. **This constrains the sample-rate choice.** `f_c,eff` depends on the corner and the duty, both
   fixed — it does **not** scale with PRF. So the anti-aliasing margin is `PRF / (2 · f_c,eff)` and
   grows linearly with PRF: 250 Hz → no margin (Nyquist sits on `f_c,eff`); 500 Hz → ~2.3–3.6×;
   1000 Hz → ~4.5–7×. **500 Hz is the floor**, and lowering PRF to buy ADC averaging (§7.1, ×1.26 at
   best, saturating at ≤ 312 Hz on the NUMAV ≤ 15 hardware limit) trades a real margin for a
   marginal SNR gain. Bad trade.
6. **NUMAV averaging cannot help with aliasing.** The ADC converts the voltage already held on
   CLEDx (§8.3.1.3), so averaging reduces ADC and quantisation noise, not aliased input noise.

**Open (bench, not blocking):** measure the actual folded interference with the probe under an LED
phototherapy lamp and under a PWM-driven heater, sweeping the interferer near 500 Hz and 1 kHz, and
confirm the pulse band stays clean. This is the only claim here that rests on theory rather than
measurement.

### 7.4 The PRF catalogue — a closed set, enforced by the type system

PRF = 500 Hz is the default and the rate every accuracy claim is declared against. Since **v0.85**
the supported rates are a **closed catalogue** (`AFE4490SampleRate`), not a range.

**Three arithmetic conditions close the candidate set:**

1. **`q` must be an integer.** `q = (afeclk / PRF) / 4` in integer arithmetic, so unless PRF divides
   10⁶ the real PRF is not the requested one (550 → 550.1, 750 → 750.2, 1500 → 1501.5 Hz). The error
   is < 0.1 %, but every τ and α in `_recalc_rate_params()` is computed from the *nominal* value, so
   the bias propagates into all EMAs and biquads.
2. **PRF must be a multiple of 50**, so both chains' 50 Hz targets stay exact with an integer decimation factor
   (`decim = PRF/50`) and HR2/HR3 keep their window lengths, lag resolution and bin resolution.
3. **Both sample windows must stay ≥ 50 µs** (datasheet §8.3.1.1). The binding one is the *ambient*
   window, giving a ceiling of **≈ 1660 Hz** (`q ≥ 602`).

**Catalogue: 500, 800, 1000, 1250, 1600 Hz** — `AFE4490SampleRate::HZ_500 … HZ_1600`, with
`kAFE_SAMPLE_RATE_HZ[]` as the single source of truth.

#### API and why a catalogue rather than a range

```cpp
void setSampleRate(AFE4490SampleRate rate);   // primary form: an unsupported rate is inexpressible
void setSampleRate(uint16_t hz);              // bench / $SET sr: accepts ONLY catalogue values
static constexpr uint8_t  sampleRateCount();
static constexpr uint16_t sampleRateHz(AFE4490SampleRate r);
static bool isValidSampleRateHz(uint16_t hz, AFE4490SampleRate* out = nullptr);
```

1. **Certifiability.** A finite, enumerable set can be validated *exhaustively*: every supported
   rate is a test case. `test_sample_rate` walks the whole catalogue asserting the four invariants
   below, which are exactly what a validation report has to state. A continuous range cannot be
   covered that way, only argued about. This is the concrete present-day benefit; the rest follows
   from it.
2. **It closes a live defect.** `setSampleRate()` used to accept 63–5000 Hz while checking only the
   numeric bounds. Above ~2487 Hz it wrote **inverted timer-compare pairs** (`ALEDxSTC` >
   `ALEDxENDC`) into the chip — from a perfectly legal API call, with no error — and below 500 Hz
   the anti-alias margin collapses (§7.3). Both ends are now closed.
3. **The acceptance policy lives in exactly one place**, `isValidSampleRateHz()`, and the numeric
   form defers to it.

**Per-entry guarantees** (asserted for every catalogue value by `test_sample_rate`):

| Invariant | Why it matters |
|---|---|
| `q = (afeclk/hz)/4` exact | the real PRF equals the requested one, so every τ and α derived from the nominal rate is correct |
| `hz % 50 == 0` | both analysis chains get an **integer** decimation factor and hold 50 Hz (§5.3.1) |
| both sampling windows ≥ 50 µs | TI's minimum (datasheet §8.3.1.1); the ambient window is binding and sets the ~1660 Hz ceiling |
| `NUMAV_max ≥ 1` | at least one ADC conversion fits in the window |

#### Changing the rate at runtime (v0.86)

A PRF change is a mutation of the signal chain, not a constant recalculation. `setSampleRate()`
therefore performs a full teardown, because recalculating coefficients does **not** fix state
accumulated at the previous rate:

| What | Why recalculation is not enough |
|---|---|
| Biquad state (`_ppgdisp_bpf`, `_hr2_bpf`, `_hr3_bpf`) | `init_bp()` / `init_lp()` explicitly leave `_v1`/`_v2` untouched — new coefficients would run on old state |
| EMA state (2 SpO2 + 6 HGAC) | `EmaChannel::init()` recomputes α but keeps the mean/var accumulated with the old α |
| `_hr2_buf` / `_hr3_buf` | hold samples decimated at the old rate: the 8 s / 10.24 s windows would mix two rates until they refill |
| Decimation phase | mid-cycle for a factor that just changed |
| Switched-RC settling | all four sampling windows are rewritten by `_apply_timing_regs()` — a signal-chain change in the sense of datasheet §7.7 t5 |

Implementation notes:

* **No-op on a no-change call.** Setting the rate it already has returns immediately; otherwise a
  redundant call would drop a full analysis window and re-arm settling for nothing.
* **Lock order `_spi_mutex` → `_state_mutex`.** This is the only place in the library holding both
  at once (`getConfig()` and `_task_body()` take them sequentially, never nested), so it defines
  the order. Do not invert it.
* **Settling is armed AFTER `_reset_algorithms()`**, which ends by clearing the countdown. That is
  safe *here* — and only here — because `_state_mutex` is held across the whole mutation, so
  `_process_sample()` cannot observe an intermediate state; the arm-before-mutate contract of
  §5.8.4 exists for the setters that do **not** hold it.
* Pinned by `test_rate_change_tears_down_old_state` and `test_same_rate_is_a_no_op`.

**HR1's moving-average capacity (resolved in v0.86b).** `hr1_ma_max_len` was 64 samples long, enough only
up to 640 Hz, so `_hr1_ma_len = round(fs / (2·hr1_ma_cutoff_hz))` clamped at four of the five
catalogue entries — effective cutoff 6.2 Hz at 800 Hz, 7.8 at 1000, 9.8 at 1250 and 12.5 at
1600 Hz instead of the configured 5 Hz — **and clamped silently**. A too-wide filter lets the
dicrotic notch reach HR1's threshold detector, which double-counts it and reports a doubled HR;
the 0.185 s refractory mitigates that unevenly and least of all in bradycardia, where the notch
arrives later and falls outside it.

Now `hr1_ma_max_len` is **160**, the worst case of the catalogue at the default cutoff
(1600 / (2·5)). The cost is RAM only — **+384 B**, measured: firmware RAM 60 444 → 60 828 B of
327 680 — because the average is incremental, so CPU is O(1) in N, and group delay stays ≈
1/(4·cutoff) ≈ 50 ms at any PRF since N scales with the rate.

No fixed capacity can cover every configuration: `setHR1MaCutoffHz()` is public and unvalidated,
so a 2 Hz cutoff at 1600 Hz would need 400 samples. That case now emits **`logE`** naming the length
needed, the length available and the resulting effective cutoff — a clamped filter is diagnosable
instead of mute. Pinned by `test_hr1_ma_not_clamped_across_catalogue`.

**If this ever becomes a range.** A `FractionalDecimator` (interpolate → filter → decimate) would
remove the `hz % 50` condition, which is the only one that excludes an otherwise valid rate: **625
Hz** is the single PRF with an exact `q`, 100 % ADC utilisation and valid windows that the
catalogue rejects today. Widening then means adding entries to `kAFE_SAMPLE_RATE_HZ[]`, or relaxing
that one predicate in `isValidSampleRateHz()`. **No public signature changes and no caller breaks** —
which is why the numeric overload was kept rather than deleted.

| PRF | q | win LED | win AMB | num | conv/s | anti-alias margin | atten. at f = PRF | samples/s |
|---|---|---|---|---|---|---|---|---|
| 250 | 4000 | 899.5 µs | 899.5 µs | 16 | 16000 | **1.1–1.7×** | −8…−11 dB | ×0.5 |
| **500** | 2000 | 449.5 µs | 399.5 µs | 10 | 20000 | 2.2–3.4× | −13…−17 dB | ×1.0 |
| 800 | 1250 | 280.8 µs | 212.0 µs | 6 | 19200 | 3.6–5.5× | −17…−21 dB | ×1.6 |
| 1000 | 1000 | 224.5 µs | 149.5 µs | 5 | 20000 | 4.5–6.9× | −19…−23 dB | ×2.0 |
| 1250 | 800 | 179.5 µs | 99.5 µs | 4 | 20000 | 5.6–8.6× | −21…−25 dB | ×2.5 |
| 1600 | 625 | 140.2 µs | 55.8 µs | 3 | 19200 | 7.1–11× | −23…−27 dB | ×3.2 |

**Raising the PRF buys** margin against the one residual risk of §7.3 — since `f_c,eff` does *not*
move (73–112 Hz; duty is 22.5 % across the whole table), a higher PRF pushes the critical interferer
at `f ≈ PRF` further down the roll-off, **+6 dB per octave** — plus better rejection of dynamic
ambient (subtraction residue `2·sin(π·f·Δt)`, Δt = ¼ PRP: −10.1 dB at 100 Hz for 500 Hz vs −18.0 dB
for 1250 Hz).

**Raising the PRF costs** CPU and transport linearly (×2 at 1000 Hz: 2 ms → 1 ms per sample, ~100 →
200 UDP pkt/s), squeezes the ambient window towards its fixed 400-count margin (at 1250 Hz the usable
window equals the margin), and — the strongest argument — **eats the thermal headroom**:

**The PRF bounds the LED duty-cycle lever.** At a fixed duty the PRF is thermally neutral
(`P_avg = I_F · V_F · duty`), but the floor under the duty is fixed *in time* — 12.5 µs of LED/cable
settling plus TI's 50 µs minimum Rx window = 62.5 µs — so the minimum reachable duty scales with PRF:

| PRF | PRP | min. duty | headroom below the present 25 % |
|---|---|---|---|
| **500** | 2000 µs | **3.1 %** | **8.0×** |
| 800 | 1250 µs | 5.0 % | 5.0× |
| 1000 | 1000 µs | 6.3 % | 4.0× |
| 1250 | 800 µs | 7.8 % | 3.2× |
| 1600 | 625 µs | 10.0 % | 2.5× |

Driving `I_F` to 100 mA at equal skin temperature would need a 12.5 % duty: comfortable at 500 Hz,
impossible at 1600 Hz. Trimming the duty is nearly free on the signal side — `f_c,eff = 500 Hz × D`
would fall to ~36–56 Hz at 12.5 % duty, still far above the PPG band (−0.3 dB at 10 Hz) and
*improving* the anti-aliasing margin — but **the ambient window must be trimmed alongside it** even
though thermals do not require it, or the LED and ambient phases end up with different bandwidths and
the digital subtraction leaves a high-frequency noise residue.

**Two non-criteria, recorded so they are not re-derived:**

- **`5000/PRF` being an integer is not a requirement.** It is worth 0.18 dB
  (`sqrt(20000/19200)` = 2 % noise), and it is contingent on the conversion window staying at 25 % of
  the PRP — it evaporates if that window is ever trimmed. 800 and 1600 Hz are *not* penalised by it.
  What is structural is the collapse below ~312 Hz, where the `NUMAV ≤ 15` hardware cap cuts
  averaging (250 Hz → 16000 conv/s → +0.97 dB).
- **ADC SNR does not depend on the PRF.** Conversions per second = `4 · num · PRF ≈ 20000`, constant
  across the useful range: what the hardware stops averaging as the PRF rises, the downstream digital
  filtering averages instead. Lowering the PRF to "buy averaging" gains nothing above 312 Hz and
  *loses* conversion capacity below it.

**Everything below 500 Hz is ruled out:** at 250 Hz the anti-aliasing margin is 1.1–1.7× (§7.3),
ambient subtraction drops to −4.2 dB and 20 % of the ADC capacity is wasted, with no compensating
advantage.

**Decision: keep 500 Hz.** Lowest CPU cost, largest thermal headroom, and an anti-aliasing margin
that is adequate unless bench measurement finds real folded interference. **1000 Hz is the reserve
step** if it does. Prerequisite before any increase: measure the real cost of `_process_sample()`
(`INCUNEST_TIMING_STATS`, §8.4) — and make the HR2/HR3 decimation adaptive first, or raising the PRF
silently changes their analysis windows.

---

## 8. Validation tooling — pulsenest_lab.py

The `pulsenest_lab.py` script is the primary validation tool for the incunest_afe4490 library.
It contains two distinct categories of windows with different purposes and lifecycles.

### 8.1 LAB windows (pre-implementation)

| Window | Purpose |
|---|---|
| `HR3LAB` | FFT + HPS algorithm design |
| `HRLab` | Autocorrelation HR2 algorithm design |
| `SpO2Lab` | SpO2 calibration and R-ratio curve fitting |

LAB windows exist **before** an algorithm is implemented in firmware. They are exploratory: free to experiment, iterate, and prototype ideas. They have no obligation to match the firmware — they are the design space where algorithms are conceived. Once an algorithm is finalised and implemented in firmware, its LAB window may be kept or retired, but it is not the verification tool.

### 8.2 TEST windows (post-implementation)

| Window | Algorithm under test |
|---|---|
| `SPO2TEST` | SpO2: AC/DC ratio, R, SQI |
| `HR1TEST` | HR1: threshold peak detection |
| `HR2TEST` | HR2: autocorrelation |
| `HR3TEST` | HR3: FFT + HPS |

TEST windows exist **after** an algorithm is implemented in firmware. Each TEST window contains an independent Python reimplementation of the exact algorithm described in this spec — same constants, same formulas, same state machine. The Python mirror is derived from this spec, not from the firmware source code.

**The spec is the contract.** Both the firmware and the Python mirror must implement this spec. Any discrepancy between the firmware output and the Python mirror output has exactly three possible causes:
1. Bug in the firmware implementation.
2. Bug in the Python mirror.
3. Ambiguity in this spec (interpreted differently in C++ vs Python).

This three-way relationship (spec → firmware, spec → Python mirror, firmware ↔ Python mirror) provides a rigorous independent verification mechanism appropriate for a medical device.

### 8.3 TEST window design rules

- **Parameters:** each TEST window exposes all algorithm constants as editable controls, with the firmware default values pre-loaded. The user can modify parameters to explore sensitivity and understand algorithm behaviour. This does not change the firmware — it only affects the Python mirror. A persistent status indicator signals whether the comparison is valid:
  - **Green — `FIRMWARE DEFAULTS`**: all parameters match the firmware defaults → comparison between firmware output and Python mirror is meaningful.
  - **Orange — `CUSTOM PARAMS`**: one or more parameters have been modified → the Python mirror no longer replicates the firmware; the window operates in exploratory mode. A `RESET TO DEFAULTS` button restores all parameters to firmware defaults.
- **Data sources:** each TEST window supports two modes:
  - **Live mode:** feeds from the active serial connection at 500 Hz (full rate, before decimation).
  - **Offline mode:** loads a recorded CSV file, processes it in batch, and displays the full time series as a static zoomable plot.
- **Comparison:** each window displays firmware output (received over serial) vs Python mirror output side by side, with a delta channel and a pass/fail indicator.
- **No code reuse from LAB windows:** TEST windows implement their algorithms from scratch following this spec. LAB windows may have evolved away from the final spec during the design phase; reusing their code would introduce uncontrolled divergence.

### 8.4 TIMING window (diagnostics)

The `TIMING` window in `pulsenest_lab.py` displays per-algorithm CPU execution times measured in firmware, parsed from `$TIMING` diagnostic frames.

**Compile-time flag:** `INCUNEST_TIMING_STATS` (default 0). Set to 1 by the consumer's build — PulseNest: `idf_build_set_property(COMPILE_DEFINITIONS "INCUNEST_TIMING_STATS=1" APPEND)` in its root `CMakeLists.txt` (project-wide, the library compiles it in); a PlatformIO consumer would use `build_flags`. When 0, all instrumentation code is compiled out with zero overhead.

**Frame format:**
```
$TIMING,hr1_mean,hr1_max,hr2_mean,hr2_max,hr3_mean,hr3_max,
        spo2_mean,spo2_max,cycle_mean,cycle_max,
        hr2cmp_mean,hr2cmp_max,hr3cmp_mean,hr3cmp_max,stack_free,
        spi_mean,spi_max*XX
```
The last pair (`spi_*`) was appended in v0.92, after `stack_free`, so a parser written for the
15-value frame keeps working.
- All time values in **µs** (measured with `esp_timer_get_time()`, resolution 1 µs)
- `stack_free`: remaining stack of the incunest_afe4490 FreeRTOS task in 4-byte words (`uxTaskGetStackHighWaterMark`)
- Checksum: XOR of all bytes between `$` and `*` (NMEA style)
- Emitted every `ts_emit_period_s = 5 s` (`_ts_emit_interval` = round(5 × fs) samples, v0.87); stats reset after each emission
- Written with `hal_console_printf()` to the console; since v0.92 also handed to the optional HAL console sink (§1.4) — PulseNest puts it on the UDP data stream

**Timing scope:**

| Field | What is measured |
|---|---|
| `hr1_mean/max` | `_hr1_update()` execution time |
| `hr2_mean/max` | `_hr2_update_sample()` execution time (fast path: biquad + decimate + buffer; autocorr itself is timed separately by `hr2_compute_mean/max`, Task B) |
| `hr3_mean/max` | `_hr3_update_sample()` execution time (fast path: LP filter + decimate + buffer; FFT+HPS itself is timed separately by `hr3_compute_mean/max`, Task C) |
| `spo2_mean/max` | `_spo2_update()` execution time |
| `cycle_mean/max` | Full sample cycle: SPI burst-read (4 channels) + SW subtraction + `_process_sample()` |
| `hr2cmp_mean/max` | `_hr2_compute()` (autocorrelation, Task B) |
| `hr3cmp_mean/max` | `_hr3_compute()` (FFT + HPS, Task C) |
| `spi_mean/max` | **v0.92** — the six 4-byte SPI frames that read one sample (2 `CONTROL0` writes + 4 register reads), from the first `_write_reg` to the last, mutex excluded. The HAL's bus cost on its own: Arduino = 4 × `SPI.transfer(uint8_t)` per frame, ESP-IDF = one polling transmit per frame |

**Budget:** cycle budget = 2000 µs (1 sample period at 500 Hz).
- `cycle_max < 1800 µs` → green (safe, 10% margin)
- `1800 ≤ cycle_max ≤ 2000 µs` → orange (tight)
- `cycle_max > 2000 µs` → red (over budget, risk of missed samples)

**Implementation notes:**
- Individual algo timers are in `_process_sample()`; cycle and SPI timers are in `_task_body()`
- `_emit_timing()` is a private method; formats and sends the frame, then resets all `TimingStat` accumulators
- `TimingStat` struct (fields: `max_us`, `sum_us`, `count`) lives in the private section of `INCUNEST_AFE4490`, guarded by `#if INCUNEST_TIMING_STATS`

---

## 9. Offline runner — `incunest_offline_runner`

### 9.1 Purpose

A native C++ command-line tool that runs the incunest_afe4490 algorithms (HR1, HR2, HR3, SpO2) on CSV files received from real Incunest incubators, without any hardware. Used to calibrate and improve the Incunest firmware using real neonatal PPG data.

### 9.2 Input CSV format (runner v0.20, 2026-09-24)

Two column dialects, detected from the header row **by name** (case-insensitive; column
order and extra columns ignored; the parser aborts with a clear error if the raw ADC
columns are missing):

| Signal | PulseNest capture | Legacy IncuNest export | fed as |
|---|---|---|---|
| IR raw          | `LED1`  | `IR`      | `test_feed_sample(led1, …)` |
| RED raw         | `LED2`  | `RED`     | `led2`  |
| IR ambient      | `ALED1` | `IR_Amb`  | `aled1` |
| RED ambient     | `ALED2` | `RED_Amb` | `aled2` |

Since v0.20 the runner feeds the **raw ADC codes** through `test_feed_sample()` — the
full `_process_sample()` path (analog state → RSQM → SpO2/HR1/HR2/HR3) — so OT, probe
state (debounce included), R, PI and SpO2 are recomputed by the library itself; the
`*_Sub` columns are derived internally and no longer read.

**Configuration comes from the capture itself:** the PulseNest capture header carries a
`# from-board: $CFG,…` line and the runner applies it (PRF, NUMAV, LED currents/range,
per-channel RF/CF/RG/STAGE2EN, AMBDAC, SpO2 coefficients, filter cutoffs) before feeding
samples; it refuses to run on a value it does not understand. `--ot-thr <A/A>` overrides
`rsqm_ot_thr` for the replay (e.g. `1.0e-4` to reproduce a capture taken before v0.95 for
equivalence checking). Parts of one session (`<stem>_pNN.csv`) share one library instance
in order, so filter/EMA state is continuous across part boundaries, as it was on the
board. Output: one `<stem>_replay.csv` per part (`SmpIdx, ProbeState, OT_LED1, OT_LED2,
R, PI, SpO2, SpO2_SQI, HR1…HR3 + SQIs`), written since runner v0.21 (2026-09-29) to
`<input dir>/derived/replay_lib<version>[_ot<thr>]/` — never beside the capture, whose
directory holds only what was recorded; the directory name records what the replay depends
on, so a replay with another library version or `--ot-thr` does not overwrite an earlier
one. `--out DIR` overrides the location.

**Optional firmware result columns:** when the capture carries the firmware's own
outputs (`SpO2`, `R`, `ProbeState` in the PulseNest dialect), the replay row also emits
them plus `delta_SpO2`/`delta_R`, for equivalence checking. Measured on
`20260923_1238_HOSPNAV_SUBJ09` (same threshold as the board, cold-start part excluded):
median delta 0.00 exactly, 94.6 % of samples within 1 pt, ProbeState match ≥ 99.7 %;
the residual tail (2.8 % of time) is micro-divergence of the variance-EMA state during
low-SQI artefact episodes, which 8-second window medians reduce to p95 = 0.84 pt. The
replay is faithful for calibration work, but it is **not bit-exact**: a cold start
cannot reproduce the board's warm state, and float rounding may differ across compilers.

### 9.3 Host build: no compile-time flag any more (v0.91)

Since v0.91 the library needs no `INCUNEST_OFFLINE`. When `ESP_PLATFORM` is not defined the main
header includes `incunest_afe4490_hal_host.h` (§1.4) and `incunest_afe4490_hal_host.cpp` provides
the pin/bus/console calls as no-ops. Everything compiles on the host — `begin()`, `stop()`,
`getData()` included — and fails honestly instead of pretending: `begin()` returns after logging
"FreeRTOS object creation failed" because the host FreeRTOS creates no objects.

What the runner needs is only **`UNIT_TEST`**, which opens the `test_feed_*` / `test_hr*` /
`test_spo2*` accessors (not part of the public API). Define it on the command line
(`-DUNIT_TEST=1`) and compile **`incunest_afe4490_hal_host.cpp` alongside `incunest_afe4490.cpp`**
so the HAL symbols link.

### 9.4 `incunest_afe4490_hal_host.h`

Host stand-ins for the ESP-IDF / FreeRTOS API the library calls, under their real names:

- `SemaphoreHandle_t`, `QueueHandle_t`, `TaskHandle_t` (`void*`), `TickType_t`, `BaseType_t`,
  `UBaseType_t`, `TaskFunction_t`; `pdTRUE`/`pdFALSE`/`pdPASS`/`pdFAIL`, `portMAX_DELAY`,
  `pdMS_TO_TICKS`, `portYIELD_FROM_ISR`.
- `xSemaphoreCreate*` → `nullptr`; `xSemaphoreTake`/`Give`/`GiveFromISR` → `pdTRUE`;
  `xQueueCreate` → `nullptr`; `xQueueSend`/`Receive` → `pdFALSE`; `xTaskCreatePinnedToCore` →
  `pdFAIL`; `vTaskDelete`, `vTaskDelay`, `vSemaphoreDelete`, `vQueueDelete` no-ops;
  `uxTaskGetStackHighWaterMark` → 0.
- `IRAM_ATTR` empty; `ESP_LOGE/W/I` to **stderr** (stdout stays clean data for the runner),
  `ESP_LOGD/V` off; `esp_timer_get_time()` on `std::chrono::steady_clock`.

It replaces `incunest_afe4490_platform_stub.h` (removed in v0.91) and PulseNest's `test/stubs/`
(removed the same day): one host path, in the library's repo.

### 9.5 History: what `INCUNEST_OFFLINE` used to do (v0.x – v0.90b)

| Former mechanism | Replaced by (v0.91) |
|---|---|
| `#ifdef INCUNEST_OFFLINE` → `platform_stub.h`, else Arduino/FreeRTOS headers | `#if defined(ESP_PLATFORM)` → real headers, else `incunest_afe4490_hal_host.h` |
| `begin()`, `stop()`, `getData()`, ISR, tasks, SPI primitives inside `#ifndef INCUNEST_OFFLINE` (63 conditionals) | none — they compile everywhere; the host HAL makes them no-ops |
| `INCUNEST_OFFLINE` implied `UNIT_TEST` | `UNIT_TEST` set explicitly by the host build |
| Hann window precomputed in `_reset_algorithms()`, not `begin()` | unchanged — still there, for the same reason |

### 9.6 File structure

```
lib/incunest_afe4490/                       (PulseNest: symlink to the library repo)
  incunest_afe4490.h / .cpp                 — library
  incunest_afe4490_hal.h                    — platform contract (§1.4)
  incunest_afe4490_hal_arduino.cpp          — active under ARDUINO
  incunest_afe4490_hal_idf.cpp              — active under ESP_PLATFORM && !ARDUINO
  incunest_afe4490_hal_host.cpp / _host.h   — active when !ESP_PLATFORM
  CMakeLists.txt                            — ESP-IDF component registration

tools/
  offline_runner/
    CMakeLists.txt           — CMake build (C++17): main.cpp + incunest_afe4490.cpp
                               + incunest_afe4490_hal_host.cpp, UNIT_TEST=1
    main.cpp                 — CSV reader, algorithm driver, output writer
```

The runner compiles the two library sources directly via CMake. No subproject, no installation,
no package manager. Known state (2026-09-15): the runner's `test_feed_*` calls predate the
OT-domain API (v0.6x) and do not compile; the HAL wiring above is in place for when it is revived.

### 9.7 Execution flow

```
incunest_offline_runner <path>     // path = CSV file or directory of CSV files

for each CSV file:
    parse header → locate required columns by name
    INCUNEST_AFE4490 afe (default constructor, no begin())
    afe._reset_algorithms()   // zeroes state + precomputes Hann window
    for each row:
        afe.test_feed_spo2(led1_aled1, led2_aled2)
        afe.test_feed_hr1(led1_aled1)
        afe.test_feed_hr2(led1_aled1)
        afe.test_feed_hr3(led1_aled1)
        write result row to <input_basename>_result.csv
    write summary row to batch_summary.csv
```

HR2 and HR3 are driven via `test_feed_hr2()` / `test_feed_hr3()`, which call the synchronous `_hr2_update_for_test()` / `_hr3_update_for_test()` paths (same computation as the async tasks, without FreeRTOS signalling). Results are available via `test_hr2()`, `test_hr2_sqi()`, etc. after each call.

### 9.8 Output files

**Per-file result:** `<input_basename>_result.csv`

```
SmpIdx,RED,IR,RED_Amb,IR_Amb,RED_Sub,IR_Sub,SpO2,SpO2_SQI,HR1,HR1_SQI,HR2,HR2_SQI,HR3,HR3_SQI
[,FW_HR1,FW_HR2,FW_HR3,FW_SpO2,delta_HR1,delta_HR2,delta_HR3,delta_SpO2]  ← if firmware columns present
```

**Batch summary:** `batch_summary.csv` (appended per run, not overwritten)

```
Timestamp,File,N_samples,SpO2_mean,SpO2_SQI_mean,HR1_mean,HR1_SQI_mean,HR2_mean,HR3_mean,valid_spo2_pct
```

### 9.9 Build

```bash
cd tools/offline_runner
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
build/incunest_offline_runner path/to/file.csv
build/incunest_offline_runner path/to/directory/
```

Requires C++17 and a standard compiler (g++ ≥ 10, MSVC ≥ 2019, clang ≥ 11). No external libraries.

---

## 10. Parameters Reference

Complete catalogue of all parameters that influence the library outputs, organised by layer. Each parameter belongs to exactly one layer; cross-references point to the section where it is defined in detail.

---

### 10.1 AFE4490 hardware configuration

Parameters written to chip registers during initialisation. Changing them requires an SPI write (via the public API); they are not re-computed per sample. Defaults are applied by `begin()` and listed in §2.2.

#### Transmitter (LED driver)

| Parameter | API | Default | Range | Effect on output |
|---|---|---|---|---|
| IR LED current (LED1) | `setLED1Current()` | ~49.8 mA | 0 – LED range max | PPG amplitude on IR channel; drives DC level and AC/DC ratio |
| RED LED current (LED2) | `setLED2Current()` | ~49.8 mA | 0 – LED range max | PPG amplitude on RED channel; directly affects SpO2 R-ratio |
| LED full-scale range | `setLEDRange()` | 150 mA | 75 or 150 mA (TX_REF=0.75 V) | Scales the 8-bit DAC resolution; affects achievable current values |

> LED currents set the signal amplitude at the photodiode. Too low → low PI, SpO2 SQI=0. Too high → TIA saturation (clipping). The optimal point maximises PI while keeping the ADC output below full-scale.

#### Receiver — TIA (I-V amplifier)

| Parameter | API | Default | Options | Effect on output |
|---|---|---|---|---|
| Feedback resistance R_F | `setTIAGain()` | RF_100K (100 kΩ) | 10k, 25k, 50k, 100k, 250k, 500k, 1MΩ | Transimpedance gain: larger R_F = higher voltage output, higher noise floor. Full-scale input current = 1 V / R_F |
| Feedback capacitance C_F | `setTIACF()` (or auto) | auto (see §7.2) | 32 steps, 5–250 pF (5 pF base + 5/15/25/50/150 pF caps) | TIA RC pole: must settle before ADC samples. Auto-selected by library from R_F and PRF |

> R_F and C_F together determine the front-end gain and noise. The library enforces `R_F × C_F ≤ Rx_sample_time / 10` via auto-CF (§7.2).

#### Receiver — Stage 2 (ambient cancellation amplifier)

| Parameter | API | Default | Options | Effect on output |
|---|---|---|---|---|
| Stage 2 gain LED1 | `setStage2GainLED1()` | RG_100K | 0, 3.5, 6, 9.5, 12 dB | STG2GAIN1[2:0] D[10:8] of TIAGAIN. Sets gain only; does NOT touch STAGE2EN1. Active when ENSEPGAIN=1. |
| Stage 2 gain LED2 | `setStage2GainLED2()` | RG_100K | 0, 3.5, 6, 9.5, 12 dB | STG2GAIN2[2:0] D[10:8] of TIA_AMB_GAIN. Sets gain only; does NOT touch STAGE2EN2. Always active. |
| Stage 2 enable LED1 | `setStage2En1(bool)` | true | true / false | STAGE2EN1 bit D14 of TIAGAIN. Decoupled from gain: can be ON at 0 dB or OFF despite non-zero gain. Active when ENSEPGAIN=1. |
| Stage 2 enable LED2 | `setStage2En2(bool)` | true | true / false | STAGE2EN2 bit D14 of TIA_AMB_GAIN. Decoupled from gain. Also forced ON by `_apply_analog_regs()` when `AMBDAC > 0` (cancellation current needs Stage 2 active). Always active. |
| Ambient cancellation DAC (I_CANCEL) | `setAmbDac()` | 0 µA | 0 – 10 µA, 1 µA steps | AMBDAC[3:0] D[19:16] of TIA_AMB_GAIN. Subtracts ambient leakage before ADC via Stage 2. When `uA > 0`, `_apply_analog_regs()` forces STAGE2EN2=1 regardless of `setStage2En2()`. Applies to both LED1 and LED2 ambient phases (single shared DAC). |

> Stage 2 gain and enable are now independent bits. RG_100K with STAGE2EN=1 → Stage 2 active at unity gain (useful for AMBDAC at minimum overhead). STAGE2EN=0 regardless of gain → Stage 2 bypassed.

#### ADC and averaging

| Parameter | API | Default | Range | Effect on output |
|---|---|---|---|---|
| Sample rate (PRF) | `setSampleRate()` | 500 Hz | 63 – 5000 Hz | Output rate; also determines LED-on window and max averages (see §7.1) |
| Number of averages (NUMAV) | `setAdcAverages()` | 8 (NUMAV=7) | 1 – NUMAV_max+1 | Hardware averaging before ADC output: SNR ∝ √N. Larger N = lower noise, higher latency. Automatically clamped to NUMAV_max for current PRF |

> Averaging is the primary noise-reduction knob that does not alter signal bandwidth. At 500 Hz, 8 averages give ×2.83 SNR improvement at no bandwidth cost.

#### Timing registers

All 22 timing-edge registers (LED on/off, sample windows, ADC conversion windows, ADC reset pulses) are **calculated automatically** by the library from `setSampleRate()`. They are not exposed via the public API. See §7 and the AFE4490 datasheet §8.3.3 for the underlying model.

---

### 10.2 Pre-processing parameters (shared across algorithms)

Applied to the raw ADC output before any algorithm. Parameters are compile-time constants.

| Parameter | Constant | Value | Used by | Effect |
|---|---|---|---|---|
| DC removal time constant | `spo2_ema_mean_tau_s` (SpO2), `hr1_dc_tau_s` (HR1) | 2.0 s (SpO2), 1.6 s (HR1) | SpO2, HR1 | EMA low-pass estimates and removes the DC component; time constant determines tracking speed vs. baseline stability |
| Input channel selection | `setPPGChannel()` default | LED1 | `ppg_disp` only (v0.69) | Selects ot_led1 vs ot_led2 for the display waveform; HR1/HR2/HR3/SpO2 always consume both OT channels directly, not gated by this selector |
| Decimation factor | `_hr2_decim.factor`, `_hr3_decim.factor` (each derived from its own target rate) | ×10 at 500 Hz | HR2 / HR3 respectively | One chain per filtered path; holds each decimated rate at 50 Hz before autocorrelation / FFT — see §5.3.1 |

---

### 10.3 SpO2 parameters

See §5.1 for algorithm description.

| Parameter | Constant | Value | Justification | Effect |
|---|---|---|---|---|
| R curve id | `spo2_r_curve_id` | `R-CURVE-U401D-MS100-20260323` | MS100 simulator / Nellcor R-curve, probe U401-D | Which curve turns R into SpO2 (§5.1, *R methods and R curves*) |
| R method of the curve | `spo2_r_curve_r_method_id` | `R-METHOD-0` | The library of 2026-03-23 | ≠ `SPO2_R_METHOD_ID` → mismatch warning (not invalidation) |
| R curve coefficient a | `spo2_r_curve_a` | 114.9208 | Empirical — MS100 simulator / Nellcor R-curve | SpO2 = a − b·R intercept |
| R curve coefficient b | `spo2_r_curve_b` | 30.5547 | Empirical — MS100 simulator / Nellcor R-curve | SpO2 = a − b·R slope |
| DC EMA time constant | `spo2_ema_mean_tau_s` | 2.0 s | f_c ≈ 0.08 Hz << 0.5 Hz (30 bpm min HR); same signal as RSQM → same τ | Speed of DC baseline tracking |
| AC² EMA time constant | `spo2_ema_var_tau_s` | 6.0 s | ISO 80601-2-61:2026 Annex JJ.2 d) (informative): 6–10 s window for a *transfer standard*; between 2–4 s (neonatal resuscitation/IH research) and 8 s (NICU default) — design rationale §4.5 | Stability of AC amplitude / PI estimate |
| Warm-up period | `spo2_warmup_s` | 18.0 s | 3 × τ_var = 3 × 6 s — EMA variance convergence | SpO2 suppressed during initial transient |
| Minimum DC level (no-finger threshold) | `spo2_min_dc` | 1000 counts | Below this DC, SpO2 and SQI are forced to 0 | No-finger detection guard |
| SQI PI lower bound | `spo2_pi_sqi_lo` | 0.5 % | Nellcor/Masimo clinical threshold | PI below this → SQI = 0 |
| SQI PI upper bound | `spo2_pi_sqi_hi` | 2.0 % | Nellcor/Masimo clinical threshold | PI above this → SQI = 1 |

---

### 10.4 HR1 parameters

See §5.2 for algorithm description.

| Parameter | Constant | Value | Effect |
|---|---|---|---|
| DC removal time constant | `hr1_dc_tau_s` | 1.6 s | Baseline tracking speed |
| Low-pass MA cutoff | `hr1_ma_cutoff_hz` | 5 Hz | Noise smoothing before peak detection; affects peak sharpness |
| Max MA length | `hr1_ma_max_len` | 64 samples | Prevents excessively long moving average at low heart rates |
| Running-max decay | `hr1_max_decay_tau_s` | 1.5 s | How long an old peak keeps the threshold up. **Recovery**, not detection margin: that is `hr1_threshold_fraction`'s job |
| Peak threshold fraction | 0.6 | 0.6 × running_max | Trade-off: higher → misses low-amplitude peaks; lower → false detections on noise |
| Refractory period | `hr1_refractory_s` | 0.185 s | Minimum interval between detected peaks; practical upper HR limit ~324 BPM |
| RR interval buffer length | 5 intervals | 5 | Number of consecutive intervals averaged for HR1 output |
| SQI CV threshold | `hr1_sqi_cv_max` | 0.15 (15 %) | CV above this → SQI = 0 (arrhythmia / motion artefact) |

**Where `hr1_max_decay_tau_s` = 1.5 s comes from (v0.93).** Until v0.93 the value was 20 s, which
was never chosen: it is the historical `× 0.9999` per-sample factor written in seconds at 500 Hz.
Two independent measurements set it, both on material where the truth is known:

- **Perturbed simulator sweep, 2026-09-08** (five MS100 recordings at 60/140/180/220/250 bpm with
  the probe squeezed at 20/30/40/50 s; `tools/hr1_probepert_sweep.py` in PulseNest). Metric:
  share of samples with `hr1_sqi > 0` **and** |HR − truth| ≤ 3 bpm, so a detector that recovers
  availability by inventing beats scores as the failure it is. τ = 20 s scores 53 % at
  220–250 bpm; τ = 1–3 s is a flat optimum (76–78 % worst case, 84 % at 220–250); τ = 0.5 s
  collapses at 60 bpm (49 %) because the threshold falls to ~8 % of the peak between beats and
  the dicrotic notch is counted. `hr1_threshold_fraction` = 0.6 survives the same sweep.
- **Three-board probe capture, 2026-09-15** (30 s, same subject at ~52 bpm on three boards, clean
  signal: `PROBE_APPLIED`, DiagCode 0, RSQI 1). `hr1_sqi` fell to 0 for one ~5.8 s run on two of
  the three boards while HR2 and HR3 kept publishing. Cause, reproduced off the capture by
  `tools/hr1_streak_forensics.py`: a single beat missed (amplitude 56 % of the previous one,
  reaching 85 % of a threshold still held up by a peak 4.5 s old), whose doubled RR interval then
  occupies the 5-interval buffer for five beats. The boards' beat-to-beat amplitude CV was 37 %
  and 43 % on the two that failed against 23 % on the one that did not — the modulation that a
  20 s memory cannot follow. At τ = 1.5 s every missed beat is recovered on all three boards with
  no extra detection.

The selection rule is the conservative one: **the largest τ that holds across 40–250 bpm**, not the
peak of the metric — a longer memory is a steadier threshold and less room to invent beats.

---

### 10.5 HR2 parameters

See §5.3 for algorithm description.

| Parameter | Constant | Value | Effect |
|---|---|---|---|
| BPF lower cutoff | `hr2_f_low_hz` | 0.5 Hz | Removes DC drift and very low-frequency baseline wander |
| BPF upper cutoff | `hr2_f_high_hz` | 5.0 Hz | Anti-aliasing before decimation; limits harmonic content |
| Decimation factor | `_hr2_decim.factor` | 10 at 500 Hz | **Derived** from `hr2_decim_target_rate_hz`, not configured — see §5.3.1 |
| Buffer length | `hr2_buf_len` | 400 samples (8 s) | Analysis window; longer → more stable at low HR, longer convergence time |
| Update interval | `hr2_update_interval` | 25 decimated samples (0.5 s) | HR2 output refresh rate |
| Lag scratch-array cap | `hr2_acorr_lag_cap` | 137 samples | Storage bound only. Swept range comes from `hr_min_bpm`/`hr_max_bpm` at runtime; the cap binds only above ~845 Hz sample rate |
| Minimum lag | `hr2_min_lag_s` | 0.185 s | Corresponds to ~324 BPM; prevents false peaks at very short lags |
| Minimum correlation threshold | `hr2_min_corr` | 0.5 | Below this normalised correlation, no HR is reported and SQI = 0 |

---

### 10.6 HR3 parameters

See §5.4 for algorithm description.

| Parameter | Constant | Value | Effect |
|---|---|---|---|
| LP filter cutoff | `hr3_f_high_hz` | 15 Hz | Anti-aliasing before decimation; passes up to the 3rd harmonic of 260 BPM (13 Hz). **Does not remove DC** — that is done by mean subtraction in `_hr3_prepare_fft_input()` |
| Decimation factor | `_hr3_decim.factor` | 10 at 500 Hz | Own chain, independent of HR2's — **derived**, see §5.3.1 |
| FFT / buffer length | `hr3_buf_len` | 512 samples (10.24 s) | Frequency resolution = 50/512 ≈ 0.098 Hz ≈ 0.35 BPM at 1 Hz |
| Update interval | `hr3_update_interval` | 25 decimated samples (0.5 s) | HR3 output refresh rate |
| Window function | Hann | — | Reduces spectral leakage; pre-computed at `_reset_algorithms()` |
| HPS harmonics | P[k]·P[2k]·P[3k] | 2nd and 3rd | Reinforces fundamental; HPS search capped at nyquist/3 to keep 3rd harmonic in band |

---

### 10.7 Range and validation parameters (transversal)

Applied by all HR algorithms equally to decide whether an output is reportable and whether SQI is forced to 0.

| Parameter | Constant | Value | Scope |
|---|---|---|---|
| Reported valid HR lower bound | `hr_min_bpm` | 40 BPM | HR1, HR2, HR3 — engineering choice; WHO-UNICEF asks for 30 (§5.2) |
| Reported valid HR upper bound | `hr_max_bpm` | 260 BPM | HR1, HR2, HR3 — engineering choice; covers WHO-UNICEF's 240 (§5.2) |
| Internal search lower bound | `hr_search_min_bpm` | 37 BPM | Guard band: hr_min − 3 |
| Internal search upper bound | `hr_search_max_bpm` | 263 BPM | Guard band: hr_max + 3 |
| SpO2 reported valid lower bound | `spo2_min` | 70 % | SpO2 |
| SpO2 reported valid upper bound | `spo2_max` | 100 % | SpO2 |
| SpO2 clamp margin | `spo2_clamp_margin` | 3 % | Values within margin are clamped, not rejected |

---

### 10.8 Out-of-scope: physical and environmental factors

The following factors affect the quality of the raw PPG signal but are **not configurable parameters of this library**. The library does not model, compensate for, or control them. They are listed here for completeness and to clarify the library's responsibility boundary.

| Factor | How it manifests in the output | Library observable |
|---|---|---|
| **Tissue perfusion (PI)** | Low PI → weak AC component → unreliable SpO2 and HR | `pi` field and `spo2_sqi` (PI-based) |
| **Ambient light** | Increases ambient channels (AmbRED, AmbIR); incomplete cancellation degrades SNR | `aled1`, `aled2`, `led1_aled1`, `led2_aled2` raw fields |
| **Patient motion / artefacts** | Introduces aperiodic components that corrupt HR estimation | All `hr*_sqi` fields (regularity/periodicity metrics) |
| **Probe type and placement** | Different optical path, tissue thickness, and spectral response; affects R-ratio calibration | `spo2_r` field; SpO2 accuracy depends on calibration matching the probe |
| **Neonatal vs adult anatomy** | Different absorption spectra and pulsatile characteristics | Affects SpO2 coefficients `a`, `b` and HR signal amplitude |

These factors belong to the test protocol, clinical validation documentation, and operator guidance for the parent IncuNest device — not to this library's specification. However, their effects are **observable** through the SQI fields and the raw signal fields exposed in `AFE4490Data`.

---

### 10.9 HGAC parameters (RF-only, two EMA per gain domain)

See §5.8 for algorithm description.

| Parameter | API | Default | Range | Effect |
|---|---|---|---|---|
| Master enable | `setHgacEnable()` | `false` | true / false | HGAC never actuates while disabled — safe default until bench-validated |
| HIGH2 guard threshold | `setHgacVTiaHigh2()` | 0.90 V | ≤ FS_V | fast-EMA `v_tia_led` above this → urgent `RF--` for that domain |
| HIGH1 leveling upper | `setHgacVTiaHigh1()` | 0.75 V | — | slow-EMA `v_tia_led` above this → `RF--` |
| LOW1 leveling lower | `setHgacVTiaLow1()` | 0.25 V | — | slow-EMA `v_tia_led` below this → `RF++` |
| Guard EMA τ (fast) | `setHgacEmaFastTauS()` | 0.1 s | 0.0–10.0 | HIGH2 estimator time constant; warmup = 3·τ |
| Leveling EMA τ (slow) | `setHgacEmaSlowTauS()` | 2.0 s | 0.0–30.0 | HIGH1/LOW1 estimator time constant; warmup = 3·τ |
| Ambient EMA τ | `setHgacEmaAmbientTauS()` | 2.0 s | 0.0–30.0 | ALED-phase estimator for the AMBIENT_HIGH alarm; sustained (avoids flicker); warmup = 3·τ |
| Settling time (computed, not `$SET`-tunable) | `_compute_switched_rc_settling_samples()` (private) | ~16 ms @ 500 Hz (datasheet §7.7 t5: ≥3 ms/phase) | — | Arms `RSQM_DIAG_SWITCHED_RC_SETTLING` after an RF change; EMA warmup does not start until this elapses (sequential, §5.8.4) |

---

## 11. AFE4490 chip register configuration

This section documents the exact value of each AFE4490 register written by the library during initialisation, compared against the three reference sources analysed.

**Reference sources:**
- **[AFE4490 Datasheet (TI)](https://www.ti.com/lit/ds/symlink/afe4490.pdf):** primary hardware reference — register map, timing formulas, electrical characteristics.
- **AFE44x0.h (EVM TI v1.4):** TI official firmware for the AFE4490EVM evaluation kit. Written for MSP430. The C code is not reusable, but the register values and timing formulas are TI's authoritative reference. See also: [AFE4490EVM User's Guide (SLAU480C)](https://www.ti.com/lit/ug/slau480c/slau480c.pdf).
- **Datasheet AFE4490 Table 2:** example values from the datasheet for PRF=500Hz, duty cycle=25%.
- **Protocentral (src/):** `src/protocentral_afe44xx.cpp` from the project — modified version of the Protocentral library adapted to the project.

---

### 11.1 Timing registers (PRF = 500 Hz, AFECLK = 4 MHz, 1 count = 0.25 µs)

The cycle is **circular**: LED2 active phase occupies the last quarter (≈6000–7999); its ADC conversion runs in the first quarter of the next cycle (0–1999), pipelined with ALED2 sampling. All registers are absolute positions in [0, 7999]. "circ." denotes a modular constraint across the cycle boundary. Table ordered by **temporal sequence**.

`PRPCOUNT` = 7999 — period register: `(AFECLK / PRF) − 1` = (4 000 000 / 500) − 1 = 7999.

| # | Identifier | Default (counts) | Rationale | Constraints |
|--:|---|---|---|---|
| 1 | **t21** ADCRSTSTCT0 — *ADC reset 0 start count* | 0 | Start of period | < t22; ≤ t13 |
| 2 | **t22** ADCRSTENDCT0 — *ADC reset 0 end count* | 3 | 3-count reset pulse (0.75 µs, −60 dB crosstalk) | > t21; < t13; (t22−t21) ≥ 4 |
| 3 | **t13** LED2CONVST — *LED2 convert start count* | 4 | 1 count after reset end (t22=3); conversion starts one AFECLK after reset | ≥ t22; ≥ t2+1 (circ.); < t14 |
| 4 | **t5** ALED2STC — *Sample ambient LED2 start count* | 400 | 400 counts (100 µs) after LED2 OFF — raised for the 50 mA default (v0.47); was 200 (adequate to ~25 mA) | > t4 (circ.); < t6; both LEDs off in [t5,t6]; RF×CF < (t6−t5)×0.25µs/10 (§8.3.1.1); ≥ t4 + 5×RF×CF/0.25µs |
| 5 | **t6** ALED2ENDC — *Sample ambient LED2 end count* | 1 998 | Uses full window; closes 2 counts before t9 (LED1 on) | > t5; < t9; ≤ t15 |
| 6 | **t14** LED2CONVEND — *LED2 convert end count* | 1 999 | 1 count before next reset (t23=2000); conv. window = 1 995 counts | > t13; (t14−t13) ≥ 1 950 (§8.3.1.2) |
| 7 | **t23** ADCRSTSTCT1 — *ADC reset 1 start count* | 2 000 | Start of phase 2 | < t24; ≤ t15 |
| 8 | **t9** LED1LEDSTC — *LED1 start count* | 2 000 | LED1 on at start of its phase (25% of period) | ≤ t7; < t10 |
| 9 | **t24** ADCRSTENDCT1 — *ADC reset 1 end count* | 2 003 | t23 + 3 counts | > t23; < t15; (t24−t23) ≥ 4 |
| 10 | **t15** ALED2CONVST — *LED2 ambient convert start count* | 2 004 | 1 count after t24=2003 | ≥ t24; ≥ t6+1; < t16 |
| 11 | **t7** LED1STC — *Sample LED1 start count* | 2 050 | 50 counts after LED1 on; TIA settle margin = max(50, 10% of LED window) | ≥ t9; < t8; RF×CF < (t8−t7)×0.25µs/10 (§8.3.1.1) |
| 12 | **t8** LED1ENDC — *Sample LED1 end count* | 3 998 | Closes 2 counts before end of LED1 phase | > t7; ≤ t10; < t17 |
| 13 | **t10** LED1LEDENDC — *LED1 end count* | 3 999 | 25% duty cycle: 2 000 counts = 500 µs → maximises photons | ≥ t8; > t9 |
| 14 | **t16** ALED2CONVEND — *LED2 ambient convert end count* | 3 999 | 1 count before t25=4000; conv. window = 1 995 counts | > t15; (t16−t15) ≥ 1 950 |
| 15 | **t25** ADCRSTSTCT2 — *ADC reset 2 start count* | 4 000 | Start of phase 3 | < t26; ≤ t17 |
| 16 | **t26** ADCRSTENDCT2 — *ADC reset 2 end count* | 4 003 | t25 + 3 counts | > t25; < t17; (t26−t25) ≥ 4 |
| 17 | **t17** LED1CONVST — *LED1 convert start count* | 4 004 | 1 count after t26=4003 | ≥ t26; ≥ t8+1; < t18 |
| 18 | **t11** ALED1STC — *Sample ambient LED1 start count* | 4 400 | 400 counts (100 µs) after LED1 OFF — raised for the 50 mA default (v0.47); same criterion as t5 | > t10; < t12; both LEDs off in [t11,t12]; RF×CF < (t12−t11)×0.25µs/10; ≥ t10 + 5×RF×CF/0.25µs |
| 19 | **t12** ALED1ENDC — *Sample ambient LED1 end count* | 5 998 | Full window; closes 2 counts before t3 (LED2 on) | > t11; < t3 (circ.); ≤ t19 |
| 20 | **t18** LED1CONVEND — *LED1 convert end count* | 5 999 | 1 count before t27=6000; conv. window = 1 995 counts | > t17; (t18−t17) ≥ 1 950 |
| 21 | **t27** ADCRSTSTCT3 — *ADC reset 3 start count* | 6 000 | Start of phase 4 | < t28; ≤ t19 |
| 22 | **t3** LED2LEDSTC — *LED2 start count* | 6 000 | LED2 on at start of its phase (75% of period) | ≤ t1; < t4 |
| 23 | **t28** ADCRSTENDCT3 — *ADC reset 3 end count* | 6 003 | t27 + 3 counts | > t27; < t19; (t28−t27) ≥ 4 |
| 24 | **t19** ALED1CONVST — *LED1 ambient convert start count* | 6 004 | 1 count after t28=6003 | ≥ t28; ≥ t12+1; < t20 |
| 25 | **t1** LED2STC — *Sample LED2 start count* | 6 050 | 50 counts after LED2 on; TIA settle margin = max(50, 10% of LED window) | ≥ t3; < t2; RF×CF < (t2−t1)×0.25µs/10 (§8.3.1.1) |
| 26 | **t2** LED2ENDC — *Sample LED2 end count* | 7 998 | Closes 2 counts before end of LED2 phase | > t1; ≤ t4; < t13+period (circ.) |
| 27 | **t4** LED2LEDENDC — *LED2 end count* | 7 999 | 25% duty cycle: 2 000 counts = 500 µs → maximises photons | ≥ t2; > t3 |
| 28 | **t20** ALED1CONVEND — *LED1 ambient convert end count* | 7 999 | = PRPCOUNT; conv. window = 1 995 counts | > t19; (t20−t19) ≥ 1 950 |

**Notes:**
- Simultaneous events are valid: t9=t23=2000, t10=t16=3999, t3=t27=6000, t4=t20=7999.
- ADC conversion minimum (§8.3.1.2): ≈ PRPCOUNT/4 − 50 ≈ 1 950 counts. Recommended values give 1 995 counts per channel.
- RF×CF (§8.3.1.1): with RF=500 KΩ, CF=5 pF → RF×CF=2.5 µs; sample windows ≈ 487 µs → limit=48.7 µs → margin ×19.
- Ambient settling (t5, t11): **400 counts (100 µs) since v0.47** (for the 50 mA default). Was 200 counts (50 µs), adequate up to ~25 mA. Valid for PRF ≤ ~2 kHz (at higher PRF the ambient sample window q−2 shrinks below the margin — revisit).
- TIA margin formula: `max(50, floor(LED_window × 0.10))` counts. Also used by auto-CF (§7.2).
- Historical comparison: TI EVM uses 20% duty cycle + 80-count margins (ALED1CONVEND overflows at 500 Hz); Protocentral uses 0 for reset and sample margins (no settling guard).

---

### 11.2 Analog configuration

| Register / Field | AFE44x0.h (EVM) | Datasheet | Protocentral (src/) | **incunest_afe4490** | **Rationale** |
|---|---|---|---|---|---|
| **TIAGAIN** | 0x00C006 | — | 0x000000 | **0x000000** | Minimal config: RF=500kΩ, CF=5pF, Stage2 off, ENSEPGAIN=0 |
| → ENSEPGAIN | 1 | — | 0 | **0** | With 0, both channels use TIAGAIN for the TIA. Simplifies configuration. Enable if independent gain per channel is needed |
| → Stage2 LED1 | EN, 0 dB | — | disabled | **disabled** | RF=500kΩ provides sufficient gain for typical PPG. Stage2 at 0dB adds no useful gain. Configurable via setter |
| → CF LED1 | 5 pF | — | 5 pF | **auto (§7.2)** | Auto-selected to maximise noise filtering while guaranteeing 5τ settling. Chosen from all 32 register codes (v0.62). Default at 500 Hz / RF_100K: 100 pF (was 55 pF) |
| → RF LED1 | 1 MΩ | — | 500 kΩ | **100 kΩ** | Chosen for the earlier 20 mA default. ✅ Verified OK at the 50 mA default (v0.47): measured v_tia < 0.5 V with a finger applied (≪ 1.0 V FS, even below the 0.6 V ideal point) — no saturation, RF_100K adequate. Behaviour with the real neonatal probe/patient (transmittance, perfusion, probe geometry) is uncharacterised — verify v_tia there. Configurable via setTIAGain() |
| **TIA_AMB_GAIN** | 0x004006 | — | 0x000001 | **0x000000** | With ENSEPGAIN=0, the RF of this register does not affect the main TIA. Relevant for FLTRCNRSEL, AMBDAC and Stage2 |
| → RF LED2/amb | 1 MΩ | — | 250 kΩ | **100 kΩ** | Ignored for TIA when ENSEPGAIN=0. Set equal to TIAGAIN to avoid surprises if ENSEPGAIN=1 is enabled later |
| → CF LED2/amb | 5 pF | — | 5 pF | **auto (§7.2)** | Same as CF LED1 (ENSEPGAIN=0 shares config) |
| → Stage2 LED2 | EN, 0 dB | — | disabled | **disabled** | Same criterion as Stage2 LED1 |
| → FLTRCNRSEL | 500 Hz | — | 500 Hz | **500 Hz** | Controlled by `kAFE_FLTRCNRSEL` (constexpr uint8_t, 0=500 Hz / 1=1000 Hz). Always written explicitly by `_build_tia_amb_gain_led2()`. Determines `kAFE_DIAG_HOLDOFF_MS` (28 ms @ 500 Hz, 16 ms @ 1000 Hz, datasheet p.11). |
| → AMBDAC | 0 µA | — | 0 µA | **0 µA** | Ambient cancellation handled in hardware (LED1−ALED1VAL, LED2−ALED2VAL). AMBDAC only needed if ambient light saturates TIA before subtraction. Exposable via setter |
| **LEDCNTRL** | 0x011414 | — | 0x001414 | **0x005555** | RANGE_0 + code 85 + TX_REF=0.75V → ~49.8 mA per LED (preliminary 50 mA dev default, v0.47) |
| → LED_RANGE | RANGE_1 | — | RANGE_0 | **RANGE_0** | Consistent with TX_REF=0.75V. FSR=150mA → resolution ~0.6mA/step |
| → LED1 code | 20 (0x14) | — | 20 (0x14) | **85 (0x55)** | ~49.8 mA: closest DAC code to the preliminary 50 mA dev default (v0.47). Configurable via setLEDCurrent() |
| → LED2 code | 20 (0x14) | — | 20 (0x14) | **85 (0x55)** | Same as LED1. In practice red and IR may need independent adjustment |
| → **Resulting current** | **≈3.9 mA** | — | **≈11.7 mA** | **≈49.8 mA** | Preliminary 50 mA dev default (v0.47); was ~19.9 mA in v0.32–v0.46 |
| **CONTROL2** | 0x020000 | — | 0x000000 | **0x000000** | All fields at reset value |
| → TX_REF | 0.5 V | 0.75 V (reset) | 0.75 V | **0.75 V** | Datasheet default. Supports up to 150mA with RANGE_0. EVM uses 0.5V due to its supply constraints |
| → PDN_AFE/RX/TX | all ON | — | all ON | **all ON** | Normal operating mode. PDN only for low-power use |
| → TXBRGMOD | H-bridge | — | H-bridge | **H-bridge** | Standard for 2-pin back-to-back LED |
| → XTAL | enabled | — | enabled | **enabled** | No crystal on in3ator but chip ignores it if not connected |
| → ADC_BYP | disabled | — | disabled | **disabled** | Normal mode; bypass only for chip testing |

---

### 11.3 Control registers

| Register / Field | AFE44x0.h (EVM) | Datasheet | Protocentral (src/) | **incunest_afe4490** | **Rationale** |
|---|---|---|---|---|---|
| **CONTROL0** | 0x000000 | — | 0x000000→0x000008 | **0x000008 in init** | SW_RST ensures known state. Self-clearing. EVM does no explicit SW_RST — risk if chip was left in inconsistent state |
| **CONTROL1** | 0x000107 | — | 0x010707 | **0x000107** | TIMEREN=1, NUMAV=7, no outputs on ALM pins |
| → TIMEREN | 1 | — | 1 | **1** | Required for the internal timer to generate control signals |
| → NUMAV | 7 (8 avg.) | — | 7 (8 avg.) | **7** | Default = 8 averaged samples → SNR ×√8 ≈ 2.8×. Configurable via `setAdcAverages()`. See NUMAV_max formula in section 7.1 |
| → CLKALMPIN | not config. | — | LED2_CONV\|LED1_CONV | **not config. (0x00)** | Protocentral outputs conversion signals on ALM pins (useful with oscilloscope). Unnecessary in production; may interfere with other uses of those pins on in3ator |
| → bit 16 | 0 | — | 1 ⚠️ | **0** | Not documented in AFE44x0.h or identified in datasheet. Protocentral enables it without apparent justification. Not replicated until its purpose is understood |
| **ALARM** | not config. | — | not config. | **not config.** | Not needed for basic operation. Will be added if chip diagnostics are implemented |

---

### 11.4 Initialisation sequence

| Step | AFE44x0.h (EVM) | Protocentral (src/) | **incunest_afe4490** | **Rationale** |
|---|---|---|---|---|
| 1 | — | PDN low 500ms → PDN high 500ms | **PDN low 500ms → PDN high 500ms** | Hard reset before any SPI communication. 500ms is conservative but safe; datasheet does not specify minimum |
| 2 | CONTROL0=0x000000 | CONTROL0=0x000000 | **CONTROL0=0x000000** | Ensures SPI write mode (SPI_READ=0) |
| 3 | — | CONTROL0=0x000008 | **CONTROL0=0x000008 (SW_RST)** | Software reset complementary to hard reset |
| 4 | Timers first | TIAGAIN, TIA_AMB_GAIN, LEDCNTRL, CONTROL2 | **TIAGAIN, TIA_AMB_GAIN, LEDCNTRL, CONTROL2** | Analog front-end before enabling timers — avoids sampling with partial configuration |
| 5 | CONTROL1 (timer ON) | Timing registers | **Timing registers** | All timing registers before starting the timer |
| 6 | CONTROL0=SPI_READ | CONTROL1 (timer ON) | **CONTROL1 (timer ON — last)** | TIMEREN activated last when everything is configured |
| 7 | — | delay 1000ms | **delay 1000ms** | Stabilisation before first read. Protocentral value not justified in datasheet — possibly reducible, but safe for startup |

---

## 12. Datasheet-to-library identifier mapping

Reference table for HGAC and RSQM algorithm design. Maps AFE4490 datasheet signal names to library identifiers.

### 12.1 Analog gain parameters

| Datasheet name | HW register | `AFE4490Config` field | Setter | Type / enum |
|---|---|---|---|---|
| **RF_LED1** | `TIAGAIN` D[2:0] | `afe_tia_rf_led1` | `setTIAGain()` | `AFE4490RF` (`RF_10K`…`RF_1M`) |
| **RF_LED2** | `TIA_AMB_GAIN` D[2:0] | `afe_tia_rf_led2` | `setTIAGain()` (shared when `ENSEPGAIN=0`) | `AFE4490RF` |
| **STG2GAIN1** | `TIAGAIN` D[10:8] | `afe_stg2_rg_led1` | `setStage2GainLED1()` | `AFE4490RG` (`RG_100K`…`RG_400K`) |
| **STG2GAIN2** | `TIA_AMB_GAIN` D[10:8] | `afe_stg2_rg_led2` | `setStage2GainLED2()` | `AFE4490RG` |
| **AMBDAC** | `TIA_AMB_GAIN` D[19:16] | `afe_ambdac_uA` | `setAmbDac(uint8_t uA)` | `uint8_t` 0–10 µA |
| **LED1** (IR current) | `LEDCNTRL` D[15:8] | `afe_led1_current_mA` | `setLED1Current(float mA)` | `float` 0–range mA (DAC-quantised) |
| **LED2** (RED current) | `LEDCNTRL` D[7:0] | `afe_led2_current_mA` | `setLED2Current(float mA)` | `float` 0–range mA (DAC-quantised) |

> **RF_LED2 note:** when `ENSEPGAIN=0` (default), `setTIAGain()` writes the same RF value to both `TIAGAIN` and `TIA_AMB_GAIN`. `afe_tia_rf_led2` acts independently only when `ENSEPGAIN=1`.

### 12.2 ADC data registers

| Datasheet name | Register address | `AFE4490Sample` field | Semantic channel | Read? |
|---|---|---|---|---|
| **LED2VAL** | `0x2A` | `led2` | RED raw | Yes |
| **ALED2VAL** | `0x2B` | `aled2` | Ambient after LED2 (RED amb) | Yes |
| **LED1VAL** | `0x2C` | `led1` | IR raw | Yes |
| **ALED1VAL** | `0x2D` | `aled1` | Ambient after LED1 (IR amb) | Yes |
| **LED2−ALED2VAL** | `0x2E` | — | *(not read — see note below)* | **No** |
| **LED1−ALED1VAL** | `0x2F` | — | *(not read — see note below)* | **No** |

The 4 raw values are sign-extended from 22-bit two's complement by `_sign_extend_22()`. Valid range: [−2 097 152, +2 097 151]. Saturation (TIA railing) produces a near-maximum constant value (~2 096 921) across all channels.

`led2_sub` and `led1_sub` (stored in `AFE4490Data`) are computed in software as `led2 − aled2` and `led1 − aled1` using int32_t arithmetic. The hardware subtraction registers (`0x2E`, `0x2F`) perform the same subtraction internally but in 22-bit arithmetic, which overflows when the operands have opposite signs near the rail (e.g. led1 ≈ +2 097 151, aled1 ≈ −2 097 152 → correct result ≈ +4 194 303 but chip wraps to −1). The software computation is exact for all physically achievable input combinations (max |led − aled| ≈ 4 194 303, well within int32_t range). The hardware registers are therefore permanently unused.

### 12.3 Hardware probe states

The hardware states are distinguishable from the raw ADC channel pattern. Used by RSQM (probe presence detection) and HGAC (inhibit control when probe not on patient).

| State | `led1` | `led2` | `aled1` | `aled2` | Physical cause |
|---|---|---|---|---|---|
| **PROBE_DISCONNECTED** | ≈ −5000 | ≈ −5000 | ≈ −5000 | ≈ −5000 | Photodiode open-circuit; TIA input floating → input offset current × RF produces small negative output |
| **PROBE_ONLY_LED_SATURATING** | saturated | saturated | low–mid | low–mid | No tissue attenuation; LEDs illuminate photodiode directly. ALED channels depend on ambient light level |
| **PROBE_OT_HIGH** | high, not saturated | high, not saturated | low–mid | low–mid | Not saturated, but both channels' OT > `rsqm_ot_thr`: little attenuation — probe in air at low gain, or very thin tissue (open calibration, §5.6.2) |
| **PROBE_AMB_SATURATING** | saturated | saturated | saturated | saturated | External light (lamp/phototherapy) saturates the LED-off phase too; presence unknown |
| **PROBE_APPLIED** | PPG signal | PPG signal | low | low | Normal operation; tissue attenuates LED light; ambient channels remain low |

**Detection method:** see §5.6. DISCONNECTED is instantaneous; AMB_SATURATING/ONLY_LED_SATURATING are decided by which phase saturates; OT_HIGH/APPLIED by optical transmittance (single threshold + 0.2 s debounce, 100 samples @ 500 Hz — derived from `rsqm_probe_state_min_s × fs`).

> These states are computed by RSQM (_rsqm_update) and gate HGAC operation (§5.8.2): HGAC acts in `PROBE_APPLIED` and `PROBE_AMB_SATURATING`, and must not act in `PROBE_DISCONNECTED` or plain `PROBE_OT_HIGH / PROBE_ONLY_LED_SATURATING`.

---

## 13. Bibliographic references

Key references for each algorithm implemented in this library.

### 13.1 HR1 — Adaptive threshold peak detection

- **Pan, J. & Tompkins, W.J. (1985).** "A Real-Time QRS Detection Algorithm." *IEEE Transactions on Biomedical Engineering*, 32(3), 230–236.
  The classical reference for cardiac peak detection with adaptive threshold and refractory period. HR1 is a simplified adaptation for PPG.

- **Elgendi, M. et al. (2013).** "Systolic Peak Detection in Acceleration Photoplethysmograms Measured from Emergency Responders in Tropical Conditions." *PLoS ONE*, 8(10), e76585.
  More specific to PPG peak detection in challenging conditions.

### 13.2 HR2 — Normalised autocorrelation

- **de Cheveigné, A. & Kawahara, H. (2002).** "YIN, a fundamental frequency estimator for speech and music." *Journal of the Acoustical Society of America*, 111(4), 1917–1930.
  The primary reference for normalised autocorrelation-based fundamental frequency estimation. HR2 is directly derived from this approach.

- **Rabiner, L.R. & Schafer, R.W. (1978).** *Digital Processing of Speech Signals.* Prentice-Hall.
  Foundational coverage of autocorrelation methods for pitch estimation.

### 13.3 HR3 — FFT + Harmonic Product Spectrum

- **Schroeder, M.R. (1968).** "Period histogram and product spectrum: New methods for fundamental-frequency measurement." *Journal of the Acoustical Society of America*, 43(4), 829–834.
  Original paper introducing the Harmonic Product Spectrum concept.

- **Noll, A.M. (1969).** "Cepstrum pitch determination." *Journal of the Acoustical Society of America*, 41(2), 293–309.
  Historical context and comparison with HPS-based approaches.

- **Paradkar, N. & Chowdhary, S.R. (2017).** "Cardiac arrhythmia detection using photoplethysmography." *IEEE EMBC 2017*.
  Application of FFT-based HR estimation on PPG signals in a wearable context similar to this project.

### 13.4 SpO2 — AC/DC ratio (R-curve method)

- **Webster, J.G. (Ed.) (1997).** *Design of Pulse Oximeters.* IOP Publishing.
  The definitive reference. Covers Beer-Lambert law, the R ratio, empirical calibration, and sources of measurement error.

- **Jubran, A. (1999).** "Pulse oximetry." *Critical Care*, 3(2), R11–R17.
  Clear clinical review of the operating principle and limitations.

- **ISO 80601-2-61.** *Medical electrical equipment — Part 2-61: Particular requirements for basic safety and essential performance of pulse oximeter equipment.*
  Normative standard defining valid measurement ranges, tolerances, and test conditions. Source of the [70–100 %] SpO2 range. It does **not** set the HR range (§201.12 only requires disclosing it) — see §5.2.

- **WHO & UNICEF (2019).** *Technical specifications and guidance for oxygen therapy devices.* WHO Medical Device
  Technical Series, ISBN 978-92-4-151691-4. §6.5 and Annex 1 Table A1.11: procurement requirements for tabletop
  NICU pulse oximeters — pulse-rate range to include 30–240 bpm, ±3 bpm, update ≤ 10 s.

- **Severinghaus, J.W. & Astrup, P.B. (1986).** "History of blood gas analysis." *Journal of Clinical Monitoring*, 2(4), 270–288.
  Historical context for non-invasive oxygen saturation measurement.

---

## 14. Version history

| Version | Description                                                                 |
|---------|-----------------------------------------------------------------------------|
| v0.98   | **R method and R curve get names (§5.1, *R methods and R curves*).**           |
|         | `SPO2_R_METHOD_ID = "R-METHOD-1"` names how R is computed; it changes only     |
|         | when R would. The coefficients become an R curve with an id and the R method   |
|         | it was fitted with: `setSpO2Coefficients(a, b)` → `setSpO2RCurve(id,           |
|         | r_method_id, a, b)` (rejects empty/oversized ids and non-finite values,        |
|         | keeping the current curve whole); `AFE4490Config::spo2_a/spo2_b` →             |
|         | `spo2_r_method_id`, `spo2_r_curve_id`, `spo2_r_curve_r_method_id`,             |
|         | `spo2_r_curve_method_match`, `spo2_r_curve_a`, `spo2_r_curve_b`. Same root in    |
|         | every layer (lib, PulseNest `$CFG` fw 0.16, capture header, PILAB). Default    |
|         | curve labelled `R-CURVE-U401D-MS100-20260323`, fitted with `R-METHOD-0` (the   |
|         | 2026-03-23 library: adc_code inputs, DC τ 1.6 s, AC τ 1.0 s), so it raises the  |
|         | **mismatch warning** by itself — warning mode: logged and reported in the       |
|         | config, never a `diag_code` bit (that would zero `rsqi`). SpO2 values are       |
|         | unchanged (same coefficients, same R). Tests: `test_spo2` +3 (default labelled  |
|         | and warning, a matching curve applies, bad input rejected whole); 9/9 suites.  |
| v0.97   | **HR3 anti-alias low-pass: 2nd → 4th-order Butterworth, 15 Hz (§5.3.1, §5.4).**   |
|         | Closes the weak point recorded in v0.83: −9.4 dB at the 25 Hz Nyquist of the ×10  |
|         | decimation becomes −18.0 dB (at a 500 Hz PRF), so less folded noise lifts the     |
|         | spectral floor that `hr3_sqi` pays for, and flatter over the harmonics the HPS    |
|         | uses (13 Hz: −1.2 vs −1.9 dB). New `LowPass4` (two cascaded biquads,              |
|         | Q = 0.5412 / 1.3066, −3 dB at the corner); `BiquadFilter::init_lp()` takes an     |
|         | optional Q (default 1/√2, unchanged behaviour for any other caller). Member        |
|         | `_hr3_bpf` renamed `_hr3_lp`: it is not a bandpass, and its comment ("0.4–15 Hz")  |
|         | had been false since v0.83. Public API unchanged (`setHR3Filter()`,               |
|         | `hr3_f_high_hz`). Tests: `test_biquad` +4 (pulse band passes, −3 dB at the corner, |
|         | < 0.14 at 25 Hz where the 2nd order leaves > 0.30, unity DC gain + precharge);    |
|         | all 9 host suites pass (87 tests). Behaviour change: HR3 only.                    |
| v0.96   | **Offline replay accessors (§9).** `test_last_ot_led2()`, `test_probe_state()`,    |
|         | `test_pi()` (UNIT_TEST only), so `incunest_offline_runner` can feed raw ADC codes   |
|         | through `test_feed_sample()` (the full `_process_sample()` path: analog state →     |
|         | RSQM → SpO2/HR) and read back OT, probe state, R, PI and SpO2 exactly as the       |
|         | firmware would produce them — e.g. to replay the 2026-09-23 HOSPNAV captures with   |
|         | the v0.95 `rsqm_ot_thr` and recover the stretches discarded as PROBE_OT_HIGH.       |
|         | No behavioral change; test surface only.                                            |
| v0.95   | **`rsqm_ot_thr` 1.0e-4 → 1.5e-4 — first real-patient calibration (§5.6.2, §10).** |
|         | HOSPNAV campaign 2026-09-23, SUBJ08: probe applied read OT_LED1 mean ≈ 120 ppm /  |
|         | max ≈ 145 ppm and kept forcing PROBE_OT_HIGH (resetting SpO2/HR) on an applied     |
|         | probe — exactly the thin-tissue case the v0.44 note anticipated. Margin ×1.03      |
|         | above the observed max (n=1 patient, recalibrate with more campaigns); ×5.3 below  |
|         | the theoretical probe-in-air OT (~8e-4), which ONLY_LED_SATURATING also detects    |
|         | independently. No code path changed — default value and comments only.             |
| v0.94   | **HGAC announces its RF moves to the host (§5.8.4).** `_hgac_change_rf()` stamps    |
|         | `_hgac_rf_change_ts_us` (esp_timer, the row clock) then increments               |
|         | `_hgac_rf_change_count`; new lock-free getter `hgacRfChangeCount(uint64_t*)`.      |
|         | The library emits nothing (measurement task never calls the network); the         |
|         | consumer's command task polls it. Prerequisite 6 of PulseNest's                   |
|         | `capture_csv_format_spec.md` (R21a: RF as a change-event, not a column).          |
|         | test_hgac: `test_hgac_rf_change_is_counted_and_stamped`.                          |
| v0.93   | **`hr1_max_decay_tau_s` 20 s → 1.5 s — the first measured value this          |
|         | parameter has ever had (§10.4).** The 20 s was the inherited `0.9999`         |
|         | per-sample literal written in seconds at 500 Hz, and v0.87 said as much.      |
|         | A threshold held up by a peak seconds old loses any beat whose amplitude has  |
|         | since fallen, and **one lost beat costs five**: its doubled RR interval sits  |
|         | in the 5-interval buffer until it is pushed out, so `hr1_sqi` is 0 for five   |
|         | beats — 5.8 s at 52 bpm, which is exactly what the bench showed on 2026-09-15 |
|         | on two of three boards while HR2/HR3 never flinched. Note what did NOT fail:  |
|         | the SQI. Averaging that buffer gives 43 bpm, and publishing it would have     |
|         | been a false bradycardia — the alarm this monitor exists to raise. The gate   |
|         | was right; the detector fed it a bad interval. Evidence: the perturbed MS100  |
|         | sweep (flat optimum 1–3 s, collapse at 0.5 s) and the three-board probe       |
|         | capture (every missed beat recovered, zero extra detections). No API or       |
|         | format change: `setHR1MaxDecayTauS()` already existed and the value is a      |
|         | default. Mirrored in PulseNest's `HR1Variant.FW_MAX_DECAY_TAU_S`.             |
| v0.92   | **`$TIMING` gains `spi_mean,spi_max` (appended last) and the HAL gains a console   |
|         | tee.** `_ts_spi` times the six 4-byte SPI frames of one sample on their own — the |
|         | measurement the ESP-IDF port of PulseNest exists to make (Arduino 4 ×            |
|         | `SPI.transfer(uint8_t)` per frame vs one polling transmit under IDF). `hal_console_ |
|         | set_sink()`: every `hal_console_printf` line is also handed to an app sink,     |
|         | NUL-terminated, on the calling task (must only queue) — PulseNest forwards       |
|         | `$TIMING`/`$TASK` to its UDP data queue, so the ESP32 TIMING window needs no UART. |
|         | The idf and host `console_printf` now format into a buffer (`fwrite`) like the   |
|         | Arduino one. motherBoard: no behaviour change (`INCUNEST_TIMING_STATS` off, no    |
|         | sink).                                                                           |
| v0.91   | **Hardware abstraction layer (§1.4): the library builds unchanged on            |
|         | Arduino-ESP32, ESP-IDF (as a CMake component) and the host.** Phase 0 of the     |
|         | ESP-IDF port of IncuNest. Measured first: only `<Arduino.h>`/`<SPI.h>` differed  |
|         | between the embedded targets — 18 call sites — while FreeRTOS, esp_timer and     |
|         | IRAM_ATTR are the same API on both, so those are not wrapped; the host gets them |
|         | from `incunest_afe4490_hal_host.h`. New: `incunest_afe4490_hal.h` (7 functions), |
|         | `_hal_arduino.cpp` (pure forwarding, byte-for-byte the v0.90b bus sequence),    |
|         | `_hal_idf.cpp` (driver/gpio + spi_master, ONE polling transmit per 4-byte       |
|         | frame instead of four), `_hal_host.cpp`, root `CMakeLists.txt`. Selection by    |
|         | detection (`ARDUINO` / `ESP_PLATFORM` / neither), no macro to set. Removed: the |
|         | 63 `INCUNEST_OFFLINE` conditionals, `incunest_afe4490_platform_stub.h`, and     |
|         | PulseNest's `test/stubs/`. `constrain()` → own `clampf()` (gnu++11 has no       |
|         | std::clamp). `_rsqm_update()` lost the four raw-channel parameters it stopped   |
|         | reading in v0.35. `begin()` now calls `hal_spi_attach()` and checks the ISR      |
|         | attach; `stop()` calls `hal_spi_detach()`.                                       |
|         | **Found while proving the refactor pure:** Arduino-ESP32 redefines `ESP_LOGx`   |
|         | (`esp32-hal-log.h`); dropping `Arduino.h` silently switched the library to IDF's |
|         | `esp_log_write` on Arduino. Fixed by including `esp32-hal-log.h` under ARDUINO.  |
|         | **Evidence:** compiler output identical for 102/113 functions, the 11 others     |
|         | being exactly the touched ones; 10 Arduino symbols out, 8 HAL symbols in.       |
|         | Verified on Arduino (V17, -Werror), ESP-IDF v6.0.1 (esp32s3, -Werror, GCC 15.2),|
|         | host (link + run) and `pio test -e native` (76/77, unchanged).                   |
|         | No behavioural change on Arduino. On ESP-IDF the library now exists at all.     |
| v0.90b  | **Warning cleanup: the library compiles clean under `-Wall -Wextra -Werror`, |
|         | and `library.json` now enforces it for every PlatformIO build of the library.** |
|         | Prompted by the ESP-IDF port of motherBoard (IncuNest branch                 |
|         | `refactor/idf-native-port`), whose vendored copy of v0.81 silences seven      |
|         | `-Wno-error=` classes. Reproduced on xtensa GCC 8.4 (Arduino) and host GCC 15 |
|         | (C++17 and C++20): only **three** were ours. (1) `-Wreorder`: the constructor |
|         | initialised `_hr1_dc_alpha` before `_spo2_warmup_samples`, against            |
|         | declaration order — list reordered; no dependency between the members.        |
|         | (2) `-Wvolatile` ×2: `--` on the volatile countdowns                            |
|         | `_switched_rc_settling_countdown` and `_diag_holdoff_samples`, deprecated in   |
|         | C++20 — rewritten as `x = x - 1`, the same non-atomic read-modify-write.       |
|         | (3) `-Wunused-parameter` ×4: `_rsqm_update()` still took the raw LED/ALED      |
|         | values it stopped reading in v0.35 (everything arrives via `AFE4490AnalogState`)|
|         | — removed from the private signature and its single call site.               |
|         | The other four (`missing-field-initializers`, `unused-variable`,               |
|         | `format-truncation`, `deprecated`) do not reproduce on either compiler and have|
|         | no candidate in the code (the only `snprintf`s live under                     |
|         | `INCUNEST_TIMING_STATS`, off in motherBoard; aggregates use `{}`; no          |
|         | deprecated IDF API): the same list is applied verbatim to every vendored       |
|         | component of that port. **No behavioural change.** Also re-syncs the four     |
|         | file headers that still read v0.88.                                            |
| v0.90   | **ProbeState renamed and split; `isProbeAbsent()`.** `PROBE_SATURATING` → |
|         | `PROBE_AMB_SATURATING`: the old name suggested "any saturation", but since |
|         | v0.61 LED-only saturation never came here — engineers (and this spec's     |
|         | author) read the name and concluded the opposite of what the code does.   |
|         | `PROBE_NOT_APPLIED` split into its two physical causes, which were         |
|         | indistinguishable in frames, CSVs and the bench: `PROBE_OT_HIGH` (keeps    |
|         | value 1 — the usual cause in historical captures, so old CSVs still read   |
|         | correctly) and `PROBE_ONLY_LED_SATURATING` (new value 4). Names describe   |
|         | what is measured, not what is inferred: "probe in air" was an inference    |
|         | that fails on thin tissue at high RF.                                     |
|         | `isProbeAbsent()` is now the one definition of "no patient", used by HGAC's|
|         | gate G0 and the diagnostics task instead of two explicit lists — the trap  |
|         | G0's own comment warned about. **No behavioural change**: the LED-only    |
|         | case is absent, exactly as it was inside NOT_APPLIED.                      |
|         | **Recorded, not fixed — open check:** thin tissue at high RF →             |
|         | ONLY_LED_SATURATING → absent → HGAC gated off → RF never lowered → stays  |
|         | absent (§5.6.2). Alex observed the sequential-RF symptom on the bench; the |
|         | capture-based verification (`tools/probe_state_saturation_check.py`) is    |
|         | pending. **Second check, recorded because the split creates it:** the      |
|         | debounce counts consecutive identical requests, so alternation between the|
|         | two absent states near the saturation boundary could keep either from     |
|         | committing (see `_rsqm_request_probe_state`). Alex doubts it occurs; if    |
|         | seen, debounce by class.                                                   |
|         | **Vocabulary (same version):** the design labels "Gate G0" and "axiom A6"  |
|         | are gone from live text and code comments — "HGAC gate" and "the RSQM/HGAC |
|         | separation axiom" say what they are without a lookup. The old labels stay  |
|         | once each, in parentheses at the definitions (§5.8, `_hgac_update()`), so |
|         | earlier documents remain traceable. History rows keep their wording.      |
|         | External consumers: motherBoard uses only `==`/`!= PROBE_APPLIED` (9 sites) |
|         | and is unaffected. PulseNest script/tests/tools updated in the same commit.|
| v0.89   | **Presence by OT requires BOTH channels (`OR` → `AND`), §5.6.2.** With    |
|         | OR, whichever channel read higher decided alone, so one channel drifting  |
|         | over `rsqm_ot_thr` declared the probe absent while the other still saw    |
|         | tissue. On thin tissue (preterm foot, MS100 simulator) a correctly        |
|         | applied probe read as absent, resetting SpO2/HR1/HR2/HR3 every sample —   |
|         | the failure mode with no downstream safety net. AND requires two          |
|         | independent channels to agree; in air neither is attenuated by tissue, so |
|         | both stay high and NOT_APPLIED holds. Margin gained on thin tissue, none  |
|         | given back on air.                                                       |
|         | *Corrected 2026-09-07:* this entry first justified the change by claiming |
|         | RED is absorbed more than IR so `ot_led1 > ot_led2` always. Measured on   |
|         | the MS100, probe applied: IR 91 ppm, RED 112 ppm — the opposite. OT is    |
|         | the whole electro-optical ratio, not tissue transmittance. The AND does   |
|         | not depend on the ordering, so the change stands. Both channels sit       |
|         | within 12 % of the threshold: working, not robust.                       |
|         | **No threshold changed** — `rsqm_ot_thr` stays at 1.0e-4 and still lacks  |
|         | a calibrated value.                                                       |
|         | **New failure mode, accepted knowingly:** one channel persistently low    |
|         | (weak LED, obstructed PD) now forces APPLIED. The LED-only-saturation     |
|         | branch and DISCONNECTED's four-channel check limit but do not remove it.  |
|         | Chosen over raising the threshold because raising it spends margin on the |
|         | air side, and because the estimate that would justify a new value carries |
|         | uncertainty wider than the decision it would settle: neonatal optical     |
|         | path length is **not published** (the 5–10 mm figure in use is an         |
|         | engineering estimate, never measured), and OT is the whole                |
|         | electro-optical ratio, not tissue transmittance alone.                    |
| v0.88   | **Two buried literals become named parameters.** Prompted by an inventory  |
|         | of what actually conditions the algorithms: of ~45 configurable values     |
|         | plus internal constants, ~15 condition a result and 7 have no documented   |
|         | provenance — but two were not even parameters.                             |
|         | **`0.6f` → `hr1_threshold_fraction`**, now configurable via                |
|         | `setHR1ThresholdFraction()` and exposed in `AFE4490Config`. It is the      |
|         | single value that most determines whether a beat is counted, and it had no |
|         | name, no presence in the config or the spec, and could not be swept        |
|         | without recompiling — which blocked precisely the sensitivity study it     |
|         | most needs. The setter rejects anything outside (0, 1]: at 0 every sample  |
|         | crosses and above 1 nothing does, so both extremes silence HR1 rather than |
|         | degrade it.                                                                |
|         | **`3.0f` → `hr_guard_band_bpm`**, one constant replacing the same literal  |
|         | at four sites across HR2's lag sweep and HR3's bin sweep, where changing   |
|         | one and forgetting another would have desynchronised the two estimators    |
|         | silently. Kept as an internal constant: nothing suggests a caller should   |
|         | tune it.                                                                   |
|         | No behavioural change — both keep their previous values.                   |
| v0.87   | **Every time-valued parameter is expressed in seconds and derived from the |
|         | rate (rationale §6).** Audit prompted by the question "are there more       |
|         | unresolved sample-rate dependencies?" — there were two live and one latent. |
|         | **`_hr1_running_max *= 0.9999f`** was a per-sample literal: tau 10 000      |
|         | samples, i.e. 20 s at 500 Hz but **6.25 s at 1600 Hz**, relaxing HR1's      |
|         | detection threshold 3.2× faster at the top of the catalogue and inviting    |
|         | false positives. Now `hr1_max_decay_tau_s` (20 s, which reproduces 0.9999   |
|         | exactly at 500 Hz, so default behaviour is unchanged) with a setter.        |
|         | **`ts_emit_interval`** was 2500 samples — its own comment admitted "5 s at  |
|         | 500 Hz", so 1.56 s at 1600 — now derived from `ts_emit_period_s`.          |
|         | **`setHR2UpdateInterval`/`setHR3UpdateInterval` → `…IntervalS(float s)`**:  |
|         | they were the only two time-valued setters in the API taking samples, and   |
|         | immune to the AFE rate only because the decimated rate is the invariant.    |
|         | **`hr1_ma_cutoff_hz` → `hr1_lp_cutoff_hz`** (and `setHR1MaCutoffHz` →      |
|         | `setHR1LpCutoffHz`): "ma" stops being literal once the filter type is under |
|         | review, and "lp" is true of both. All breaking, all with zero external      |
|         | callers (checked in src/, tests and the lab script).                        |
|         | **The lasting part is the net, not the fixes:** two catalogue-sweeping      |
|         | tests assert every duration is preserved in seconds. Both had to be         |
|         | rewritten after passing with the defects reintroduced — a test must observe |
|         | the EFFECT (not the derived coefficient, which stays correct while unused)  |
|         | and must exercise the setter with a NON-DEFAULT value (a constant equal to  |
|         | the default is indistinguishable otherwise).                                |
| v0.86b  | **HR1 moving-average length 64 → 160 samples, clamp no longer silent      |
|         | (§7.4).** 64 samples covered only up to 640 Hz, so four of the five        |
|         | catalogue rates ran with an effective cutoff of 6.2–12.5 Hz instead of the |
|         | configured 5 Hz — and said nothing. That filter exists to keep the dicrotic |
|         | notch away from HR1's threshold detector; too wide, the notch is           |
|         | double-counted and HR reads double. The 0.185 s refractory mitigates it     |
|         | unevenly and least of all in bradycardia, where the notch arrives later     |
|         | and falls outside the refractory — i.e. it fails in the clinically         |
|         | critical case. 160 = worst case of the catalogue at the default cutoff.     |
|         | Cost is RAM only (+384 B measured: 60 444 → 60 828 B): the average is       |
|         | incremental so CPU is O(1) in N, and group delay stays ≈ 50 ms at any PRF.  |
|         | Since `setHR1MaCutoffHz()` is public and unvalidated no fixed capacity can  |
|         | cover every case (2 Hz at 1600 Hz would need 400 samples), so the clamp now    |
|         | emits `logE` with the length needed, the length available and the effective |
|         | by `test_hr1_ma_not_clamped_across_catalogue` (verified to fail with the    |
|         | cap back at 64: 64 samples instead of 80 at 800 Hz).                       |
| v0.86   | **A PRF change now tears down state accumulated at the previous rate       |
|         | (§7.4 "Changing the rate at runtime").** `setSampleRate()` recalculated    |
|         | coefficients but left four things belonging to the old rate in play: the   |
|         | biquad state (`init_bp`/`init_lp` deliberately do not reset it), the EMA   |
|         | state (`init()` recomputes α but keeps mean/var), the decimated HR2/HR3    |
|         | buffers (an 8 s window mixing two rates until it refilled) and the         |
|         | decimation phase. It also did not arm switched-RC settling, even though a  |
|         | rate change rewrites all four sampling windows — a signal-chain change in  |
|         | the sense of datasheet §7.7 t5, and the same reason RF, CF, ILED, AMBDAC,  |
|         | RG and Stage2En arm it. Now: no-op when the rate is unchanged, otherwise   |
|         | `_reset_algorithms()` plus an arm, under both mutexes.                     |
|         | **Lock order defined: `_spi_mutex` → `_state_mutex`** — the only place     |
|         | holding both at once, hence the order for the whole library. Holding       |
|         | `_state_mutex` across the mutation is also why settling can be armed AFTER |
|         | the reset here (which clears the countdown) rather than before the         |
|         | mutation as §5.8.4 requires of setters that do not hold it.                |
|         | Pinned by `test_rate_change_tears_down_old_state` (verified to fail with   |
|         | the fix reverted: buffer count 400 instead of 0) and                       |
|         | `test_same_rate_is_a_no_op`. **Recorded, NOT fixed:** HR1's moving average |
|         | saturates at 64 samples above 640 Hz, so four entries run with an          |
|         | effective cutoff of 6.2–12.5 Hz instead of 5 Hz.                           |
| v0.85   | **`AFE4490SampleRate`: the supported PRFs become a closed catalogue        |
|         | {500, 800, 1000, 1250, 1600 Hz} enforced by the type system (§7.4).**      |
|         | `setSampleRate()` previously accepted 63–5000 Hz while checking only the   |
|         | numeric bounds: above ~2487 Hz a legal call wrote INVERTED timer-compare   |
|         | pairs (`ALEDxSTC` > `ALEDxENDC`) into the chip, silently, and below        |
|         | 500 Hz the anti-alias margin collapses (§7.3). Both ends are now closed.   |
|         | Primary form takes the enum, so an unvalidated rate is inexpressible; the  |
|         | numeric overload is kept for the bench and `$SET sr` and accepts only      |
|         | catalogue values. The acceptance policy lives in ONE predicate,            |
|         | `isValidSampleRateHz()`. Introspection (`sampleRateCount()`,               |
|         | `sampleRateHz()`) is static, so a host application or a validation report  |
|         | can enumerate every supported rate before `begin()`.                      |
|         | **Present-day benefit: certifiability.** A finite set can be validated     |
|         | exhaustively — new `test_sample_rate` walks the whole catalogue asserting  |
|         | exact `q`, integer decimation factor, both sampling windows ≥ TI's 50 µs,  |
|         | non-inverted ambient timer pair and NUMAV_max ≥ 1, plus that rejected      |
|         | rates leave the configuration untouched and that both analysis chains hold |
|         | 50 Hz at every entry. A continuous range cannot be covered that way.       |
|         | **Designed to widen:** a FractionalDecimator would drop the `hz % 50`      |
|         | condition — the only one excluding an otherwise valid rate (625 Hz) — so   |
|         | widening means adding table entries or relaxing that predicate. No public  |
|         | signature changes, which is why the numeric overload was kept.             |
|         | `test_hr2`'s off-grid case (700 Hz) was retired: it is no longer reachable |
|         | through the API, by design. The rounding branch in `Decimator::configure()` |
|         | stays as defensive code and as the hook for that future decimator.        |
| v0.84c  | **`_hr2_seg` → `_hr2_snapshot`, `_hr2_linearize()` →                       |
|         | `_hr2_take_snapshot()`, `_hr3_linearize()` → `_hr3_prepare_fft_input()`.** |
|         | No functional change. `seg` said nothing, and `linearize` named the         |
|         | mechanism (walking a circular buffer) rather than the purpose: the copy    |
|         | exists so Task B sees a COHERENT window while Task A keeps writing         |
|         | `_hr2_buf` — the isolation that `_hr2_computing` guards. The comments       |
|         | already called it a "snapshot"; now the identifier does too, and a future  |
|         | memcpy-based implementation would still be a snapshot while no longer      |
|         | being a "linearize". The twin names also implied the two functions did the |
|         | same thing: HR2's only copies, HR3's copies AND subtracts the mean AND     |
|         | applies the Hann window, which `_hr3_prepare_fft_input()` now states.      |
| v0.84b  | **Decimation chains renamed `_pulse_band_decim` / `_harmonic_band_decim` → |
|         | `_hr2_decim` / `_hr3_decim`** (with their target-rate constants and test    |
|         | accessors). No functional change. The band-based names described a split    |
|         | that does not exist: a "pulse band" reaching 5 Hz covers the 5th harmonic   |
|         | of a 60 BPM pulse, and a "harmonic band" includes the fundamental. Naming   |
|         | after the consumer is accurate for all three lag estimators, since          |
|         | HR2_ACF / HR2_MPM / HR2_YIN share HR2's chain and buffer. Accepted risk,    |
|         | recorded in §5.3.1: the name goes stale if a non-HR2_* estimator ever feeds |
|         | off that chain, and the honest rename would then be after what each chain   |
|         | PRESERVES (HR2's: the fundamental across [30, 300] BPM; HR3's: up to the    |
|         | 3rd harmonic of 260 BPM, which its HPS multiplies).                        |
| v0.84   | **One decimation chain per filtered path (§5.3.1).** The single             |
|         | `_decim_factor` served both HR2 and HR3, implying a coupling that does not  |
|         | exist: the two paths share no filter, no buffer and no phase counter, so    |
|         | nothing forces a common rate and running them at one rate saves nothing —   |
|         | no shared convolution, no shared buffer, not one CPU cycle. Replaced by a   |
|         | `Decimator` struct (factor, phase, achieved rate) instantiated twice:       |
|         | `_hr2_decim` (BP 0.5–5 Hz → HR2_ACF, and MPM/YIN when they land)     |
|         | and `_hr3_decim` (LP 15 Hz → HR3), each deriving its factor from  |
|         | its own target. Both target 50 Hz today, so **no behavioural change** — but |
|         | §5.3.1 now records that this is a coincidence of requirements, not a        |
|         | constraint: only the harmonic chain needs 50 Hz (its 15 Hz low-pass stops   |
|         | protecting the decimation below that); the pulse chain has 5× of spare      |
|         | margin and at 25 Hz would cut the O(N·lags) autocorrelation by 3.9×,        |
|         | tripled once three estimators sweep the same buffer. What holds that back   |
|         | is lag resolution (4.8 vs 2.4 BPM at 120 BPM), which is precisely what MPM  |
|         | and YIN depend on — to be settled with the offline replay experiment, not   |
|         | by reasoning, and not during the campaign that compares the three.          |
|         | New `test_decim_chains_derive_independently` pins per-chain derivation at   |
|         | 500, 1000 and 700 Hz (the last one off-grid, to check the achieved rate is  |
|         | reported honestly rather than as the nominal target).                       |
|         | §5.5 now expands ACF / NSDF / MPM / CMND / YIN / HPS.                       |
| v0.83   | **The decimated analysis rate is now the invariant; the factor derives from |
|         | it (§5.3.1).** `_decim_factor = round(sample_rate / decim_target_rate_hz)`, |
|         | with `decim_target_rate_hz = 50 Hz` as the contract. Four constants are     |
|         | calibrated against that rate (`hr2_buf_len` = 8 s, `hr3_buf_len` = 10.24 s, |
|         | the update intervals = 0.5 s, `hr2_acorr_lag_cap` = 22 BPM); with the       |
|         | previous fixed factor of 10, changing the AFE rate moved all four at once   |
|         | and silently — at 1000 Hz the lag cap made 40 BPM unreachable, reporting    |
|         | real bradycardia as "no periodicity". Pinned by the new                     |
|         | `test_hr2_decimated_rate_invariant_to_sample_rate` (40 BPM at 1000 Hz — the |
|         | discriminating case; a 60 BPM check passes either way).                     |
|         | **§5.3.1 is new** and records, for the first time, WHY 50 Hz: it is the GCD |
|         | of the validated PRF grid (the highest rate with an integer factor at every |
|         | admissible PRF); below it HR3's 15 Hz low-pass stops protecting the         |
|         | decimation (Nyquist falls under the filter corner) and the windows grow     |
|         | into physiological non-stationarity; above it the fixed buffers grow too    |
|         | short in time. Weak point recorded: −9.4 dB of anti-alias attenuation at    |
|         | Nyquist, paid in `hr3_sqi`; the lever would be a 4th-order low-pass.        |
|         | **Naming (no functional change):** `fs2` (HR2) and `fs_dec` (HR3), two      |
|         | spellings of the same quantity, become `_decim_sample_rate_hz`, symmetric   |
|         | with `_afe_sample_rate_hz`; `hr2_decim_factor` + `hr3_decim_factor` become  |
|         | one `_decim_factor` (a third, dead copy lived in the .cpp anonymous         |
|         | namespace, shadowed); the local `fs` — which meant *full scale in mA* in    |
|         | two places and *sample rate in Hz* in another — becomes `full_scale_mA` /   |
|         | `sample_rate_hz`. Version checklist: the .cpp header still read v0.80, so   |
|         | it had been missed in v0.81 and v0.82 too.                                  |
|         | **API cleanup (breaking, deliberate):** `setHR3Filter()` loses its         |
|         | `f_low_hz` argument and `AFE4490Config` loses `hr3_f_low_hz`. Both were    |
|         | inert — `init_lp()` never reads a lower corner — and actively misleading:  |
|         | the reported value was a plausible 0.4 Hz left over from the constructor,  |
|         | so a caller passing another value got 0.4 back and could believe a         |
|         | bandpass was in effect. HR3 is a pure low-pass BY DESIGN (§5.4): the FFT   |
|         | needs the harmonics preserved in phase for the HPS, and the DC is removed  |
|         | by mean subtraction in `_hr3_prepare_fft_input()`. Neither field crossed the wire  |
|         | (no $CFG/$LCFG frame carried it), so no protocol or script change.        |
|         | Spec §5.4 also corrected: the low-pass corner is 15 Hz, documented as      |
|         | 10 Hz, and the parameter table claimed the filter "removes DC/baseline     |
|         | drift" — it does not; a low-pass passes DC.                                |
| v0.82   | **HR2 autocorrelation lag sweep: upper bound derived from `hr_min_bpm`      |
|         | instead of a hardcoded 137 samples (§5.3).** The constant dated from v0.11, |
|         | when the HR range was [25, 300] BPM (guard band 22 BPM at 50 Hz); the range |
|         | later moved to [40, 260] BPM but the lag bound was never revisited. Being   |
|         | fixed in *samples* while `fs2` is not, it also silently encoded a 50 Hz     |
|         | assumption. Now `max_lag = 60/(hr_min_bpm-3)*fs2`, symmetric with the       |
|         | existing `min_lag`, capped by storage (`hr2_acorr_max_lag` renamed to       |
|         | `hr2_acorr_lag_cap`) and by `hr2_buf_len/2-1` (unbiased-ACF noise limit),   |
|         | plus an explicit guard when fewer than 3 lags remain. **No output change at |
|         | 500 Hz** — peaks beyond the old bound were rejected by the `hr >=           |
|         | hr_min_bpm` check anyway — but the sweep drops from 127 to 71 lags, ~39 %   |
|         | less work in the O(N·lags) inner loop, and now tracks `setHrRange()`.       |
|         | New test `test_hr2_40bpm_lower_bound` pins the bottom of the range.         |
|         | **Documentation added in the same version (no code change):** new §7.3      |
|         | (anti-aliasing — the effective Rx bandwidth is the duty-cycled corner,      |
|         | `f_c,eff ≈ f_c,nominal × D` ≈ 70–110 Hz, not the nominal 500 Hz; margin     |
|         | 2.3–3.6× over Nyquist at PRF 500 Hz; datasheet Fig. 54) and new §7.4        |
|         | (the validated PRF grid {500, 800, 1000, 1250, 1600} Hz, the thermal        |
|         | headroom the PRF leaves under the LED duty, and two recorded non-criteria). |
|         | §7.2 corrected: it claimed the pre-sampler filter left "no out-of-band      |
|         | noise to alias" — false for a first-order pole; the CF conclusion stands.   |
| v0.81   | **Rename: `AFE4490DebugData::hgac_rf_led1/led2` → `rf_led1/led2` (§3d).**  |
|         | Alex, reviewing the new PulseNest `HGAC_RF1/RF2` CSV columns: the name      |
|         | implies HGAC computed the value, but it's just the live RF config          |
|         | (identical to `_afe_tia_rf_led1/led2`) — it can equally come from a manual  |
|         | `$SET`. Consumer-side rename in PulseNest (`main.cpp`, `pulsenest_lab.py`,  |
|         | `pulsenest_lab_spec.md`) done in the same pass. No behaviour change.        |
| v0.80   | **Bug fix: cross-thread race let a manual `$SET` gain/current/AMBDAC       |
|         | change get combined with a stale raw ADC code before the settling window    |
|         | was even armed (§5.8.4).** Distinct from v0.79: that fix covered            |
|         | *recomputing* while already frozen; this one covers how the frozen snapshot |
|         | gets *saved* in the first place. The six arming setters mutated their live  |
|         | circuit-parameter member, *then* called `_arm_switched_rc_settling()` —     |
|         | protected only by `_spi_mutex`, never `_state_mutex` (`_process_sample()`'s |
|         | lock). A manual `$SET` runs on `Cmd_Task`, a different FreeRTOS task; if    |
|         | `_task_body()` ran `_process_sample()` in the mutate→arm gap, it saw the    |
|         | countdown still at 0, recomputed fresh, and combined an old raw code with   |
|         | the already-new parameter — latching the wrong value into                  |
|         | `_last_valid_analog_state` for the whole settling window. HGAC was never    |
|         | exposed (mutates+arms sequentially, single-threaded, under `_state_mutex`). |
|         | Suspected by Alex from a real capture ("a veces FW_OT_LED1 parece congelado |
|         | con la RF nueva"). Fix: reorder to arm-before-mutate in all six setters     |
|         | (`setTIAGainLED1/2`, `setLED1/2Current`, `setAmbDac`,                       |
|         | `setStage2Gain`/`setStage2GainLED1/2`, `setStage2En1/2`) — safe because the |
|         | countdown value depends only on `_afe_sample_rate_hz`, never on which       |
|         | parameter changed. No new locks (rejected taking `_state_mutex` in the      |
|         | setters: `_hgac_change_rf()` already nests a setter call under a           |
|         | caller-held `_state_mutex`, so `Cmd_Task` acquiring `_spi_mutex` then       |
|         | `_state_mutex` would invert the lock order → deadlock). Not covered by a    |
|         | new automated test — the native suite is single-threaded and can't          |
|         | reproduce cross-thread interleaving; the fix rests on the ordering argument |
|         | above. Verified: `pio test -e native` 63/64 (unrelated pre-existing         |
|         | `test_biquad` failure untouched); ESP32-S3 build (`incunest_V16`) SUCCESS.  |
| v0.79   | **Bug fix: freezing raw ADC codes during switched-RC settling wasn't        |
|         | enough — `_compute_analog_state()` still combined them with LIVE (already   |
|         | changed) circuit parameters, producing a stepped `OT` instead of a flat one |
|         | (§5.8.4).** Found by Alex from a real capture: `ppg_disp` showed an abrupt  |
|         | pulse (two steps bracketing the settle window) instead of the intended      |
|         | smooth decay — RSQM/SpO2/HR1-3 were silently exposed to the same wrongly-   |
|         | scaled `OT` for the whole window, not just the display. Fix: new member    |
|         | `_last_valid_analog_state` (the full `AFE4490AnalogState`, not just raw     |
|         | codes) saved every normal cycle; `_process_sample()` reuses it verbatim     |
|         | while `_should_freeze_input()` instead of recomputing from stale-code +    |
|         | live-parameter combinations. Chosen over freezing all 9 individual circuit  |
|         | parameters (RF/RG/EN/AMBDAC/ILED separately) — fixes the mismatch by       |
|         | construction, one new member instead of nine, `_compute_analog_state()`     |
|         | untouched. New test `test_analog_state_stays_flat_during_settle_window`     |
|         | (PulseNest `test_hgac.cpp`) proves it the strong way: feeds a bogus raw     |
|         | code while frozen and asserts `OT` doesn't move. Verified: `pio test -e     |
|         | native` 63/64 (unrelated pre-existing `test_biquad` failure untouched);     |
|         | ESP32-S3 build (`incunest_V16`) SUCCESS.                                    |
| v0.78   | **Docs only: comment on `_apply_analog_regs()` explaining why it's always   |
|         | called unconditionally, even when a setter's value didn't change** — asked  |
|         | by Alex while reviewing the `changed` guards added in v0.74-v0.77. Several  |
|         | fields share one register (e.g. `TIAGAIN` packs `RF_LED1`+`CF_LED1`+        |
|         | `STG2GAIN1`+`STAGE2EN1`+`ENSEPGAIN`), so any SPI write already retransmits  |
|         | unchanged neighbours; rewriting the same bits has no physical effect (no    |
|         | voltage step, nothing re-settles) and doubles as a forced resync — unlike   |
|         | `_arm_switched_rc_settling()`, which IS guarded because it has a real cost  |
|         | (freezes live data ~16 ms). No code change.                                |
| v0.77   | **`setStage2En1()`/`setStage2En2()` also arm switched-RC settling (§5.8.4)  |
|         | — the follow-up decision left open in v0.76.** Bypassing Stage 2            |
|         | (`STAGE2EN=0` → unity gain) vs. enabling it (`STAGE2EN=1` → gain `RG`)      |
|         | steps the effective gain exactly like `setStage2Gain()` itself, so the same |
|         | protection applies (same no-op guard: toggling to the current enable state  |
|         | doesn't re-arm). New tests `test_stage2_en_toggle_arms_settling`,           |
|         | `test_setting_same_stage2_en_does_not_rearm_settling` (PulseNest            |
|         | `test_hgac.cpp`). Verified: `pio test -e native`; ESP32-S3 build            |
|         | (`incunest_V16`) SUCCESS.                                                    |
| v0.76   | **Switched-RC settling extended to ILED, AMBDAC, and RG — not just RF       |
|         | (§5.8.4).** Datasheet §7.7 t5 footnote (1) names "LED current setting, TIA  |
|         | gain, and so forth" as triggers — RF was the only one wired up through      |
|         | v0.75. `setLED1Current()`/`setLED2Current()`, `setAmbDac()`, and            |
|         | `setStage2Gain()`/`setStage2GainLED1/2()` now call                          |
|         | `_arm_switched_rc_settling()` themselves too, with the same no-op guard as  |
|         | RF (re-applying the same value doesn't re-arm). AMBDAC and RG sit in the    |
|         | identical feedback path as RF (Eq. 2) — a step in either produces the same  |
|         | kind of transient. `setTIACF*()` deliberately excluded: CF only shapes      |
|         | transient response, not the DC operating point (no CF term in Eq. 2).       |
|         | `setStage2En1/2()` (which also steps effective gain, unity↔RG) noticed but  |
|         | left for a follow-up decision, not folded in silently. New tests            |
|         | `test_led_current_change_arms_settling`, `test_ambdac_change_arms_settling`,|
|         | `test_stage2_gain_change_arms_settling`,                                    |
|         | `test_setting_same_led_ambdac_rg_does_not_rearm_settling` (PulseNest        |
|         | `test_hgac.cpp`). Verified: `pio test -e native`; ESP32-S3 build            |
|         | (`incunest_V16`) SUCCESS.                                                    |
| v0.75   | **Rename: `_rsqm_settling_countdown`/`RSQM_DIAG_HW_SETTLING`/               |
|         | `_arm_settling()`/`_compute_afe_settle_samples()` → their                   |
|         | `*_switched_rc_settling_*` equivalents (§5.8.4).** Found by Alex: "settling"|
|         | alone was ambiguous among three unrelated phenomena in this library — the   |
|         | TIA's own RC pole (τ=RF·CF, one sample), the LED/cable transient (also one  |
|         | sample), and this one (several PRP cycles, the datasheet's 3 ms minimum).   |
|         | "Switched RC" is the datasheet's own term (§7.7 t5 footnote (1): "four      |
|         | switched RC filters"), chosen over `gain_settling` (names the trigger, not  |
|         | the mechanism — would misdescribe a future ILED-triggered case, since ILED  |
|         | is a current, not a gain) and `sample_hold_settling` (not the datasheet's   |
|         | term). Also fixed the `RSQM_DIAG_*` bit comment (§3) to explain the         |
|         | distinction from the other two settling phenomena. Historical changelog    |
|         | rows below (v0.35–v0.72) intentionally still use the old names — they      |
|         | describe what those commits did at the time, not the current identifiers.  |
|         | Internal-only, no behaviour change. Verified: `pio test -e native`;         |
|         | ESP32-S3 build (`incunest_V16`) SUCCESS.                                    |
| v0.74   | **A manual RF change now arms the settle window too, not just an HGAC one   |
|         | (§5.8.4).** Found by Alex: `_rsqm_settling_countdown` was only armed inside |
|         | `_hgac_change_rf()` — calling `setTIAGainLED1/2()`/`setTIAGain()` directly  |
|         | (e.g. a manual `$SET,tiagain1,...` from the host) changed RF with zero      |
|         | input-freeze protection (v0.72), even though the same physical transient   |
|         | occurs regardless of who initiated the change. New `_arm_settling()`        |
|         | (guarded: re-applying the same value is a no-op) is now called from the     |
|         | three public setters; `_hgac_change_rf()`'s own separate arm removed (gets  |
|         | it for free through the setter it already calls).                          |
|         | **`_rsqm_settling_countdown` is now `volatile`** — with the arm reachable   |
|         | from any caller, it can be written from a different thread (e.g. `Cmd_Task` |
|         | on a manual `$SET`) than the one that reads/decrements it (`_task_body()`). |
|         | No mutex: both sides only ever do a plain 32-bit store or decrement, never  |
|         | a cross-thread read-modify-write, so the worst-case race is benign (±1     |
|         | sample of settle duration) — same reasoning already applied to             |
|         | `_diag_active`/`_diag_holdoff_samples`.                                     |
|         | New tests `test_manual_rf_change_also_arms_settling` and                   |
|         | `test_setting_same_gain_does_not_rearm_settling` (PulseNest `test_hgac.cpp`). |
|         | Verified: `pio test -e native`; ESP32-S3 build (`incunest_V16`) SUCCESS.    |
| v0.73   | **Docs only: `tia_settle_min` PRF dependence reviewed (§7.2).** Confirmed   |
|         | it does NOT scale with PRF (flat `constexpr`, always 12.5 µs regardless of  |
|         | `_afe_sample_rate_hz`) — and that this is almost certainly the right shape, |
|         | since its purpose (LED/cable turn-on settling, §8.3.1.3) is a hardware      |
|         | property independent of pulse rate. Sharpened the existing provenance note  |
|         | (`9fb2831`): datasheet Table 2's own footnote (1), p.36, states its values  |
|         | are "based off PRF = 500 Hz and duty cycle = 25%" — the "50" `tia_settle_min` |
|         | was lifted from is TI's single worked example, never stated as a rule for  |
|         | any other rate. No code change: the formula's structure (fixed floor +     |
|         | PRF-proportional term) was already correct, only the exact magnitude has   |
|         | weak provenance, same class of finding as `tia_settle_fraction`/`tia_n_tau`. |
| v0.72   | **`RSQM_DIAG_HW_SETTLING` is now enforced, not just labelled** (found by    |
|         | Alex from a visible spike in `ppg_disp` whenever HGAC changes RF).         |
|         | `_task_body()`'s freeze condition — `_diag_active \|\| _diag_holdoff_samples`|
|         | `> 0 \|\| _rsqm_settling_countdown > 0` — now also covers the settling window |
|         | opened by `_hgac_change_rf()` (§5.8.4), reusing the exact same frozen-      |
|         | last-valid-input mechanism already used for the post-diagnostic holdoff:   |
|         | "same physical phenomenon (analog front-end re-settling), two different    |
|         | triggers." Before this, the bit was set in `diag_code` but nothing         |
|         | actually stopped SpO2/HR1/HR2/HR3/`ppg_disp` from processing the real      |
|         | (transient) values during that window — only HGAC's own Gate G0 checked    |
|         | it, to pause its own actuation. `_rsqm_settling_countdown` is decremented   |
|         | inside `_process_sample()` (`_rsqm_update()`) unconditionally, whether fed  |
|         | frozen or fresh inputs, so no extra bookkeeping was needed beyond the OR    |
|         | term. Renamed `_diag_last_*` → `_last_valid_*` (no longer diagnostics-      |
|         | specific) across `.h`/`.cpp`. Fixes `ppg_disp`'s visible spike AND a        |
|         | latent, previously-unreported issue: SpO2/HR1/HR2/HR3 were silently        |
|         | exposed to the same RF-change transient. Verified: `pio test -e native`;   |
|         | ESP32-S3 build (`incunest_V16`) SUCCESS.                                    |
| v0.71   | **Docs only: plausible physical derivation of `tia_axis::LIN_V` = 1.8 V**   |
|         | (§3c), previously purely empirical (`tia_linearity_sweep.py`, no theory     |
|         | recorded). Alex connected two independently-documented datasheet facts      |
|         | never linked before: §8.3.4 regulates the receive analog rail to 1.8 V      |
|         | internally, and §8.3.1.1's ambient-DAC common-mode (0.9 V, "set             |
|         | internally", p.11) is exactly half of it. A symmetric fully-differential    |
|         | output centred at 0.9 V on a 0–1.8 V rail reaches its maximum differential  |
|         | swing (one node at 1.8 V, the other at 0 V) at exactly 1.8 V — matching     |
|         | `LIN_V` exactly, with `FS_V` (1.0 V, TI-guaranteed) sitting inside it as    |
|         | guardband, and the measured hard clip (~1.94 V) slightly past it as         |
|         | expected real-silicon headroom, not a contradiction. Not verified against a |
|         | schematic — recorded as the most plausible explanation found, not a         |
|         | certainty. No code change.                                                  |
| v0.70   | **500 Hz post-stage2 filter pole vs. settle window (§5.8.4) reviewed —      |
|         | closes the last open item from the v0.65 settling work.** Computed          |
|         | numerically across the full `setSampleRate()` range (63–5000 Hz): at every  |
|         | rate this library is actually used at (500 Hz default), the settle window  |
|         | covers the filter's 5τ (1.59 ms) with 10×–39× margin. Docs only, no code    |
|         | change for this part.                                                       |
|         | **Found in the process (docs only, not fixed):** the settle-window          |
|         | calculation itself collapses above ~2487 Hz (still within the documented   |
|         | valid range of `setSampleRate()`) — a more severe, now-quantified           |
|         | re-statement of the pre-existing "Valid for PRF ≤ ~2 kHz" caveat already on |
|         | `ambient_margin`. Above that threshold `_compute_ambient_sample_window_counts()` |
|         | returns 0, collapsing `_compute_afe_settle_samples()` to its 1-sample floor  |
|         | (under even the raw 3 ms datasheet requirement) and, at the register level,  |
|         | `_apply_timing_regs()` would write an inverted `ALEDxSTC > ALEDxENDC` pair.  |
|         | Out of scope here (this project only runs at 500 Hz) — tracked in memory.   |
| v0.69   | **`ppg_disp` migrated to the OT domain — the last consumer of raw           |
|         | `led1_sub`/`led2_sub` joins the gain-invariance migration HR1/HR2/HR3/SpO2  |
|         | already went through (§5.1/§5.2, since v0.38).** Found by Alex: `ppg_disp`  |
|         | was the only signal still scaled by RF, so it visibly jumped whenever HGAC  |
|         | changed gain — the exact artefact OT fixed everywhere else.                 |
|         | `AFE4490Data::ppg_disp`: `int32_t` → `float` (OT magnitude ~1e-5..1e-6      |
|         | truncates to 0 as an int). Source: bandpass-filtered `ot_led1`/`ot_led2`    |
|         | (was `led1_sub`/`led2_sub`). `AFE4490Channel` enum shrunk from 6 values to  |
|         | 2 (`LED1`/`LED2`): the four single-ended raw options (bare LED1/LED2/       |
|         | ALED1/ALED2) have no OT equivalent — OT is structurally an ambient-         |
|         | corrected ratio (`AFE4490AnalogState` has no `ot_aled1`/`ot_aled2`) — and   |
|         | `setPPGChannel()` was their only consumer, so they were dropped rather than |
|         | kept meaningless. NOT normalized for display (deliberately deferred — a    |
|         | candidate `ppg_disp_norm`, PI-weighted [0..1], is backlog, not this         |
|         | change). §1.3/§2.1/§3 corrected in the same pass: fixed pre-existing spec   |
|         | drift found while touching this code — stale field names                   |
|         | (`led2_aled2`/`led1_aled1`, actual code has used `led2_sub`/`led1_sub` for  |
|         | some time), a phantom `int32_t ppg` struct field that no longer exists in   |
|         | the header, and a processing-chain diagram that still routed HR1 through    |
|         | `setPPGChannel()` when it has consumed `ot_led1` directly since v0.38.      |
|         | PulseNest: wire frames `$M1`-`$M4` change `PPG_DISP`'s format from `%ld`    |
|         | to `%.4e` (units A/A, was ADC counts); `channel_str()` in `main.cpp`        |
|         | updated for the 2-value enum. `pulsenest_lab.py`/`pulsenest_lab_spec.md`:   |
|         | `PPG_DISP` tooltip/type corrected (parsing was already `float(x)`, no      |
|         | code change needed there); `LED1_SUB`/`LED2_SUB` tooltips also corrected    |
|         | in the same pass — they still claimed to feed HR1/HR2/HR3/SpO2, another    |
|         | pre-existing drift from the same v0.38 OT migration that was never         |
|         | reflected in the lab's tooltips.                                            |
| v0.68   | **CF/RF atomicity reviewed (§5.8.4) — no bug, closes an open question from  |
|         | the v0.65 settling work.** `RF_LEDx`/`CF_LEDx` share the same 24-bit         |
|         | register (`TIAGAIN`/`TIA_AMB_GAIN`), and `setTIAGainLED1/2()` recomputes CF  |
|         | for the new RF *before* the single SPI write in `_apply_analog_regs()` — the |
|         | chip never observes a transient RF-new/CF-old combination, and `getConfig()` |
|         | reads both fields under the same `_spi_mutex` (no torn read either).         |
|         | Documented inline in `_hgac_change_rf()` and in §5.8.4. New regression test  |
|         | `test_hgac_change_rf_recalculates_cf` (PulseNest `test_hgac.cpp`) exercises  |
|         | CF recalculation through the HGAC entry point specifically (previously only  |
|         | covered via the public `setTIAGain()` in `test_tia_cf.cpp`). Also fixes      |
|         | `INCUNEST_AFE4490_VERSION` macro, stuck at "0.64" since v0.65 (comment/      |
|         | `library.json` were bumped correctly each time, this `#define` was missed). |
|         | Comments/tests only, no behaviour change.                                    |
| v0.67   | **Rename: `_compute_settle_margin()` → `_compute_led_on_to_sample_margin_counts()`** |
|         | (found by Alex reviewing v0.66). "Settle" was ambiguous two ways: (1) the    |
|         | datasheet's stated cause is LED/cable settling (§8.3.1.3), but the library   |
|         | also reuses the same figure as the TIA-τ budget for auto-CF — a name        |
|         | asserting either cause alone would be partly wrong for the other call site;  |
|         | (2) it collided with the unrelated `_compute_afe_settle_samples()` (v0.65,   |
|         | the datasheet-t5 post-register-change wait) — same word, different          |
|         | timescale (µs-level per-phase margin vs. ms-level post-config countdown).    |
|         | New name describes what the value structurally IS (`LEDxSTC = LEDxLEDSTC +` |
|         | this margin) without taking a side on WHY, and adds the `_counts` suffix    |
|         | for consistency with `_compute_led_sample_window_counts()` /                |
|         | `_compute_ambient_sample_window_counts()` (v0.66). Internal-only, no        |
|         | behaviour change.                                                           |
| v0.66   | **Rename: `_compute_sample_window_counts()` only ever covered the LED phase** |
|         | (found by Alex reviewing v0.65) — renamed to `_compute_led_sample_window_counts()`, |
|         | paired with a new symmetric `_compute_ambient_sample_window_counts()` (was  |
|         | inline arithmetic duplicated inside `_compute_afe_settle_samples()`). Both now |
|         | delegate to a shared `_window_counts(margin)` clamp, removing the duplicated |
|         | `(q > margin+2) ? q-2-margin : 0` formula. `_compute_afe_settle_samples()` is  |
|         | now `min(led(), ambient())` — no inline `q`/margin arithmetic left in it.    |
|         | `getCFMaxEq1PF()` updated to the new LED-window name (datasheet Eq. 1's "Rx  |
|         | Sample Time" is specifically the LED window, per Fig. 58 — never ambient).   |
|         | Internal-only, no behaviour change; `pio test -e native` unaffected.        |
| v0.65   | **`afe_settle_time_s` (0.15 s fixed guess) replaced by `_compute_afe_settle_samples()`,** |
|         | derived from datasheet §7.7 t5 (p.17): "> 3 ms of cumulative sampling time  |
|         | in each phase" after any signal-chain change, footnote (1) naming "TIA     |
|         | gain" explicitly. New private method computes `ceil(3 ms / phase_window_s)` |
|         | from the shorter of the LED/ambient sample windows (both symmetric between  |
|         | LED1/LED2, so one figure covers either HGAC color domain); depends only on  |
|         | `_afe_sample_rate_hz`, recomputed live instead of hardcoded. At the default  |
|         | 500 Hz config this yields 8 samples (~16 ms) vs. the old 150 ms — ≈9×       |
|         | shorter and, unlike the old value, traceable to the datasheet instead of an |
|         | undocumented guess (§5.8.4, §7.2). New named constant                      |
|         | `afe_ambient_margin_counts` (was a bare `400` literal in                    |
|         | `_apply_timing_regs()`) shared by both call sites. §5.8.4 total-delay        |
|         | figures (guard/leveling re-arm after an RF step) updated accordingly.       |
| v0.64   | **HGAC: `hgac_v_tia_low1` default 0.25→0.20 V** (§5.8.1). Widens the        |
|         | leveling dead-band by lowering LOW1, not raising HIGH1: HIGH1 is capped by   |
|         | its 0.15 V margin to HIGH2/GUARD (safety against saturation), while LOW1 has |
|         | no symmetric lower guard (`v_tia_diff ≥ 0` always) — widening there only     |
|         | costs SNR, never safety. Also fixes the tightest anti-hunting margin (0.05 V |
|         | at the RF ×2.5 step boundaries) found for the old value; new stability      |
|         | invariant `HIGH1/LOW1 = 3.75 > 2.5×` (was 3.0). PulseNest's `test_hgac.cpp`  |
|         | no longer hardcodes a raw ADC code for its weak-signal case — it derives it  |
|         | from the object's own `hgac_v_tia_low1` (via `getConfig()`), so this default |
|         | can move again without silently breaking the test.                          |
| v0.63   | **Datasheet Equation 1 exposed as a check (no behaviour change).** New       |
|         | `getCFMaxEq1PF(rf)` returns the chip's own CF ceiling                         |
|         | (`RF × CF ≤ Rx Sample Time / 10`, §8.3.1.1), plus private                     |
|         | `_compute_sample_window_counts()` (= q − 2 − margin). Auto-CF is ≈4.5×        |
|         | stricter and provably stays inside Eq. 1 (`test_autocf_always_within_eq1`);   |
|         | a MANUAL `setTIACF*()` can exceed it and is deliberately not clamped —        |
|         | PulseNest now emits `# WARN … applied anyway` after such a $SET.              |
|         | Spec §7.2 documents that `tia_settle_fraction` and Eq. 1's `/10` govern       |
|         | DIFFERENT decisions and must not be substituted: per §8.3.1.3 the sample-     |
|         | start delay exists for **LED/cable** settling, not TIA settling, so Eq. 1     |
|         | (which only bounds CF) cannot supply a criterion for it. Both numbers kept    |
|         | unchanged: the margin is conservative but free, and shrinking it without      |
|         | measuring the LED/cable transient risks sampling before the LED settles.      |
|         | Also corrects the strictness ratio stated in v0.62: ≈4.5×, not ~10×.          |
|         | `test_tia_cf` grows 9 → 13 cases.                                            |
|         | §7.2 additionally records the provenance of all three settle constants:       |
|         | NONE comes from the datasheet. `tia_settle_min`=50 is TI's Table 2 example    |
|         | value; `tia_settle_fraction`=0.10 is probably Eq. 1's `/10` misapplied;       |
|         | `tia_n_tau`=5.0 is the generic five-time-constants rule of thumb (its         |
|         | "< 0.7% error" comment is circular — 0.7% IS e⁻⁵). The datasheet's implied    |
|         | count is 10τ, so raising `tia_n_tau` 5→10 "to match TI" while keeping the     |
|         | margin basis would make the criterion twice as STRICT, not laxer — the three  |
|         | constants only make sense together. All kept unchanged.                       |
| v0.62   | **TIA CF: expose all 32 register codes (was 6).** Found by Alex reviewing     |
|         | `_recalc_afe_tia_cf_led1()` against datasheet Figure 114. `CF_LED[4:0]` is    |
|         | not a 6-value enum: it switches five capacitors in parallel over a fixed 5 pF |
|         | base (weights 5/15/25/50/150 pF), so all 32 combinations are legal —          |
|         | 5…250 pF. The old `enum class AFE4490TIACF` covered only the six single-bit   |
|         | settings; its values were CORRECT but the grid was incomplete, making auto-CF |
|         | under-select. Changes: `AFE4490TIACF` → `using AFE4490CFCode = uint8_t` (the  |
|         | code IS the value); `kAFE_CF_PF[6]` → `[32]`; new `kAFE_CF_STR[32]`,          |
|         | `afeCFCodeToPF()`, `afeCFPFToCode()` (quantise DOWN — never overshoot the 5τ  |
|         | budget); `cf_code[]` LUT deleted (now `(code & 0x1F) << 3`); setters take pF  |
|         | (`setTIACF(float)`), mirroring `_quantize_led_mA()`; `AFE4490Config` exposes  |
|         | `afe_tia_cf_ledN_code` + `_pF`. `afeStrToCF()` still accepts the six pre-v0.62 |
|         | strings → no $CFG/$SET protocol break. `kAFE_CF_MATCH_TOL` (1e-4) added: the  |
|         | budget computes 99.9999924 instead of 100.0 in float, which would have        |
|         | silently dropped a whole step and negated the change. **Behaviour change:**   |
|         | auto-CF at the default 500 Hz / RF_100K now picks **100 pF (was 55 pF)**;     |
|         | f_pole ≈ 15.9 kHz, far above the PPG band. Every RF/Fs combination improves   |
|         | or is unchanged; none violates 5τ.                                            |
|         | New `test_tia_cf` (9 cases) covers the LUT vs. datasheet weights, quantise-   |
|         | down, round-trips, legacy strings and the float-tolerance regression.         |
|         | **Verified on 16.A (2026-08-17):** sweeping CF over its full 5–250 pF range   |
|         | changes neither V_TIA (±1 mV = drift) nor `LED1_SUB` %SD/Mean (0.01 → 0.01).  |
|         | An earlier claim in this changelog that the change bought "~1.8× more noise   |
|         | filtering" was NOT supported by measurement and has been removed: the         |
|         | internal 500 Hz filter (FLTRCNRSEL) dominates the noise bandwidth and sits    |
|         | before the sampler. v0.62's value is fixing an incomplete register grid, not  |
|         | improving SNR. See §7.2 "Measured behaviour" for the data, the write-path     |
|         | control, and why the auto-CF criterion is ≈4.5× stricter than datasheet Eq.1. |
| v0.61-experiment (doc fix) | **§5.8.4 corrected: `afe_settle_time_s` and the EMA warmup are SEQUENTIAL, not overlapped.** |
|         | No code change — the spec previously claimed the 3·τ EMA warmup "comfortably absorbs"  |
|         | the 150 ms settling window (implying they run in parallel). Wrong: Gate G0             |
|         | (`_hgac_update()`) closes off while `RSQM_DIAG_HW_SETTLING` is active and `reset()`s the |
|         | four EMAs on EVERY sample during that window (not once) — `count` stays pinned at 0, so  |
|         | `EmaChannel::update()` is never called and the warmup counter cannot advance until the  |
|         | settling countdown reaches 0. Corrected total delay after an RF change: guard operative  |
|         | again at `afe_settle_time_s` + 3×τ_fast = 0.45 s (was documented as 0.3 s); leveling/    |
|         | ambient at + 3×τ_slow = 6.15 s (was 6 s). Found by Alex while reviewing the design.      |
| v0.61-experiment | **ProbeState classifier: discriminate saturation by phase (probe-in-air vs external light).** |
|         | Fixes HGAC driving RF to the floor when the finger is removed. Was: *any* positive     |
|         | saturation → PROBE_SATURATING → HGAC acts. Now (`_rsqm_update`): if not saturated, OT   |
|         | classifies presence as before; if saturated, look at WHICH phase clips — an ambient    |
|         | (ALED) phase also clipping → external light → PROBE_SATURATING (HGAC corrects); only    |
|         | LED phases clipping, ambient clean → the LED's own light hits the PD with no tissue →   |
|         | **PROBE_NOT_APPLIED** (probe in air), so HGAC freezes RF instead of chasing it down.    |
|         | OT is the discriminator's basis (ambient-invariant, subtracts ALED). Limitation:       |
|         | LED-only clip can't tell air from a rare no-ambient tissue over-saturation → assumed    |
|         | air. Spec §5.6.2 rewritten; PROBE_SATURATING enum comment updated. HGAC unchanged.      |
|         | test_hgac: the old "LED-dominated → descend" test became "LED-only sat = probe in air → |
|         | HGAC frozen". 38/38 native tests, ESP32-S3 V16 build OK.                                 |
| v0.60-experiment | **Move the ambient alarm out of `_hgac_track()` (separate actuation/diagnostic).** |
|         | `_hgac_track()` is pure actuation again (returns void); the AMBIENT_HIGH condition moves|
|         | to a new `_hgac_ambient_high(color, ambient)` evaluated in `_hgac_update()`. Same        |
|         | producer (HGAC — depends on RF state, so not RSQM), but no longer nested in the guard   |
|         | branch. Behaviour change: the alarm no longer requires the LED guard (`fast`) to have    |
|         | tripped — condition is now just `RF==floor && ambient≥HIGH2` (ALED is the direct ambient |
|         | indicator). Cleaner separation and drops the spurious `fast` dependency. Tests unchanged |
|         | (8/8), ESP32-S3 V16 build OK.                                                            |
| v0.59-experiment | **HGAC ambient-light alarm (`RSQM_DIAG_AMBIENT_HIGH`, bit 14).**            |
|         | When the guard is tripped but RF is already at the floor, HGAC now discriminates the  |
|         | cause with a third per-domain EMA on the **ALED** phase (`hgac_ema_ambient_tau_s`,     |
|         | default 2 s, sustained): if ambient alone ≥ HIGH2, lowering RF/ILED can't help         |
|         | (`v_tia_led ≥ v_tia_aled`) → raise bit 14 "TOO MUCH AMBIENT LIGHT" (global OR across    |
|         | domains). Reuses the bit freed by SIGNAL_WEAK's removal; distinct from `AMB_SAT`        |
|         | (per-sample hard rail). `_hgac_track()` returns the alarm and `_hgac_update()` publishes|
|         | it into `_current_data.diag_code`. Consumer: pulsenest_lab DiagCode tooltip (operator-  |
|         | actionable). test_hgac +2 (alarm at floor; no alarm when LED-dominated). 38/38 native   |
|         | tests, ESP32-S3 V16 build OK.                                                           |
| v0.58-experiment | **HGAC rebuilt: RF-only, two EMA per gain domain (guard + leveling).**      |
|         | Replaces the single TIAGUARD guard + debounce + descent-only design. Per domain (LED |
|         | phase only): a **fast** EMA (τ 0.1 s) drives an urgent HIGH2 guard (0.90 V), a **slow**|
|         | EMA (τ 2 s) drives a bidirectional HIGH1/LOW1 leveling dead-band (0.75/0.25 V) — so RF |
|         | now both descends AND ascends. No debounce (the EMA τ is the temporal filter).        |
|         | `EmaChannel` gains optional `warmup_s` + `valid()`; HGAC uses 3·τ warmup, and          |
|         | `_hgac_change_rf()` **resets** the domain's EMAs on a gain step (warmup doubles as     |
|         | cooldown; no analytic rescale). Renames: `tia_axis::GUARD_V`→`HIGH2_V`;                |
|         | `hgac_v_tia_thr`→`hgac_v_tia_high2`; added `hgac_v_tia_high1`, `hgac_v_tia_low1`,      |
|         | `hgac_ema_fast_tau_s`, `hgac_ema_slow_tau_s`; removed `hgac_tiaguard_debounce_min_s`.  |
|         | Setters renamed/added accordingly; `_hgac_step_rf_down`/`_hgac_check_tiaguard` →       |
|         | `_hgac_change_rf`/`_hgac_track`. Only led1/led2 tracked (ALED reserved for the         |
|         | deferred ambient alarm). API-breaking (config fields, setters, wire keys). test_hgac   |
|         | rewritten (6 behavioural tests: guard→floor, warmup, disabled, leveling→raises RF,     |
|         | reset+rewarm, SpO2-EMA-untouched). 36/36 native tests, ESP32-S3 V16 build OK.          |
| v0.57-experiment | **Remove RSQM's EMA estimators, the `ready` gate, and the RSQM EMA τ config.** |
|         | Follow-on to v0.56: once SIGNAL_WEAK was gone, `_rsqm_led1/2_sub_ema` fed only their   |
|         | sample `.count` (the `ready = count ≥ rsqm_ready_count` gate), and `ready` fed only     |
|         | `rsqi`'s 12 s warm-up — itself a vestige of SIGNAL_WEAK (no statistic left to mature).   |
|         | Removed: the two `EmaChannel`s, `rsqm_ready_count`, `ready`, and (cascade) the now-inert |
|         | `rsqm_ema_mean_tau_s`/`rsqm_ema_var_tau_s` config + `setRsqmEmaTau()` + `getConfig()`    |
|         | copies + `$LCFG`/`$SET` wiring + LIBConfig controls. `rsqi` is now                        |
|         | `PROBE_APPLIED && diag_code==0` (responsive, no warm-up — probe-state is instantaneous,  |
|         | and SpO2/HR keep their own warm-ups). Spec §5.6.1 rewritten. API-breaking (config fields  |
|         | + setter + wire keys). 36/36 native tests, ESP32-S3 V16 build OK.                         |
| v0.56-experiment | **Remove `RSQM_DIAG_SIGNAL_WEAK` (bit 14) end-to-end.**                    |
|         | No actionable consumer (only the aggregate `rsqi` gate + display), not fixable by |
|         | gain (RF scales AC and DC together → PI/ratio invariant), and redundant with the  |
|         | PI-based `spo2_sqi`. Removed: the bit, `rsqm_signal_weak_std` config field + `2000`|
|         | default + `setRsqmSignalWeakStd()` + `getConfig()` copy, the producer block in     |
|         | `_rsqm_update()`, and the `$LCFG`/`$SET` wiring + LIBConfig control in PulseNest.  |
|         | Bit 14 (0x4000) left free — reintroduce as `RSQM_DIAG_AC_LOW` if a pulsatile-      |
|         | amplitude flag is ever needed. API-breaking (config field + setter + bit + wire).  |
|         | Note: `_rsqm_led1/2_sub_ema` now feed only `.count` (for `ready`); their var/mean   |
|         | are unconsumed (candidate to reduce to a plain sample counter). 36/36 native tests, |
|         | ESP32-S3 V16 build OK.                                                              |
| v0.55-experiment | **`EmaChannel::update()` seeds on the first sample.**                      |
|         | `count == 0` → `mean = x`, `var = 0` (was: ramp `mean` from 0). Ramping from 0 left    |
|         | `d = x − mean ≈ x` for ~3·τ, spuriously inflating `var` (→ PI, RSQM SIGNAL_WEAK) during |
|         | warm-up. Fixes the start-up bias; does NOT shorten `spo2_warmup_s` / `rsqm_ready_count` |
|         | (var still needs ~3·τ_var to be representative). Steady state unchanged. Behaviour      |
|         | change only in the transient after construction/`reset()`. Affects all EmaChannel users |
|         | (RSQM, SpO2). 36/36 native tests (same as baseline), no regression.                     |
| v0.54-experiment | **Remove the `IDEAL` (0.6 V) v_tia threshold — unused.**                    |
|         | `tia_axis::IDEAL_V` and the `hgac_v_tia_ideal` config field (default + `getConfig()` |
|         | copy) deleted; the ascending `static_assert` drops the `IDEAL_V <` term. It was      |
|         | telemetry-only in v1 and never consumed by any control logic. RF is a discrete 7-step |
|         | actuator (×2–3 jumps) → no fine setpoint is reachable, so a "target DC" is meaningless: |
|         | v1 control is a dead-band `[LOW, HIGH]` + `GUARD`, no attractor at the centre. Would    |
|         | only matter for a future feed-forward jump (Phase 5). API-breaking: `AFE4490Config`     |
|         | loses a field (no wire/firmware consumer — `$CFG` never sent it). 36/36 native tests,   |
|         | ESP32-S3 (incunest_V16) build OK.                                                       |
| v0.53-experiment | **Consolidate ADC constants into `namespace adc`.**                        |
|         | `kAFE_ADC_FS_CODE`/`kAFE_ADC_FSR` (file-scope) and `kAdcSatPos`/`kAdcSatNeg`     |
|         | (`AFE4490AnalogState`) moved to `namespace adc`: `FS_CODE`/`FSR`/`SCALE`, NOMINAL |
|         | vs ACTUAL code rails, and guarded `SAT_POS`/`SAT_NEG` (guard now explicit).      |
|         | Removed `tia_axis::ADC_SAT_V` (duplicate 1.2 V) — `tia_axis` references          |
|         | `adc::FSR`. Internal reorg, no API consumer outside the library.                 |
| v0.52-experiment | **Rename `v_tia_diff` → `v_tia` (drop the `_diff` suffix).**                |
|         | All identifiers, config fields (`hgac_v_tia_thr`/`hgac_v_tia_ideal`), the       |
|         | setter (`setHgacVTiaThr`), comments and CSV column headers (`V_TIA_*`).          |
|         | Rationale (Alex): only the differential value exists, so `_diff` is redundant   |
|         | and falsely implies a `v_tia_branch` that is nowhere in the code. Removed the    |
|         | old "naming rule 4b". API-breaking: config/setter renamed; CSV headers change    |
|         | from `V_TIA_DIFF_*` to `V_TIA_*` (breaks by-name parsing of older captures).      |
| v0.51-experiment | **Removed `AFE4490AnalogState::kVTiaDiffFs`/`kVTiaDiffLin` aliases.**       |
|         | They only re-exposed `tia_axis::FS_V`/`LIN_V` (the single source of truth); the   |
|         | `tia_over_fs`/`tia_over_lin` masks now compare against `tia_axis::` directly.     |
| v0.50-experiment | **PROBE_SATURATING checks POSITIVE saturation only (`anyPositiveSaturation`).**|
|         | RSQM's saturation branch changed from `!allValidRange()` to                     |
|         | `anyPositiveSaturation() = (adc_sat_pos \| tia_over_fs)`. Rationale (Alex): the  |
|         | probe can only saturate from above (photocurrent unipolar); the ADC only rails  |
|         | NEGATIVE via AMBDAC (unused in v1, and its neg rail was itself characterised via |
|         | an AMBDAC sweep), and a negative rail wouldn't be cleared by reducing RF/ILED.   |
|         | Identical behaviour today (AMBDAC=0 → adc_sat_neg always 0); differs only if     |
|         | AMBDAC is enabled (layer v2). New helper `anyPositiveSaturation()`;              |
|         | `allValidRange()` removed. No behaviour change. 36/36 tests, build OK.        |
|         | Also renamed `_rsqm_set_probe_state()` to `_rsqm_request_probe_state()`:    |
|         | a debounce gate that commits a state only after N consecutive identical     |
|         | requests, not an imperative setter; the name now matches its `requested`    |
|         | parameter and contract. Private method, no signature/behaviour change.      |
|         | Removed the diag-task on-state-change trigger (`xTaskNotify` from           |
|         | `_rsqm_request_probe_state`); `_diag_task_body` now polls via `vTaskDelay`  |
|         | every rsqm_diag_period_ms (5 s). The notification only advanced the first   |
|         | diagnostic by <=5 s while no patient was present (no value) and woke the    |
|         | task pointlessly on transitions into APPLIED/SATURATING. Behaviour change.  |
|         | Detector: the two `_rsqm_request_probe_state()` call sites (DISCONNECTED    |
|         | branch + else) collapsed into one flat if/else-if priority ladder with a    |
|         | single call, matching the priority order already in §5.6.                   |
|         | Readability only, no behaviour change.                                      |
|         | Diag task gate is now a whitelist (DISCONNECTED || NOT_APPLIED) instead     |
|         | of a blacklist (not APPLIED, not SATURATING). Runs runAfeDiagnostics only   |
|         | in states with no patient confirmed. Any future ProbeState defaults to      |
|         | NOT diagnosing → never silently interrupts acquisition. Equivalent for      |
|         | today's four states; no behaviour change.                                   |
|         | Gate G0 in _hgac_update() rewritten to an explicit gate-OFF blacklist:      |
|         | `NOT_APPLIED || DISCONNECTED || HW_SETTLING` instead of the `probe_active`  |
|         | helper + double negation. Equivalent today (4 states); readability per      |
|         | Alex. Trade-off recorded: a future ProbeState is NOT gated off (fail-open), |
|         | unlike the diag-task whitelist above — revisit the list when adding states. |
|         | Also in _hgac_check_tiaguard(): removed the single-use `tiaguard` bool +    |
|         | `!tiaguard`; the early-return now tests the condition directly (De Morgan). |
|         | Removed HGAC gain-floor signal end-to-end: bit HGAC_DIAG_GAIN_FLOOR         |
|         | (0x10000), members _hgac_alarm_gain_floor_led1/2, param alarm_flag of       |
|         | _hgac_check_tiaguard, AFE4490DebugData.hgac_alarm_gain_floor, getter        |
|         | test_hgac_alarm_gain_floor, the $M4 HGAC_ALARM field (frame 37->36 cols),   |
|         | and §5.8.6. No consumer ever decoded the bit (script parses <=col33;        |
|         | motherBoard only prints diag_code). Behaviour change (bit gone from         |
|         | diag_code, one fewer $M4 field). test_hgac Test 5 repurposed to assert      |
|         | RF reaches the floor (RF_10K).                                              |
|         | Renamed hgac_settle_time_s -> afe_settle_time_s: it arms the RSQM settling  |
|         | countdown and models AFE front-end re-settling after any register change,   |
|         | not an HGAC-specific quantity (comment already anticipates config-change    |
|         | producers). Private constexpr, no public API break.                         |
| v0.49-experiment | **Rename TIASAT → TIAGUARD + debounce naming; `adc_code` convention.**  |
|         | The HGAC over-threshold condition was named "TIASAT" (TIA saturation) — it   |
|         | collided with RSQM's `PROBE_SATURATING` and misled: the guard trips at        |
|         | `GUARD_V`=0.9 V, BELOW full-scale, so it is *preventive*, not saturation.     |
|         | Renamed the condition to **TIAGUARD**. Debounce state/threshold renamed to    |
|         | drop the count/samples ambiguity: `_hgac_tiasat_count_led1/2` →               |
|         | `_hgac_tiaguard_debounce_count_led1/2`, `_hgac_tiasat_min_samples`  |
|         | → `_hgac_tiaguard_debounce_min_samples`, `hgac_tiasat_persist_s` /            |
|         | `setHgacTiasatPersistS()` → `hgac_tiaguard_debounce_min_s` /                  |
|         | `setHgacTiaguardDebounceMinS()`, `_hgac_check_tiasat()` →                     |
|         | `_hgac_check_tiaguard()`. `$SET`/`$LCFG` wire key updated in `src/main.cpp`   |
|         | (no consumer in `pulsenest_lab.py`). Naming rule adopted: ADC output uses     |
|         | `adc_code`, NEVER `count`; `count` freed for dynamic sample counters (project  |
|         | convention, e.g. `_sample_count`) → state `_hgac_tiaguard_debounce_count`;     |
|         | fixed sample thresholds use `_min_samples`. No behaviour change.              |
|         | 36/36 native tests, ESP32-S3 (incunest_V16) build OK.                         |
| v0.48-experiment | **Single source of truth for the v_tia axis: `namespace tia_axis`.**|
|         | The five ordered thresholds (IDEAL 0.6 / GUARD 0.9 / FS 1.0 / ADC_SAT 1.2 / LIN  |
|         | 1.8 V) were scattered across `AFE4490AnalogState` (physics constexpr) and        |
|         | `AFE4490Config` (HGAC policy defaults). Now defined ONCE in `tia_axis`, with a    |
|         | `static_assert` enforcing the strict ascending order at compile time.            |
|         | `kVTiaFs/Lin` and the `hgac_v_tia_thr/ideal` defaults derive from it.    |
|         | `setHgacVTiaThr()` now clamps to ≤ FS_V (guard can't sit above the reliable   |
|         | range at runtime). Values unchanged → no behaviour change except that clamp.     |
|         | ADC_SAT_V is nominal; `kAdcSatPos` stays the empirical adc_code rail. 36/36 tests.  |
| v0.47b-experiment | **Readability: RSQM's PROBE_SATURATING check uses `allValidRange()`.** |
|         | `_rsqm_update()` classified saturation via `!otLed1Valid() \|\| !otLed2Valid()`  |
|         | (two locals). Replaced by `!as.allValidRange()` — exact equivalent, but asks the |
|         | channels' RANGE directly instead of borrowing OT-validity semantics (a separate  |
|         | concern that may evolve independently of "in range"). Aligns the code with its   |
|         | own comment ("channel not CH_VALID_RANGE → SATURATING"). No behaviour change.     |
|         | `otLed1Valid()`/`otLed2Valid()` removed — unused after this change, no dead API.  |
|         | 36/36 native tests, build OK.                                                     |
| v0.47-experiment | **Default LED current 20 mA → 50 mA (preliminary, dev-phase).** Both**  |
|         | `_afe_led1/2_current_mA` = 50.0 → DAC code 85 (0x55), ~49.8 mA. Purpose:    |
|         | gain field experience at higher drive. Rationale (2026-07-27): the LED      |
|         | rating is NOT the limit — at the AFE's pulsed ~25 % duty, 50 mA peak sits    |
|         | ≪ I_FP 100 mA and ~23 mW ≪ Pd, so there is ample margin (datasheet Medle,    |
|         | reliability-caveated). Side-effects: (a) TIA does NOT saturate — measured   |
|         | v_tia < 0.5 V, finger, 50 mA/RF_100K (neonatal case uncharacterised); |
|         | (b) §8 timing note,                                                         |
|         | ambient settling t5/t11 raised 200→400 counts (100 µs) for the 50 mA drive |
|         | (longer LED extinction / CF discharge tail); (c) skin-thermal envelope      |
|         | (ISO ≤41 °C) still uncharacterised. Spec §3/§8 default+timing tables updated.|
|         | Also: **inlined Gate G0 into `_hgac_update()`** — removed the single-use    |
|         | `_hgac_gate_ok()` helper so the gate's condition and its consequences (reset |
|         | TIAGUARD debounce + return) read together; the design rationale lives here in  |
|         | §5.8, not in a 12-line code comment. No behaviour change.                    |
| v0.46-experiment | **Rename `AFE4490ChState` to an increasing-range scale (all suffixed**  |
|         | `_RANGE`): `CH_CLEAN`→`CH_VALID_RANGE`, `CH_OFF_SPEC`→`CH_EXTENDED_RANGE`,  |
|         | `CH_CLIPPED`→`CH_CLIPPED_RANGE`; helpers `chClean()`→`chValidRange()` and    |
|         | `allClean()`→`allValidRange()`. Motivation (Alex, 2026-07-26): the old       |
|         | `CLEAN`/`OFF_SPEC` mix wasn't parallel; the new trio reads as one monotone   |
|         | scale of `v_tia` bands and drops the earlier `chInValidRange` proposal  |
|         | (ambiguous "in-valid"). "VALID" = TI's guaranteed-linear range (±1.0 V), NOT |
|         | that EXTENDED is invalid — enum comment clarifies the nuance. Pure rename:   |
|         | no behavior change. Propagated to `pulsenest_lab.py`, `pulsenest_lab_spec.md`|
|         | and `test_hgac.cpp`. 36/36 native tests, ESP32-S3 (incunest_V16) build OK.  |
| v0.45-experiment | **New `ProbeState::PROBE_SATURATING` — splits the TIAGUARD condition out**|
|         | of `PROBE_NOT_APPLIED`, resolving the open question from v0.44d/§5.8.2      |
|         | about HGAC's gate (Alex, 2026-07-23: "no entiendo por qué un fallo en la    |
|         | detección de PROBE_APPLIED se deba corregir modificando RF o ILED").        |
|         | `_rsqm_update()`: `!as.otLed1Valid()`/`!as.otLed2Valid()` (channel not      |
|         | `CH_VALID_RANGE`) now yields `PROBE_SATURATING` directly, checked BEFORE the      |
|         | `rsqm_ot_thr` comparison — no longer OR'd into the same `NOT_APPLIED` as    |
|         | "OT valid but reads high" (genuine no-tissue). Enum value appended (0-2    |
|         | unchanged) to keep `$M2`/`$M3`/`$M4` wire encoding stable. Consumers:       |
|         | SpO2/HR1/HR2/HR3/RSQI need no change (already gate on `!= PROBE_APPLIED`); |
|         | `_hgac_gate_ok()` simplified to `== PROBE_APPLIED \|\| == PROBE_SATURATING` |
|         | (§5.8.2) — no longer needs the coarse `!= PROBE_DISCONNECTED` and no       |
|         | longer opens during ordinary probe-idle `NOT_APPLIED`; `_diag_task_body()` |
|         | (§5.7) updated to also skip periodic diagnostics during `PROBE_SATURATING` |
|         | (a real saturation event may involve a real patient, and HGAC needs to act |
|         | undisturbed). `pulsenest_lab.py`: `PROBE_SATURATING=3` added to the 4      |
|         | mirror TestCalc classes and the `AFESweepTestWindow` combo; new dedicated  |
|         | background color in SIGNAL STATS (orange, distinct from NOT_APPLIED's     |
|         | amber). Test stimulus in `test_hgac.cpp` (`SAT_LED1_CODE`) raised from     |
|         | ~0.973 V to ~1.087 V so it actually trips `PROBE_SATURATING` under the new |
|         | gate (the old value stayed CH_VALID_RANGE, which now yields plain NOT_APPLIED —  |
|         | correctly excluded from HGAC's gate, so the old tests would otherwise      |
|         | fail). Verified: 36/36 native tests (`test_biquad` pre-existing unrelated  |
|         | failure), ESP32-S3 V16 build OK.                                          |
| v0.44d-experiment | **Renamed "descent" → "TIASAT" (TIA saturation) for the condition,**    |
|         | kept a plain action verb for the response. Alex: the "descent" concept      |
|         | conflated the CONDITION being debounced (TIA saturating) with the ACTION    |
|         | taken in response (step RF down) — and doesn't appear in the datasheet.    |
|         | Split cleanly: `_hgac_descent_count_led1/2` → `_hgac_tiasat_count_led1/2`,  |
|         | `hgac_descent_persist_s`/`setHgacDescentPersistS()` →                      |
|         | `hgac_tiasat_persist_s`/`setHgacTiasatPersistS()`, `_hgac_descent_min_      |
|         | samples` → `_hgac_tiasat_min_samples`, `_hgac_check_descent()` →            |
|         | `_hgac_check_tiasat()` (condition side); `_hgac_descend_rf()` →             |
|         | `_hgac_step_rf_down()` (action side, unrelated to TIASAT — a plain verb).   |
|         | `$SET`/`$LCFG` wire key `hgac_descent_persist_s` → `hgac_tiasat_persist_s`   |
|         | *(these `tiasat` names were later renamed to `tiaguard` in v0.49 — see below)* |
|         | in PulseNest `src/main.cpp` (no consumer in `pulsenest_lab.py` yet, so no   |
|         | Python-side break). Test accessors `test_hgac_descend_rf_led1/2` →         |
|         | `test_hgac_step_rf_down_led1/2`. Test *function names* (e.g.               |
|         | `test_hgac_descent_after_persistence`) and prose describing the general    |
|         | English action ("descent"/"step down") were left alone — only the         |
|         | conflated identifier family was in scope. Verified: 36/36 native tests     |
|         | (`test_biquad` pre-existing unrelated failure), ESP32-S3 V16 build OK.     |
| v0.44c-experiment | **Removed `_hgac_invalid_ema` — Gate G0's invalid-fraction clause was**|
|         | self-defeating, not just redundant (§5.8.2). It gated HGAC's corrective     |
|         | action on the EMA'd fraction of `!allValidRange()` samples — the same signal    |
|         | HGAC exists to fix — with a faster time constant (0.2 s window, crosses    |
|         | 0.5 after ~139 ms of continuous saturation) than the descent debounce it   |
|         | gates (0.5 s persistence), so sustained saturation closed the gate and     |
|         | reset the descent counters before the debounce could ever complete —      |
|         | blocking the fix exactly when needed most. Never caught by tests: the only |
|         | descent test uses a stimulus that stays below `kVTiaFs` so `allValidRange()`|
|         | never went false. Removed: `_hgac_invalid_ema` member, `hgac_gate_invalid_ |
|         | frac_max`/`hgac_gate_window_s` config fields + `setHgacGateInvalidFracMax()`|
|         | setter + its `$SET`/`$LCFG` wiring in PulseNest `src/main.cpp`. `_hgac_gate_|
|         | ok()` keeps only the `probe_state`/`HW_SETTLING` clauses (§5.8.2 deviation  |
|         | discussion unchanged, still open). Verified: 36/36 native tests (`test_    |
|         | biquad` pre-existing unrelated failure), ESP32-S3 V16 build OK.            |
| v0.44b-experiment | **Readability refactor of `_hgac_check_descent()`, no behavior change**|
|         | (`incunest_afe4490.cpp`). The O8.1 debounce increment/check was duplicated  |
|         | across the "at gain floor → alarm" and "not at floor → descend" branches,   |
|         | with two different increment idioms (capped vs unconditional pre-increment) |
|         | that made the two paths hard to compare. Hoisted the debounce check once,  |
|         | shared by both outcomes; the `cur_rf == RF_10K` branch now only decides     |
|         | alarm-vs-descend after debounce is already satisfied. Verified equivalent:  |
|         | 6/6 `test_hgac.cpp` (incl. `test_hgac_descent_debounce`,                    |
|         | `test_hgac_alarm_at_gain_floor`), ESP32-S3 V16 build OK.                    |
| v0.44-experiment | **EXPERIMENT (branch `experiment/ot-domain-inputs`): `rsqm_ot_thr` widened**|
|         | 8.5e-5 → 1.0e-4 (§5, §10). First real-HW test (CONTEC MS100 simulator,      |
|         | probe applied) found `probe_state` never reached PROBE_APPLIED at the       |
|         | theoretical 8.5e-5 estimate — the simulator is very sensitive to probe      |
|         | placement and reads OT above that value routinely. Practical mitigation,   |
|         | not an empirical calibration with a real neonatal probe/patient.           |
| v0.43-experiment | **Naming cleanup, no behavior change: `_verb_algorithm()` → `_algorithm_verb()`**|
|         | across the whole class, matching the prefix convention already used by     |
|         | every member variable (`_hr2_buf`, `_spo2_a`, `_rsqm_led1_sub_ema`, ...)    |
|         | and by the HR2/HR3 task methods (`_hr2_task_body`). 15 renames: `_update_rsqm`|
|         | → `_rsqm_update`; `_update_hgac` → `_hgac_update`; `_update_spo2` →         |
|         | `_spo2_update`; `_update_hr1` → `_hr1_update`; `_update_hr2_sample` →       |
|         | `_hr2_update_sample`; `_linearize_hr2` → `_hr2_take_snapshot`; `_compute_hr2`   |
|         | → `_hr2_compute`; `_update_hr2` (test-only sync wrapper) →                 |
|         | `_hr2_update_for_test` (name now signals it is never called from           |
|         | production — see §5.3); `_hr2_calc_sem` → `_hr2_compute_sem` (names the    |
|         | function it gates, `_hr2_compute()`, instead of a generic "calc"); same    |
|         | 5 renames mirrored for HR3 (`_update_hr3_sample`/`_linearize_hr3`/         |
|         | `_compute_hr3`/`_update_hr3`/`_hr3_calc_sem`); `_set_probe_state` →        |
|         | `_rsqm_set_probe_state` (RSQM's debounce gate — previously had no          |
|         | algorithm prefix at all). Pure rename: no signatures, semantics, or        |
|         | timings changed. Verified: 36/36 native tests (`test_biquad` pre-existing  |
|         | unrelated failure), ESP32-S3 V16 build OK (18.4% RAM / 30.1% flash).       |
|         | Living spec sections (§1, §3d, §5, §9, §11) updated to the new names;      |
|         | this table (historical record) keeps the old names as they were at each   |
|         | past version. Spec §1.3, §3d, §5.1-§5.4, §5.6-§5.8, §9.3/§9.7, §11, §14.  |
| v0.42-experiment | **EXPERIMENT (branch `experiment/ot-domain-inputs`): same `probe_state`  |
|         | design as v0.41's SpO2 applied to HR1/HR2/HR3.** `_update_hr1(float ir,    |
|         | ProbeState probe_state)`, `_update_hr2_sample(...)`, `_update_hr3_sample(  |
|         | ...)` all gained the `probe_state` parameter — consumed, never computed.  |
|         | While `probe_state != PROBE_APPLIED`: all internal state (HR1's DC filter/ |
|         | running max/threshold flag/peak+interval indices/RR buffer/MA buffer;     |
|         | HR2's bandpass filter + circular buffer; HR3's low-pass filter + circular  |
|         | buffer) resets every sample (idempotent, same pattern as SpO2 — no stored |
|         | "previous state" needed), and `hr1/hr2/hr3` + their `_sqi` fields are     |
|         | forced to `NaN`/`0`, unifying the sentinel across all 4 algorithms. For   |
|         | HR2/HR3 the gate lives in the fast per-sample path (shared by the         |
|         | production call site and the sync test wrapper), so the slow autocorr/FFT |
|         | task (Task B/C) is never signalled while not applied. Test wrappers       |
|         | `test_feed_hr1/hr2/hr3()` gained the `probe_state` parameter. New tests   |
|         | `test_hr1/hr2/hr3_not_applied_resets` (reset + fresh-buffer/interval-count |
|         | on reapply). `pulsenest_lab.py` mirrors: `HR1TestCalc`/`HR2TestCalc`/      |
|         | `HR3TestCalc.update()` all gained `probe_state`, resetting + returning    |
|         | `NaN`/`0` identically; all call sites (offline CSV loaders, live feeds,   |
|         | `PPGMonitor`) updated — the unrelated `HRFFTCalc`/`self.hr3_calc`         |
|         | (HR3LAB diagnostic window) deliberately left untouched (separate class,  |
|         | no inheritance, out of scope). Verified: 36/36 native tests (test_biquad  |
|         | pre-existing unrelated failure), `py_compile` clean. Spec §5.2-§5.4, §14. |
| v0.41-experiment | **EXPERIMENT (branch `experiment/ot-domain-inputs`): presence detection  |
|         | fully delegated to RSQM — `spo2_min_ot_dc` removed too.** `_update_spo2()`  |
|         | gained a third parameter, `ProbeState probe_state` (RSQM's classification, |
|         | consumed — never computed internally). While `probe_state != PROBE_APPLIED`|
|         | (covers both `PROBE_NOT_APPLIED` and `PROBE_DISCONNECTED`): both EMAs are  |
|         | reset every sample (idempotent — no stored "previous state" needed; warmup |
|         | restarts naturally via `ir_ema.count` once `PROBE_APPLIED` resumes), and   |
|         | `pi`/`spo2`/`spo2_r` are set to `NaN` (not `-1.0f` — a forgotten `sqi`      |
|         | check on `NaN` fails safe: comparisons are always false, so e.g. a naive   |
|         | `spo2 < 70` alarm never fires on `NaN`, whereas it would on `-1.0f`).       |
|         | `spo2_min_ot_dc`/`spo2_ac_div_eps` collapsed into one `spo2_div_eps`        |
|         | (`1e-9`), a purely numerical division-safety guard on the actual divisors  |
|         | (`dc_ir` for PI, `dc_red`/`ac_ir` for R) — not physiological, since that's  |
|         | now `probe_state`'s job. `_spo2_sample_count` removed (redundant with      |
|         | `EmaChannel::count`, always incremented in lockstep). Output contract now  |
|         | uniform: `sqi==0 ⟹ pi==spo2==spo2_r==NaN`, across warmup, not-applied, and |
|         | the division guard alike (previously only the near-zero-mean PI branch    |
|         | used a sentinel; the rest left outputs stale). Verified: 7/7 `test_spo2`   |
|         | (new `test_spo2_not_applied_resets` covers reset + fresh-warmup-on-reapply)|
|         | + 33/33 other native tests, ESP32-S3 build (16.4 KB RAM / 828 KB flash).   |
|         | Spec §5.1, §14.                                                            |
| v0.40-experiment | **EXPERIMENT (branch `experiment/ot-domain-inputs`): removed the        |
|         | `spo2_min_i_pd_a` "no-finger" gate added in v0.38.** Analysis (2026-07-15)  |
|         | showed it rarely detected an actual no-finger condition (a transmittance    |
|         | probe with no finger shows HIGH OT, not low i_pd) and mostly duplicated,    |
|         | with a weaker/uncalibrated criterion, RSQM's own `PROBE_DISCONNECTED`.      |
|         | `_update_spo2()` signature simplified to `(ot_ir, ot_red)` — no more        |
|         | `i_pd_ir`/`i_pd_red`. Replaced by `spo2_min_ot_dc` (A/A, `1e-6` placeholder)|
|         | applied only to `dc_ir`/`dc_red`; the AC side keeps a separate purely      |
|         | numerical guard `spo2_ac_div_eps` (`1e-9`, on `ac_ir` only) — conflating   |
|         | the two into one shared threshold was tried first and broke 4/7           |
|         | `test_spo2` cases (AC legitimately gets tiny at low PI). `setSpO2MinIPdA()`|
|         | removed; `spo2_min_i_pd_a` removed from `AFE4490Config`. Spec §5.1, §14.   |
| v0.39-experiment | **EXPERIMENT (branch `experiment/ot-domain-inputs`): HR1/HR2/HR3     |
|         | input migrated to OT**, completing the migration started in v0.38 (SpO2).   |
|         | `_update_hr1/hr2/hr3()` and their fast-path helpers now take `float`        |
|         | (`as.ot_led1`) instead of `int32_t led1_sub`. HR1 (peak timing) and HR3     |
|         | (HPS ratio) needed no threshold recalibration — pure domain/type change.    |
|         | HR2's near-zero-energy guard (`acorr0 < 1.0f`) recalibrated to              |
|         | `acorr0 < hr2_ot_energy_eps` (1e-16) for OT-scale magnitudes. All 3 test    |
|         | suites rewritten with OT-realistic magnitudes (previously still used raw   |
|         | count-scale values that happened to pass by coincidence, since all three   |
|         | algorithms are scale-invariant). Spec §5.2/§5.3/§5.4, §14.                  |
| v0.38-experiment | **EXPERIMENT (branch `experiment/ot-domain-inputs`): SpO2 input      |
|         | migrated from raw adc_code to OT.** `_update_spo2()` now takes            |
|         | `(ot_ir, ot_red, i_pd_ir, i_pd_red)` instead of `(led1_sub, led2_sub)`.      |
|         | New `spo2_min_i_pd_a` (replaces `spo2_min_dc`) — no-finger gate on          |
|         | absolute i_pd magnitude, not the OT ratio (unreliable near zero signal).    |
|         | `_hgac_descend_rf()` no longer rescales the SpO2 EMA — OT is gain-invariant |
|         | by construction, so `EmaChannel::rescale()` is now dead code (removed).     |
|         | `library.json` platforms/frameworks unblocked earlier (v0.37) already      |
|         | covers native testing of this change. Spec §5.1 (rewritten), §14.          |
|         | Not yet migrated: HR1/HR2/HR3 (still raw-domain, lower priority — see      |
|         | project memory), `pulsenest_lab.py` mirror classes (impact analyzed, not   |
|         | resolved — see project memory).                                            |
| v0.37   | **HGAC Phase 1: RF-only saturation guard + descent + mandatory SpO2 EMA    |
|         | rescale** (`_update_hgac()`, runs after `_update_rsqm()` in                 |
|         | `_process_sample()`). New `AFE4490Config` fields `hgac_enable`,             |
|         | `hgac_v_tia_thr`, `hgac_v_tia_ideal`, `hgac_descent_persist_s`,   |
|         | `hgac_gate_invalid_frac_max`, `hgac_gate_window_s` (promoted from the       |
|         | former fixed `hgac_v_tia_thr/ideal` constexpr in the .cpp anonymous    |
|         | namespace). New bit `HGAC_DIAG_GAIN_FLOOR` (0x10000). First real producer  |
|         | of `RSQM_DIAG_HW_SETTLING` (previously unreachable). `EmaChannel::rescale()`|
|         | added. Gate G0 implemented as `probe_state != PROBE_DISCONNECTED` (not     |
|         | literally `== PROBE_APPLIED` as in the original design axiom — avoids a    |
|         | deadlock where saturation-induced OT invalidity would block the descent    |
|         | that clears it). No ascent yet (manual recovery) — disabled by default.    |
|         | `AFE4490DebugData` gained `hgac_rf_led1/led2`/`hgac_alarm_gain_floor`.      |
|         | New unit tests `test/test_hgac/test_hgac.cpp`. Include-hygiene fix:        |
|         | added missing `<cstring>`/`<cmath>` to the non-OFFLINE branch (the plain    |
|         | `UNIT_TEST`/native path lacked them; real Arduino.h happened to provide    |
|         | them transitively). Spec §5.8 (new), §3d, §5.6.3, §10.9 (new), §14.        |
| v0.36   | DiagCode RSQM flags renamed `RSQM_*` → `RSQM_DIAG_*` (symmetric with        |
|         | `AFE_DIAG_*`; avoids collision with operative `rsqm_*` config params).      |
|         | Documented as ANTICIPATORY CODE: no programmatic consumer beyond the        |
|         | aggregate `rsqi` gate; intended consumer is HGAC, which may never read      |
|         | them. `RSQM_DIAG_HW_SETTLING` noted as currently unreachable (nothing       |
|         | sets `_rsqm_settling_countdown > 0`). Spec §5.6.3.                          |
| v0.35   | `AFE4490AnalogState`: value-always invalidity policy. New fields             |
|         | `ot_led1`/`ot_led2` (OT moved from `_update_rsqm()` to                       |
|         | `_compute_analog_state()`); 4 validity masks (adc_sat_pos/neg,               |
|         | tia_over_fs/lin, bit=channel per new `AFE4490Ch` enum); helpers +            |
|         | `chState()` (`AFE4490ChState`: CH_VALID_RANGE/CH_EXTENDED_RANGE/CH_CLIPPED_RANGE,              |
|         | precedence CLIPPED>OFF_SPEC>CLEAN); physical constants in struct             |
|         | (kVTiaFs=1.0, kVTiaLin=1.8, kAdcSatPos/Neg=±2096700: empirical       |
|         | rails +2096921/−2096919 with ~220-code guard — ALED codes near the          |
|         | negative rail oscillate without pinning due to ADC averaging).               |
|         | Removed `rsqm_ot_saturated` sentinel and `rsqm_adc_positive_sat`;            |
|         | `hgac_v_tia_fs` superseded by `kVTiaFs`. RSQM saturation            |
|         | policy now explicit and stricter (`otLedxValid()`: both channels             |
|         | CH_VALID_RANGE, both rails, TIA linearity). Spec §3c, §5.6.2, §5.6.3.              |
| v0.32   | Default values changed: LED1/LED2 current 11.7 mA → ~19.9 mA (code 34,  |
|         | range 150 mA), TIA gain RF_500K → RF_100K. Matched to Medle probe at     |
|         | 20 mA. Spec tables updated (§3, §8 TX/RX params, §9 register decisions). |
| v0.31   | HR3 SQI: redesigned as two-bin HPS fraction in a local window. Replaces    |
|         | P[k] local prominence (v0.30), which was limited to SQI ≤ 0.19 due to     |
|         | the Hann window spreading FFT energy across 3 bins (P[k]/P[k±1] ≈ 4,      |
|         | not the assumed 10³). New metric: numerator = hps(peak_bin)+hps(b_other)  |
|         | where b_other is the adjacent bin in the direction of fractional delta;     |
|         | denominator = Σhps over ±hr3_sqi_local_w bins around both peak bins.       |
|         | Bin-position-independent: off-bin energy split cancels in the ratio.        |
|         | Removed `hr3_sqi_local_ratio`. Spec §5.4 updated. Header bumped to v0.31. |
| v0.30   | HR3 SQI: switched from HPS to P[k] for local prominence computation.       |
|         | HPS product (cubic amplification) saturated SQI to 1 for any real signal.  |
|         | P[k] has linear dynamic range; `hr3_sqi_local_ratio` raised 10→100.        |
|         | `p_interp` reuses yp/y0/yn already computed for frequency interpolation.   |
|         | Spec §5.4 updated. `incunest_afe4490_spec.md` header bumped to v0.30.      |
| v0.29   | HR3 true-peak validation widened: `hr3_peak_valid_w=5` — peak_bin must be  |
|         | strict maximum in ±5 bins (including outside search range). Replaces ±1     |
|         | neighbor check. Detects slope maxima and boundary humps more robustly.      |
|         | HR3 SQI simplified: global SQI and geometric mean removed; only local       |
|         | prominence (±hr3_sqi_local_w) remains. Eliminates false-high SQI when       |
|         | sqi_local stayed near 1.0 while the signal was absent. Spec §5.4.          |
| v0.28   | HR3 SQI redesigned: combined global fraction + local prominence (geometric  |
|         | mean). `hr3_sqi_local_w=5` (±0.49 Hz window), `hr3_sqi_local_ratio=10.0`.  |
|         | Detects movement-artifact humps that fool the global metric. Spec §5.4.    |
|         | True-peak validation: reject boundary maxima by checking HPS neighbors      |
|         | outside search range — fixes false HR/SQI from low-freq movement artifacts. |
| v0.27   | RSQM (Raw Signal Quality Monitor). `ProbeState` enum                        |
|         | (PROBE_DISCONNECTED / PROBE_NOT_APPLIED / PROBE_APPLIED). `DiagCode`        |
|         | bitmask constants RSQM_AMB_SAT, RSQM_SIGNAL_WEAK, RSQM_HW_SETTLING         |
|         | (renamed RSQM_DIAG_* in v0.36).                                             |
|         | `AFE4490Data` gains `rsqi`, `diag_code`, `probe_state`. EMA estimators      |
|         | (α_mean=0.02, α_var=0.004, ready after 750 samples). Probe detection via   |
|         | `optical_transmittance` (single threshold 0.30, no hysteresis) + 100-sample  |
|         | debounce in `_set_probe_state()`. RSQM runs first in `_process_sample()`    |
|         | at 500 Hz. Spec §5.6 added.                                                 |
| v0.26   | Decouple STAGE2EN1/STAGE2EN2 from Stage 2 gain. `stg2_gain_code[]` now         |
|         | contains only D[10:8] gain bits (no enable). `setStage2En1(bool)` and          |
|         | `setStage2En2(bool)` expose STAGE2EN1 (D14 of TIAGAIN) and STAGE2EN2           |
|         | (D14 of TIA_AMB_GAIN) as independent explicit parameters. `AFE4490Config`      |
|         | gains `afe_stg2_en_led1` and `afe_stg2_en_led2`. `$CFG` frame gains                |
|         | `stage2en1` and `stage2en2` fields. `$SET,stage2en1/2,<0\|1>` commands added.  |
|         | PulseNest HW CONFIG TIA section gains STAGE2EN1/STAGE2EN2 checkboxes.          |
|         | STAGE2EN2 is still forced ON when AMBDAC > 0.                                  |
| v0.25b  | Fix `setAmbDac()`: `_apply_analog_regs()` now sets STAGE2EN2 (D14 of          |
|         | TIA_AMB_GAIN) automatically when `uA > 0`. Stage 2 must be active for AMBDAC  |
|         | to have effect; STAGE2EN2 is independent of TIAGAIN's Stage 2 enable even    |
|         | with ENSEPGAIN=0.                                                            |
| v0.25   | `setAmbDac(uint8_t uA)`: exposes AMBDAC[3:0] in TIA_AMB_GAIN D19:D16.      |
|         | `AFE4490Config` gains `afe_ambdac_uA`. `$CFG` frame gains `ambdac` field.   |
|         | PulseNest HW CONFIG window gains "Ambient Cancellation" group with AMBDAC   |
|         | spinbox and `$SET,ambdac,<value>` command.                                  |
| v0.25   | LED current setters now store the actual DAC-quantised value (nearest step  |
|         | of range/256 mA) instead of the requested float. `setLEDRange()` re-        |
|         | quantises both currents to the new grid. `getConfig()` thus returns the      |
|         | real programmed current. New private helper `_quantize_led_mA()`.           |
| v0.25   | HR valid lower bound raised: `hr_min_bpm` 30→**40 BPM**.                  |
|         | `hr_search_min_bpm` 27→37 BPM (guard band: hr_min − 3). Python:           |
|         | `FW_HR_MIN_BPM` (HR1/HR2/HR3TestCalc) and `HR_MIN_HZ` (HRFFTCalc)         |
|         | updated accordingly.                                                       |
| v0.24   | Algorithm parameters exposed in AFE4490Config and runtime-configurable      |
|         | via 13 new setters (setSpO2WarmupS, setSpO2EmaMeanTauS, setSpO2EmaVarTauS,   |
|         | setSpO2Range, setSpO2MinDC, setSpO2PiSqiThresholds, setHR1DcTauS,         |
|         | setHR1MaCutoffHz, setHR1SqiCvMax, setHR2MinCorr, setHR2UpdateInterval,    |
|         | setHR3UpdateInterval, setHRValidRange). hr_search_{min,max}_bpm now        |
|         | derived from hr_{min,max}_bpm ± 3 BPM guard band.                         |
| v0.23   | `runAfeDiagnostics(diag_holdoff_ms = 10)`: post-diagnostic holdoff window.     |
|         | After DIAG_EN is cleared, `_task_body()` feeds frozen last-valid raw ADC   |
|         | values into `_process_sample()` for `diag_holdoff_ms` ms, replacing analog |
|         | re-settling artefacts with neutral constant input that filters absorb       |
|         | gracefully. Default 10 ms covers worst-case TIA settling (RF_1M+CF_155P:   |
|         | 5τ ≈ 775 µs). Set to 0 to observe raw artefacts. Spec §2.5b updated.       |
| v0.22   | `runAfeDiagnostics()` redesigned: `_diag_active` flag replaces long mutex hold. |
|         | `_spi_mutex` held only for brief SPI transactions, eliminating priority       |
|         | inversion. During the ~10 ms diagnostic window, `_task_body()` repeats the   |
|         | last valid sample into the queue (under `_state_mutex`) on each DRDY instead  |
|         | of dropping it, preserving temporal equi-spacing for HR2/HR3 and reducing     |
|         | IIR filter discontinuity at resumption. Spec §2.5b updated.                  |
| v0.21   | Parameters Reference added (§10): consolidated catalogue of all parameters  |
|         | that influence library outputs, organised by layer: AFE hardware config     |
|         | (§10.1), pre-processing (§10.2), SpO2 (§10.3), HR1 (§10.4), HR2 (§10.5),  |
|         | HR3 (§10.6), range/validation (§10.7), and out-of-scope environmental       |
|         | factors (§10.8). Version history moved to §13 (end of document).            |
|         | §11 AFE4490 register config, §12 Bibliographic references renumbered.       |
| v0.20   | Auto-CF: `_recalc_tia_cf()` selects the largest `AFE4490TIACF` that settles  |
|         | within the adaptive TIA margin (10% of LED-on window, floor 12.5 µs).       |
|         | `_compute_settle_margin()` shared by timing regs and auto-CF — always in     |
|         | sync. Called from `_recalc_rate_params()` and `setTIAGain()`. `setTIACF()`  |
|         | manual override still available. Default at 500 Hz / RF_500K: CF_20P        |
|         | (was CF_5P). Spec §7.2 added.                                               |
| v0.19   | Configuration readback: `AFE4490Config` struct + `getConfig()` method.      |
|         | Returns a thread-safe snapshot of all runtime-configurable parameters       |
|         | (sample rate, averages, LED currents, LED range, TIA gain/CF, stage 2 gain,|
|         | PPG channel, filter type/cutoffs, HR2/HR3 filter cutoffs, SpO2 a/b).       |
|         | Takes `_spi_mutex` + `_state_mutex` when initialized; safe before `begin()`.|
| v0.18   | HR2_SQI: unbiased normalised autocorrelation. Previous biased estimator     |
|         | systematically capped SQI at (N−τ)/N ≈ 0.875 at 60 BPM (τ=50, N=400)     |
|         | even for a perfectly clean signal. Fix: divide by `acorr0·(N−τ)/N`         |
|         | instead of `acorr0`. Clean signal now yields SQI ≈ 1.0 at all HR.          |
|         | Updated: `_compute_hr2()` in `incunest_afe4490.cpp`, `HR2TestCalc.update()` |
|         | in `pulsenest_lab.py`, `test_hr2.cpp` (thresholds raised to >0.95 clean,     |
|         | >0.80 noisy). Spec §5.3. Observed with MS100 simulator: HR2 SQI ~0.9       |
|         | vs HR1/HR3 ~1.0 — now consistent.                                           |
| v0.17   | HR3_SQI: parabolic interpolation on HPS values (not P[k]) in numerator.    |
|         | Fixes SQI collapse at inter-bin frequencies (e.g. 85 BPM = bin 14.5):      |
|         | cubic product loss in HPS reduced SQI to ~0.5 for clean signals. Now SQI   |
|         | > 0.80 across all frequencies. New unit test: `test_hr3_85bpm`. (§5.4)     |
| v0.16   | Offline runner specification added (§9): `incunest_offline_runner` native C++   |
|         | tool. `INCUNEST_OFFLINE` compile flag stubs Arduino/FreeRTOS and enables        |
|         | `UNIT_TEST` API for algorithm-only builds. Hann window precomputation       |
|         | moved to `_reset_algorithms()`. Input CSV: column-name-based parser,        |
|         | format-agnostic, requires RED/IR/RED_Amb/IR_Amb/RED_Sub/IR_Sub. Output:        |
|         | per-file `_result.csv` + `batch_summary.csv`. Build: CMake, C++17, no      |
|         | external deps. Optional firmware delta columns for equivalence checking.    |
| v0.15   | HR3_SQI redesigned: from linear spectral concentration to HPS peak           |
|         | prominence (`HPS[peak_bin] / Σ HPS[k]`). Eliminates harmonic inflation of   |
|         | the denominator. No new buffer — `hps_sum` accumulated in the existing loop. |
|         | Updated: `incunest_afe4490.cpp` `_compute_hr3()`, `incunest_afe4490.h` comment,       |
|         | `HR3TestCalc` in `pulsenest_lab.py`, spec §5.4.                               |
| v0.14   | Timing instrumentation: `INCUNEST_TIMING_STATS` compile-time flag, `TimingStat` |
|         | struct, `_emit_timing()` private method. Measures HR1/HR2/HR3/SpO2 per-call |
|         | time and full cycle time (SPI + all algos) with `esp_timer_get_time()`.      |
|         | `$TIMING` serial frame emitted every ~5 s. `TIMING` window added to         |
|         | `pulsenest_lab.py` with green/orange/red budget indicator (budget = 2000 µs). |
|         | Spec §8.4 added. Enabled in `platformio.ini` via `-DINCUNEST_TIMING_STATS=1`.    |
| v0.13   | SQI continuous [0–1] for all algorithms. HR1_SQI: CV-based. HR2_SQI:       |
|         | normalised autocorrelation peak. HR3_SQI: spectral concentration.           |
|         | SpO2_SQI: PI-based (clamp PI from 0.5%–2.0%). All SQI fields added to      |
|         | `AFE4490Data` and $M1 frame. 4 TEST windows completed (SPO2TEST, HR1TEST,  |
|         | HR2TEST, HR3TEST). pulsenest_lab.py spec §8 updated.                          |
| v0.12   | Added HR3 algorithm (FFT + HPS): `hr3`/`hr3_valid` in `AFE4490Data`,     |
|         | `setHR3Filter()` in API, section 5.4. Self-contained radix-2 DIT FFT     |
|         | in anonymous namespace. `_recalc_biquad_lp()` for LP anti-aliasing.      |
|         | `INCUNEST_AFE4490_TASK_STACK` default 4096→8192. Architecture diagrams         |
|         | (sections 1.1 and 1.3) updated. $M1 serial trama field 17 = HR3.        |
| v0.11   | HR reported range corrected per ISO 80601-2-61 + neonatal use:           |
|         | **[30, 250] → [25, 300] BPM**. Guard band updated to [22, 303] BPM.      |
|         | `hr1_refractory_s` 0.200→0.185 s (covers 303 BPM guard: 198 ms period).   |
|         | `hr2_min_lag_s` 0.22→0.185 s. `hr2_acorr_max_lag` 111→137 samples.      |
|         | Python `HR_MIN_HZ`/`HR_MAX_HZ`/`HR_SEARCH_*` updated accordingly.        |
| v0.10   | Guard band: internal HR search range extended to [27, 253] BPM (±3 BPM  |
|         | beyond reported [30, 250] BPM). HR2: `hr2_acorr_max_lag` 100→111        |
|         | (27 BPM @ 50 Hz). HR3 Python: `HR_SEARCH_MIN_HZ`/`HR_SEARCH_MAX_HZ`     |
|         | added. `hr_search_min_bpm`/`hr_search_max_bpm` constants documented.     |
| v0.9    | HR measurement range extended: 40–240 BPM → **30–250 BPM**.             |
|         | `hr_min_bpm` 40→30, `hr_max_bpm` 240→250, `hr1_refractory_s` 0.3→0.2 s  |
|         | (allows up to ~300 BPM), `hr2_acorr_max_lag` 75→100 samples (30 BPM    |
|         | at 50 Hz). Spec header updated to v0.9.                                  |
| v0.8    | Added HR2 algorithm (autocorrelation): `hr2`/`hr2_valid` in `AFE4490Data`, |
|         | `setHR2Filter()` in API, section 5.3. Updated architecture diagrams        |
|         | (sections 1.1 and 1.3). HR1/HR2 run in parallel in `_process_sample()`.   |
|         | BiquadFilter refactored as struct; `_biquad_process()` extracted.          |
|         | Section 5.2 (HR1) fully documented: DC removal IIR, moving average LP,    |
|         | running-max threshold, refractory period, 5-interval averaging, peak       |
|         | marker diagnostic, and key constants table.                                |
| v0.7    | Added `stop()` (section 2.6): clean shutdown — detaches ISR, deletes       |
|         | internal task and FreeRTOS objects, resets algorithm state. Allows          |
|         | `begin()` to be called again. Added `_reset_algorithms()` (private).       |
|         | Enables hot-swap between incunest_afe4490 and protocentral at runtime.          |
| v0.6    | Biquad coefficients computed dynamically (`_recalc_biquad()`). Formula:    |
|         | bilinear transform on a 2nd-order Butterworth bandpass prototype.          |
|         | Coefficients moved from namespace constants to members `_bq_b0..a2`.       |
|         | Called from `_recalc_rate_params()` and `setPPGDispFilter()`. The "not yet        |
|         | implemented" warning removed. `setPPGDispFilter()` is now fully functional for    |
|         | any fs, f_low, f_high.                                                     |
| v0.5    | Internal constants reorganised at the top of the anonymous namespace in    |
|         | `.cpp`, in snake_case. Algorithm time parameters expressed in physical      |
|         | units (seconds): `spo2_warmup_s`, `dc_iir_tau_s`, `ac_ema_tau_s`,         |
|         | `hr1_refractory_s`. New members `_spo2_warmup_samples`,                     |
|         | `_hr1_refractory_samples`, `_dc_iir_alpha`, `_ac_ema_beta` derived from     |
|         | `_sample_rate_hz` via `_recalc_rate_params()`, called in constructor and   |
|         | in `setSampleRate()`. SpO2 and HR algorithms now correct at any PRF.       |
| v0.4    | `AFE4490Data` extended with the 6 raw AFE4490 signals (led1, led2,         |
|         | aled1, aled2, led1_aled1, led2_aled2). Struct is now equivalent in         |
|         | information to the protocentral output.                                    |
| v0.3    | Configurable NUMAV: added `setAdcAverages(uint8_t num)` to the API.        |
|         | Section 11.1 with NUMAV_max formula, table by PRF, clamping behaviour and  |
|         | SNR effect. `setSampleRate()` recalculates NUMAV_max.                      |
|         | **First implementation:** `include/incunest_afe4490.h` and                      |
|         | `src/incunest_afe4490.cpp` generated. Pending hardware validation.              |
| v0.2    | Added section 11 with comparative register tables (timing, analog, control,|
|         | init sequence). Sources: AFE44x0.h EVM TI v1.4, Datasheet AFE4490 Table 2,|
|         | Protocentral src/. incunest_afe4490 values justified register by register.      |
|         | Updated defaults: NUMAV=7, ENSEPGAIN=0, Stage2 disabled.                   |
| v0.1    | Initial complete specification                                              |
