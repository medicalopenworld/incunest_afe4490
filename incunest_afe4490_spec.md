# incunest_afe4490 — Specification v0.53

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
    ├─→ _compute_analog_state()  →  AFE4490AnalogState (V_ADC, V_TIA, I_PD, OT)  — see §3c
    │       ├─→ _rsqm_update()      (probe state, DiagCode, RSQI)
    │       └─→ _hgac_update()      (TIAGUARD guard + RF step-down — Phase 1, §5.8)
    │
    ├─→ selected channel (setPPGChannel)
    │       └─→ bandpass filter (setPPGDispFilter) → ppg + HR1 (peak detection)
    │
    ├─→ LED1-ALED1 (IR corr.) → bandpass 0.5–5 Hz → decimate ×10 → HR2 (autocorrelation)
    │
    ├─→ LED1-ALED1 (IR corr.) → LP 10 Hz → decimate ×10 → 512-sample Hann → FFT → HPS → HR3
    │
    └─→ LED1-ALED1 (IR corr.) + LED2-ALED2 (RED corr.) → AC/DC → SpO2
```

---

## 2. Public API

### 2.1 Data struct
```cpp
struct AFE4490Data {
    // Field order mirrors the $M1/$P1 serial frame: raw signals first, then processed outputs
    // Raw ADC outputs (6 signals from AFE4490)
    int32_t led2;       // LED2VAL  — RED raw           (frame: RED)
    int32_t led1;       // LED1VAL  — IR raw            (frame: IR)
    int32_t aled2;      // ALED2VAL — ambient after LED2 (frame: RED_Amb)
    int32_t aled1;      // ALED1VAL — ambient after LED1 (frame: IR_Amb)
    int32_t led2_aled2; // LED2-ALED2 — RED ambient-corrected (frame: RED_Sub)
    int32_t led1_aled1; // LED1-ALED1 — IR ambient-corrected  (frame: IR_Sub)
    // Processed outputs
    int32_t ppg;        // filtered PPG of selected channel (IR, for HR1 peak detection)
    float   ppg_disp;   // PPG for display only: normalized [0..1], PI-weighted (low PI → range < 1). Never use for SpO2 or HR calculations.
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
    uint32_t   diag_code;    // DiagCode bitmask: RSQM_DIAG_AMB_SAT | RSQM_DIAG_SIGNAL_WEAK | RSQM_DIAG_HW_SETTLING | ...
    ProbeState probe_state;  // PROBE_DISCONNECTED, PROBE_NOT_APPLIED, PROBE_APPLIED, or PROBE_SATURATING
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
| TIA CF         | auto (see §7.2): 55 pF @ 500 Hz / RF_100K |
| Stage 2 gain   | disabled                      |
| ENSEPGAIN      | 0 (single gain)               |
| NUMAV          | 7 (8 averages, max 9 at 500Hz)|
| PPG channel    | LED1_ALED1                    |
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
void setTIACF(AFE4490TIACF cf);         // manual override; overridden again if setTIAGain()/setSampleRate() is called
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
    AFE4490TIACF      afe_tia_cf_led1;       // TIA CF for LED1 (IR) — TIAGAIN; active when ENSEPGAIN=1
    AFE4490RG afe_stg2_rg_led1;  // STG2GAIN1 for LED1 — gain bits only, no enable
    bool              afe_stg2_en_led1;        // STAGE2EN1 (D14 of TIAGAIN) — explicit enable
    AFE4490RF    afe_tia_rf_led2;     // TIA RF for LED2 (RED) — TIA_AMB_GAIN; always active
    AFE4490TIACF      afe_tia_cf_led2;       // TIA CF for LED2 (RED) — TIA_AMB_GAIN; always active
    AFE4490RG afe_stg2_rg_led2;  // STG2GAIN2 for LED2 — gain bits only, no enable
    bool              afe_stg2_en_led2;        // STAGE2EN2 (D14 of TIA_AMB_GAIN) — explicit enable; also forced ON when AMBDAC > 0
    uint8_t           afe_ambdac_uA;         // Ambient cancellation DAC current [0–10 µA]; AMBDAC[3:0] in TIA_AMB_GAIN D19:D16
    AFE4490Channel    ppgdisp_channel;       // PPG display channel selection
    float             ppgdisp_f_low_hz;      // PPG display bandpass lower cutoff [Hz]
    float             ppgdisp_f_high_hz;     // PPG display bandpass upper cutoff [Hz]
    float             hr2_f_low_hz;          // HR2 bandpass lower cutoff [Hz]
    float             hr2_f_high_hz;         // HR2 bandpass upper cutoff [Hz]
    float             hr3_f_low_hz;          // HR3 bandpass lower cutoff [Hz]
    float             hr3_f_high_hz;         // HR3 bandpass upper cutoff [Hz]
    float             spo2_a;                   // SpO2 calibration: SpO2 = a - b*R
    float             spo2_b;
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
    float             rsqm_signal_weak_std;      // LED1_Sub_std threshold for SIGNAL_WEAK [adc_code]
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
void setHR3Filter(float f_low_hz = 0.4f, float f_high_hz = 15.0f);
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
void setRsqmSignalWeakStd(float v);             // LED1_Sub_std threshold for SIGNAL_WEAK [adc_code]
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
After DIAG_EN is cleared the analog front-end (TIA, stage 2) needs a few PRP cycles to re-settle. During holdoff, `_task_body()` skips the SPI read and instead calls `_process_sample()` with the frozen last-valid raw ADC values (`_diag_last_led1/2`, `_diag_last_aled1/2`, `_diag_last_led1/2_aled1/2`). A constant input is absorbed by all IIR/BPF/LP filters with near-zero AC contribution, making it far less disruptive to HR1/HR2/HR3/SpO2 than a transient outlier. The raw values are saved on every normal `_task_body()` cycle and are only accessed from that same task — no mutex is needed.

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
enum class AFE4490Channel {
    LED1,        // IR raw
    LED2,        // RED raw
    ALED1,       // ambient after LED1
    ALED2,       // ambient after LED2
    LED1_ALED1,  // IR ambient-corrected (default)
    LED2_ALED2   // RED ambient-corrected
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

enum class AFE4490TIACF {
    CF_5P,     // 5 pF (default)
    CF_10P,
    CF_20P,
    CF_30P,
    CF_55P,
    CF_155P,
    // any combination supported by register CF_LED[4:0]
};

enum class AFE4490RG {
    RG_100K,    // 0 dB   — ×1   — RG=100 kΩ  (Ri=100 kΩ fixed, datasheet Table 1)
    RG_150K,  // 3.5 dB — ×1.5 — RG=150 kΩ
    RG_200K,    // 6 dB   — ×2   — RG=200 kΩ
    RG_300K,  // 9.5 dB — ×3   — RG=300 kΩ
    RG_400K    // 12 dB  — ×4   — RG=400 kΩ
};

// Appended at the end (values 0-2 unchanged) to keep the $M2/$M3/$M4 wire encoding
// ((int)probe_state) stable for existing consumers/CSVs.
enum class ProbeState {
    PROBE_DISCONNECTED,  // cable or probe connector not plugged in
    PROBE_NOT_APPLIED,   // probe connected, OT indicates no tissue in the optical path
    PROBE_APPLIED,       // probe on patient, normal measurement
    PROBE_SATURATING     // a channel is saturated/off-spec (!CH_VALID_RANGE) — presence unknown;
                         // saturation can happen with or without a patient (e.g. an unapplied
                         // probe under a phototherapy lamp), so this does NOT imply APPLIED.
                         // Algorithms (SpO2/HR1/HR2/HR3) and RSQI treat it like any non-
                         // APPLIED state. HGAC treats it like PROBE_APPLIED (§5.8.2).
};

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
// Bits 13+: RSQM software checks — updated continuously at 500 Hz by _rsqm_update()
constexpr uint32_t RSQM_DIAG_AMB_SAT     = 0x00002000UL;  // bit 13 — ambient channel near positive rail
constexpr uint32_t RSQM_DIAG_SIGNAL_WEAK = 0x00004000UL;  // bit 14 — differential signal amplitude too low
constexpr uint32_t RSQM_DIAG_HW_SETTLING = 0x00008000UL;  // bit 15 — hardware settling after config change
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

// TIA feedback capacitor — indexed by AFE4490TIACF (0=CF_5P … 5=CF_155P)
constexpr float kAFE_CF_PF[6]  = { 5.0f, 10.0f, 20.0f, 30.0f, 55.0f, 155.0f };

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

Register-code arrays (`rf_code[]`, `cf_code[]`, `stg2_gain_code[]`) remain in an anonymous namespace in `incunest_afe4490.cpp` — internal only, never exposed.

### String codecs (inline)

Convert between enum values and the label strings used in the `$CFG`/`$SET` protocol.

```cpp
// TIA feedback resistor RF
inline const char* afeRFToStr(AFE4490RF g);          // → "10K"/"25K"/"50K"/"100K"/"250K"/"500K"/"1M"
inline bool        afeStrToRF(const char* s, AFE4490RF& out);  // returns false if unrecognised

// TIA feedback capacitor CF
inline const char* afeCFToStr(AFE4490TIACF cf);            // → "5p"/"10p"/"20p"/"30p"/"55p"/"155p"
inline bool        afeStrToCF(const char* s, AFE4490TIACF& out);

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
    AFE4490RF hgac_rf_led1;          // current RF, IR domain — HGAC Phase 1 (§5.8)
    AFE4490RF hgac_rf_led2;          // current RF, RED domain
};
```

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
discontinuity in SpO2's input, so `_hgac_step_rf_down()` no longer needs to rescale the SpO2
`EmaChannel` by k (removed; see §5.8, `EmaChannel::rescale()` deleted as dead code) — R is
never corrupted by a gain step, without the reactive rescale mechanism HGAC Phase 1 used.
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
what they did catch overlapped with RSQM's own `PROBE_DISCONNECTED`/`PROBE_NOT_APPLIED`
classification, just with a narrower/uncalibrated criterion and no debounce. Having SpO2
(and, in principle, HR1/HR2/HR3) each maintain their own copy of "is there a probe/finger"
detection duplicates RSQM's one job across N places with N different, uncoordinated
thresholds — architecturally wrong even before considering whether any single one of them
works correctly.

**v0.41: `_spo2_update(float ot_ir, float ot_red, ProbeState probe_state)`** — a third
parameter, RSQM's classification, consumed but never computed here. While
`probe_state != PROBE_APPLIED` (covers `PROBE_NOT_APPLIED`, `PROBE_DISCONNECTED`, and (since
2026-07-24) `PROBE_SATURATING` identically — same treatment RSQI already gives them): both EMAs are reset **every sample**
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

Coefficients `a` and `b` are empirical (calibration). Configurable via `setSpO2Coefficients()`.

**DC and AC² estimation via EmaChannel:**

Both DC and AC power are extracted from a single `EmaChannel` per signal (IR and RED), using two independent time constants:

- **`spo2_ema_mean_tau_s` = 2.0 s** — tracks the DC baseline (slow IIR low-pass, f_c ≈ 0.08 Hz). Must satisfy f_c << 0.5 Hz (minimum physiological HR at 30 bpm) so the mean does not follow the pulsatile component. Same value as `rsqm_ema_mean_tau_s` — both track the DC of the same signal with the same physical requirement.
- **`spo2_ema_var_tau_s` = 6.0 s** — tracks the AC² power (EMA of `(x − mean)²`). Minimum averaging time required by **ISO 80601-2-61:2026 Annex JJ.2 d)**, which mandates ≥ 6 s averaging for the SpO2 transfer standard reference. A shorter τ would give a fluctuating AC estimate that violates the norm.
- **`spo2_warmup_s` = 18.0 s** — SpO2 output suppressed until 3 × τ_var = 3 × 6 s, time for the EMA variance estimator to converge.

The two τ values are intentionally different because DC and AC² have distinct temporal requirements: DC must track slow baseline drift (τ = 2 s), while AC² must be stable enough for a reliable SpO2 estimate (τ ≥ 6 s per ISO).

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
3. **Running maximum:** exponential decay tracker (`× 0.9999` per sample) keeps amplitude reference current.
4. **Threshold crossing:** rising edge detected when signal crosses `0.6 × running_max`. A refractory period (0.185 s, ~324 BPM practical limit) prevents double-detection.
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
| `hr1_refractory_s` | 0.185 s | Refractory period between peaks (~324 BPM practical limit) |
| `hr1_peak_marker_samples` | 10 samples | Duration of peak marker (hr1_ppg=0) |
| `hr_min_bpm` | 40 BPM | Reported valid HR range minimum (ISO 80601-2-61; neonatal use) |
| `hr_max_bpm` | 260 BPM | Reported valid HR range maximum (neonatal tachycardia) |
| `hr_search_min_bpm` | 37 BPM | Internal search lower bound (guard band: hr_min − 3) |
| `hr_search_max_bpm` | 263 BPM | Internal search upper bound (guard band: hr_max + 3) |

> **Guard band design:** all HR algorithms search internally in [37, 263] BPM and force `hr*_sqi = 0.0` when the result falls outside [40, 260] BPM. This prevents boundary-clipping errors where a signal at exactly 40 BPM could be missed due to small algorithmic offsets. The ±3 BPM margin is consistent with the ISO 80601-2-61 measurement tolerance.

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
1. Biquad bandpass filter 0.5–5 Hz at 500 Hz (eliminates DC, high-frequency noise, and acts as anti-aliasing)
2. Decimate by factor 10 → effective rate 50 Hz
3. Accumulate in circular buffer of 400 samples (8 s at 50 Hz)
4. Every 25 decimated samples (0.5 s), compute normalised autocorrelation over lags corresponding to 30–272 BPM

**Peak selection:** first local maximum above `hr2_min_corr = 0.5`. Sub-sample resolution via parabolic interpolation.

**Key constants:**

| Constant | Value | Meaning |
|---|---|---|
| `hr2_buf_len` | 400 | Buffer length (8 s at 50 Hz) |
| `hr2_acorr_max_lag` | 137 | Max lag searched (guard band 22 BPM at 50 Hz: 50×60/22=137) |
| `hr2_decim_factor` | 10 | 500 Hz → 50 Hz |
| `hr2_update_interval` | 25 | Recompute every 0.5 s |
| `hr2_min_lag_s` | 0.22 s | Min RR lag (~272 BPM max) |
| `hr2_min_corr` | 0.5 | Normalised correlation threshold for valid peak |

`hr2_sqi` is a continuous [0–1] quality metric equal to the **unbiased** normalised autocorrelation value at the dominant lag:

```
acorr_unbiased[τ] = Σ_{i=0}^{N-τ-1} x[i]·x[i+τ]  /  (acorr0 · (N−τ)/N)
acorr0            = Σ_{i=0}^{N-1} x[i]²
SQI               = acorr_unbiased[peak_lag]
```

The unbiased correction factor `N/(N−τ)` compensates for the finite-window effect: the biased estimator (dividing by `acorr0` alone) systematically underestimates because its numerator sums only `N−τ` terms while `acorr0` reflects `N` terms. Without this correction a perfectly periodic signal at 60 BPM (τ=50, N=400) would yield SQI = 350/400 = 0.875 even with zero noise. With the correction, a clean periodic signal yields SQI ≈ 1.0 regardless of the HR.

A value close to 1 means the signal is strongly periodic at the detected RR period. `hr2_min_corr = 0.5` acts as the minimum threshold below which no peak is reported and SQI is forced to 0. If the buffer is not yet full or no valid peak is found, SQI = 0.

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
1. 2nd-order Butterworth low-pass filter at 10 Hz (anti-aliasing before decimation)
2. Decimate by factor 10 → effective rate 50 Hz
3. Accumulate in circular buffer of 512 samples (10.24 s at 50 Hz)
4. Every 25 decimated samples (0.5 s): mean subtraction (DC removal) → Hann window → in-place radix-2 DIT FFT → HPS → parabolic interpolation

**Harmonic Product Spectrum (HPS):** multiplies the power spectrum by its 2nd and 3rd harmonic downsampled versions: `HPS[k] = P[k] · P[2k] · P[3k]`. Reinforces the fundamental frequency; suppresses dominant harmonics that can mislead simpler peak-finding.

**Self-contained FFT:** radix-2 DIT FFT (Cooley-Tukey) implemented in the anonymous namespace to avoid external dependencies. Works in both `env:in3ator_V15` and `env:native` test environments.

**Key constants:**

| Constant | Value | Meaning |
|---|---|---|
| `hr3_buf_len` | 512 | Buffer length (10.24 s at 50 Hz) |
| `hr3_decim_factor` | 10 | 500 Hz → 50 Hz |
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
| HR2 | Autocorrelation on decimated signal (50 Hz, 8 s window) | Implemented |
| HR3 | FFT + HPS on decimated signal (50 Hz, 10.24 s window) | Implemented |
| HR4 | True peak detection via derivative (max of derivative = steepest ascending slope) | Planned |

HR1–HR4 all operate on `led1_aled1` (IR ambient-corrected) and run in parallel. `AFE4490Data` will expose `hr4`/`hr4_valid` when implemented.

> All HR algorithms developed from scratch. Protocentral code is not used as a base.

---

### 5.6 RSQM — Raw Signal Quality Monitor

Runs in `_process_sample()` after `_compute_analog_state()`, once per sample at 500 Hz. Fills `AFE4490Data::rsqi`, `diag_code`, and `probe_state`. Receives `AFE4490AnalogState` by `const&` (pre-computed OT values).

#### 5.6.1 EMA estimators

Two per-channel EMA estimators track mean and variance of the differential signal (LED_Sub = LED − ALED):

```
ema_mean += α_mean × (x − ema_mean)          α_mean = 1−exp(−1/(τ_mean·fs)), τ_mean = rsqm_ema_mean_tau_s (default 2.0 s)
d = x − ema_mean
ema_var  += α_var  × (d² − ema_var)           α_var  = 1−exp(−1/(τ_var·fs)),  τ_var  = rsqm_ema_var_tau_s  (default 4.0 s)
```

Both `rsqm_ema_mean_tau_s` and `rsqm_ema_var_tau_s` are runtime-configurable via `setRsqmEmaTau()`; changing them triggers `_recalc_rate_params()`.

Estimators are considered **ready** after `rsqm_ready_count` = 6000 samples (3 × τ_var = 12 s @ 500 Hz). During warmup, `probe_state = PROBE_DISCONNECTED` and `rsqi = 0`.

> Reset required on hardware config change (RF / RG / AMBDAC): `ema_mean = x_current; ema_var = 0; count = 0`. Implemented as part of HGAC (not yet active in v0.27).

#### 5.6.2 Probe state machine

**Priority order:** PROBE_DISCONNECTED > PROBE_SATURATING > PROBE_NOT_APPLIED > PROBE_APPLIED.

**PROBE_DISCONNECTED** — instantaneous criterion, ALL of:
- `LED1_raw < 0` AND `ALED1_raw < 0` AND `LED2_raw < 0` AND `ALED2_raw < 0`
- `|LED1_Sub| < 5000` AND `|LED2_Sub| < 5000`

Physical cause: when cable is disconnected, TIA input is floating; input-offset current drives all four channels (LED1, ALED1, LED2, ALED2) to the same large negative value, so `LED_Sub = LED − ALED ≈ 0`. No EMA warmup required; responds in one sample.

Validated with afe_sweep_test.csv: DISCONNECTED → all channels negative, LED_Sub_mean ≈ −592 (range −3117..+65).

**PROBE_SATURATING** (added 2026-07-24, distinct from PROBE_NOT_APPLIED since — see below) —
checked before the OT threshold, when not DISCONNECTED:

```
PROBE_SATURATING  ⟺  !as.otLed1Valid() || !as.otLed2Valid()
```

i.e. either color's OT source channels (its own phase or its ambient phase) are not `CH_VALID_RANGE`
(ADC saturated either rail, or TIA beyond linearity — see §3c). Under saturation the OT ratio
itself is not trustworthy, so it is not even evaluated against `rsqm_ot_thr` in this case.

**Not the same as "no tissue".** Saturation is a property of signal amplitude, not of patient
presence — an unapplied probe under strong or flickering light (e.g. a phototherapy lamp) can
saturate with nobody connected, and a genuinely applied probe can saturate from motion or an
ambient spike. So `PROBE_SATURATING` deliberately does **not** imply `PROBE_APPLIED`; it only
asserts "presence unknown, channel out of range". Before 2026-07-24 this case was folded into
`PROBE_NOT_APPLIED` (see former text below) — separating it is what makes HGAC's gate (§5.8.2)
able to act on real saturation without also acting during ordinary probe-idle periods.

Consumers: SpO2/HR1/HR2/HR3 and RSQI treat `PROBE_SATURATING` like any other non-`PROBE_APPLIED`
state (invalid output, internal state reset — no code change needed, they already gate on
`!= PROBE_APPLIED`). `_diag_task_body()` treats it like `PROBE_APPLIED` (skips periodic HW
diagnostics — running one during a real saturation event would disrupt HGAC's attempt to clear
it). HGAC (§5.8.2) treats it like `PROBE_APPLIED` (acts to correct the saturation).

**PROBE_NOT_APPLIED** — detected via *optical transmittance*, when not DISCONNECTED and not
SATURATING (i.e. both color channels are already known `CH_VALID_RANGE`):

```
OT = (I_PD_LED − I_PD_ALED) / I_LED     [A/A, dimensionless]
```

Since v0.35 OT is computed in `_compute_analog_state()` and stored in `AFE4490AnalogState` (`ot_led1`/`ot_led2`, see §3c); `_rsqm_update()` only consumes it. Responds in one sample; PPG modulation (~2%) produces negligible OT variation.

OT is the **full electro-optical system ratio** — it combines LED wall-plug efficiency, tissue transmittance, and photodiode responsivity into a single dimensionless number. It is not the tissue optical transmittance alone.

- Threshold: `as.ot_led1 > rsqm_ot_thr` OR `as.ot_led2 > rsqm_ot_thr` → PROBE_NOT_APPLIED; otherwise PROBE_APPLIED *(OR logic: one channel is enough)*. Default `rsqm_ot_thr = 1.0e-4` (v0.44, widened from 8.5e-5) — runtime-configurable via `setRsqmOtThr()`.
- Single threshold (no hysteresis) — state oscillation is prevented by the `rsqm_probe_state_min_s` debounce in `_rsqm_request_probe_state()` instead.
- **Until 2026-07-24**, this check also folded in `!as.otLedxValid()` (saturation) via OR — collapsing "channel saturated" and "channel clean but reads high" into the same `PROBE_NOT_APPLIED` value. Split into the dedicated `PROBE_SATURATING` state above (see its rationale).

Estimated values (theoretical, need empirical calibration with HW): APPLIED ≈ 1.4e-5, NOT_APPLIED ≈ 8e-4 (factor ~×54). First real-HW test (2026-07-19, CONTEC MS100 simulator, probe applied) showed the theoretical APPLIED estimate does not hold in practice: the simulator is highly sensitive to probe placement and OT frequently reads above 8.5e-5 with the probe applied, so `probe_state` never reached APPLIED at that threshold. `rsqm_ot_thr` was widened to 1.0e-4 as a practical mitigation — still not a calibrated value from a real neonatal probe/patient.

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
| 0x4000 | `RSQM_DIAG_SIGNAL_WEAK` | `ProbeState == PROBE_APPLIED` AND (`LED1_Sub_std < 2000` OR `LED2_Sub_std < 2000`) — both channels needed for SpO2 (needs HW calibration) |
| 0x8000 | `RSQM_DIAG_HW_SETTLING` | Countdown > 0 (`_rsqm_settling_countdown`) |

HGAC publishes no DiagCode bit (the former bit 16 `HGAC_DIAG_GAIN_FLOOR` was removed in v0.50 — no consumer decoded it; the "at RF floor and still saturating" condition is derivable at the point of use as `rf == RF_10K` while TIAGUARD persists, and its actuator, ILED, is Phase 4).

Multiple bits may be set simultaneously.

**Update (v0.37):** `RSQM_DIAG_HW_SETTLING` is no longer unreachable — HGAC (§5.8) is its first real producer, arming `_rsqm_settling_countdown` after every RF step-down. `RSQM_DIAG_AMB_SAT` and `RSQM_DIAG_SIGNAL_WEAK` still have no programmatic consumer beyond the aggregate `rsqi` gate and human inspection.

#### 5.6.4 RSQI

```
rsqi = 1  iff  (estimators ready) AND (probe_state == PROBE_APPLIED) AND (diag_code == 0)
rsqi = 0  otherwise
```

---

### 5.7 Autonomous diagnostics task (`_diag_task_body`)

A dedicated internal FreeRTOS task (`incunest_diag`, low priority) calls `runAfeDiagnostics()` autonomously on a periodic schedule:

- **Periodic** — the task wakes every `rsqm_diag_period_ms` (default 5000 ms) via `vTaskDelay`. Diagnostics run only in states with no patient confirmed in the optical path: `probe_state == PROBE_DISCONNECTED` OR `probe_state == PROBE_NOT_APPLIED`. `PROBE_APPLIED` and `PROBE_SATURATING` are excluded to avoid disrupting HR/SpO2 acquisition with the 10 ms blackout window — and, for `PROBE_SATURATING` specifically, to avoid disrupting HGAC's attempt to clear the saturation while a real patient may be connected. This is a whitelist by design (since v0.50): any future `ProbeState` defaults to NOT diagnosing, so a new state can never silently interrupt acquisition — it must be added here deliberately. Equivalent to the former `≠ APPLIED AND ≠ SATURATING` blacklist for the four states that exist today.

There is no on-state-change trigger: it only advanced the first post-transition diagnostic by ≤ 5 s while no patient was present (no clinical value) and woke the task pointlessly on transitions into APPLIED/SATURATING. Removed in v0.50 (see §14).

The task is created in `begin()` and deleted in `stop()`. It runs on core 1 at `INCUNEST_AFE4490_DIAG_TASK_PRIORITY` (= main task priority − 2).

`runAfeDiagnostics()` remains a public API callable independently from external code (e.g. via `$DIAG?`). Both callers serialise through `_spi_mutex`.

---

### 5.8 HGAC — Hardware Gain Control (Phase 1: RF-only TIAGUARD response)

**Status (v0.37): Phase 1 implemented — RF-only guard stepping RF down on TIAGUARD
(`v_tia` above the guard threshold, below full-scale), no ascent yet (manual recovery).
Disabled by default (`hgac_enable = false`).**
Runs in `_process_sample()`
immediately after `_rsqm_update()`, once per sample at 500 Hz. This is the first of a planned
multi-phase rollout (full design, closed architecturally, in the project's design notes):
Phase 1 (this), Phase 2 (RF ascent with prediction/hysteresis), Phase 3 (telemetry closure),
Phase 4 (ILED actuation + thermal envelope), Phases 5-7 (feed-forward, P² estimator, UI —
optional/deferred).

> **Why HGAC is RF-only in v1 — ILED is a weak de-saturation actuator at the gain floor
> (analysis 2026-08-04).** In the LED phase, `v_tia_led = 2·RF·(i_amb + i_led)` with
> `i_led = OT·ILED`; only `i_led` scales with ILED (ambient current does not — see the ILED-settling
> note above). The maximum voltage that lowering ILED can shed at RF = 10K (the floor) is
> `Δv = 2·RF·OT·(ILED_max − ILED_min) = 800·OT` V. For an APPLIED neonatal probe (OT < `rsqm_ot_thr`
> ≈ 1e-4, typically 1e-5…1e-4) that is only **~0.01–0.08 V** — negligible. The reason: at RF = 10K
> the LED-driven part of `v_tia` is itself tiny (~0.1 V), so if a channel is *saturating* there
> (~1.2 V ADC rail), the bulk is **ambient**, which ILED cannot touch. This is why the descent
> ladder's response to "RF at floor and still saturating" is the **excessive-ambient-light alarm**
> (discriminated by the ALED phase), not ILED reduction — and it motivates keeping v1 RF-only.
> Lowering ILED only sheds meaningful voltage in the rare LED-dominated case (high OT, very
> translucent tissue, ALED not saturating). OT thresholds still need empirical calibration, so
> Δv is order-of-magnitude. Design rationale lives in the project's HGAC design notes.

#### 5.8.1 TIAGUARD — TIA over-threshold guard (single guard, v1)

With Stage 2 frozen at ×1 gain and AMBDAC=0 (v1 defaults), `v_tia == v_adc` — the ADC
clips before the TIA's own linearity limit, so ADC saturation and TIA-linearity saturation
collapse into a single guard trip. **TIAGUARD** names this condition — `v_tia` above the
guard threshold (`GUARD_V` = 0.9 V, which sits BELOW full-scale; see `tia_axis`) — not the
response to it:

```
tiaguard(channel) = v_tia > hgac_v_tia_thr   (default 0.9 V)
```

Evaluated per gain domain (IR = LED1+ALED1; RED = LED2+ALED2 — the two domains ENSEPGAIN=1
creates, see §5.6/§11 TIAGAIN/TIA_AMB_GAIN): either the LED phase or its ambient phase
saturating trips that domain's guard (both share one RF).

#### 5.8.2 Gate G0

```
// Implemented as a gate-OFF in _hgac_update() (reset TIAGUARD debounce counters + return):
gate_off = (probe_state == PROBE_NOT_APPLIED OR probe_state == PROBE_DISCONNECTED)
           OR (diag_code & RSQM_DIAG_HW_SETTLING)
// Idle states listed explicitly (not !(APPLIED||SATURATING)): a future ProbeState is NOT
// gated off here and would let HGAC act — revisit this list when adding a ProbeState.
// Equivalent today to: gate_ok = (APPLIED OR SATURATING) AND !HW_SETTLING
```

**Removed in v0.44c (2026-07-21):** a third clause `invalid_fraction_ema.mean <=
hgac_gate_invalid_frac_max` (a dedicated `EmaChannel` fed `1.0` when
`!AFE4490AnalogState::allValidRange()`, else `0.0`, window 0.2 s) used to gate HGAC on a recent
history of "clean" samples. Removed because it was self-defeating for a corrective actuator:
`allValidRange()` IS the TIAGUARD condition HGAC exists to correct, so the gate blocked the fix
in proportion to how much the problem it fixes had been occurring. Worse, its time constant
(0.2 s window, crosses the 0.5 threshold after ~139 ms of continuous TIAGUARD — exact EMA
step-response math) is faster than the TIAGUARD persistence debounce (0.5 s, §5.8.1) that
commits an actual RF step — so under *sustained* TIAGUARD the gate closed and reset the
TIAGUARD counters (see `_hgac_update()`) before the debounce could ever complete, blocking the
step-down exactly when it was most needed. No test ever caught this: the one test that
exercises the step-down path (`test_hgac_descent_after_persistence`) deliberately used a
stimulus (`v_tia ≈ 0.973 V`, since changed — see below) that stayed below `kVTiaFs`
(1.0 V) — i.e. `allValidRange()` stayed `true` throughout — so the EMA clause was never actually
exercised by continuous TIAGUARD in practice. HGAC's own persistence debounce
inside `_hgac_check_tiaguard()` (O8.1) already exists specifically to reject one-sample blips,
making this upstream smoothing redundant even in the cases where it didn't backfire.

**Resolved 2026-07-24 (was an open question 2026-07-21-23):** the gate used to be
`probe_state != PROBE_DISCONNECTED` — a deviation from the closed design's literal "gate on
PROBE_APPLIED", needed because RSQM's (then-overloaded) `PROBE_NOT_APPLIED` classifier also
fired when a channel's optical-transmittance ratio became invalid due to clipping (`otLedxValid()`
requires `CH_VALID_RANGE`) — i.e. exactly the TIAGUARD condition HGAC exists to correct. Gating on
`PROBE_APPLIED` literally would have let a severe/fast TIAGUARD event flip `probe_state` to
`NOT_APPLIED` and then block the very RF step-down that would clear it (a deadlock). But that
gate was broader than the deadlock required: it also let HGAC act during *any*
`PROBE_NOT_APPLIED`, including the common "probe idle, no patient" state — and since Phase 1
has no ascent, an unrelated optical event in that state (e.g. a phototherapy lamp on an
unapplied probe) could have driven RF down irreversibly before a real patient was connected.

**The actual fix was upstream, not in this gate:** RSQM now reports the TIAGUARD condition as
its own state, `PROBE_SATURATING` (§5.6.2), distinct from `PROBE_NOT_APPLIED`. This gate simply
treats `PROBE_SATURATING` like `PROBE_APPLIED` — no need to reason about which `NOT_APPLIED`
causes are "safe" to include, because the ambiguous cause no longer reports as `NOT_APPLIED`
at all. `PROBE_DISCONNECTED` is excluded as before (structurally incompatible with TIAGUARD —
all 4 channels near zero, §5.6.2); plain `PROBE_NOT_APPLIED` (probe idle, not saturating) is
now correctly excluded too.

#### 5.8.3 TIAGUARD response algorithm

Per gain domain, debounced by `hgac_tiaguard_debounce_min_s` (default 0.5 s = 250 samples @ 500 Hz):

1. If not TIAGUARD → reset the debounce counter for that domain.
2. If TIAGUARD and RF is not at the floor (`RF_10K`) → after the debounce window, step RF
   down one LUT level (`kAFE_RF_OHM`) via `_hgac_step_rf_down()`.
3. If TIAGUARD and RF is already at `RF_10K` → nothing to do (no further step possible; ILED
   actuation is Phase 4). The debounce counter stays latched (not reset) so it doesn't
   retrigger the instant TIAGUARD clears. No DiagCode bit is published (the former
   `HGAC_DIAG_GAIN_FLOOR` was removed in v0.50 — see §14; the condition is derivable at the
   point of use as `rf == RF_10K` while TIAGUARD persists).

No ascent in Phase 1 — recovery from a lowered RF is manual (a documented limitation of this
phase, not an oversight; Phase 2 adds predicted/hysteresis-protected ascent).

#### 5.8.4 Atomic RF change + SpO2 EMA rescale (`_hgac_step_rf_down`)

A gain change must not corrupt SpO2's R ratio. `_spo2_ch_ir_ema`/`_spo2_ch_red_ema`
(`EmaChannel`, §5.1) are linear EMAs, so a gain step by a known factor `k` rescales exactly:

```
k = kAFE_RF_OHM[new_rf] / kAFE_RF_OHM[old_rf]
mean ← mean × k          // EmaChannel::rescale(k)
var  ← var × k²
```

`_hgac_step_rf_down()` performs the register write (via the existing `setTIAGainLED1`/`LED2`
setters — reused, not duplicated) and the corresponding EMA rescale as one operation, then
arms `_rsqm_settling_countdown` (≈150 ms, `afe_settle_time_s`) — the first real producer of
`RSQM_DIAG_HW_SETTLING` (§5.6.3). **This ordering is mandatory, not an optimization:** without
it, an RF change would leave the EMA's stale pre-change state in place for ~3×`spo2_ema_var_tau_s`
(≈18 s), during which R can be corrupted by tens to ~100% (documented in the project's O10/C1
analysis). There is no shippable intermediate state where automatic RF actuation exists without
this rescale.

Only applies to RF changes (clean rescale, since RF is linear in the signal chain). ILED
changes (Phase 4) do NOT get this treatment — ambient current does not scale with ILED
(asymmetric residual, tracked separately) — so ILED changes must instead gate SpO2 invalid
during settling, not rescale.

**Concurrency:** `_hgac_update()` and everything it calls take no lock of their own — they
run from `_process_sample()`, already under `_state_mutex`. `_hgac_step_rf_down()` calls
`setTIAGainLED1/2()`, which take `_spi_mutex` internally; nesting `_spi_mutex` inside the
caller's `_state_mutex` is safe because no code path acquires `_spi_mutex` first and then
blocks on `_state_mutex`. **Contract:** the `_hgac_*` actuation primitives must only ever be
called from `_hgac_update()` — never directly from application code or another task.

#### 5.8.5 Parameters (see §10.9) and telemetry

`AFE4490Config` fields: `hgac_enable`, `hgac_v_tia_thr`, `hgac_v_tia_ideal`
(telemetry only in v1), `hgac_tiaguard_debounce_min_s`. `$SET`/`$LCFG` wiring in PulseNest's
`src/main.cpp`. `$M4` gained
`HGAC_RF1,HGAC_RF2` (current RF per domain), sourced from `AFE4490DebugData` fields
`hgac_rf_led1`/`hgac_rf_led2` (§3d), populated alongside `analog` in `_process_sample()`.
(A third field `HGAC_ALARM` / `hgac_alarm_gain_floor` existed through v0.49 and was removed in
v0.50 — see §14.)

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

This margin is used both by `_apply_timing_regs()` (to set LED2STC/LED1STC) and by `_recalc_tia_cf()` (to select CF). Both functions always use the same value — they are in sync.

**`_recalc_tia_cf()`** selects the **largest** `AFE4490TIACF` value whose physical capacitance satisfies the constraint. It is called automatically from `_recalc_rate_params()` (triggered by `setSampleRate()` and constructor) and from `setTIAGain()`. `setTIACF()` can override afterwards.

**CF selection table at 500 Hz** (settle = 200 counts = 50 µs):

| RF (AFE4490RF) | RF (Ω) | CF_max (pF) | Auto CF |
|---|---|---|---|
| RF_10K  | 10 kΩ  | 1000 | CF_155P |
| RF_25K  | 25 kΩ  | 400  | CF_155P |
| RF_50K  | 50 kΩ  | 200  | CF_155P |
| RF_100K | 100 kΩ | 100  | CF_55P  |
| RF_250K | 250 kΩ | 40   | CF_30P  |
| RF_500K | 500 kΩ | 20   | CF_20P  |
| RF_1M   | 1 MΩ   | 10   | CF_10P  |

At lower Fs the settle window is larger and CF increases further (e.g. at 100 Hz, settle = 250 µs → RF_500K gets CF_55P).

**TIA pole frequency** (for reference, not used in the calculation):
```
f_pole = 1 / (2π × RF × CF)
```
At 500 Hz / RF_500K / CF_20P: f_pole ≈ 15.9 kHz — well above the PPG signal band (0.5–20 Hz).

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

**Compile-time flag:** `INCUNEST_TIMING_STATS` (default 0). Set to 1 via `platformio.ini` `build_flags` to enable. When 0, all instrumentation code is compiled out with zero overhead.

**Frame format:**
```
$TIMING,hr1_mean,hr1_max,hr2_mean,hr2_max,hr3_mean,hr3_max,
        spo2_mean,spo2_max,cycle_mean,cycle_max,stack_free*XX
```
- All time values in **µs** (measured with `esp_timer_get_time()`, resolution 1 µs)
- `stack_free`: remaining stack of the incunest_afe4490 FreeRTOS task in 4-byte words (`uxTaskGetStackHighWaterMark`)
- Checksum: XOR of all bytes between `$` and `*` (NMEA style)
- Emitted every `ts_emit_interval = 2500` samples (~5 s at 500 Hz); stats reset after each emission

**Timing scope:**

| Field | What is measured |
|---|---|
| `hr1_mean/max` | `_hr1_update()` execution time |
| `hr2_mean/max` | `_hr2_update_sample()` execution time (fast path: biquad + decimate + buffer; autocorr itself is timed separately by `hr2_compute_mean/max`, Task B) |
| `hr3_mean/max` | `_hr3_update_sample()` execution time (fast path: LP filter + decimate + buffer; FFT+HPS itself is timed separately by `hr3_compute_mean/max`, Task C) |
| `spo2_mean/max` | `_spo2_update()` execution time |
| `cycle_mean/max` | Full sample cycle: SPI burst-read (4 channels) + SW subtraction + `_process_sample()` |

**Budget:** cycle budget = 2000 µs (1 sample period at 500 Hz).
- `cycle_max < 1800 µs` → green (safe, 10% margin)
- `1800 ≤ cycle_max ≤ 2000 µs` → orange (tight)
- `cycle_max > 2000 µs` → red (over budget, risk of missed samples)

**Implementation notes:**
- Individual algo timers are in `_process_sample()`; cycle timer is in `_task_body()`
- `_emit_timing()` is a private method; formats and sends the frame, then resets all `TimingStat` accumulators
- `TimingStat` struct (fields: `max_us`, `sum_us`, `count`) lives in the private section of `INCUNEST_AFE4490`, guarded by `#if INCUNEST_TIMING_STATS`

---

## 9. Offline runner — `incunest_offline_runner`

### 9.1 Purpose

A native C++ command-line tool that runs the incunest_afe4490 algorithms (HR1, HR2, HR3, SpO2) on CSV files received from real Incunest incubators, without any hardware. Used to calibrate and improve the Incunest firmware using real neonatal PPG data.

### 9.2 Input CSV format

Incunest incubators export 10-second CSV files at 500 Hz (5000 samples). The format is not fixed — it may vary across firmware versions — but the following six columns are always present:

| Column name | Signal | `AFE4490Data` field |
|---|---|---|
| `RED`    | LED2VAL — RED raw          | `led2`       |
| `IR`     | LED1VAL — IR raw           | `led1`       |
| `RED_Amb` | ALED2VAL — ambient after LED2 | `aled2`   |
| `IR_Amb`  | ALED1VAL — ambient after LED1 | `aled1`   |
| `RED_Sub` | LED2-ALED2 — RED corrected | `led2_aled2` |
| `IR_Sub`  | LED1-ALED1 — IR corrected  | `led1_aled1` |

**Parser rule:** the parser reads the CSV header row and finds the above columns **by name** (case-insensitive). Column order and extra columns are ignored. The parser aborts with a clear error if any of the six required columns is missing.

**Optional firmware result columns:** if the CSV also contains columns `FW_HR1`, `FW_HR2`, `FW_HR3`, `FW_SpO2`, the runner includes them in the output and adds delta columns (`delta_HR1`, etc.) for algorithm equivalence checking. These columns do not affect calibration — their sole purpose is to verify that the offline C++ runner and the firmware produce identical results.

### 9.3 Compile-time flag: `INCUNEST_OFFLINE`

To compile `incunest_afe4490.cpp` without Arduino, SPI, or FreeRTOS, define `INCUNEST_OFFLINE` before including the header. This flag:

1. Replaces all platform includes with `incunest_afe4490_platform_stub.h` (type stubs only).
2. Disables `begin()`, `stop()`, `getData()`, the ISR, and all FreeRTOS objects.
3. Implicitly enables `UNIT_TEST`, exposing `test_feed_*` / `test_hr*` / `test_spo2*` methods.
4. Keeps all algorithm methods (`_hr1_update`, `_hr2_update_for_test`, `_hr3_update_for_test`, `_spo2_update`, `_reset_algorithms`, `_recalc_rate_params`, `_biquad_process`) fully functional.

`INCUNEST_OFFLINE` is never defined in firmware builds. It is only used by the offline runner and any future native unit tests.

### 9.4 `incunest_afe4490_platform_stub.h`

Thin stub (~30 lines) located at `lib/incunest_afe4490/incunest_afe4490_platform_stub.h` — alongside the library header that includes it. Provides:

- Standard integer types (`uint8_t`, `int32_t`, etc.) via `<cstdint>`
- `inline uint32_t millis() { return 0; }` (not used by algorithms, present to avoid linker error)
- `SemaphoreHandle_t`, `QueueHandle_t`, `TaskHandle_t` as `void*` typedefs (never accessed under `INCUNEST_OFFLINE`)
- `IRAM_ATTR` as empty macro
- `portTICK_PERIOD_MS` as `1`

No Arduino-specific classes (`Serial`, `SPI`, `SPIClass`) are needed — they are all guarded by `#ifndef INCUNEST_OFFLINE` in the library.

### 9.5 Changes required in `incunest_afe4490.h` / `.cpp`

| Change | Location | Reason |
|---|---|---|
| Wrap platform includes in `#ifdef INCUNEST_OFFLINE` / `#else` | `incunest_afe4490.h` top | Avoid Arduino/FreeRTOS headers on native build |
| Wrap `begin()`, `stop()`, `getData()`, ISR, task methods in `#ifndef INCUNEST_OFFLINE` | `incunest_afe4490.h` + `.cpp` | These are hardware-only |
| Move Hann window precomputation from `begin()` to `_reset_algorithms()` | `incunest_afe4490.cpp` | `begin()` is disabled under `INCUNEST_OFFLINE`; `_reset_algorithms()` is available |
| `#define UNIT_TEST` when `INCUNEST_OFFLINE` is defined | `incunest_afe4490.h` | Expose `test_feed_*` API |

### 9.6 File structure

```
lib/incunest_afe4490/
  incunest_afe4490.h              — library header (includes incunest_afe4490_platform_stub.h when INCUNEST_OFFLINE)
  incunest_afe4490.cpp            — library implementation
  incunest_afe4490_platform_stub.h    — Arduino/FreeRTOS type stubs (used by the library, not the runner)

tools/
  offline_runner/
    CMakeLists.txt           — CMake build (C++17, no external deps)
    main.cpp                 — CSV reader, algorithm driver, output writer
```

The runner includes `../../lib/incunest_afe4490/incunest_afe4490.cpp` directly via CMake `target_sources`. No subproject, no installation, no package manager.

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
| Feedback capacitance C_F | `setTIACF()` (or auto) | auto (see §7.2) | 5, 10, 20, 30, 55, 155 pF | TIA RC pole: must settle before ADC samples. Auto-selected by library from R_F and PRF |

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
| Input channel selection | `setPPGChannel()` default | LED1_ALED1 | All algorithms | Selects which of the 6 raw signals enters the processing chain |
| Decimation factor | `hr2_decim_factor`, `hr3_decim_factor` | ×10 | HR2, HR3 | Reduces 500 Hz → 50 Hz before autocorrelation / FFT; sets effective frequency resolution |

---

### 10.3 SpO2 parameters

See §5.1 for algorithm description.

| Parameter | Constant | Value | Justification | Effect |
|---|---|---|---|---|
| Calibration coefficient a | `spo2_a` | 114.9208 | Empirical — MS100 simulator / Nellcor R-curve | SpO2 = a − b·R intercept |
| Calibration coefficient b | `spo2_b` | 30.5547 | Empirical — MS100 simulator / Nellcor R-curve | SpO2 = a − b·R slope |
| DC EMA time constant | `spo2_ema_mean_tau_s` | 2.0 s | f_c ≈ 0.08 Hz << 0.5 Hz (30 bpm min HR); same signal as RSQM → same τ | Speed of DC baseline tracking |
| AC² EMA time constant | `spo2_ema_var_tau_s` | 6.0 s | **ISO 80601-2-61:2026 Annex JJ.2 d)**: averaging time ≥ 6 s for SpO2 reference | Stability of AC amplitude / PI estimate |
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
| Peak threshold fraction | 0.6 | 0.6 × running_max | Trade-off: higher → misses low-amplitude peaks; lower → false detections on noise |
| Refractory period | `hr1_refractory_s` | 0.185 s | Minimum interval between detected peaks; practical upper HR limit ~324 BPM |
| RR interval buffer length | 5 intervals | 5 | Number of consecutive intervals averaged for HR1 output |
| SQI CV threshold | `hr1_sqi_cv_max` | 0.15 (15 %) | CV above this → SQI = 0 (arrhythmia / motion artefact) |

---

### 10.5 HR2 parameters

See §5.3 for algorithm description.

| Parameter | Constant | Value | Effect |
|---|---|---|---|
| BPF lower cutoff | `hr2_f_low_hz` | 0.5 Hz | Removes DC drift and very low-frequency baseline wander |
| BPF upper cutoff | `hr2_f_high_hz` | 5.0 Hz | Anti-aliasing before decimation; limits harmonic content |
| Decimation factor | `hr2_decim_factor` | 10 | 500 Hz → 50 Hz |
| Buffer length | `hr2_buf_len` | 400 samples (8 s) | Analysis window; longer → more stable at low HR, longer convergence time |
| Update interval | `hr2_update_interval` | 25 decimated samples (0.5 s) | HR2 output refresh rate |
| Max autocorrelation lag | `hr2_acorr_max_lag` | 137 samples | Corresponds to 22 BPM guard band at 50 Hz |
| Minimum lag | `hr2_min_lag_s` | 0.185 s | Corresponds to ~324 BPM; prevents false peaks at very short lags |
| Minimum correlation threshold | `hr2_min_corr` | 0.5 | Below this normalised correlation, no HR is reported and SQI = 0 |

---

### 10.6 HR3 parameters

See §5.4 for algorithm description.

| Parameter | Constant | Value | Effect |
|---|---|---|---|
| BP filter cutoffs | `hr3_f_low_hz` / `hr3_f_high_hz` | 0.4–15 Hz | Removes DC/baseline drift; passes up to 3rd harmonic of 260 BPM (13 Hz) |
| Decimation factor | `hr3_decim_factor` | 10 | 500 Hz → 50 Hz |
| FFT / buffer length | `hr3_buf_len` | 512 samples (10.24 s) | Frequency resolution = 50/512 ≈ 0.098 Hz ≈ 0.35 BPM at 1 Hz |
| Update interval | `hr3_update_interval` | 25 decimated samples (0.5 s) | HR3 output refresh rate |
| Window function | Hann | — | Reduces spectral leakage; pre-computed at `_reset_algorithms()` |
| HPS harmonics | P[k]·P[2k]·P[3k] | 2nd and 3rd | Reinforces fundamental; HPS search capped at nyquist/3 to keep 3rd harmonic in band |

---

### 10.7 Range and validation parameters (transversal)

Applied by all HR algorithms equally to decide whether an output is reportable and whether SQI is forced to 0.

| Parameter | Constant | Value | Scope |
|---|---|---|---|
| Reported valid HR lower bound | `hr_min_bpm` | 40 BPM | HR1, HR2, HR3 — ISO 80601-2-61; neonatal |
| Reported valid HR upper bound | `hr_max_bpm` | 260 BPM | HR1, HR2, HR3 — neonatal tachycardia |
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

### 10.9 HGAC parameters (Phase 1: RF-only TIAGUARD response)

See §5.8 for algorithm description.

| Parameter | API | Default | Range | Effect |
|---|---|---|---|---|
| Master enable | `setHgacEnable()` | `false` | true / false | HGAC never actuates while disabled — safe default until bench-validated |
| Saturation guard threshold | `setHgacVTiaThr()` | 0.9 V | — | `v_tia` above this trips the TIAGUARD guard for that domain |
| Ideal operating point | — (config field only) | 0.6 V | — | Telemetry only in v1 — not consumed by the TIAGUARD response logic yet |
| TIAGUARD persistence | `setHgacTiaguardDebounceMinS()` | 0.5 s (250 samples @ 500 Hz) | 0.0–10.0 | Debounce before an RF step-down — urgent path (O8.1) |
| Settling time (fixed, not `$SET`-tunable) | `afe_settle_time_s` (private) | 0.15 s | — | Arms `RSQM_DIAG_HW_SETTLING` after an RF change; coarse diagnostic window, distinct from the sub-sample-period `_compute_settle_margin()` used for CF auto-selection |

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
| → CF LED1 | 5 pF | — | 5 pF | **auto (§7.2)** | Auto-selected to maximise noise filtering while guaranteeing 5τ settling. Default at 500 Hz / RF_100K: 55 pF |
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

Three distinct hardware states are distinguishable from the raw ADC channel pattern. Used by RSQM (probe presence detection) and HGAC (inhibit control when probe not on patient).

| State | `led1` | `led2` | `aled1` | `aled2` | Physical cause |
|---|---|---|---|---|---|
| **PROBE_DISCONNECTED** | ≈ −5000 | ≈ −5000 | ≈ −5000 | ≈ −5000 | Photodiode open-circuit; TIA input floating → input offset current × RF produces small negative output |
| **PROBE_NOT_APPLIED** | saturated | saturated | low–mid | low–mid | No tissue attenuation; LEDs illuminate photodiode directly. ALED channels depend on ambient light level |
| **PROBE_APPLIED** | PPG signal | PPG signal | low | low | Normal operation; tissue attenuates LED light; ambient channels remain low |

**Detection method:** see §5.6. DISCONNECTED is instantaneous; SATURATING/NOT_APPLIED/APPLIED use optical transmittance and channel validity (single threshold + 0.2 s debounce, 100 samples @ 500 Hz — derived from `rsqm_probe_state_min_s × fs`); APPLIED uses 750-sample EMA warmup for RSQM_DIAG_SIGNAL_WEAK.

> These states are computed by RSQM (_rsqm_update) and gate HGAC operation (§5.8.2): HGAC acts in `PROBE_APPLIED` and `PROBE_SATURATING`, and must not act in `PROBE_DISCONNECTED` or plain `PROBE_NOT_APPLIED`.

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
  Normative standard defining valid measurement ranges, tolerances, and test conditions. Source of the [70–100 %] SpO2 range; HR bounds [40–260 BPM] are derived from ISO neonatal criteria.

- **Severinghaus, J.W. & Astrup, P.B. (1986).** "History of blood gas analysis." *Journal of Clinical Monitoring*, 2(4), 270–288.
  Historical context for non-invasive oxygen saturation measurement.

---

## 14. Version history

| Version | Description                                                                 |
|---------|-----------------------------------------------------------------------------|
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
|         | `_hr2_update_sample`; `_linearize_hr2` → `_hr2_linearize`; `_compute_hr2`   |
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
