# incunest_afe4490 — Specification v0.31

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
6 raw signals
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
    int32_t ppg;        // filtered PPG of selected channel
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
    // RSQM outputs (filled every sample by _update_rsqm)
    uint8_t    rsqi;         // Raw Signal Quality Index: 0=invalid, 1=valid
    uint32_t   diag_code;    // DiagCode bitmask: RSQM_AMB_SAT | RSQM_SIGNAL_WEAK | RSQM_HW_SETTLING | ...
    ProbeState probe_state;  // PROBE_DISCONNECTED, PROBE_NOT_APPLIED, or PROBE_APPLIED
};
```

### 2.2 Initialization
```cpp
void begin(int pin_cs, int pin_drdy);
```
Initializes SPI, configures the chip with default values, attaches ISR to DRDY, and starts the internal FreeRTOS task.

**Default values after begin():**

| Parameter      | Default value                  |
|----------------|-------------------------------|
| Sample rate    | 500 Hz                        |
| LED1 current   | 11.7 mA                       |
| LED2 current   | 11.7 mA                       |
| LED range      | 150 mA (TX_REF = 0.75V)       |
| TIA gain (RF)  | 500 kΩ                        |
| TIA CF         | auto (see §7.2): 20 pF @ 500 Hz / RF_500K |
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
void setTIAGain(AFE4490TIAGain gain);   // auto-recalculates CF (see §7.2); setTIACF() can override
void setTIACF(AFE4490TIACF cf);         // manual override; overridden again if setTIAGain()/setSampleRate() is called
void setStage2Gain(AFE4490Stage2Gain gain);      // sets gain for both channels; does NOT touch STAGE2EN1/2
void setStage2GainLED1(AFE4490Stage2Gain gain);  // STG2GAIN1 only; does NOT touch STAGE2EN1
void setStage2GainLED2(AFE4490Stage2Gain gain);  // STG2GAIN2 only; does NOT touch STAGE2EN2
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
    AFE4490TIAGain    afe_tia_gain_led1;      // TIA RF for LED1 (IR) — TIAGAIN; active when ENSEPGAIN=1
    AFE4490TIACF      afe_tia_cf_led1;       // TIA CF for LED1 (IR) — TIAGAIN; active when ENSEPGAIN=1
    AFE4490Stage2Gain afe_stage2_gain_led1;  // STG2GAIN1 for LED1 — gain bits only, no enable
    bool              afe_stage2_en1;        // STAGE2EN1 (D14 of TIAGAIN) — explicit enable
    AFE4490TIAGain    afe_tia_gain_led2;     // TIA RF for LED2 (RED) — TIA_AMB_GAIN; always active
    AFE4490TIACF      afe_tia_cf_led2;       // TIA CF for LED2 (RED) — TIA_AMB_GAIN; always active
    AFE4490Stage2Gain afe_stage2_gain_led2;  // STG2GAIN2 for LED2 — gain bits only, no enable
    bool              afe_stage2_en2;        // STAGE2EN2 (D14 of TIA_AMB_GAIN) — explicit enable; also forced ON when AMBDAC > 0
    uint8_t           afe_ambdac_uA;         // Ambient cancellation DAC current [0–10 µA]; AMBDAC[3:0] in TIA_AMB_GAIN D19:D16
    AFE4490Channel    ppgdisp_channel;       // PPG display channel selection
    AFE4490Filter     ppgdisp_filter_type;   // PPG display filter type
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
    float             spo2_dc_iir_tau_s;         // DC removal IIR time constant [s]
    float             spo2_ac_ema_tau_s;         // AC² EMA time constant [s]
    float             spo2_min;                  // valid output lower bound [%]
    float             spo2_max;                  // valid output upper bound [%]
    float             spo2_min_dc;               // no-finger DC threshold [ADC counts]
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
};
AFE4490Config getConfig();
```

**Thread safety:** takes `_spi_mutex` (for hardware fields) and `_state_mutex` (for signal/filter fields) if `_initialized == true`. Safe before and after `begin()`.

**Use case:** allows the host application (e.g. `pulsenest_lab.py`) to read back the active chip configuration for display, logging, or verification.

### 2.4 Signal and filter configuration
```cpp
void setPPGChannel(AFE4490Channel channel);
void setPPGDispFilter(AFE4490Filter type, float f_low_hz = 0.5f, float f_high_hz = 20.0f);
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
void setSpO2DcIirTauS(float tau_s);                        // DC IIR time constant [s] — recalc
void setSpO2AcEmaTauS(float tau_s);                        // AC² EMA time constant [s] — recalc
void setSpO2Range(float min_pct, float max_pct);           // valid SpO2 output range [%]
void setSpO2MinDC(float counts);                           // no-finger DC threshold [ADC counts]
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
```

### 2.6 Data retrieval
```cpp
bool getData(AFE4490Data& data);
// Returns true and fills data if an item is available in the queue.
// Returns false immediately if the queue is empty (non-blocking).
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
uint32_t runAfeDiagnostics(uint32_t diag_holdoff_ms = 100);
```
Runs the AFE4490 hardware self-test sequence (datasheet section 8.4.3.3, Table 3).

**Parameters:**
- `diag_holdoff_ms` — duration (ms) of the post-diagnostic holdoff window. Default: 10 ms (~5 samples at 500 Hz), which covers the worst-case TIA settling time (RF_1M + CF_155P: 5τ ≈ 775 µs). Set to 0 to observe raw artefacts from the re-settling analog front-end.

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

enum class AFE4490Filter {
    NONE,
    MOVING_AVERAGE,
    BUTTERWORTH
};

enum class AFE4490TIAGain {
    RF_10K,
    RF_25K,
    RF_50K,
    RF_100K,
    RF_250K,
    RF_500K,   // default
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

enum class AFE4490Stage2Gain {
    GAIN_0DB,    // 0 dB (×1, unity gain) — Stage 2 active; disable via setEnStage2GainLED1/2(false)
    GAIN_3_5DB,
    GAIN_6DB,
    GAIN_9_5DB,
    GAIN_12DB
};

enum class ProbeState {
    PROBE_DISCONNECTED,  // cable or probe connector not plugged in
    PROBE_NOT_APPLIED,   // probe connected but not on patient tissue
    PROBE_APPLIED        // probe on patient, normal measurement
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
// Bits 13+: RSQM software checks — updated continuously at 500 Hz by _update_rsqm()
constexpr uint32_t RSQM_AMB_SAT     = 0x00002000UL;  // bit 13 — ambient channel near positive rail
constexpr uint32_t RSQM_SIGNAL_WEAK = 0x00004000UL;  // bit 14 — differential signal amplitude too low
constexpr uint32_t RSQM_HW_SETTLING = 0x00008000UL;  // bit 15 — hardware settling after config change
// bits 16–31 reserved
```

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
Based on the ratio R of the AC and DC components of the ambient-corrected signals:

```
R = (AC_RED / DC_RED) / (AC_IR / DC_IR)

SpO2 = a - b × R
```

Coefficients `a` and `b` are empirical (calibration). Configurable via `setSpO2Coefficients()`.
`spo2_sqi` is a continuous [0–1] quality metric based on the Perfusion Index (PI):

```
sqi = clamp((PI − 0.5) / (2.0 − 0.5), 0, 1)
```

PI < 0.5 % → SQI = 0 (absent or very weak signal, no finger contact). PI ≥ 2.0 % → SQI = 1 (full quality). Thresholds from Nellcor/Masimo clinical reference. If SpO2 is outside the valid range [70–100 %] or the DC level is below the no-finger threshold, SQI is forced to 0.

### 5.2 HR1 — Peak detection

**Processing chain:**
1. **DC removal:** IIR low-pass filter (τ = 1.6 s) estimates DC; subtracted from raw signal. Signal is negated for conventional PPG polarity (peaks up).
2. **Low-pass filter:** moving average with cutoff ~5 Hz (`len = fs / (2 × 5)`; at 500 Hz → 50 samples). Capped at 64 samples max.
3. **Running maximum:** exponential decay tracker (`× 0.9999` per sample) keeps amplitude reference current.
4. **Threshold crossing:** rising edge detected when signal crosses `0.6 × running_max`. A refractory period (0.2 s, ~300 BPM max) prevents double-detection.
5. **RR interval buffer:** last 5 consecutive intervals stored. HR1 computed as average:

```
HR1 (bpm) = fs × 60 / mean(last 5 RR intervals in samples)
```

`hr1_sqi` is a continuous [0–1] quality metric based on the coefficient of variation (CV) of the 5 most recent RR intervals:

```
CV  = std(RR[0..4]) / mean(RR[0..4])
SQI = clamp(1 − CV / 0.15, 0, 1)
```

CV = 0 (perfectly regular rhythm) → SQI = 1. CV ≥ 15 % → SQI = 0 (arrhythmia, artefact, or loss of signal). The 15 % threshold is the standard clinical criterion for rhythm regularity. If fewer than 5 intervals have been detected, or if HR1 is outside [25, 300] BPM, SQI is forced to 0. The refractory period of 200 ms naturally supports detection up to ~300 BPM, providing a guard band above the 250 BPM limit at no cost.

> **Implementation note:** peak location is currently approximated by the rising threshold crossing (first sample above `0.6 × running_max`). This introduces a timing error that depends on signal slope and amplitude — if either changes (low perfusion, motion), the detected instant shifts relative to the true peak, introducing jitter in the RR interval. Planned improvement: apply derivative to the filtered signal and detect the rising edge as the maximum of the derivative (steepest ascending slope), which gives a more stable and precise timing reference independent of signal amplitude.

**Diagnostic field `hr1_ppg`:** after each detected peak, `hr1_ppg` is forced to 0 for 10 samples. This produces a visible marker in the serial stream / plotter that survives serial downsampling.

**Key constants:**

| Constant | Value | Meaning |
|---|---|---|
| `hr1_dc_tau_s` | 1.6 s | IIR DC removal time constant |
| `hr1_ma_cutoff_hz` | 5 Hz | Moving average low-pass cutoff |
| `hr1_ma_max_len` | 64 samples | Max moving average length |
| `hr1_refractory_s` | 0.2 s | Refractory period between peaks (~300 BPM max) |
| `hr1_peak_marker_samples` | 10 samples | Duration of peak marker (hr1_ppg=0) |
| `hr_min_bpm` | 40 BPM | Reported valid HR range minimum (ISO 80601-2-61; neonatal use) |
| `hr_max_bpm` | 300 BPM | Reported valid HR range maximum (neonatal tachycardia) |
| `hr_search_min_bpm` | 37 BPM | Internal search lower bound (guard band: hr_min − 3) |
| `hr_search_max_bpm` | 303 BPM | Internal search upper bound (guard band: hr_max + 3) |

> **Guard band design:** all HR algorithms search internally in [22, 303] BPM and force `hr*_sqi = 0.0` when the result falls outside [25, 300] BPM. This prevents boundary-clipping errors where a signal at exactly 25 BPM could be missed due to small algorithmic offsets. The ±3 BPM margin is consistent with the ISO 80601-2-61 measurement tolerance.

### 5.3 HR2 — Autocorrelation
Independent second HR algorithm running in parallel with HR1 on the same `led1_aled1` signal.

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

`hr3_sqi` is a continuous [0–1] metric based on the **two-bin HPS fraction** in a local window. This design is bin-position-independent: a signal at a non-integer bin spreads HPS energy across two adjacent bins (b1 = `peak_bin`, b2 = `b_other`), and the ratio of those two bins to their local context is the same regardless of fractional offset.

```
b_other   = peak_bin + sign(delta)          clipped to [1, nyquist/3]
hps_num   = hps_at(peak_bin) + hps_at(b_other)
b_lo      = min(peak_bin, b_other) − hr3_sqi_local_w
b_hi      = max(peak_bin, b_other) + hr3_sqi_local_w
hps_win   = Σ hps_at(k) for k in [b_lo, b_hi] ∩ [1, nyquist/3]
n_win     = number of bins summed
fraction  = hps_num / hps_win
baseline  = 2 / n_win               (uniform-noise reference: 2 signal bins out of n_win)
SQI       = clamp((fraction − baseline) / (1 − baseline), 0, 1)
```

Parameters: `hr3_sqi_local_w = 5` bins (window extends ±5 bins beyond the two peak bins, ~±0.49 Hz). If `hps_win = 0` or `n_win ≤ 2`, SQI = 0 (conservative).

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

Runs first in `_process_sample()`, once per sample at 500 Hz. Fills `AFE4490Data::rsqi`, `diag_code`, and `probe_state`.

#### 5.6.1 EMA estimators

Two per-channel EMA estimators track mean and variance of the differential signal (LED_Sub = LED − ALED):

```
ema_mean += α_mean × (x − ema_mean)          α_mean = 0.02  (τ ≈ 100 ms @ 500 Hz)
d = x − ema_mean
ema_var  += α_var  × (d² − ema_var)           α_var  = 0.004 (τ ≈ 500 ms @ 500 Hz)
```

Estimators are considered **ready** after 750 samples (3 × τ_var = 1500 ms). During warmup, `probe_state = PROBE_DISCONNECTED` and `rsqi = 0`.

> Reset required on hardware config change (RF / RG / AMBDAC): `ema_mean = x_current; ema_var = 0; count = 0`. Implemented as part of HGAC (not yet active in v0.27).

#### 5.6.2 Probe state machine

**Priority order:** PROBE_DISCONNECTED > PROBE_NOT_APPLIED > PROBE_APPLIED.

**PROBE_DISCONNECTED** — instantaneous criterion, ALL of:
- `LED1_raw < 0` AND `ALED1_raw < 0` AND `LED2_raw < 0` AND `ALED2_raw < 0`
- `|LED1_Sub| < 5000` AND `|LED2_Sub| < 5000`

Physical cause: when cable is disconnected, TIA input is floating; input-offset current drives all four channels (LED1, ALED1, LED2, ALED2) to the same large negative value, so `LED_Sub = LED − ALED ≈ 0`. No EMA warmup required; responds in one sample.

Validated with afe_sweep_test.csv: DISCONNECTED → all channels negative, LED_Sub_mean ≈ −592 (range −3117..+65).

**PROBE_NOT_APPLIED** — detected via *optical transmittance* when not DISCONNECTED:

```
optical_transmittance = LED_Sub / (LED_mA × RF_Ω × RG_linear)
```

- **OT computed from instantaneous `LED_Sub` values** (not EMA). Responds in one sample; margin of ×54 makes PPG noise (2% modulation → ±0.001 OT) negligible.
- When `LED_raw ≥ 2 096 921` (near ADC positive rail): `ot = 100.0` (sentinel — prevents false APPLIED from saturated channel)
- Threshold: `ot1 > 0.30` OR `ot2 > 0.30` → PROBE_NOT_APPLIED; otherwise PROBE_APPLIED *(OR logic: one channel is enough)*
- Single threshold (no hysteresis) — state oscillation is prevented by the 50-sample debounce in `_set_probe_state()` instead.

Measured values (afe_sweep_test.csv, clip probe): APPLIED ≈ 0.05, NOT_APPLIED ≈ 2.7–2.96 (factor ×54).

> Valid for clip-type reusable probes only. Disposable adhesive probes not validated.

**PROBE_APPLIED** — neither of the above.

**State transition debounce (`_set_probe_state`):** any state change requires 100 consecutive samples (~200 ms @ 500 Hz) with the same requested state before being committed. A single different sample resets the counter. This eliminates false transient states during cable connect/disconnect events.

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

**Bits 13+ — RSQM software (updated continuously at 500 Hz by `_update_rsqm()`):**

| Bit | Constant | Condition |
|---|---|---|
| 0x2000 | `RSQM_AMB_SAT` | `aled1 ≥ 2 096 921` OR `aled2 ≥ 2 096 921` |
| 0x4000 | `RSQM_SIGNAL_WEAK` | `ProbeState == PROBE_APPLIED` AND (`LED1_Sub_std < 2000` OR `LED2_Sub_std < 2000`) — both channels needed for SpO2 (needs HW calibration) |
| 0x8000 | `RSQM_HW_SETTLING` | Countdown > 0 (set externally by HGAC; not yet active in v0.27) |

Multiple bits may be set simultaneously.

#### 5.6.4 RSQI

```
rsqi = 1  iff  (estimators ready) AND (probe_state == PROBE_APPLIED) AND (diag_code == 0)
rsqi = 0  otherwise
```

---

### 5.7 Autonomous diagnostics task (`_diag_task_body`)

A dedicated internal FreeRTOS task (`incunest_diag`, low priority) calls `runAfeDiagnostics()` autonomously in two situations:

1. **On probe state change** — `_set_probe_state()` sends `xTaskNotify` when a new state is committed. The task wakes immediately and runs diagnostics (regardless of state, including APPLIED).
2. **Periodic timeout** — `xTaskNotifyWait` times out every `rsqm_diag_period_ms` (default 5000 ms). Diagnostics run only if `probe_state ≠ PROBE_APPLIED`, to avoid disrupting HR/SpO2 acquisition with the 10 ms blackout window.

The task is created in `begin()` and deleted in `stop()`. It runs on core 1 at `INCUNEST_AFE4490_DIAG_TASK_PRIORITY` (= main task priority − 2).

`runAfeDiagnostics()` remains a public API callable independently from external code (e.g. via `$DIAG?`). Both callers serialise through `_spi_mutex`.

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

| RF (AFE4490TIAGain) | RF (Ω) | CF_max (pF) | Auto CF |
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
| `hr1_mean/max` | `_update_hr1()` execution time |
| `hr2_mean/max` | `_update_hr2()` execution time (biquad + autocorr combined) |
| `hr3_mean/max` | `_update_hr3()` execution time (LP filter + FFT + HPS combined) |
| `spo2_mean/max` | `_update_spo2()` execution time |
| `cycle_mean/max` | Full sample cycle: SPI burst-read (6 channels) + `_process_sample()` |

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
4. Keeps all algorithm methods (`_update_hr1`, `_update_hr2`, `_update_hr3`, `_update_spo2`, `_reset_algorithms`, `_recalc_rate_params`, `_biquad_process`) fully functional.

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

HR2 and HR3 are driven via `test_feed_hr2()` / `test_feed_hr3()`, which call the synchronous `_update_hr2()` / `_update_hr3()` paths (same computation as the async tasks, without FreeRTOS signalling). Results are available via `test_hr2()`, `test_hr2_sqi()`, etc. after each call.

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
| IR LED current (LED1) | `setLED1Current()` | 11.7 mA | 0 – LED range max | PPG amplitude on IR channel; drives DC level and AC/DC ratio |
| RED LED current (LED2) | `setLED2Current()` | 11.7 mA | 0 – LED range max | PPG amplitude on RED channel; directly affects SpO2 R-ratio |
| LED full-scale range | `setLEDRange()` | 150 mA | 75 or 150 mA (TX_REF=0.75 V) | Scales the 8-bit DAC resolution; affects achievable current values |

> LED currents set the signal amplitude at the photodiode. Too low → low PI, SpO2 SQI=0. Too high → TIA saturation (clipping). The optimal point maximises PI while keeping the ADC output below full-scale.

#### Receiver — TIA (I-V amplifier)

| Parameter | API | Default | Options | Effect on output |
|---|---|---|---|---|
| Feedback resistance R_F | `setTIAGain()` | RF_500K (500 kΩ) | 10k, 25k, 50k, 100k, 250k, 500k, 1MΩ | Transimpedance gain: larger R_F = higher voltage output, higher noise floor. Full-scale input current = 1 V / R_F |
| Feedback capacitance C_F | `setTIACF()` (or auto) | auto (see §7.2) | 5, 10, 20, 30, 55, 155 pF | TIA RC pole: must settle before ADC samples. Auto-selected by library from R_F and PRF |

> R_F and C_F together determine the front-end gain and noise. The library enforces `R_F × C_F ≤ Rx_sample_time / 10` via auto-CF (§7.2).

#### Receiver — Stage 2 (ambient cancellation amplifier)

| Parameter | API | Default | Options | Effect on output |
|---|---|---|---|---|
| Stage 2 gain LED1 | `setStage2GainLED1()` | GAIN_0DB | 0, 3.5, 6, 9.5, 12 dB | STG2GAIN1[2:0] D[10:8] of TIAGAIN. Sets gain only; does NOT touch STAGE2EN1. Active when ENSEPGAIN=1. |
| Stage 2 gain LED2 | `setStage2GainLED2()` | GAIN_0DB | 0, 3.5, 6, 9.5, 12 dB | STG2GAIN2[2:0] D[10:8] of TIA_AMB_GAIN. Sets gain only; does NOT touch STAGE2EN2. Always active. |
| Stage 2 enable LED1 | `setStage2En1(bool)` | true | true / false | STAGE2EN1 bit D14 of TIAGAIN. Decoupled from gain: can be ON at 0 dB or OFF despite non-zero gain. Active when ENSEPGAIN=1. |
| Stage 2 enable LED2 | `setStage2En2(bool)` | true | true / false | STAGE2EN2 bit D14 of TIA_AMB_GAIN. Decoupled from gain. Also forced ON by `_apply_analog_regs()` when `AMBDAC > 0` (cancellation current needs Stage 2 active). Always active. |
| Ambient cancellation DAC (I_CANCEL) | `setAmbDac()` | 0 µA | 0 – 10 µA, 1 µA steps | AMBDAC[3:0] D[19:16] of TIA_AMB_GAIN. Subtracts ambient leakage before ADC via Stage 2. When `uA > 0`, `_apply_analog_regs()` forces STAGE2EN2=1 regardless of `setStage2En2()`. Applies to both LED1 and LED2 ambient phases (single shared DAC). |

> Stage 2 gain and enable are now independent bits. GAIN_0DB with STAGE2EN=1 → Stage 2 active at unity gain (useful for AMBDAC at minimum overhead). STAGE2EN=0 regardless of gain → Stage 2 bypassed.

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
| DC removal time constant | `dc_iir_tau_s` (SpO2), `hr1_dc_tau_s` (HR1) | 1.6 s | SpO2, HR1 | IIR low-pass estimates and removes the DC component; time constant determines tracking speed vs. baseline stability |
| Input channel selection | `setPPGChannel()` default | LED1_ALED1 | All algorithms | Selects which of the 6 raw signals enters the processing chain |
| Decimation factor | `hr2_decim_factor`, `hr3_decim_factor` | ×10 | HR2, HR3 | Reduces 500 Hz → 50 Hz before autocorrelation / FFT; sets effective frequency resolution |

---

### 10.3 SpO2 parameters

See §5.1 for algorithm description.

| Parameter | Constant | Value | Effect |
|---|---|---|---|
| Calibration coefficient a | `spo2_a` | 114.9208 | SpO2 = a − b·R intercept; calibrated with MS100 simulator / Nellcor R-curve |
| Calibration coefficient b | `spo2_b` | 30.5547 | SpO2 = a − b·R slope |
| DC IIR time constant | `dc_iir_tau_s` | 1.6 s | Speed of DC baseline tracking |
| AC EMA time constant | `ac_ema_tau_s` | 1.0 s | Smoothing of AC amplitude estimate |
| Minimum DC level (no-finger threshold) | `spo2_min_dc` | 1000 counts | Below this DC, SpO2 and SQI are forced to 0 |
| Warm-up period | `warmup_s` | 5.0 s | SpO2 suppressed during initial filter transient |
| SQI PI lower bound | `spo2_pi_sqi_lo` | 0.5 % | PI below this → SQI = 0 |
| SQI PI upper bound | `spo2_pi_sqi_hi` | 2.0 % | PI above this → SQI = 1 |

---

### 10.4 HR1 parameters

See §5.2 for algorithm description.

| Parameter | Constant | Value | Effect |
|---|---|---|---|
| DC removal time constant | `hr1_dc_tau_s` | 1.6 s | Baseline tracking speed |
| Low-pass MA cutoff | `hr1_ma_cutoff_hz` | 5 Hz | Noise smoothing before peak detection; affects peak sharpness |
| Max MA length | `hr1_ma_max_len` | 64 samples | Prevents excessively long moving average at low heart rates |
| Peak threshold fraction | 0.6 | 0.6 × running_max | Trade-off: higher → misses low-amplitude peaks; lower → false detections on noise |
| Refractory period | `hr1_refractory_s` | 0.2 s | Minimum interval between detected peaks; sets practical upper HR limit (~300 BPM) |
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
| Reported valid HR upper bound | `hr_max_bpm` | 300 BPM | HR1, HR2, HR3 — neonatal tachycardia |
| Internal search lower bound | `hr_search_min_bpm` | 37 BPM | Guard band: hr_min − 3 |
| Internal search upper bound | `hr_search_max_bpm` | 303 BPM | Guard band: hr_max + 3 |
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
| 4 | **t5** ALED2STC — *Sample ambient LED2 start count* | 200 | 200 counts (50 µs) after LED2 OFF for TIA settling; use 400+ at ≥ 50 mA | > t4 (circ.); < t6; both LEDs off in [t5,t6]; RF×CF < (t6−t5)×0.25µs/10 (§8.3.1.1); ≥ t4 + 5×RF×CF/0.25µs |
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
| 18 | **t11** ALED1STC — *Sample ambient LED1 start count* | 4 200 | 200 counts (50 µs) after LED1 OFF for TIA settling; same criterion as t5 | > t10; < t12; both LEDs off in [t11,t12]; RF×CF < (t12−t11)×0.25µs/10; ≥ t10 + 5×RF×CF/0.25µs |
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
- Ambient settling (t5, t11): 200 counts (50 µs) is adequate up to ~25 mA LED current. At ≥ 50 mA increase to 400+ counts.
- TIA margin formula: `max(50, floor(LED_window × 0.10))` counts. Also used by auto-CF (§7.2).
- Historical comparison: TI EVM uses 20% duty cycle + 80-count margins (ALED1CONVEND overflows at 500 Hz); Protocentral uses 0 for reset and sample margins (no settling guard).

---

### 11.2 Analog configuration

| Register / Field | AFE44x0.h (EVM) | Datasheet | Protocentral (src/) | **incunest_afe4490** | **Rationale** |
|---|---|---|---|---|---|
| **TIAGAIN** | 0x00C006 | — | 0x000000 | **0x000000** | Minimal config: RF=500kΩ, CF=5pF, Stage2 off, ENSEPGAIN=0 |
| → ENSEPGAIN | 1 | — | 0 | **0** | With 0, both channels use TIAGAIN for the TIA. Simplifies configuration. Enable if independent gain per channel is needed |
| → Stage2 LED1 | EN, 0 dB | — | disabled | **disabled** | RF=500kΩ provides sufficient gain for typical PPG. Stage2 at 0dB adds no useful gain. Configurable via setter |
| → CF LED1 | 5 pF | — | 5 pF | **auto (§7.2)** | Auto-selected to maximise noise filtering while guaranteeing 5τ settling. Default at 500 Hz / RF_500K: 20 pF |
| → RF LED1 | 1 MΩ | — | 500 kΩ | **500 kΩ** | Validated by Protocentral on real hardware. EVM's 1MΩ risks saturation. Configurable via setTIAGain() |
| **TIA_AMB_GAIN** | 0x004006 | — | 0x000001 | **0x000000** | With ENSEPGAIN=0, the RF of this register does not affect the main TIA. Relevant for FLTRCNRSEL, AMBDAC and Stage2 |
| → RF LED2/amb | 1 MΩ | — | 250 kΩ | **500 kΩ** | Ignored for TIA when ENSEPGAIN=0. Set equal to TIAGAIN to avoid surprises if ENSEPGAIN=1 is enabled later |
| → CF LED2/amb | 5 pF | — | 5 pF | **auto (§7.2)** | Same as CF LED1 (ENSEPGAIN=0 shares config) |
| → Stage2 LED2 | EN, 0 dB | — | disabled | **disabled** | Same criterion as Stage2 LED1 |
| → FLTRCNRSEL | 500 Hz | — | 500 Hz | **500 Hz** | Matches PRF. 1000 Hz option only makes sense if PRF is raised |
| → AMBDAC | 0 µA | — | 0 µA | **0 µA** | Ambient cancellation handled in hardware (LED1−ALED1VAL, LED2−ALED2VAL). AMBDAC only needed if ambient light saturates TIA before subtraction. Exposable via setter |
| **LEDCNTRL** | 0x011414 | — | 0x001414 | **0x001414** | RANGE_0 + code 20 + TX_REF=0.75V → 11.7mA per LED |
| → LED_RANGE | RANGE_1 | — | RANGE_0 | **RANGE_0** | Consistent with TX_REF=0.75V. FSR=150mA → resolution ~0.6mA/step |
| → LED1 code | 20 (0x14) | — | 20 (0x14) | **20 (0x14)** | 11.7mA: standard starting point for finger SpO2 validated by Protocentral. Configurable via setLEDCurrent() |
| → LED2 code | 20 (0x14) | — | 20 (0x14) | **20 (0x14)** | Same as LED1. In practice red and IR may need independent adjustment |
| → **Resulting current** | **≈3.9 mA** | — | **≈11.7 mA** | **≈11.7 mA** | EVM uses TX_REF=0.5V + RANGE_1 → 3.9mA; insufficient as a general default |
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
| **RF_LED1** | `TIAGAIN` D[2:0] | `afe_tia_gain_led1` | `setTIAGain()` | `AFE4490TIAGain` (`RF_10K`…`RF_1M`) |
| **RF_LED2** | `TIA_AMB_GAIN` D[2:0] | `afe_tia_gain_led2` | `setTIAGain()` (shared when `ENSEPGAIN=0`) | `AFE4490TIAGain` |
| **STG2GAIN1** | `TIAGAIN` D[10:8] | `afe_stage2_gain_led1` | `setStage2GainLED1()` | `AFE4490Stage2Gain` (`GAIN_0DB`…`GAIN_12DB`) |
| **STG2GAIN2** | `TIA_AMB_GAIN` D[10:8] | `afe_stage2_gain_led2` | `setStage2GainLED2()` | `AFE4490Stage2Gain` |
| **AMBDAC** | `TIA_AMB_GAIN` D[19:16] | `afe_ambdac_uA` | `setAmbDac(uint8_t uA)` | `uint8_t` 0–10 µA |
| **LED1** (IR current) | `LEDCNTRL` D[15:8] | `afe_led1_current_mA` | `setLED1Current(float mA)` | `float` 0–range mA (DAC-quantised) |
| **LED2** (RED current) | `LEDCNTRL` D[7:0] | `afe_led2_current_mA` | `setLED2Current(float mA)` | `float` 0–range mA (DAC-quantised) |

> **RF_LED2 note:** when `ENSEPGAIN=0` (default), `setTIAGain()` writes the same RF value to both `TIAGAIN` and `TIA_AMB_GAIN`. `afe_tia_gain_led2` acts independently only when `ENSEPGAIN=1`.

### 12.2 ADC data registers

| Datasheet name | Register address | `AFE4490Sample` field | Semantic channel |
|---|---|---|---|
| **LED2VAL** | `0x2A` | `led2` | RED raw |
| **ALED2VAL** | `0x2B` | `aled2` | Ambient after LED2 (RED amb) |
| **LED1VAL** | `0x2C` | `led1` | IR raw |
| **ALED1VAL** | `0x2D` | `aled1` | Ambient after LED1 (IR amb) |
| **LED2−ALED2VAL** | `0x2E` | `led2_aled2` | RED ambient-corrected |
| **LED1−ALED1VAL** | `0x2F` | `led1_aled1` | IR ambient-corrected *(primary PPG/SpO2 signal)* |

All 6 values are sign-extended from 22-bit two's complement by `_sign_extend_22()`. Valid range: [−2 097 152, +2 097 151]. Saturation (TIA railing) produces a near-maximum constant value (~2 096 921) across all channels.

### 12.3 Hardware probe states

Three distinct hardware states are distinguishable from the raw ADC channel pattern. Used by RSQM (probe presence detection) and HGAC (inhibit control when probe not on patient).

| State | `led1` | `led2` | `aled1` | `aled2` | Physical cause |
|---|---|---|---|---|---|
| **PROBE_DISCONNECTED** | ≈ −5000 | ≈ −5000 | ≈ −5000 | ≈ −5000 | Photodiode open-circuit; TIA input floating → input offset current × RF produces small negative output |
| **PROBE_NOT_APPLIED** | saturated | saturated | low–mid | low–mid | No tissue attenuation; LEDs illuminate photodiode directly. ALED channels depend on ambient light level |
| **PROBE_APPLIED** | PPG signal | PPG signal | low | low | Normal operation; tissue attenuates LED light; ambient channels remain low |

**Detection method:** see §5.6. DISCONNECTED is instantaneous; NOT_APPLIED/APPLIED use optical transmittance (single threshold + 50-sample debounce); APPLIED uses 750-sample EMA warmup for RSQM_SIGNAL_WEAK.

> These states are computed by RSQM (_update_rsqm) and gate HGAC operation: HGAC must not act in `PROBE_DISCONNECTED` or `PROBE_NOT_APPLIED` states.

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
  Normative standard defining valid measurement ranges, tolerances, and test conditions. Source of the [70–100 %] SpO2 and [25–300 BPM] HR bounds used in this library.

- **Severinghaus, J.W. & Astrup, P.B. (1986).** "History of blood gas analysis." *Journal of Clinical Monitoring*, 2(4), 270–288.
  Historical context for non-invasive oxygen saturation measurement.

---

## 14. Version history

| Version | Description                                                                 |
|---------|-----------------------------------------------------------------------------|
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
|         | bitmask constants RSQM_AMB_SAT, RSQM_SIGNAL_WEAK, RSQM_HW_SETTLING.        |
|         | `AFE4490Data` gains `rsqi`, `diag_code`, `probe_state`. EMA estimators      |
|         | (α_mean=0.02, α_var=0.004, ready after 750 samples). Probe detection via   |
|         | `optical_transmittance` (single threshold 0.30, no hysteresis) + 100-sample  |
|         | debounce in `_set_probe_state()`. RSQM runs first in `_process_sample()`    |
|         | at 500 Hz. Spec §5.6 added.                                                 |
| v0.26   | Decouple STAGE2EN1/STAGE2EN2 from Stage 2 gain. `stg2_gain_code[]` now         |
|         | contains only D[10:8] gain bits (no enable). `setStage2En1(bool)` and          |
|         | `setStage2En2(bool)` expose STAGE2EN1 (D14 of TIAGAIN) and STAGE2EN2           |
|         | (D14 of TIA_AMB_GAIN) as independent explicit parameters. `AFE4490Config`      |
|         | gains `afe_stage2_en1` and `afe_stage2_en2`. `$CFG` frame gains                |
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
|         | via 13 new setters (setSpO2WarmupS, setSpO2DcIirTauS, setSpO2AcEmaTauS,   |
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
