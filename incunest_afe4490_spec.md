# incunest_afe4490 — Specification v0.18

Medical Open World proprietary library for the AFE4490 chip (PPG/SpO2 pulse oximeter).
Designed for ESP32-S3 with Arduino + FreeRTOS. Phase 2 of the AFE4490 test project.
Coexists with `protocentral-afe4490-arduino` for behavioral comparison.

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
    │       └─→ bandpass filter (setFilter) → ppg + HR1 (peak detection)
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
| TIA CF         | 5 pF                          |
| Stage 2 gain   | disabled                      |
| ENSEPGAIN      | 0 (single gain)               |
| NUMAV          | 7 (8 averages, max 9 at 500Hz)|
| PPG channel    | LED1_ALED1                    |
| Filter         | BUTTERWORTH, 0.5–20 Hz        |

### 2.3 Chip configuration
```cpp
void setSampleRate(uint16_t hz);        // valid range: 63–5000 Hz. Updates PRF and recalculates NUMAV_max.
void setNumAverages(uint8_t num);       // num = number of samples to average: 1 (no averaging) .. NUMAV_max+1
                                        // NUMAV_max = floor(5000 / PRF) - 1  (e.g.: PRF=500Hz → max=9 → up to 10 samples)
                                        // Absolute hardware limit: 16 samples (NUMAV=15)
                                        // If num > NUMAV_max for current PRF, clamped and logE emitted
void setLED1Current(float mA);          // depends on setLEDRange()
void setLED2Current(float mA);          // depends on setLEDRange()
void setLEDRange(uint8_t mA);           // 75 or 150 mA (with TX_REF=0.75V default)
void setTIAGain(AFE4490TIAGain gain);
void setTIACF(AFE4490TIACF cf);
void setStage2Gain(AFE4490Stage2Gain gain);
```

### 2.4 Signal and filter configuration
```cpp
void setPPGChannel(AFE4490Channel channel);
void setFilter(AFE4490Filter type, float f_low_hz = 0.5f, float f_high_hz = 20.0f);
// HR2 bandpass filter cutoffs (default 0.5–5 Hz); callable before or after begin()
void setHR2Filter(float f_low_hz = 0.5f, float f_high_hz = 5.0f);
// HR3 low-pass filter cutoff (default 10 Hz); callable before or after begin()
void setHR3Filter(float f_high_hz = 10.0f);
```

### 2.5 Data retrieval
```cpp
bool getData(AFE4490Data& data);
// Returns true and fills data if an item is available in the queue.
// Returns false immediately if the queue is empty (non-blocking).
```

### 2.6 Shutdown
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
    GAIN_0DB,    // default (stage 2 disabled)
    GAIN_3_5DB,
    GAIN_6DB,
    GAIN_9_5DB,
    GAIN_12DB
};
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
| `hr_refractory_s` | 0.2 s | Refractory period between peaks (~300 BPM max) |
| `hr1_peak_marker_samples` | 10 samples | Duration of peak marker (hr1_ppg=0) |
| `hr_min_bpm` | 25 BPM | Reported valid HR range minimum (ISO 80601-2-61; neonatal use) |
| `hr_max_bpm` | 300 BPM | Reported valid HR range maximum (neonatal tachycardia) |
| `hr_search_min_bpm` | 22 BPM | Internal search lower bound (guard band: hr_min − 3) |
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

`hr3_sqi` is a continuous [0–1] quality metric based on HPS peak prominence: the fraction of total HPS energy at the interpolated peak, relative to the sum of all HPS values across the search range:

```
HPS_interp = parabolic interpolation of HPS at (peak_bin-1, peak_bin, peak_bin+1)
fraction   = HPS_interp / Σ HPS[k]   (k across search range)
baseline   = 1 / N_bins               (flat-HPS reference)
SQI        = clamp((fraction − baseline) / (1 − baseline), 0, 1)
```

`baseline` is the fraction a single bin would hold if HPS were uniformly distributed across the `N_bins` bins of the search range (~48 bins). Using the HPS domain instead of the linear spectrum avoids harmonic inflation of the denominator: since HPS = P[k]·P[2k]·P[3k], only the true fundamental accumulates power from all three harmonics simultaneously, so the peak bin naturally dominates the HPS sum when the signal is periodic.

**Why parabolic interpolation on HPS (not on P[k]):** when the true fundamental falls between FFT bins (e.g. 85 BPM = bin 14.5), the Hann window splits energy between adjacent bins. Because HPS is a *product* of three power spectra, the split is cubic — a 50 % energy split at each harmonic yields only ~12 % of the ideal single-bin HPS, collapsing SQI to ~0.5 even for a clean signal. Applying parabolic interpolation directly to the HPS values recovers the true peak height at the fractional bin position, consistent with how `delta` is already used to interpolate the frequency estimate.

A clean PPG signal → SQI approaches 1. A diffuse or noisy spectrum → SQI ≈ 0. If the buffer is not yet full, the HPS peak is outside [25, 300] BPM, or the HPS sum is zero, SQI = 0.

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

Source: AFE4490 datasheet section 8.4.1, Equation 5.

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

**`setNumAverages(uint8_t num)` behaviour:**
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

## 10. Version history

| Version | Description                                                                 |
|---------|-----------------------------------------------------------------------------|
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
|         | `hr_refractory_s` 0.200→0.185 s (covers 303 BPM guard: 198 ms period).   |
|         | `hr2_min_lag_s` 0.22→0.185 s. `hr2_acorr_max_lag` 111→137 samples.      |
|         | Python `HR_MIN_HZ`/`HR_MAX_HZ`/`HR_SEARCH_*` updated accordingly.        |
| v0.10   | Guard band: internal HR search range extended to [27, 253] BPM (±3 BPM  |
|         | beyond reported [30, 250] BPM). HR2: `hr2_acorr_max_lag` 100→111        |
|         | (27 BPM @ 50 Hz). HR3 Python: `HR_SEARCH_MIN_HZ`/`HR_SEARCH_MAX_HZ`     |
|         | added. `hr_search_min_bpm`/`hr_search_max_bpm` constants documented.     |
| v0.9    | HR measurement range extended: 40–240 BPM → **30–250 BPM**.             |
|         | `hr_min_bpm` 40→30, `hr_max_bpm` 240→250, `hr_refractory_s` 0.3→0.2 s  |
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
|         | Called from `_recalc_rate_params()` and `setFilter()`. The "not yet        |
|         | implemented" warning removed. `setFilter()` is now fully functional for    |
|         | any fs, f_low, f_high.                                                     |
| v0.5    | Internal constants reorganised at the top of the anonymous namespace in    |
|         | `.cpp`, in snake_case. Algorithm time parameters expressed in physical      |
|         | units (seconds): `spo2_warmup_s`, `dc_iir_tau_s`, `ac_ema_tau_s`,         |
|         | `hr_refractory_s`. New members `_spo2_warmup_samples`,                     |
|         | `_hr_refractory_samples`, `_dc_iir_alpha`, `_ac_ema_beta` derived from     |
|         | `_sample_rate_hz` via `_recalc_rate_params()`, called in constructor and   |
|         | in `setSampleRate()`. SpO2 and HR algorithms now correct at any PRF.       |
| v0.4    | `AFE4490Data` extended with the 6 raw AFE4490 signals (led1, led2,         |
|         | aled1, aled2, led1_aled1, led2_aled2). Struct is now equivalent in         |
|         | information to the protocentral output.                                    |
| v0.3    | Configurable NUMAV: added `setNumAverages(uint8_t num)` to the API.        |
|         | Section 11.1 with NUMAV_max formula, table by PRF, clamping behaviour and  |
|         | SNR effect. `setSampleRate()` recalculates NUMAV_max.                      |
|         | **First implementation:** `include/incunest_afe4490.h` and                      |
|         | `src/incunest_afe4490.cpp` generated. Pending hardware validation.              |
| v0.2    | Added section 11 with comparative register tables (timing, analog, control,|
|         | init sequence). Sources: AFE44x0.h EVM TI v1.4, Datasheet AFE4490 Table 2,|
|         | Protocentral src/. incunest_afe4490 values justified register by register.      |
|         | Updated defaults: NUMAV=7, ENSEPGAIN=0, Stage2 disabled.                   |
| v0.1    | Initial complete specification                                              |

---

## 11. AFE4490 chip register configuration

This section documents the exact value of each AFE4490 register written by the library during initialisation, compared against the three reference sources analysed.

**Reference sources:**
- **AFE44x0.h (EVM TI v1.4):** TI official firmware for the AFE4490EVM evaluation kit. Written for MSP430. The C code is not reusable, but the register values and timing formulas are TI's authoritative reference.
- **Datasheet AFE4490 Table 2:** example values from the datasheet for PRF=500Hz, duty cycle=25%.
- **Protocentral (src/):** `src/protocentral_afe44xx.cpp` from the project — modified version of the Protocentral library adapted to the project.

---

### 11.1 Timing registers (PRF = 500 Hz, AFECLK = 4 MHz, 1 count = 0.25 µs)

| Register | AFE44x0.h (PRF=500) | Datasheet Table 2 | Protocentral (src/) | **incunest_afe4490** | **Rationale** |
|---|---|---|---|---|---|
| PRPCOUNT | 7999 | 7999 (t29) | 7999 | **7999** | PRF=500Hz: (4,000,000/500)−1=7999 |
| LED2LEDSTC | 6000 | 6000 (t3) | 6000 | **6000** | LED2 starts exactly at the beginning of its phase (75% of period) |
| LED2LEDENDC | 7599 | 7999 (t4) | 7999 | **7999** | 25% duty cycle: 2000 counts = 500µs → maximises photons |
| LED2STC | 6080 | 6050 (t1) | 6000 | **6050** | 50 counts (12.5µs) margin for TIA settling. TI EVM uses 80 (excessive at 500Hz); Protocentral uses 0 (risky) |
| LED2ENDC | 7598 | 7998 (t2) | 7998 | **7998** | Closes 2 counts before end of phase |
| ALED2STC | 80 | 200 (t5) | 0 | **200** | 200 counts (50 µs) after LED2 OFF — timing margin; ambient ripple confirmed as optical/electrical crosstalk (not timing) |
| ALED2ENDC | 1598 | 1998 (t6) | 1998 | **1998** | Uses full window except last 2 counts |
| LED1LEDSTC | 2000 | 2000 (t9) | 2000 | **2000** | LED1 starts at beginning of its phase (25% of period) |
| LED1LEDENDC | 3599 | 3999 (t10) | 3999 | **3999** | 25% duty cycle, same as LED2 |
| LED1STC | 2080 | 2050 (t7) | 2000 | **2050** | Same criterion as LED2STC: 50-count margin |
| LED1ENDC | 3598 | 3998 (t8) | 3998 | **3998** | Same as LED2ENDC |
| ALED1STC | 4080 | 4200 (t11) | 4000 | **4200** | 200 counts (50 µs) after LED1 OFF — same criterion as ALED2STC |
| ALED1ENDC | 5598 | 5998 (t12) | 5998 | **5998** | Same as ALED2ENDC |
| LED2CONVST | 7 | 4 (t13) | 2 | **4** | 1 count after ADC reset end (end=3). Required: "Must start one AFE clock cycle after the ADC reset pulse ends" |
| LED2CONVEND | 2000 | 1999 (t14) | 1999 | **1999** | 1 count before next reset (ADCRSTSTCT1=2000) |
| ALED2CONVST | 2007 | 2004 (t15) | 2002 | **2004** | 1 count after ADCRSTENDCT1=2003 |
| ALED2CONVEND | 4000 | 3999 (t16) | 3999 | **3999** | 1 count before ADCRSTSTCT2=4000 |
| LED1CONVST | 4007 | 4004 (t17) | 4002 | **4004** | 1 count after ADCRSTENDCT2=4003 |
| LED1CONVEND | 6000 | 5999 (t18) | 5999 | **5999** | 1 count before ADCRSTSTCT3=6000 |
| ALED1CONVST | 6007 | 6004 (t19) | 6002 | **6004** | 1 count after ADCRSTENDCT3=6003 |
| ALED1CONVEND | ⚠️ 8000 | 7999 (t20) | 7999 | **7999** | =PRPCOUNT. TI EVM formula overflows at 500Hz (designed for 100Hz) |
| ADCRSTSTCT0 | 0 | 0 (t21) | 0 | **0** | Start of period |
| ADCRSTENDCT0 | 5 | 3 (t22) | 0 | **3** | 3 counts = 0.75µs → −60 dB crosstalk. Protocentral uses 0 (unquantified risk). 6 counts would eliminate crosstalk entirely but unnecessary with hardware ambient cancellation |
| ADCRSTSTCT1 | 2000 | 2000 (t23) | 2000 | **2000** | Start of phase 2 |
| ADCRSTENDCT1 | 2005 | 2003 (t24) | 2000 | **2003** | ADCRSTSTCT1 + 3 counts |
| ADCRSTSTCT2 | 4000 | 4000 (t25) | 4000 | **4000** | Start of phase 3 |
| ADCRSTENDCT2 | 4005 | 4003 (t26) | 4000 | **4003** | ADCRSTSTCT2 + 3 counts |
| ADCRSTSTCT3 | 6000 | 6000 (t27) | 6000 | **6000** | Start of phase 4 |
| ADCRSTENDCT3 | 6005 | 6003 (t28) | 6000 | **6003** | ADCRSTSTCT3 + 3 counts |

**Notes on the Table 2 formula:**
- `PRPCOUNT = (AFECLK / PRF) − 1`
- 25% duty cycle: LED window = `(PRPCOUNT+1) / 4` counts
- TIA margin: 50 fixed counts regardless of PRF
- ADC reset: 3 fixed counts. For 6 counts (zero crosstalk), set CONVST to reset_end+1
- TI EVM formula uses 20% duty cycle and 80-count margin → **not valid at 500Hz** (ALED1CONVEND overflows)

---

### 11.2 Analog configuration

| Register / Field | AFE44x0.h (EVM) | Datasheet | Protocentral (src/) | **incunest_afe4490** | **Rationale** |
|---|---|---|---|---|---|
| **TIAGAIN** | 0x00C006 | — | 0x000000 | **0x000000** | Minimal config: RF=500kΩ, CF=5pF, Stage2 off, ENSEPGAIN=0 |
| → ENSEPGAIN | 1 | — | 0 | **0** | With 0, both channels use TIAGAIN for the TIA. Simplifies configuration. Enable if independent gain per channel is needed |
| → Stage2 LED1 | EN, 0 dB | — | disabled | **disabled** | RF=500kΩ provides sufficient gain for typical PPG. Stage2 at 0dB adds no useful gain. Configurable via setter |
| → CF LED1 | 5 pF | — | 5 pF | **5 pF** | Datasheet minimum → maximum TIA bandwidth. Adjust if instability occurs |
| → RF LED1 | 1 MΩ | — | 500 kΩ | **500 kΩ** | Validated by Protocentral on real hardware. EVM's 1MΩ risks saturation. Configurable via setTIAGain() |
| **TIA_AMB_GAIN** | 0x004006 | — | 0x000001 | **0x000000** | With ENSEPGAIN=0, the RF of this register does not affect the main TIA. Relevant for FLTRCNRSEL, AMBDAC and Stage2 |
| → RF LED2/amb | 1 MΩ | — | 250 kΩ | **500 kΩ** | Ignored for TIA when ENSEPGAIN=0. Set equal to TIAGAIN to avoid surprises if ENSEPGAIN=1 is enabled later |
| → CF LED2/amb | 5 pF | — | 5 pF | **5 pF** | Same criterion as CF LED1 |
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
| → NUMAV | 7 (8 avg.) | — | 7 (8 avg.) | **7** | Default = 8 averaged samples → SNR ×√8 ≈ 2.8×. Configurable via `setNumAverages()`. See NUMAV_max formula in section 7.1 |
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

## 12. Bibliographic references

Key references for each algorithm implemented in this library.

### 12.1 HR1 — Adaptive threshold peak detection

- **Pan, J. & Tompkins, W.J. (1985).** "A Real-Time QRS Detection Algorithm." *IEEE Transactions on Biomedical Engineering*, 32(3), 230–236.
  The classical reference for cardiac peak detection with adaptive threshold and refractory period. HR1 is a simplified adaptation for PPG.

- **Elgendi, M. et al. (2013).** "Systolic Peak Detection in Acceleration Photoplethysmograms Measured from Emergency Responders in Tropical Conditions." *PLoS ONE*, 8(10), e76585.
  More specific to PPG peak detection in challenging conditions.

### 12.2 HR2 — Normalised autocorrelation

- **de Cheveigné, A. & Kawahara, H. (2002).** "YIN, a fundamental frequency estimator for speech and music." *Journal of the Acoustical Society of America*, 111(4), 1917–1930.
  The primary reference for normalised autocorrelation-based fundamental frequency estimation. HR2 is directly derived from this approach.

- **Rabiner, L.R. & Schafer, R.W. (1978).** *Digital Processing of Speech Signals.* Prentice-Hall.
  Foundational coverage of autocorrelation methods for pitch estimation.

### 12.3 HR3 — FFT + Harmonic Product Spectrum

- **Schroeder, M.R. (1968).** "Period histogram and product spectrum: New methods for fundamental-frequency measurement." *Journal of the Acoustical Society of America*, 43(4), 829–834.
  Original paper introducing the Harmonic Product Spectrum concept.

- **Noll, A.M. (1969).** "Cepstrum pitch determination." *Journal of the Acoustical Society of America*, 41(2), 293–309.
  Historical context and comparison with HPS-based approaches.

- **Paradkar, N. & Chowdhary, S.R. (2017).** "Cardiac arrhythmia detection using photoplethysmography." *IEEE EMBC 2017*.
  Application of FFT-based HR estimation on PPG signals in a wearable context similar to this project.

### 12.4 SpO2 — AC/DC ratio (R-curve method)

- **Webster, J.G. (Ed.) (1997).** *Design of Pulse Oximeters.* IOP Publishing.
  The definitive reference. Covers Beer-Lambert law, the R ratio, empirical calibration, and sources of measurement error.

- **Jubran, A. (1999).** "Pulse oximetry." *Critical Care*, 3(2), R11–R17.
  Clear clinical review of the operating principle and limitations.

- **ISO 80601-2-61.** *Medical electrical equipment — Part 2-61: Particular requirements for basic safety and essential performance of pulse oximeter equipment.*
  Normative standard defining valid measurement ranges, tolerances, and test conditions. Source of the [70–100 %] SpO2 and [25–300 BPM] HR bounds used in this library.

- **Severinghaus, J.W. & Astrup, P.B. (1986).** "History of blood gas analysis." *Journal of Clinical Monitoring*, 2(4), 270–288.
  Historical context for non-invasive oxygen saturation measurement.
