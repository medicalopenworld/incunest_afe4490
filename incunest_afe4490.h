#pragma once

// incunest_afe4490 — Medical Open World AFE4490 driver + PPG algorithms (HR, SpO2)
// Library version: v0.39-experiment — ESP32-S3, Arduino + FreeRTOS
// Spec: incunest_afe4490_spec.md
// Chip datasheet: https://www.ti.com/lit/ds/symlink/afe4490.pdf
// Author: Medical Open World — http://medicalopenworld.org — <contact@medicalopenworld.org>

#define INCUNEST_AFE4490_VERSION "0.39-experiment"

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
  // Explicit — this header uses strcmp() (afeStrToRF/CF/RG) and expf() (EmaChannel::init())
  // directly. Real Arduino.h happens to pull these in transitively, but that must not be
  // relied upon: the plain UNIT_TEST/native path (pio test -e native, distinct from
  // INCUNEST_OFFLINE) uses a minimal Arduino.h stub that does not.
  #include <cstring>
  #include <cmath>
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

#ifndef INCUNEST_AFE4490_DIAG_TASK_STACK
#define INCUNEST_AFE4490_DIAG_TASK_STACK  2048
#endif

#ifndef INCUNEST_AFE4490_DIAG_TASK_PRIORITY
#define INCUNEST_AFE4490_DIAG_TASK_PRIORITY  (INCUNEST_AFE4490_TASK_PRIORITY - 2)
#endif

#ifndef INCUNEST_TIMING_STATS
#define INCUNEST_TIMING_STATS 0
#endif

// ── Probe state ───────────────────────────────────────────────────────────────
enum class ProbeState {
    PROBE_DISCONNECTED,  // cable or probe connector not plugged in
    PROBE_NOT_APPLIED,   // probe connected but not on patient tissue
    PROBE_APPLIED        // probe on patient, normal measurement
};

// DiagCode bitmask flags — returned in AFE4490Data::diag_code (uint32_t, 32 bits)
// Multiple bits may be set simultaneously.
//
// Bits 0–12: AFE4490 hardware DIAG register (REG_DIAG 0x30) — set by runAfeDiagnostics()
constexpr uint32_t AFE_DIAG_PD_ALM          = 0x00000001UL;  // bit  0 — photodiode out of range
constexpr uint32_t AFE_DIAG_LED_ALM         = 0x00000002UL;  // bit  1 — LED out of range
constexpr uint32_t AFE_DIAG_OUT             = 0x00000004UL;  // bit  2 — general diagnostic fault
constexpr uint32_t AFE_DIAG_LED2_ALM        = 0x00000008UL;  // bit  3 — LED2 driver fault
constexpr uint32_t AFE_DIAG_LED3_ALM        = 0x00000010UL;  // bit  4 — LED3 driver fault (if used)
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
// Bits 13+: RSQM software checks — set continuously by _update_rsqm() at 500 Hz.
// ANTICIPATORY CODE (2026-07-11): these flags currently have NO programmatic consumer
// beyond the aggregate rsqi gate (diag_code == 0) and human inspection in pulsenest_lab.
// Their intended consumer is HGAC, which may end up doing its own internal checks and
// never read them — they may remain unused. HW_SETTLING is currently unreachable: nobody
// sets _rsqm_settling_countdown > 0 yet (intended producer: HGAC / config changes).
constexpr uint32_t RSQM_DIAG_AMB_SAT     = 0x00002000UL;  // bit 13 — ambient channel near positive rail
constexpr uint32_t RSQM_DIAG_SIGNAL_WEAK = 0x00004000UL;  // bit 14 — differential signal amplitude too low
constexpr uint32_t RSQM_DIAG_HW_SETTLING = 0x00008000UL;  // bit 15 — hardware settling after config change
//
// Bit 16: HGAC (Hardware Gain Control) — set by _update_hgac(), first real producer of
// RSQM_DIAG_HW_SETTLING above. HGAC_DIAG_GAIN_FLOOR: RF at the lowest LUT level (10k) and
// the channel is still saturating — ILED actuation not implemented yet (Phase 1: RF-only
// descent), so this is HGAC's "excessive ambient light" alarm for v1.
constexpr uint32_t HGAC_DIAG_GAIN_FLOOR  = 0x00010000UL;  // bit 16 — RF at floor, still saturating

// ── Public data struct ────────────────────────────────────────────────────────
struct AFE4490Data {
    // Field order mirrors the $M1/$P1 serial frame: raw signals first, then processed outputs
    // Raw ADC outputs (6 signals from AFE4490)
    int32_t led2;        // LED2VAL  — RED raw          (frame: LED2)
    int32_t led1;        // LED1VAL  — IR raw           (frame: LED1)
    int32_t aled2;       // ALED2VAL — ambient after LED2 (frame: ALED2)
    int32_t aled1;       // ALED1VAL — ambient after LED1 (frame: ALED1)
    int32_t led2_sub;  // LED2-ALED2 — RED ambient-corrected (frame: LED2_SUB)
    int32_t led1_sub;  // LED1-ALED1 — IR ambient-corrected  (frame: LED1_SUB)
    // Processed outputs
    int32_t ppg_disp;         // PPG display signal (BPF + negated; display only — not an input to algorithms)
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
    // RSQM outputs
    uint8_t    rsqi;         // Raw Signal Quality Index: 0=invalid, 1=valid
    uint32_t   diag_code;    // DiagCode bitmask: RSQM_DIAG_AMB_SAT | RSQM_DIAG_SIGNAL_WEAK | RSQM_DIAG_HW_SETTLING | HGAC_DIAG_GAIN_FLOOR | ...
    ProbeState probe_state;  // PROBE_DISCONNECTED, PROBE_NOT_APPLIED, or PROBE_APPLIED
};

// ── Enumerations ──────────────────────────────────────────────────────────────
enum class AFE4490Channel {
    LED1,       // IR raw
    LED2,       // RED raw
    ALED1,      // ambient after LED1
    ALED2,      // ambient after LED2
    LED1_SUB, // IR ambient-corrected (default)
    LED2_SUB  // RED ambient-corrected
};

enum class AFE4490RF {
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

enum class AFE4490RG {
    RG_100K,    // 0 dB   — ×1   — RG=100 kΩ  (Ri=100 kΩ fixed, datasheet Table 1)
    RG_150K,  // 3.5 dB — ×1.5 — RG=150 kΩ
    RG_200K,    // 6 dB   — ×2   — RG=200 kΩ
    RG_300K,  // 9.5 dB — ×3   — RG=300 kΩ
    RG_400K    // 12 dB  — ×4   — RG=400 kΩ
};

// ── Physical parameter tables (indexed by enum cast to int) ──────────────────
// Single source of truth for AFE4490 physical values.
// Do NOT duplicate these in application code — include this header and index by enum.
// Usage: float rf_ohm = kAFE_RF_OHM[(int)cfg.afe_tia_rf_led1];

// AFE4490RF → TIA feedback resistance [Ω]
constexpr float kAFE_RF_OHM[7] = {
    10e3f,   // RF_10K
    25e3f,   // RF_25K
    50e3f,   // RF_50K
   100e3f,   // RF_100K
   250e3f,   // RF_250K
   500e3f,   // RF_500K
     1e6f    // RF_1M
};

// AFE4490TIACF → TIA feedback capacitance [pF]
constexpr float kAFE_CF_PF[6] = {
     5.0f,   // CF_5P
    10.0f,   // CF_10P
    20.0f,   // CF_20P
    30.0f,   // CF_30P
    55.0f,   // CF_55P
   155.0f    // CF_155P
};

// AFE4490RG → Stage 2 feedback resistance RG [Ω]
constexpr float kAFE_RG_OHM[5] = {
   100e3f,   // RG_100K   (×1)
   150e3f,   // RG_150K (×1.5)
   200e3f,   // RG_200K   (×2)
   300e3f,   // RG_300K (×3)
   400e3f    // RG_400K  (×4)
};

// AFE4490RG → linear voltage gain [dimensionless] = RG / Ri
constexpr float kAFE_RG_GAIN[5] = {
    1.0f,    // RG_100K
    1.5f,    // RG_150K
    2.0f,    // RG_200K
    3.0f,    // RG_300K
    4.0f     // RG_400K
};

// Stage 2 input resistance Ri [Ω] — fixed by chip design (datasheet p.28)
constexpr float kAFE_RI_OHM = 100e3f;

// ADC full-scale constants — used for code-to-voltage reconstruction
// (datasheet p.11: ADC full-scale voltage ±1.2 V, 22-bit resolution;
//  p.43 §8.4.1: 22-bit twos complement output format)
constexpr float kAFE_ADC_FS_COUNTS = 2097151.0f;  // 2^21 − 1 (positive full-scale code, 22-bit signed ADC)
constexpr float kAFE_ADC_FSR       = 1.2f;         // V — ADC full-scale range (±1.2 V differential)

// Low-pass filter corner frequency selection (FLTRCNRSEL bit D15 of TIA_AMB_GAIN, datasheet p.75).
// 0 = 500 Hz (reset default); 1 = 1000 Hz.
// Written unconditionally to TIA_AMB_GAIN in _build_tia_amb_gain_led2().
constexpr uint8_t  kAFE_FLTRCNRSEL      = 0;
// Diagnostic holdoff: filter settling time after diagnostics mode (datasheet p.11).
// 500 Hz corner → 28 ms; 1000 Hz corner → 16 ms.
constexpr uint32_t kAFE_DIAG_HOLDOFF_MS = kAFE_FLTRCNRSEL ? 16U : 28U;

/// Physical quantities derived from ADC codes for one sample — purely the signal chain.
/// Computed in _process_sample() before RSQM and HGAC; not stored in AFE4490Data.
/// Datasheet Eq.2 (p.30):  V_DIFF = 2 × (I_PD × RF/Ri − I_CANCEL) × RG
/// Inversions:
///   V_ADC      = code / kAFE_ADC_FS_COUNTS × kAFE_ADC_FSR              [V]
///   V_TIA_DIFF = 2 × (V_ADC / (2 × RG_ohm) + I_CANCEL_A) × Ri          [V]  (= 2 × I_PD × RF)
///   I_PD       = V_TIA_DIFF / (2 × RF_ohm)                             [A]
/// V_TIA convention: DIFFERENTIAL (datasheet §9.2.2: target 0.6 V, V_OD(fs) = 1.0 V).
/// Per-branch value = v_tia_diff / 2 (TI linear FS 0.5 V; measured hard clip ~0.97 V/branch).
/// Naming rule 4b: bare "v_tia" is forbidden — always suffix _diff or _branch.
/// ALED channels use the same RF and RG as the corresponding LED channel.
/// When STAGE2EN=0: RG_ohm = kAFE_RI_OHM (unity gain).

/// Channel identifier for AFE4490AnalogState per-channel queries.
/// The value is the bit index used in the validity masks (bit = 1 << (uint8_t)ch).
enum class AFE4490Ch : uint8_t { LED1 = 0, ALED1 = 1, LED2 = 2, ALED2 = 3 };

/// Per-channel measurement state, derived from the validity masks (see chState()).
/// Precedence: CH_CLIPPED > CH_OFF_SPEC > CH_CLEAN.
enum class AFE4490ChState : uint8_t {
    CH_CLEAN    = 0,  // trustworthy measurement: inside every guaranteed-linear range
    CH_OFF_SPEC = 1,  // v_tia_diff in (kVTiaDiffFs, kVTiaDiffLin]: outside TI guarantee but
                      // empirically linear (sweep 2026-07-08) — value most likely correct
    CH_CLIPPED  = 2,  // ADC saturated or TIA beyond empirical linearity: the stored value is
                      // a BOUND, not a measurement (sat pos: real ≥ stored; sat neg: real ≤ stored)
};

/// Invalidity policy: VALUE-ALWAYS. Every field is always computed from the raw codes,
/// even when part of the chain is saturated — the validity masks (one bit per channel,
/// bit index = AFE4490Ch) tell the consumer whether each value is a measurement or a bound.
/// Consumers must query through the helpers (chState/chClean/otLedxValid/...), never the
/// masks directly, so the packing (per-stage masks) can change without touching consumers.
/// The thresholds below are PHYSICS (datasheet / silicon / per-board characterization),
/// not policy — HGAC actuation thresholds (hgac_v_tia_diff_thr/ideal) live in the controller.
struct AFE4490AnalogState {
    // ── Physical detection thresholds ────────────────────────────────────────
    static constexpr float   kVTiaDiffFs  = 1.0f;      // V — V_OD(fs): TI-guaranteed linearity (§9.2.2, Fig. 135)
    static constexpr float   kVTiaDiffLin = 1.8f;      // V — empirical linearity limit (tia_linearity_sweep.py 2026-07-08, IncuNest 16.A)
    // ADC saturation thresholds: rails measured empirically via an AMBDAC sweep test
    // on IncuNest 16.A hardware (2026-07-10, driving the ADC into both rails), with a
    // ~220-count guard band inside the rail. Rails are NOT the datasheet code limits
    // (±2^21): empirical positive rail = +2096921 (constant, all 4 channels);
    // empirical negative rail = -2096919 (constant on LED1/LED2; ALED1/ALED2 publish
    // averaged codes that oscillate down to ~-2096810 without pinning — ADC averaging
    // mixes clipped and non-clipped conversions).
    // Guard = 2x the worst observed ALED-to-rail distance
    // (109 counts), applied to BOTH polarities (the averaging mechanism is sign-
    // independent). False positives are impossible in practice: a legitimate code
    // within 220 counts of the rail (0.01% of range) is functionally saturated anyway.
    static constexpr int32_t kAdcSatPos   =  2096700;  // counts — positive rail +2096921 minus guard
    static constexpr int32_t kAdcSatNeg   = -2096700;  // counts — negative rail -2096919 plus guard

    // ── Values (always computed — see validity masks below) ─────────────────
    float v_adc_led1,      v_adc_led2,      v_adc_aled1,      v_adc_aled2;       // V
    float v_tia_diff_led1, v_tia_diff_led2, v_tia_diff_aled1, v_tia_diff_aled2;  // V  (= 2 × I_PD × RF)
    float i_pd_led1,       i_pd_led2,       i_pd_aled1,       i_pd_aled2;        // A
    float ot_led1,         ot_led2;  // A/A — optical transmittance: (i_pd_led − i_pd_aled) / ILEDx_A.
                                     // Full electro-optical system ratio (LED efficiency × tissue ×
                                     // PD responsivity). Validity: otLed1Valid()/otLed2Valid().

    // ── Validity masks — bit = channel (1 << (uint8_t)AFE4490Ch) ────────────
    uint8_t adc_sat_pos  = 0;  // ADC code ≥ kAdcSatPos
    uint8_t adc_sat_neg  = 0;  // ADC code ≤ kAdcSatNeg
    uint8_t tia_over_fs  = 0;  // v_tia_diff > kVTiaDiffFs (photocurrent unipolar → no lower bound)
    uint8_t tia_over_lin = 0;  // v_tia_diff > kVTiaDiffLin

    // ── Helpers (mandatory access path — packing may change) ────────────────
    bool adcSatPos(AFE4490Ch ch)  const { return (adc_sat_pos  & _bit(ch)) != 0; }
    bool adcSatNeg(AFE4490Ch ch)  const { return (adc_sat_neg  & _bit(ch)) != 0; }
    bool adcSat(AFE4490Ch ch)     const { return adcSatPos(ch) || adcSatNeg(ch); }
    bool tiaOverFs(AFE4490Ch ch)  const { return (tia_over_fs  & _bit(ch)) != 0; }
    bool tiaOverLin(AFE4490Ch ch) const { return (tia_over_lin & _bit(ch)) != 0; }

    AFE4490ChState chState(AFE4490Ch ch) const {
        if (adcSat(ch) || tiaOverLin(ch)) return AFE4490ChState::CH_CLIPPED;
        if (tiaOverFs(ch))                return AFE4490ChState::CH_OFF_SPEC;
        return AFE4490ChState::CH_CLEAN;
    }
    bool chClean(AFE4490Ch ch) const { return chState(ch) == AFE4490ChState::CH_CLEAN; }

    // OT validity is strict (both source channels CH_CLEAN): OT is a ratio of two
    // measurements — an OFF_SPEC or CLIPPED operand makes the ratio unreliable.
    bool otLed1Valid() const { return chClean(AFE4490Ch::LED1) && chClean(AFE4490Ch::ALED1); }
    bool otLed2Valid() const { return chClean(AFE4490Ch::LED2) && chClean(AFE4490Ch::ALED2); }

    bool anyAdcSat()    const { return (adc_sat_pos | adc_sat_neg) != 0; }              // HGAC O1
    bool anyTiaOverFs() const { return tia_over_fs != 0; }                              // HGAC O2
    bool allClean()     const { return (adc_sat_pos | adc_sat_neg | tia_over_fs | tia_over_lin) == 0; }

private:
    static uint8_t _bit(AFE4490Ch ch) { return (uint8_t)(1u << (uint8_t)ch); }
};

/// Debug/test snapshot — aggregates internal signals not published via AFE4490Data.
/// Not in the production data path (not in queue). Intended for test tools (e.g. PulseNest).
/// Retrieved via getData(data, dbg); updated every sample under _state_mutex.
struct AFE4490DebugData {
    AFE4490AnalogState analog;  // physical reconstruction: V_ADC, V_TIA_DIFF, I_PD for all 4 channels
    AFE4490RF hgac_rf_led1;         // current RF for LED1 (IR) domain — HGAC Phase 1 (RF-only descent)
    AFE4490RF hgac_rf_led2;         // current RF for LED2 (RED) domain
    bool      hgac_alarm_gain_floor; // true if either domain is at RF_10K and still saturating
};

// ── String codecs for enum ↔ $CFG/$SET protocol labels ───────────────────────
// These functions convert between the enum values and the abbreviated string labels
// used in the $CFG serial frame and $SET command protocol.
// String labels: RF → "10K"/"25K"/"50K"/"100K"/"250K"/"500K"/"1M"
//                CF → "5p"/"10p"/"20p"/"30p"/"55p"/"155p"
//                STG2 → "0dB"/"3.5dB"/"6dB"/"9.5dB"/"12dB"

inline const char* afeRFToStr(AFE4490RF g) {
    switch (g) {
        case AFE4490RF::RF_10K:  return "10K";
        case AFE4490RF::RF_25K:  return "25K";
        case AFE4490RF::RF_50K:  return "50K";
        case AFE4490RF::RF_100K: return "100K";
        case AFE4490RF::RF_250K: return "250K";
        case AFE4490RF::RF_500K: return "500K";
        case AFE4490RF::RF_1M:   return "1M";
        default:                      return "?";
    }
}

inline bool afeStrToRF(const char* s, AFE4490RF& out) {
    if      (strcmp(s, "10K")  == 0) { out = AFE4490RF::RF_10K;  return true; }
    else if (strcmp(s, "25K")  == 0) { out = AFE4490RF::RF_25K;  return true; }
    else if (strcmp(s, "50K")  == 0) { out = AFE4490RF::RF_50K;  return true; }
    else if (strcmp(s, "100K") == 0) { out = AFE4490RF::RF_100K; return true; }
    else if (strcmp(s, "250K") == 0) { out = AFE4490RF::RF_250K; return true; }
    else if (strcmp(s, "500K") == 0) { out = AFE4490RF::RF_500K; return true; }
    else if (strcmp(s, "1M")   == 0) { out = AFE4490RF::RF_1M;   return true; }
    return false;
}

inline const char* afeCFToStr(AFE4490TIACF cf) {
    switch (cf) {
        case AFE4490TIACF::CF_5P:   return "5p";
        case AFE4490TIACF::CF_10P:  return "10p";
        case AFE4490TIACF::CF_20P:  return "20p";
        case AFE4490TIACF::CF_30P:  return "30p";
        case AFE4490TIACF::CF_55P:  return "55p";
        case AFE4490TIACF::CF_155P: return "155p";
        default:                    return "?";
    }
}

inline bool afeStrToCF(const char* s, AFE4490TIACF& out) {
    if      (strcmp(s, "5p")   == 0) { out = AFE4490TIACF::CF_5P;   return true; }
    else if (strcmp(s, "10p")  == 0) { out = AFE4490TIACF::CF_10P;  return true; }
    else if (strcmp(s, "20p")  == 0) { out = AFE4490TIACF::CF_20P;  return true; }
    else if (strcmp(s, "30p")  == 0) { out = AFE4490TIACF::CF_30P;  return true; }
    else if (strcmp(s, "55p")  == 0) { out = AFE4490TIACF::CF_55P;  return true; }
    else if (strcmp(s, "155p") == 0) { out = AFE4490TIACF::CF_155P; return true; }
    return false;
}

inline const char* afeRGToStr(AFE4490RG g) {
    switch (g) {
        case AFE4490RG::RG_100K:   return "0dB";
        case AFE4490RG::RG_150K: return "3.5dB";
        case AFE4490RG::RG_200K:   return "6dB";
        case AFE4490RG::RG_300K: return "9.5dB";
        case AFE4490RG::RG_400K:  return "12dB";
        default:                            return "?";
    }
}

inline bool afeStrToRG(const char* s, AFE4490RG& out) {
    if      (strcmp(s, "0dB")   == 0) { out = AFE4490RG::RG_100K;   return true; }
    else if (strcmp(s, "3.5dB") == 0) { out = AFE4490RG::RG_150K; return true; }
    else if (strcmp(s, "6dB")   == 0) { out = AFE4490RG::RG_200K;   return true; }
    else if (strcmp(s, "9.5dB") == 0) { out = AFE4490RG::RG_300K; return true; }
    else if (strcmp(s, "12dB")  == 0) { out = AFE4490RG::RG_400K;  return true; }
    return false;
}

// ── Timing registers snapshot ─────────────────────────────────────────────────
// Returned by getTimingConfig(). Raw register values (counts, AFECLK = 4 MHz → 1 count = 0.25 µs).
// Field names follow datasheet Table 2 notation (t1–t28). t29 = PRPCOUNT is in AFE4490Config.
struct AFE4490TimingConfig {
    uint32_t t1,  t2;   // LED2STC,    LED2ENDC    — LED2 sampling window
    uint32_t t3,  t4;   // LED2LEDSTC, LED2LEDENDC — LED2 LED pulse
    uint32_t t5,  t6;   // ALED2STC,   ALED2ENDC   — ambient LED2 sampling
    uint32_t t7,  t8;   // LED1STC,    LED1ENDC    — LED1 sampling window
    uint32_t t9,  t10;  // LED1LEDSTC, LED1LEDENDC — LED1 LED pulse
    uint32_t t11, t12;  // ALED1STC,   ALED1ENDC   — ambient LED1 sampling
    uint32_t t13, t14;  // LED2CONVST,  LED2CONVEND  — LED2 ADC conversion
    uint32_t t15, t16;  // ALED2CONVST, ALED2CONVEND — ambient LED2 ADC conversion
    uint32_t t17, t18;  // LED1CONVST,  LED1CONVEND  — LED1 ADC conversion
    uint32_t t19, t20;  // ALED1CONVST, ALED1CONVEND — ambient LED1 ADC conversion
    uint32_t t21, t22;  // ADCRSTSTCT0, ADCRSTENDCT0 — ADC reset pulse 0
    uint32_t t23, t24;  // ADCRSTSTCT1, ADCRSTENDCT1 — ADC reset pulse 1
    uint32_t t25, t26;  // ADCRSTSTCT2, ADCRSTENDCT2 — ADC reset pulse 2
    uint32_t t27, t28;  // ADCRSTSTCT3, ADCRSTENDCT3 — ADC reset pulse 3
};

// ── Chip configuration snapshot ───────────────────────────────────────────────
// Returned by getConfig(). Groups all runtime-configurable parameters.
struct AFE4490Config {
    uint16_t          afe_sample_rate_hz;    // ADC sample rate [Hz]
    uint8_t           afe_adc_averages;      // ADC hardware averages (1 = no averaging)
    float             afe_led1_current_mA;   // IR LED drive current [mA]
    float             afe_led2_current_mA;   // RED LED drive current [mA]
    uint8_t           afe_led_range_mA;      // LED full-scale range: 75 or 150 mA
    // TIA gain — separate per-channel (ENSEPGAIN=1) or shared (ENSEPGAIN=0)
    // When afe_sep_tia_en=false: hardware uses LED2 fields for both channels (TIAGAIN LED1 fields ignored by chip)
    // When afe_sep_tia_en=true:  TIAGAIN register controls LED1 (IR); TIA_AMB_GAIN controls LED2 (RED)
    bool              afe_sep_tia_en;         // ENSEPGAIN bit D15 in TIAGAIN register
    AFE4490RF    afe_tia_rf_led1;     // RF for LED1 (IR)  — active when afe_sep_tia_en=true
    AFE4490TIACF      afe_tia_cf_led1;       // CF for LED1 (IR)  — active when afe_sep_tia_en=true
    AFE4490RG afe_stg2_rg_led1;  // Stage 2 STG2GAIN1 for LED1 (IR) — gain only, no enable
    bool              afe_stg2_en_led1;        // STAGE2EN1 (D14 of TIAGAIN) — explicit Stage 2 enable for LED1
    AFE4490RF    afe_tia_rf_led2;     // RF for LED2 (RED) — always active (both channels when ENSEPGAIN=0)
    AFE4490TIACF      afe_tia_cf_led2;       // CF for LED2 (RED) — always active
    AFE4490RG afe_stg2_rg_led2;  // Stage 2 STG2GAIN2 for LED2 (RED) — gain only, no enable
    bool              afe_stg2_en_led2;        // STAGE2EN2 (D14 of TIA_AMB_GAIN) — explicit Stage 2 enable for LED2
    uint8_t           afe_ambdac_uA;         // Ambient cancellation DAC [0–10 µA]; AMBDAC[3:0] in TIA_AMB_GAIN D19:D16
    AFE4490Channel    ppgdisp_channel;       // PPG display channel selection
    float             ppgdisp_f_low_hz;      // PPG display bandpass lower cutoff [Hz]
    float             ppgdisp_f_high_hz;     // PPG display bandpass upper cutoff [Hz]
    float             hr2_f_low_hz;      // HR2 bandpass lower cutoff [Hz]
    float             hr2_f_high_hz;     // HR2 bandpass upper cutoff [Hz]
    float             hr3_f_low_hz;      // HR3 bandpass lower cutoff [Hz]
    float             hr3_f_high_hz;     // HR3 bandpass upper cutoff [Hz]
    float             spo2_a;                   // SpO2 calibration: SpO2 = a - b*R
    float             spo2_b;
    // SpO2 algorithm parameters
    float             spo2_warmup_s;             // warmup before reporting SpO2 [s]
    float             spo2_ema_mean_tau_s;         // EMA mean (DC) time constant [s]
    float             spo2_ema_var_tau_s;          // EMA variance (AC²) time constant [s]
    float             spo2_min;                  // valid output lower bound [%]
    float             spo2_max;                  // valid output upper bound [%]
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
    // RSQM algorithm parameters
    float             rsqm_ot_thr;               // OT threshold: NOT_APPLIED vs APPLIED [A/A]
    float             rsqm_signal_weak_std;      // LED1_Sub_std threshold for SIGNAL_WEAK [ADC counts]
    float             rsqm_disconn_led_sub_thr;  // |led_sub| min for DISCONNECTED detection [ADC counts]
    float             rsqm_disconn_i_pd_thr;     // |i_pd| min for DISCONNECTED detection [A]
    float             rsqm_probe_state_min_s;    // debounce time to change probe state [s]
    float             rsqm_ema_mean_tau_s;        // RSQM EMA mean time constant [s]
    float             rsqm_ema_var_tau_s;         // RSQM EMA variance time constant [s]
    // HGAC parameters (Phase 1: RF-only descent — see incunest_afe4490_spec.md §5.8)
    bool              hgac_enable;                    // master enable; false = HGAC never actuates
    float             hgac_v_tia_diff_thr;            // V — saturation guard (A3): descend RF above this
    float             hgac_v_tia_diff_ideal;          // V — ideal operating point (telemetry only, v1)
    float             hgac_descent_persist_s;         // s — debounce before a descent step (urgent path)
    float             hgac_gate_invalid_frac_max;     // Gate G0: max fraction of non-clean samples allowed
    float             hgac_gate_window_s;             // s — EMA window for the invalid-fraction estimator
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
    // debug=true: queue items carry AFE4490DebugData alongside AFE4490Data — enables
    // getData(data, dbg) with true sample-level atomicity. Zero overhead when false (default).
    void begin(int pin_cs, int pin_drdy, bool debug = false);
#endif

    // Chip configuration setters (callable before or after begin())
    void setSampleRate(uint16_t hz);        // 63–5000 Hz; recalculates NUMAV_max
    void setAdcAverages(uint8_t num);       // 1=no averaging; clamped to floor(5000/PRF)
    void setLED1Current(float mA);
    void setLED2Current(float mA);
    void setLEDRange(uint8_t mA);           // 75 or 150 mA
    // Joint setters — apply the same value to both LED1 and LED2 channels; also sets ENSEPGAIN=0
    void setTIAGain(AFE4490RF gain);   // sets afe_tia_rf_led1 = afe_tia_rf_led2 = gain
    void setTIACF(AFE4490TIACF cf);         // sets afe_tia_cf_led1 = afe_tia_cf_led2 = cf
    void setStage2Gain(AFE4490RG gain); // sets afe_stg2_rg_led1 = afe_stg2_rg_led2 = gain
    // Separate gain mode — enable independent LED1/LED2 TIA settings (ENSEPGAIN bit D15 in TIAGAIN)
    void setEnSepGain(bool enable);
    // Per-channel setters — only meaningful when ENSEPGAIN=1 (setEnSepGain(true))
    void setTIAGainLED1(AFE4490RF gain);   // RF for LED1 (IR)  — writes TIAGAIN RF_LED1[2:0]
    void setTIACFLED1(AFE4490TIACF cf);         // CF for LED1 (IR)  — writes TIAGAIN CF_LED1[4:0]
    void setStage2GainLED1(AFE4490RG gain); // STG2GAIN1[2:0] for LED1 (IR) — gain only, does not touch STAGE2EN1
    void setStage2En1(bool en);             // STAGE2EN1 (D14 of TIAGAIN) — explicit Stage 2 enable for LED1
    void setTIAGainLED2(AFE4490RF gain);   // RF for LED2 (RED) — writes TIA_AMB_GAIN RF_LED2[2:0]
    void setTIACFLED2(AFE4490TIACF cf);         // CF for LED2 (RED) — writes TIA_AMB_GAIN CF_LED2[4:0]
    void setStage2GainLED2(AFE4490RG gain); // STG2GAIN2[2:0] for LED2 (RED) — gain only, does not touch STAGE2EN2
    void setStage2En2(bool en);             // STAGE2EN2 (D14 of TIA_AMB_GAIN) — explicit Stage 2 enable for LED2
    void setAmbDac(uint8_t uA);             // Ambient cancellation DAC current [0–10 µA]; AMBDAC[3:0] in TIA_AMB_GAIN D19:D16

    // Signal and filter configuration
    void setPPGChannel(AFE4490Channel channel);
    void setPPGDispFilter(float f_low_hz = 0.5f, float f_high_hz = 20.0f);

    // HR2 bandpass filter cutoffs (default 0.5–5 Hz); callable before or after begin()
    void setHR2Filter(float f_low_hz = 0.5f, float f_high_hz = 5.0f);

    // HR3 bandpass filter cutoffs (default 0.4–15 Hz); callable before or after begin()
    void setHR3Filter(float f_low_hz = 0.4f, float f_high_hz = 15.0f);

    // SpO2 algorithm setters (callable before or after begin())
    void setSpO2WarmupS(float s);                              // warmup period before reporting SpO2
    void setSpO2EmaMeanTauS(float tau_s);                      // EMA mean (DC) time constant; triggers recalc
    void setSpO2EmaVarTauS(float tau_s);                       // EMA variance (AC²) time constant; triggers recalc
    void setSpO2Range(float min_pct, float max_pct);           // valid SpO2 output range [%]
    void setSpO2PiSqiThresholds(float lo_pct, float hi_pct);  // PI-based SQI thresholds [%]

    // HR1 algorithm setters
    void setHR1DcTauS(float tau_s);                            // DC removal IIR time constant; triggers recalc
    void setHR1MaCutoffHz(float hz);                           // peak detection LP MA cutoff; triggers recalc
    void setHR1SqiCvMax(float cv);                             // SQI CV threshold [dimensionless]

    // HR2 algorithm setters
    void setHR2MinCorr(float corr);                            // min autocorrelation for SQI=1
    void setHR2UpdateInterval(uint32_t decimated_samples);     // autocorrelation recompute period

    // HR3 algorithm setters
    void setHR3UpdateInterval(uint32_t decimated_samples);     // FFT+HPS recompute period

    // HR valid range setter (applies to HR1 + HR2 + HR3)
    void setHRValidRange(float min_bpm, float max_bpm);

    // RSQM algorithm setters (callable before or after begin())
    void setRsqmOtThr(float v);                     // OT threshold: NOT_APPLIED vs APPLIED [A/A]
    void setRsqmSignalWeakStd(float v);             // LED1_Sub_std threshold for SIGNAL_WEAK [ADC counts]
    void setRsqmDisconnLedSubThr(float v);          // |led_sub| min for DISCONNECTED detection [ADC counts]
    void setRsqmDisconnIPdThr(float v);             // |i_pd| min for DISCONNECTED detection [A]
    void setRsqmProbeStateMinS(float s);            // debounce time to change probe state [s]; triggers recalc
    void setRsqmEmaTau(float mean_tau_s, float var_tau_s); // RSQM EMA time constants [s]; triggers recalc

    // HGAC setters (callable before or after begin()) — Phase 1: RF-only descent
    void setHgacEnable(bool en);                    // master enable; false = HGAC never actuates
    void setHgacVTiaDiffThr(float v);               // V — saturation guard threshold (A3)
    void setHgacDescentPersistS(float s);           // s — debounce before a descent step; triggers recalc
    void setHgacGateInvalidFracMax(float v);        // Gate G0: max fraction of non-clean samples allowed

    /**
     * @brief Retrieves the next available sample from the internal queue (non-blocking).
     *
     * The library samples internally at the configured rate (default 500 Hz) regardless
     * of how often this function is called.
     *
     * The internal queue holds up to INCUNEST_AFE4490_QUEUE_SIZE (10) samples. When full,
     * the oldest sample is automatically discarded to make room for the newest.
     *
     * Behaviour by consumption rate:
     *  - **> 500 Hz**: most calls return false — no data available yet.
     *  - **≈ 500 Hz**: ideal — queue stays near-empty, samples consumed in order, no discards.
     *  - **Occasional jitter**: queue buffers up to 10 samples without discards.
     *  - **Consistently < 500 Hz**: queue stays full in steady state; samples are returned in
     *    FIFO order with a fixed latency of 10 × T_sample (20 ms at 500 Hz). Intermediate
     *    samples are discarded (e.g. at 100 Hz, 1 in every 5 samples is received).
     *
     * The internal algorithms (HR1, HR2, HR3, SpO2) process every sample at the configured
     * rate regardless of consumption rate — call frequency does not affect calculation quality.
     *
     * @param data  Output: filled with the oldest queued sample if available.
     * @return true if data was available, false if the queue was empty.
     */
#ifndef INCUNEST_OFFLINE
    // dbg: if non-null and begin(debug=true) was used, filled atomically with the debug
    // snapshot for the same sample. Ignored (not filled) when debug mode is inactive.
    bool getData(AFE4490Data& data, AFE4490DebugData* dbg = nullptr);
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

    // Returns all timing register values read directly from the AFE4490 chip via SPI.
    // Requires begin() to have been called. Returns zeros if not initialized.
    // Thread-safe: takes _spi_mutex.
    AFE4490TimingConfig getTimingConfig();

    // Writes a single timing register directly to the chip (takes effect on next PRP cycle).
    // addr: register address (0x01–0x1C). value: 16-bit count (AFECLK = 4 MHz).
    // No validation is performed — caller is responsible for constraint checking.
    // Requires begin() to have been called. No-op if not initialized.
    // Thread-safe: takes _spi_mutex.
    void setTimingReg(uint8_t addr, uint32_t value);

    // Runs the AFE4490 hardware diagnostics sequence (Table 3, section 8.4.3.3).
    // Holds _spi_mutex for ~10 ms to prevent the data task from clearing CONTROL0.
    // Returns the raw 24-bit DIAG register (0x30) with all 13 diagnostic flags.
    // Returns 0 if the library is not initialized.
    //
    // diag_holdoff_ms: after diagnostics end, _task_body() feeds the last valid
    // raw ADC values into _process_sample() for this many milliseconds instead of
    // reading the chip. This replaces transient outliers (analog front-end re-settling
    // after DIAG_EN is cleared) with neutral frozen input that the IIR/BPF/LP filters
    // absorb gracefully. Set to 0 to observe the raw diagnostic artefacts.
    // Default: kAFE_DIAG_HOLDOFF_MS — derived from kAFE_FLTRCNRSEL (datasheet p.11):
    //   500 Hz corner → 28 ms; 1000 Hz corner → 16 ms.
#ifndef INCUNEST_OFFLINE
    uint32_t runAfeDiagnostics(uint32_t diag_holdoff_ms = kAFE_DIAG_HOLDOFF_MS);
#endif

    // ISR entry point (must be public for static trampoline)
#ifndef INCUNEST_OFFLINE
    void _drdy_isr();
#endif

private:
    // ── Private types ─────────────────────────────────────────────────────────

    // 2nd-order Butterworth filter (bandpass or lowpass) using DF-II transposed form.
    // init_bp() / init_lp() compute coefficients via bilinear transform; process() runs
    // one sample; reset() clears state and schedules precharge on the next process() call.
    struct BiquadFilter {
        float f_low  = 0.0f;   // bandpass lower cutoff [Hz] (unused for LP)
        float f_high = 0.0f;   // upper cutoff / LP cutoff [Hz]

        // Design a 2nd-order Butterworth bandpass. Recalculates coefficients; does not reset state.
        void init_bp(float f_low_hz, float f_high_hz, float fs);

        // Design a 2nd-order Butterworth lowpass. Recalculates coefficients; does not reset state.
        void init_lp(float f_high_hz, float fs);

        // Clear state and schedule precharge on next process() call.
        void reset() { _v1 = 0.0f; _v2 = 0.0f; _needs_precharge = true; }

        // Process one sample (DF-II transposed). Precharges to steady-state on first call after reset().
        inline float process(float x) {
            if (_needs_precharge) {
                float denom = 1.0f + _a1 + _a2;
                float y_ss  = (denom != 0.0f) ? x * (_b0 + _b1 + _b2) / denom : 0.0f;
                _v1 = y_ss - _b0 * x;
                _v2 = _b2 * x - _a2 * y_ss;
                _needs_precharge = false;
            }
            float y = _b0 * x + _v1;
            _v1     = _b1 * x - _a1 * y + _v2;
            _v2     = _b2 * x - _a2 * y;
            return y;
        }

    private:
        float _b0 = 0.0f, _b1 = 0.0f, _b2 = 0.0f;   // DF-II transposed coefficients
        float _a1 = 0.0f, _a2 = 0.0f;
        float _v1 = 0.0f, _v2 = 0.0f;                 // filter state
        bool  _needs_precharge = true;
    };

    // EMA (Exponential Moving Average) estimator for one signal channel (mean + variance).
    // EMA low-pass: y += α·(x − y)  ≡  y = (1−α)·y + α·x   (α small = slow, α→1 = fast)
    // Configured by time constants τ (seconds): α = 1 − exp(−1/(τ·fs))
    struct EmaChannel {
        float    mean  = 0.0f;
        float    var   = 0.0f;
        uint32_t count = 0;

        // Set time constants and sample rate; computes α internally.
        // Must be called before the first update(). Safe to call again to reconfigure.
        void init(float mean_tau_s, float var_tau_s, float fs) {
            _mean_alpha = 1.0f - expf(-1.0f / (mean_tau_s * fs));
            _var_alpha  = 1.0f - expf(-1.0f / (var_tau_s  * fs));
        }

        // Zero mean, var and count; keeps α unchanged (no need to re-init after reset).
        void reset() { mean = 0.0f; var = 0.0f; count = 0; }

        inline void update(float x) {
            mean += _mean_alpha * (x - mean);
            float d = x - mean;
            var  += _var_alpha  * (d * d - var);
            count++;
        }

    private:
        float _mean_alpha = 0.0f;
        float _var_alpha  = 0.0f;
    };

    // HGAC gain domain — ENSEPGAIN=1 splits RF into two independent domains (IR=LED1+ALED1,
    // RED=LED2+ALED2; see A4/O10 in incunest_afe4490_spec.md §5.8). Distinct from AFE4490Ch
    // (per-channel validity masks, 4 values) since the RF actuator is per-color, not per-channel.
    enum class HgacColor : uint8_t { IR = 0, RED = 1 };

    // ── RSQM constants ─────────────────────────────────────────────────────────
    // EMA mean τ: 2000 ms, f_c = 0.08 Hz.
    // Must be slow enough that f_c << 0.5 Hz (min physiological HR) so the mean tracks only
    // the DC baseline and does not follow the pulsatile component — otherwise d = x−mean is
    // underestimated, biasing the variance estimate and causing false RSQM_DIAG_SIGNAL_WEAK.
    // Also used for OT (optical transmittance): responds to hardware level changes (HGAC
    // RF/LED adjustments, settling ~150 ms) within 3τ ≈ 6 s — acceptable because
    // RSQM_DIAG_HW_SETTLING gates HGAC actions during that period.
    float             rsqm_ema_mean_tau_s    = 2.0f;     // τ_mean = 2 s → α ≈ 0.001 @ 500 Hz

    // EMA variance τ: 4000 ms, f_c = 0.04 Hz.
    // Must be slower than mean (τ_var > τ_mean) because the variance estimate needs to
    // accumulate several complete cardiac cycles to be reliable:
    //   @ 60 bpm (1 Hz, period 1000 ms): τ_var covers 4 full cycles
    //   @ 30 bpm (0.5 Hz, period 2000 ms): τ_var covers 2 full cycles (minimum acceptable)
    // Using τ_var = τ_mean = 2 s would cover only 1 cycle at 30 bpm — borderline.
    // τ_var > τ_mean is intentional asymmetry justified by different physical requirements.
    float             rsqm_ema_var_tau_s     = 4.0f;     // τ_var  = 4 s → α ≈ 0.0005 @ 500 Hz

    static constexpr uint32_t rsqm_ready_count      = 6000;     // samples before estimators trusted (3 × τ_var = 12 s @ 500 Hz)
    float             rsqm_disconn_led_sub_thr = 5000.0f;  // |led_sub| < this on both ch → disconnected; measured worst case: LED2_Sub=4013 (×1.25 margin)
    float             rsqm_disconn_i_pd_thr = 150e-9f; // |i_pd| < this on all 4 ch → disconnected; worst case 52 nA (ambdac=2 µA, RF=50K) → ×2.9 margin
                                                                 // Min i_pd LED ch in PROBE_APPLIED: 218 nA → ×1.45 below; sub criterion guards against false positives
    float             rsqm_ot_thr           = 8.5e-5f;  // OT > this → PROBE_NOT_APPLIED; ≤ this → PROBE_APPLIED
                                                                 // Physical scale (A/A): APPLIED ≈ 1.4e-5, NOT_APPLIED ≈ 8e-4 (needs empirical calibration)
    // ADC saturation thresholds live in AFE4490AnalogState (kAdcSatPos/kAdcSatNeg) since v0.35.
    float             rsqm_signal_weak_std  = 2000.0f;  // LED1_Sub_std < this → RSQM_DIAG_SIGNAL_WEAK (needs HW calibration)
    float             rsqm_probe_state_min_s = 0.2f;    // debounce time to change probe state [s] (= 100 samples @ 500 Hz)
    static constexpr uint32_t rsqm_diag_period_ms        = 5000; // periodic runAfeDiagnostics() interval in DISCONNECTED/NOT_APPLIED

    // ── HGAC constants (Phase 1: RF-only descent) ────────────────────────────
    // Defaults promoted from the former file-scope hgac_v_tia_diff_thr/ideal constants
    // (now runtime-tunable, matching the RSQM pattern above).
    bool              hgac_enable                = false; // OFF by default until bench-validated
    float             hgac_v_tia_diff_thr        = 0.9f;  // V — HGAC actuation threshold (A3); reduce RF before kVTiaDiffFs (1.0V)
    float             hgac_v_tia_diff_ideal      = 0.6f;  // V — Ideal Operating Point, diff (Fig. 135, §9.2.2); telemetry only in v1
    float             hgac_descent_persist_s     = 0.5f;  // s — O8.1: descent is urgent (~500 ms)
    float             hgac_gate_invalid_frac_max = 0.5f;  // Gate G0 (A6): n_invalid/n ≤ this
    float             hgac_gate_window_s         = 0.2f;  // s — EMA window for the invalid-fraction estimator
    // Fixed (not $SET-tunable in v1, like rsqm_diag_period_ms): approximate analog front-end
    // + downstream settling time after an RF change, used to arm RSQM_DIAG_HW_SETTLING.
    // NOT derived from _compute_settle_margin() (that figure is a sub-sample-period AFECLK-count
    // budget used only for CF auto-selection within one PRP cycle) — this is a separate,
    // coarser, sample-granularity window for diagnostic/telemetry purposes.
    static constexpr float hgac_settle_time_s  = 0.15f;    // s — ~150 ms

    // ── SPI primitives ────────────────────────────────────────────────────────
#ifndef INCUNEST_OFFLINE
    void     _write_reg(uint8_t addr, uint32_t data);
    uint32_t _read_spi_raw(uint8_t addr);   // assumes SPI_READ already enabled
    uint32_t _read_reg(uint8_t addr);       // handles SPI_READ enable/disable
#endif

    // Sign-extend 22-bit two's complement ADC output
    static int32_t _sign_extend_22(uint32_t raw);

    // Recomputes rate-dependent algorithm parameters from _afe_sample_rate_hz
    void _recalc_rate_params();
    // Returns TIA settle margin in AFECLK counts: max(tia_settle_min, 10% of LED-on window)
    uint32_t _compute_settle_margin() const;
    // Selects the largest AFE4490TIACF that settles within _compute_settle_margin() for the given RF.
    // Called automatically by _recalc_rate_params() and setTIAGain*(); setTIACF*() overrides.
    void _recalc_afe_tia_cf_led1();   // updates _afe_tia_cf_led1 from _afe_tia_rf_led1
    void _recalc_afe_tia_cf_led2();   // updates _afe_tia_cf_led2 from _afe_tia_rf_led2
    // Returns mA rounded to the nearest DAC step: round(mA/range*256)/256*range
    float _quantize_led_mA(float mA) const;

    // Chip init
#ifndef INCUNEST_OFFLINE
    void _chip_init();
    void _apply_timing_regs();
    void _apply_analog_regs();
    void _apply_control_regs();
    uint32_t _build_tiagain_led1();        // TIAGAIN register value (LED1 fields + ENSEPGAIN bit)
    uint32_t _build_tia_amb_gain_led2();   // TIA_AMB_GAIN register value (LED2 fields + AMBDAC)
#endif

    // FreeRTOS task
#ifndef INCUNEST_OFFLINE
    static void _task_trampoline(void* pv);
    static void _diag_task_trampoline(void* pv);
    void        _diag_task_body();
    void _task_body();
#endif

    // Signal processing
    void  _process_sample(int32_t led1, int32_t led2, int32_t aled1, int32_t aled2,
                          int32_t led1_sub, int32_t led2_sub);

    // Analog reconstruction — runs first in _process_sample(); provides V_ADC, V_TIA_DIFF, I_PD, OT
    AFE4490AnalogState _compute_analog_state(int32_t led1, int32_t led2,
                                             int32_t aled1, int32_t aled2) const;

    // RSQM — runs after _compute_analog_state(); fills rsqi, diag_code, probe_state
    void _update_rsqm(int32_t led1, int32_t led2, int32_t aled1, int32_t aled2,
                      int32_t led1_sub, int32_t led2_sub, const AFE4490AnalogState& as);
    void _set_probe_state(ProbeState requested); // debounce gate: commits state only after _rsqm_probe_state_min_samples consecutive requests

    // HGAC — runs after _update_rsqm(); Phase 1: RF-only descent (see incunest_afe4490_spec.md §5.8).
    // Called from _process_sample(), which already holds _state_mutex — these methods take no
    // lock of their own; _hgac_descend_rf() nests _spi_mutex (via the existing setTIAGainLED1/2()
    // setters) inside the caller's _state_mutex, which is safe because no code path acquires
    // _spi_mutex first and then blocks on _state_mutex.
    void _update_hgac(const AFE4490AnalogState& as);
    bool _hgac_gate_ok() const; // Gate G0 (A6): probe not disconnected, no HW_SETTLING, invalid fraction within bound
    void _hgac_check_descent(HgacColor color, float v_tia_diff_led, float v_tia_diff_aled,
                              uint32_t& descent_count, bool& alarm_flag);
    // Atomic "change RF + arm settling" primitive. CONTRACT: only ever called from
    // _update_hgac() (i.e. from inside _process_sample(), under _state_mutex) — never call
    // this directly from application code or another task.
    // EXPERIMENT (OT-domain input, branch experiment/ot-domain-inputs): SpO2 now consumes
    // as.ot_led1/ot_led2 (gain-invariant by construction — v_tia_diff/RF cancels), so this no
    // longer rescales the SpO2 EmaChannel by k as it did in the raw-ADC-domain version. See
    // incunest_afe4490_spec.md §5.1/§5.8.
    void _hgac_descend_rf(HgacColor color, AFE4490RF new_rf);

    // Algorithms — synchronous (used by unit tests and SpO2/HR1)
    // EXPERIMENT (OT-domain input): ot_ir/ot_red replace led1_sub/led2_sub as the
    // gain-invariant signal. probe_state is consumed (RSQM's classification), never computed
    // here — see incunest_afe4490.cpp for the reset-on-not-applied / division-safety design.
    void _update_spo2(float ot_ir, float ot_red, ProbeState probe_state);
    // EXPERIMENT (OT-domain input): ir is now as.ot_led1 (float, gain-invariant) instead of
    // led1_sub (int32_t raw ambient-corrected counts). HR1/HR2/HR3 are periodicity/timing-based
    // (peak timing, autocorrelation lag, FFT dominant frequency) — a uniform gain scale does
    // not change their result in steady state (unlike SpO2's amplitude ratio R), so this is a
    // pure type/domain change, no threshold recalibration needed for HR1/HR3. HR2's acorr0
    // near-zero guard was recalibrated for OT magnitudes (see hr2_ot_energy_eps).
    void _update_hr1(float ir);
    // HR2/HR3 synchronous entry points (unit-test only; production uses split paths below)
    void _update_hr2(float ir);
    void _update_hr3(float ir);
    void _reset_algorithms();

    // HR2 async split: fast per-sample path + linearise + compute
    bool _update_hr2_sample(float ir); // filter+decimate+buffer; returns true when interval fires
    void _linearize_hr2();                        // copy _hr2_buf → _hr2_seg (call under _state_mutex)
    void _compute_hr2();                          // autocorr on _hr2_seg → _hr2_result/_hr2_result_sqi

    // HR3 async split
    bool _update_hr3_sample(float ir); // filter+decimate+buffer; returns true when interval fires
    void _linearize_hr3();                        // DC+Hann into _hr3_fft (call under _state_mutex)
    void _compute_hr3();                          // FFT+HPS on _hr3_fft → _hr3_result/_hr3_result_sqi

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
    SemaphoreHandle_t _state_mutex;  // protects internal processing state (_ppgdisp_channel, filter
                                     // buffers, SpO2/HR accumulators) shared between _process_sample()
                                     // and the config setters that do not access the SPI bus
    QueueHandle_t     _data_queue;
    TaskHandle_t      _task_handle;
    bool              _initialized;
    uint32_t          _diag_code         { 0 };  // combined diag state: bits 0-12 = AFE DIAG reg, bits 13+ = RSQM flags
    volatile bool     _diag_active;          // true while runAfeDiagnostics() holds DIAG_EN; _task_body() skips reads
    volatile uint32_t _diag_holdoff_samples; // countdown after diagnostics end; _task_body() feeds frozen raw input
    int32_t _diag_last_led1, _diag_last_led2;          // last valid raw ADC values, saved on every normal cycle
    int32_t _diag_last_aled1, _diag_last_aled2;
    int32_t _diag_last_led1_sub, _diag_last_led2_sub;

    // ── HR2/HR3 async computation tasks ──────────────────────────────────────
    // Task A (_task_body) runs the fast per-sample path and signals Task B/C when
    // the computation window fires. Task B/C run the slow autocorr / FFT+HPS
    // outside the real-time loop, then write results under _state_mutex.
    SemaphoreHandle_t _hr2_calc_sem;    // given by Task A, taken by Task B
    SemaphoreHandle_t _hr3_calc_sem;    // given by Task A, taken by Task C
    TaskHandle_t      _hr2_task_handle;
    TaskHandle_t      _hr3_task_handle;
    TaskHandle_t      _diag_task_handle;
    volatile bool     _hr2_computing;   // true while Task B holds _hr2_seg; prevents Task A from overwriting
    volatile bool     _hr3_computing;   // true while Task C uses _hr3_fft; prevents Task A from overwriting
    float             _hr2_result;      // written by Task B, copied to _current_data under _state_mutex
    float             _hr2_result_sqi;
    float             _hr3_result;      // written by Task C
    float             _hr3_result_sqi;

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
    uint16_t          _afe_sample_rate_hz;
    uint8_t           _afe_adc_averages;     // user-visible count (1 = no averaging)
    float             _afe_led1_current_mA;
    float             _afe_led2_current_mA;
    uint8_t           _afe_led_range_mA;     // 75 or 150
    // TIA gain — separate per-channel when _afe_sep_tia_en=true
    // When false: chip uses LED2 fields for both channels; LED1 fields written but ignored by hardware
    // When true:  TIAGAIN→LED1(IR), TIA_AMB_GAIN→LED2(RED)
    bool              _afe_sep_tia_en;
    AFE4490RF    _afe_tia_rf_led1;    // RF_LED1 in TIAGAIN
    AFE4490TIACF      _afe_tia_cf_led1;      // CF_LED1 in TIAGAIN
    AFE4490RG _afe_stg2_rg_led1; // STG2GAIN1[2:0] in TIAGAIN D[10:8]
    bool              _afe_stg2_en_led1;       // STAGE2EN1 (D14 of TIAGAIN)
    AFE4490RF    _afe_tia_rf_led2;    // RF_LED2 in TIA_AMB_GAIN
    AFE4490TIACF      _afe_tia_cf_led2;      // CF_LED2 in TIA_AMB_GAIN
    AFE4490RG _afe_stg2_rg_led2; // STG2GAIN2[2:0] in TIA_AMB_GAIN D[10:8]
    bool              _afe_stg2_en_led2;       // STAGE2EN2 (D14 of TIA_AMB_GAIN); also forced ON when _afe_ambdac_uA > 0
    uint8_t           _afe_ambdac_uA;        // AMBDAC[3:0] value (0–10), written to TIA_AMB_GAIN D19:D16

    // ── Signal processing configuration ──
    AFE4490Channel    _ppgdisp_channel;

    // ── PPG display filter (Butterworth bandpass, configurable via setPPGDispFilter()) ──
    BiquadFilter      _ppgdisp_bpf;          // default: 0.5–20 Hz

    // ── HR1 moving average state (independent of PPG display filter) ──
    static constexpr int hr1_ma_max_len = 64;  // supports up to 640 Hz @ 5 Hz cutoff
    float    _hr1_ma_buf[hr1_ma_max_len];
    uint32_t _hr1_ma_len;   // computed from sample_rate in _recalc_rate_params()
    int      _hr1_ma_idx;
    float    _hr1_ma_sum;

    // ── Rate-dependent algorithm parameters (derived from _afe_sample_rate_hz) ──
    uint32_t          _spo2_warmup_samples;
    uint32_t          _hr1_refractory_samples;
    float             _hr1_dc_alpha;

    // ── SpO2 state ──
    // DC and AC² power for IR and RED channels, tracked via EmaChannel. EXPERIMENT (OT-domain
    // input, branch experiment/ot-domain-inputs): x is now OT (as.ot_led1/ot_led2), gain-
    // invariant by construction, instead of the raw ambient-corrected counts (led1_sub/led2_sub)
    // used before this experiment. See incunest_afe4490_spec.md §5.1/§5.8.
    //   mean  = DC component (slow IIR low-pass of x)
    //           α_mean = 1 − exp(−1/(τ_mean·fs)),  τ_mean = spo2_ema_mean_tau_s (default 1.6 s)
    //           NOTE: future alternative — bandpass filter for DC tracking
    //   var   = AC power estimate = E[(x − mean)²] = EMA of AC²
    //           α_var  = 1 − exp(−1/(τ_var·fs)),   τ_var  = spo2_ema_var_tau_s  (default 1.0 s)
    //           NOTE: future alternative — bandpass-filtered AC² instead of DC-subtraction AC²
    //   count = sample count used directly for warmup (ir_ema/red_ema always updated
    //           together, so either's count works — no separate _spo2_sample_count needed).
    //           Naturally restarts warmup after a reset (see _update_spo2() in the .cpp).
    //   PI    = sqrt(var) / mean × 100  [%]  (Perfusion Index = RMS_AC / DC)
    EmaChannel _spo2_ch_ir_ema;
    EmaChannel _spo2_ch_red_ema;
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
    // Bandpass-filters led1_sub (0.5–5 Hz), decimates by hr2_decim_factor,
    // accumulates a circular buffer of hr2_buf_len samples, then periodically
    // computes normalised autocorrelation to find the fundamental RR period.
    static constexpr int hr2_buf_len         = 400;  // 8 s at 50 Hz (fs/hr2_decim_factor)
    static constexpr int hr2_acorr_max_lag   = 137;  // guard band lower bound 22 BPM at 50 Hz: 50*60/22 = 137 samples
    static constexpr int hr2_decim_factor    = 10;   // 500 Hz → 50 Hz

    BiquadFilter _hr2_bpf;                   // bandpass filter (default 0.5–5 Hz)
    float    _hr2_buf[hr2_buf_len];          // circular buffer of decimated filtered samples
    float    _hr2_seg[hr2_buf_len];          // linearized copy for autocorrelation (avoids stack pressure)
    int      _hr2_buf_idx;                   // next write position in _hr2_buf
    uint32_t _hr2_buf_count;                 // samples written (capped at hr2_buf_len)
    uint32_t _hr2_decim_counter;             // decimation phase counter
    uint32_t _hr2_update_counter;            // decimated samples since last autocorr computation

    // ── HR3 — FFT + Harmonic Product Spectrum HR algorithm ───────────────────
    // Low-pass-filters led1_sub (10 Hz anti-aliasing), decimates by hr3_decim_factor,
    // accumulates 512 samples, then every hr3_update_interval decimated samples applies
    // a Hann window, computes the real FFT, and finds the dominant peak via the
    // Harmonic Product Spectrum (fundamental × 2nd harmonic × 3rd harmonic).
    static constexpr int hr3_buf_len         = 512;  // 10.24 s at 50 Hz → freq resolution 0.098 Hz ≈ 5.9 BPM/bin
    static constexpr int hr3_decim_factor    = 10;   // 500 Hz → 50 Hz

    BiquadFilter _hr3_bpf;                     // bandpass filter (default 0.4–15 Hz)
    float    _hr3_buf[hr3_buf_len];            // circular buffer of decimated LP-filtered samples
    float    _hr3_hann[hr3_buf_len];           // precomputed Hann window coefficients (computed once in begin())
    float    _hr3_fft[hr3_buf_len * 2];        // complex FFT buffer (interleaved re/im), also scratch for windowed input
    int      _hr3_buf_idx;                     // next write position in _hr3_buf
    uint32_t _hr3_buf_count;                   // samples written (capped at hr3_buf_len)
    uint32_t _hr3_decim_counter;               // decimation phase counter
    uint32_t _hr3_update_counter;              // decimated samples since last FFT computation

    // ── Algorithm tuning parameters (runtime-configurable) ──────────────────
    float    _spo2_warmup_s;
    float    _spo2_ema_mean_tau_s;
    float    _spo2_ema_var_tau_s;
    float    _spo2_min;
    float    _spo2_max;
    float    _spo2_pi_sqi_lo;
    float    _spo2_pi_sqi_hi;
    float    _hr1_dc_tau_s;
    float    _hr1_ma_cutoff_hz;
    float    _hr1_sqi_cv_max;
    float    _hr2_min_corr;
    uint32_t _hr2_update_interval;
    uint32_t _hr3_update_interval;
    float    _hr_min_bpm;
    float    _hr_max_bpm;

    // ── RSQM state ────────────────────────────────────────────────────────────
    EmaChannel _rsqm_led1_sub_ema;     // EMA estimator for led1_sub (IR differential)
    EmaChannel _rsqm_led2_sub_ema;     // EMA estimator for led2_sub (RED differential)
    ProbeState     _rsqm_probe_state         { ProbeState::PROBE_DISCONNECTED }; // current probe state (output of _set_probe_state)
    ProbeState     _rsqm_probe_state_pending { ProbeState::PROBE_DISCONNECTED }; // last requested state (debounce accumulator)
    uint32_t       _rsqm_probe_state_count   { 0 };                              // consecutive samples with same requested state
    uint32_t       _rsqm_probe_state_min_samples { 100 };                        // cached from rsqm_probe_state_min_s × fs
    uint32_t       _rsqm_settling_countdown; // samples remaining in HW_SETTLING window (0 = inactive)

    // ── HGAC state (Phase 1: RF-only descent) ────────────────────────────────
    uint32_t   _hgac_descent_min_samples;        // cached from hgac_descent_persist_s × fs
    uint32_t   _hgac_descent_count_led1 { 0 };   // debounce counter, IR domain (LED1/ALED1)
    uint32_t   _hgac_descent_count_led2 { 0 };   // debounce counter, RED domain (LED2/ALED2)
    EmaChannel _hgac_invalid_ema;                 // fraction of !allClean() samples — Gate G0 only, not the guard trip itself
    bool       _hgac_alarm_gain_floor_led1 { false }; // IR domain at RF_10K, still saturating
    bool       _hgac_alarm_gain_floor_led2 { false }; // RED domain at RF_10K, still saturating

    bool             _debug_enabled; // set by begin(debug=true); enables combined queue items
    // ── Output snapshot (written by task, pushed to queue) ──
    AFE4490Data      _current_data;
    AFE4490DebugData _debug_data;   // debug snapshot; written under _state_mutex each sample

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

    // HR1
    void  test_feed_hr1(float ot_ir) { _update_hr1(ot_ir); }
    float test_hr1()                        { return _current_data.hr1; }
    float test_hr1_sqi()                    { return _current_data.hr1_sqi; }

    // HR2
    void  test_feed_hr2(float ot_ir) { _update_hr2(ot_ir); }
    float test_hr2()                        { return _current_data.hr2; }
    float test_hr2_sqi()                    { return _current_data.hr2_sqi; }

    // HR3
    void  test_feed_hr3(float ot_ir) { _update_hr3(ot_ir); }
    float test_hr3()                        { return _current_data.hr3; }
    float test_hr3_sqi()                    { return _current_data.hr3_sqi; }

    // SpO2
    void  test_feed_spo2(float ot_ir, float ot_red, ProbeState probe_state) {
        _update_spo2(ot_ir, ot_red, probe_state);
    }
    float test_spo2()                       { return _current_data.spo2; }
    float test_spo2_r()                     { return _current_data.spo2_r; }
    float test_spo2_sqi()                   { return _current_data.spo2_sqi; }
    float test_spo2_ir_ema_mean()  const { return _spo2_ch_ir_ema.mean; }
    float test_spo2_ir_ema_var()   const { return _spo2_ch_ir_ema.var; }
    float test_spo2_red_ema_mean() const { return _spo2_ch_red_ema.mean; }
    float test_spo2_red_ema_var()  const { return _spo2_ch_red_ema.var; }
    void  test_set_spo2_ir_ema(float mean, float var)  { _spo2_ch_ir_ema.mean = mean; _spo2_ch_ir_ema.var = var; }
    void  test_set_spo2_red_ema(float mean, float var) { _spo2_ch_red_ema.mean = mean; _spo2_ch_red_ema.var = var; }

    // HGAC — full-pipeline entry point (HGAC hooks inside _process_sample(), not a narrower path)
    void  test_feed_sample(int32_t led1, int32_t led2, int32_t aled1, int32_t aled2) {
        _process_sample(led1, led2, aled1, aled2, led1 - aled1, led2 - aled2);
    }
    AFE4490RF test_hgac_rf_led1() const { return _afe_tia_rf_led1; }
    AFE4490RF test_hgac_rf_led2() const { return _afe_tia_rf_led2; }
    bool test_hgac_alarm_gain_floor() const { return _hgac_alarm_gain_floor_led1 || _hgac_alarm_gain_floor_led2; }
    // Isolated access to the RF-change primitive, bypassing the full sample pipeline.
    void test_hgac_descend_rf_led1(AFE4490RF new_rf) { _hgac_descend_rf(HgacColor::IR, new_rf); }
    void test_hgac_descend_rf_led2(AFE4490RF new_rf) { _hgac_descend_rf(HgacColor::RED, new_rf); }
    uint32_t test_diag_code() const { return _current_data.diag_code; }
#endif
};
