// incunest_afe4490.cpp — Medical Open World AFE4490 driver + PPG algorithms (HR, SpO2, PI)
// Library version: v0.104 — ESP32-S3, Arduino-ESP32 or ESP-IDF, FreeRTOS
// Spec: incunest_afe4490_spec.md
// Chip datasheet: https://www.ti.com/lit/ds/symlink/afe4490.pdf
// Author: Medical Open World — http://medicalopenworld.org — <contact@medicalopenworld.org>

#include "incunest_afe4490.h"
#include <math.h>
#include <string.h>

static const char* TAG = "incunest_afe4490";

namespace {
    // ── Math ──────────────────────────────────────────────────────────────────
    constexpr float    kPi                  = 3.14159265358979f;
    // Arduino's constrain macro, as a function. The Arduino build is gnu++11, so std::clamp
    // (C++17) is not available; unlike the macro, each argument is evaluated once.
    inline float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

    // ── SpO2 ──────────────────────────────────────────────────────────────────
    // EMA mean τ (DC baseline): 2.0 s, f_c ≈ 0.08 Hz.
    // f_c << min physiological HR (0.5 Hz @ 30 bpm) so the DC estimate does not follow the
    // pulsatile component; if it did, AC = x−DC would be underestimated, biasing the R ratio and SpO2.
    constexpr float    spo2_ema_mean_tau_s   = 2.0f;
    // SpO2 averaging window (v0.102, rationale §12): a true sliding window of 6.0 s over the
    // summands of R-METHOD-2 (dx·dy, dx², dy²) and the (x − mean)² of R-METHOD-1. Replaces the EMA with τ = 6 s, which
    // in noise and group delay was equivalent to a 12 s window (an EMA of τ averages like a
    // window of 2τ) and so was slower than the 8 s NICU default and outside the 6–10 s of ISO
    // 80601-2-61:2026 Annex JJ.2 d) it was taken from. 6 s: Nellcor "Normal" 5–7 s, between the
    // 2–4 s of neonatal resuscitation / intermittent-hypoxaemia work and the 8 s NICU default
    // (rationale §4.5). Measured on HOSPNAV SUBJ08/09: same accuracy as the EMA, 36 % more noise
    // (0.34 vs 0.25 SpO2 points), ~2 s less lag, exact settling. The window IS the warm-up.
    // Runtime range [alg_window_s_min, alg_window_s_max] — 2 s keeps the "short window + alarm
    // delay" configuration of the neonatal literature (McClure 2016) reachable.
    constexpr float    spo2_window_s         = 6.0f;
    // Default R curve (v0.100): R-CURVE-STS0163-HOSPNAV-20261004 — fitted with R-METHOD-2 as this
    // library computes it (offline_runner replay of the 2026-09-23 HOSPNAV campaigns, two neonates,
    // Medle ST-S0163 probe, reference = the ward monitor read by VideoNest, 3455 stationary 8 s
    // windows): SpO2 = 123.98 − 39.13·R, Spearman ρ −0.80, scatter 1.65 points, median |residual|
    // 0.91; per subject 129.2 − 46.7·R and 118.8 − 32.1·R, leave-one-subject-out median |error|
    // 0.95–1.38. Reference range 85–100 % (31 windows below 85, where it reads −3.4 points);
    // a simulator-free, hospital-monitor-referenced curve, not a controlled desaturation study
    // (ISO 80601-2-61 Annex DD). Only valid with R-METHOD-2 and for that probe. Spec §5.1,
    // rationale §9. The previous default (MS100 simulator, R-METHOD-0: 114.9208 − 30.5547·R)
    // stays in the spec's registry. The label's date is the date of the FIT (2026-10-04), not of
    // the campaign: until v0.105 it read -20260923, the same label the discarded R-METHOD-1
    // proposal of 2026-09-24 had used — one label, one (a, b) (PulseNest docs/spo2_r_curves.md).
    constexpr const char* spo2_r_curve_id_default          = "R-CURVE-STS0163-HOSPNAV-20261004";
    constexpr float    spo2_r_curve_a_default = 123.98f;   // SpO2 = a - b·R: the (a, b) that label names
    constexpr float    spo2_r_curve_b_default =  39.13f;
    constexpr const char* spo2_r_curve_r_method_id_default = "R-METHOD-2";  // the R method it was fitted with
    constexpr float    spo2_min            =  70.0f;   // % — valid output lower bound
    constexpr float    spo2_max            = 100.0f;   // % — valid output upper bound
    constexpr float    spo2_clamp_margin   =  10.0f;   // % — clamp to spo2_max if within margin above;
                                                       //     beyond it the curve is off the physiology
                                                       //     (3 → 10 in v0.107: a 3-pt curve offset at
                                                       //     SaO2 100 % withheld SpO2 — rationale §15)
    // EXPERIMENT (OT-domain input, branch experiment/ot-domain-inputs): SpO2's DC/AC now
    // live in OT units (~1e-5 typical, dimensionless A/A) instead of adc_code (~1e5).
    // Presence detection (is there a real signal / finger applied?) is RSQM's responsibility
    // alone (ProbeState, passed in) — _spo2_update() consumes it, does not compute its own.
    // This removes two earlier mechanisms that both tried to do RSQM's job from inside SpO2:
    // the absolute-i_pd "no-finger" gate (spo2_min_i_pd_a — rarely caught an actual no-finger
    // condition: a transmittance probe with no finger shows HIGH OT, not low i_pd) and the
    // absolute-OT-DC "no-signal floor" (spo2_min_ot_dc — same duplication, just on a
    // different quantity). Both removed.
    // spo2_div_eps is what remains: a single, purely numerical division-safety guard (NOT a
    // physiological threshold) on the actual divisors in the code below — dc_ir (PI calc) and
    // dc_red/ac_ir (R calc). Kept far below any real operating value (~1e-5 DC, ~1e-8 AC at
    // the lowest valid PI) so it only ever trips on true near-zero/numerical noise, e.g. right
    // after an EmaChannel reset.
    constexpr float    spo2_div_eps        = 1e-9f;    // A/A — division-safety guard only
    // SQI: Perfusion Index thresholds (Nellcor/Masimo clinical reference).
    // PI < spo2_pi_sqi_lo → SQI=0. PI ≥ spo2_pi_sqi_hi → SQI=1. Linear ramp in between.
    constexpr float    spo2_pi_sqi_lo      = 0.5f;     // % — PI below this → SQI = 0
    constexpr float    spo2_pi_sqi_hi      = 2.0f;     // % — PI above this → SQI = 1
    // Both in PEAK-TO-PEAK units since v0.101 (the units Nellcor/Masimo quote them in; the library's
    // PI was RMS-based until then, i.e. ÷2.8–3.2, so these gates fired ~3× too early — rationale §11).
    // Upper bound: the industry's DISPLAYED range tops at 20 % — Masimo 0.02–20 (LAB-9204A, Rad-57
    // operator's manual; lab3410f "Perfusion Index" whitepaper defines PI only as pulsatile over
    // non-pulsatile), Mindray 0.05–20 (BeneVision N-Series operator's manual, SpO2 chapter),
    // Nellcor 0.03–20 (PM-1000N service manual, Table 2-1 note 6, "detected IR pulse modulation").
    // Links in PulseNest docs/ppg_neonatal_manufacturer_references.md. None of the three discloses
    // its estimator (peak-to-peak per beat vs RMS); ISO 80601-2-61:2026 201.3.245 defines %mod only
    // as "approximately AC/DC × 100". So 20 % is a display convention, not a physiological limit
    // (open: where the 20 % ceiling really comes from — BACKLOG 2026-10-22). Above it the "PI" is a
    // DC transient (probe coming off before probe_state notices, an RF step), not perfusion: SpO2
    // is withheld instead of reported with SQI = 1.
    constexpr float    spo2_pi_max         = 20.0f;    // % — PI above this → SpO2 invalid, SQI = 0
    // Pulse gate (v0.110, rationale §17): SpO2 only while HR3 has identified a pulse. Measured on
    // the 21 referenced sessions (2026-10-20): the moving neonate SUBJ10 went from 81 % of windows
    // published at −16 points to 2.6 % at +2; quiet subjects kept 78–98 %; what the gate withholds
    // there had a median error of 2.5–3.8 points. Costs: desaturations that coincide with movement
    // are withheld (−27 points of availability below 90 % in SUBJ08/09) and the first value after
    // applying the probe waits for HR3's 10.24 s buffer (median 10.5 s, p90 36.6 s). No hold of the
    // last value: HR3 stays valid into the first seconds of motion, so a hold re-admits the error.
    constexpr bool     spo2_pulse_gate_enable = true;
    // R-METHOD-2 (v0.100, design rationale §9–§10). Band-pass = the pulse band HR2 uses, one
    // biquad: 0.3/0.7–4/8 Hz and 2 vs 4 poles measured equal for R on HOSPNAV (scatter 1.63 vs
    // 1.62 points), so no new filter type enters the library for it. r below spo2_r_method2_reg_corr_min
    // invalidates SpO2: on the corpus r never fell below 0.83 with a pulse (p1 0.949), and the
    // threshold is provisional until r has been measured on pulse-less input (probe off, motion).
    constexpr uint8_t  spo2_r_method_default = 2;
    constexpr float    spo2_r_method2_reg_corr_min       = 0.8f;
    constexpr float    spo2_bp_f_low_hz      = 0.5f;    // Hz — 30 BPM, the HR floor (§8)
    constexpr float    spo2_bp_f_high_hz     = 5.0f;    // Hz — R does not depend on the cut (MS100 40–250 BPM)
    constexpr float    spo2_reg_eps          = 1e-30f;  // EMA(dx²)/EMA(dy²) at or below this: no slope yet

    // ── PI (Perfusion Index) ─────────────────────────────────────────────────
    // Decoupled from SpO2 (v0.101) — see the PI state block in the header and design rationale
    // §11. Own private BiquadFilter instance ("paths share nothing", §5.3.1). Floor = the HR floor,
    // as HR2/R-METHOD-2; ceiling = HR3's 15 Hz, NOT R-METHOD-2's 5 Hz: PI is a magnitude, so the
    // harmonics count — measured on the MS100 at constant modulation, a 5 Hz ceiling read PI 25 %
    // lower at 220 BPM than at 60 BPM (the 2nd harmonic leaves the band); 15 Hz removes the HR
    // dependence, and the noise it admits was unmeasurable even in the lowest-PI decile of a
    // neonatal capture. R keeps 5 Hz because there the same filter sits on both channels and cancels.
    constexpr float    pi_bp_f_low_hz        = 0.5f;    // Hz — 30 BPM floor, same reasoning as spo2_bp_f_low_hz
    constexpr float    pi_bp_f_high_hz       = 15.0f;   // Hz — 3rd harmonic of hr_max_bpm (13 Hz) with margin, as HR3
    constexpr float    pi_window_s           = 6.0f;    // s — sliding-window length (runtime since v0.102, setPIWindowS)
    // RMS → peak-to-peak, exact for a sinusoid. The band-passed PPG measured 2.79/2.92 on two
    // neonates and 3.0–3.2 on the MS100, so this under-reads a per-beat Max−Min by 0–12 %; the
    // alternative (per-beat peaks) needs a beat detector and is kept for PILAB (rationale §11).
    constexpr float    pi_rms_to_pp_ratio          = 2.8284271f;   // 2·√2
    constexpr float    pi_div_eps            = 1e-9f;   // A/A — division-safety guard only (mirrors spo2_div_eps)

    // ── HR1 ───────────────────────────────────────────────────────────────────
    constexpr float    hr1_dc_tau_s        = 1.6f;     // s  — DC removal IIR time constant
    constexpr float    hr1_lp_cutoff_hz    = 5.0f;     // Hz — low-pass cutoff before peak detection
    // s — running-max decay: how long an old peak keeps holding the detection threshold up.
    // MEASURED (v0.93). The former 20 s was not a choice: it is the historical 0.9999
    // per-sample factor expressed in seconds at 500 Hz (exp(-1/(20*500)) = 0.9999). At 20 s the
    // threshold still reflects a peak from several seconds ago, so a beat whose amplitude has
    // since fallen — the pulse amplitude of a real subject swings far more than the textbook
    // 10-20 % respiratory modulation — does not reach it and is lost; one lost beat then sits in
    // the 5-interval RR buffer for five beats, and hr1_sqi is 0 for all of them (5.8 s at
    // 52 bpm, observed on the bench 2026-09-15 on two of three boards).
    // 1.5 s is the largest value that keeps the whole 40-250 bpm range: the perturbed MS100
    // sweep (2026-09-08) shows a flat optimum from 1 to 3 s (76-78 % worst case vs 53 % at 20 s)
    // that collapses at 0.5 s (49 % at 60 bpm — the threshold falls so far between beats that
    // the dicrotic notch is counted), and the three-board probe capture (2026-09-15) confirms it
    // on a real subject: every lost beat recovered, no extra detection.
    constexpr float    hr1_max_decay_tau_s = 1.5f;
    // SQI: CV = std/mean of the 5 most recent RR intervals.
    // CV=0 (perfectly regular) → SQI=1. CV ≥ hr1_sqi_cv_max → SQI=0.
    // Detection threshold, as a FRACTION of the decaying running max. Was a bare 0.6f literal
    // inside _hr1_update() until v0.88: the single parameter that most determines whether a beat
    // is counted had no name, was absent from AFE4490Config and from the spec, and could not be
    // swept without recompiling — which blocked the sensitivity study it most needs.
    // Lower → more sensitive (risks counting the dicrotic notch); higher → misses weak beats.
    constexpr float    hr1_threshold_fraction = 0.6f;
    constexpr float    hr1_sqi_cv_max      = 0.15f;    // dimensionless — 15% CV threshold
    // REFRACTORY PERIOD — the blind window after an accepted beat, during which any further
    // threshold crossing is ignored. Named after the cardiac refractory period (the myocardium
    // cannot be re-excited for a while after a contraction); here it is the guard that stops the
    // DICROTIC NOTCH — the bump left by the aortic valve closing, on the pulse's way down — from
    // being counted as a second beat and doubling the reported rate.
    //
    // 0.185 s allows beats up to 60/0.185 = 324 BPM. The binding constraint is the other end: at
    // hr_max_bpm + 3 = 263 BPM beats are 228 ms apart, so a longer refractory would start
    // rejecting legitimate ones. Hence "covers the 263 BPM guard band" — it means "does not get in
    // the way at 263 BPM", not "protects up to 263". 43 ms of that margin are unused.
    //
    // KNOWN LIMITATION: the notch does not arrive at a fixed delay — roughly a third of systole,
    // so ~150 ms at 200 BPM but 250-350 ms at 60 BPM. A fixed refractory therefore guards well in
    // tachycardia (where double-counting is least likely anyway) and poorly in BRADYCARDIA, which
    // is the event that most needs detecting in a neonate. No fixed value covers both ends: that
    // needs a refractory proportional to the estimated RR interval — see the detector strategy
    // catalogue in incunest_afe4490_design_rationale.md §5.3.
    constexpr float    hr1_refractory_s    = 0.185f;   // s

    // ── HR2 ───────────────────────────────────────────────────────────────────
    constexpr float    hr2_min_corr        = 0.5f;     // normalised autocorrelation threshold for SQI=1
    // EXPERIMENT (OT-domain input): acorr0 (sum of ~400 squared bandpass-filtered OT samples,
    // each ~1e-6 typical AC amplitude) is now ~1e-10 for a real signal, not ~1e9 as in the raw
    // adc_code domain. This is a pure near-zero-energy guard (rejects a flat/disconnected
    // signal before dividing by acorr0), not a calibrated physiological threshold — picked
    // conservatively low (real signal energy is many orders of magnitude above it).
    constexpr float    hr2_ot_energy_eps  = 1e-16f;

    // ── HR3 ───────────────────────────────────────────────────────────────────
    // SQI window half-width W [bins] on each side of {b1, b2}.
    // Window = [b1-W .. b2+W], total width = 2W+2 bins (~±0.49 Hz at 50 Hz/512).
    // Wider W → SQI sensitive to broader spectral context; narrower W → more local.
    constexpr int      hr3_snr_local_w     = 5;

    // ── HR (all algorithms) ───────────────────────────────────────────────────
    // hr_search_{min,max}_bpm are derived inline as hr_{min,max}_bpm ± 3 BPM guard band.
    // Valid output range. Neither bound has a normative source: ISO 80601-2-61 §201.12 only requires
    // the manufacturer to DISCLOSE the displayed pulse-rate range. The citable requirement is
    // WHO-UNICEF 2019 (oxygen therapy devices, §6.5 / Annex 1 Table A1.11): detection range to
    // include 30–240 bpm. 30 (v0.99; 40 from v0.25) meets the bottom and lets a bradycardia below
    // 40 BPM raise the rate alarm instead of a signal-quality one. 260 (2026-05-02) covers the top
    // with room for neonatal SVT, typically 220–300 bpm (PALS: infant SVT > 220 bpm).
    // The HR2 band-pass corner stays at 0.5 Hz = 30 bpm: its autocorrelation is normalised, so the
    // −3 dB there costs nothing (measured 2026-10-03, rationale §8); a lower corner only admits drift.
    constexpr float    hr_min_bpm          =  30.0f;   // bpm — valid lower bound
    constexpr float    hr_max_bpm          = 260.0f;   // bpm — valid upper bound
    // Widens the SEARCH range beyond the valid output range, so a rate sitting exactly at a
    // bound is still found rather than cut off. Shared by HR2 (lag sweep) and HR3 (bin sweep);
    // it was the same 3.0f literal repeated at four sites until v0.88, where changing one and
    // forgetting another would have desynchronised the two estimators silently.
    constexpr float    hr_guard_band_bpm   = 3.0f;     // bpm

    // ── HGAC — TIA operating range (DIFFERENTIAL domain) ─────
    // Reference: AFE4490 datasheet Fig. 135 ("AGC Loop"), §9.2.2.
    // TIA differential output range = ±1.0 V; ADC full-scale = ±1.2 V (22-bit).
    // To recover v_tia from an ADC reading at any Stage 2 gain and AMBDAC setting:
    //   v_tia_volt = (adc_value * adc::SCALE) / rg_gain + ambdac_val * 0.2f
    // where rg_gain is the linear Stage 2 gain (1/1.5/2/3/4), ambdac_val is AMBDAC[3:0]
    // (0–10 µA, 1 µA/step), and 0.2 V/step = 2 × Ri × 1 µA = 2 × 100 kΩ × 1 µA.
    // adc::SCALE = adc::FSR / 2^21 (bipolar twos-complement LSB; see namespace adc in .h).
    // When STAGE2EN = 0 (Stage 2 bypassed): v_tia_volt = adc_value * adc::SCALE.
    // Physical thresholds (TIA full-scale differential 1.0 V, empirical lin 1.8 V) live in
    // tia_axis (FS_V / LIN_V), the single source of truth, since v0.35 — physics belongs to
    // the reconstruction layer; only HGAC actuation POLICY stays here.
    // hgac_v_tia_high2 is now a runtime-tunable AFE4490Config field (promoted
    // from a fixed constexpr here — see incunest_afe4490.h private defaults,
    // matching the rsqm_* pattern) so their defaults live in the header, not here.
    // Empirical hard clip ~1.94 V diff (TIA output node hits the rail, pre-ambient-
    // subtraction; independent of RF and AMBDAC; IncuNest 16.A sweep 2026-07-08).
    // Reserved for future use if HGAC ever needs the physical clip:
    // static constexpr float hgac_v_tia_clip = 1.94f;  // V — empirical hard clip (2026-07-08)

    // ── HR3 FFT — radix-2 Cooley-Tukey DIT (in-place, complex interleaved) ──
    // x: float array of 2N elements [re0,im0, re1,im1, ..., re(N-1),im(N-1)]
    // N must be a power of two. Twiddle factors computed per stage (9 calls to
    // cosf/sinf for N=512), not per butterfly — negligible overhead at 0.5 s update rate.
    static void _fft_r2(float* x, int N) {
        // Bit-reversal permutation
        for (int i = 1, j = 0; i < N; i++) {
            int bit = N >> 1;
            for (; j & bit; bit >>= 1) j ^= bit;
            j ^= bit;
            if (i < j) {
                float t;
                t = x[2*i];   x[2*i]   = x[2*j];   x[2*j]   = t;
                t = x[2*i+1]; x[2*i+1] = x[2*j+1]; x[2*j+1] = t;
            }
        }
        // Butterfly stages
        for (int len = 2; len <= N; len <<= 1) {
            float w_re = cosf(-2.0f * kPi / (float)len);
            float w_im = sinf(-2.0f * kPi / (float)len);
            for (int i = 0; i < N; i += len) {
                float c_re = 1.0f, c_im = 0.0f;
                for (int j = 0; j < len / 2; j++) {
                    int u = 2 * (i + j), v = 2 * (i + j + len / 2);
                    float vt_re = c_re * x[v]   - c_im * x[v + 1];
                    float vt_im = c_re * x[v + 1] + c_im * x[v];
                    x[v]     = x[u]     - vt_re;
                    x[v + 1] = x[u + 1] - vt_im;
                    x[u]     = x[u]     + vt_re;
                    x[u + 1] = x[u + 1] + vt_im;
                    float tmp = c_re * w_re - c_im * w_im;
                    c_im      = c_re * w_im + c_im * w_re;
                    c_re      = tmp;
                }
            }
        }
    }

    // ── AFE4490 register addresses ────────────────────────────────────────────
    constexpr uint8_t REG_CONTROL0      = 0x00;
    constexpr uint8_t REG_LED2STC       = 0x01;
    constexpr uint8_t REG_LED2ENDC      = 0x02;
    constexpr uint8_t REG_LED2LEDSTC    = 0x03;
    constexpr uint8_t REG_LED2LEDENDC   = 0x04;
    constexpr uint8_t REG_ALED2STC      = 0x05;
    constexpr uint8_t REG_ALED2ENDC     = 0x06;
    constexpr uint8_t REG_LED1STC       = 0x07;
    constexpr uint8_t REG_LED1ENDC      = 0x08;
    constexpr uint8_t REG_LED1LEDSTC    = 0x09;
    constexpr uint8_t REG_LED1LEDENDC   = 0x0A;
    constexpr uint8_t REG_ALED1STC      = 0x0B;
    constexpr uint8_t REG_ALED1ENDC     = 0x0C;
    constexpr uint8_t REG_LED2CONVST    = 0x0D;
    constexpr uint8_t REG_LED2CONVEND   = 0x0E;
    constexpr uint8_t REG_ALED2CONVST   = 0x0F;
    constexpr uint8_t REG_ALED2CONVEND  = 0x10;
    constexpr uint8_t REG_LED1CONVST    = 0x11;
    constexpr uint8_t REG_LED1CONVEND   = 0x12;
    constexpr uint8_t REG_ALED1CONVST   = 0x13;
    constexpr uint8_t REG_ALED1CONVEND  = 0x14;
    constexpr uint8_t REG_ADCRSTSTCT0   = 0x15;
    constexpr uint8_t REG_ADCRSTENDCT0  = 0x16;
    constexpr uint8_t REG_ADCRSTSTCT1   = 0x17;
    constexpr uint8_t REG_ADCRSTENDCT1  = 0x18;
    constexpr uint8_t REG_ADCRSTSTCT2   = 0x19;
    constexpr uint8_t REG_ADCRSTENDCT2  = 0x1A;
    constexpr uint8_t REG_ADCRSTSTCT3   = 0x1B;
    constexpr uint8_t REG_ADCRSTENDCT3  = 0x1C;
    constexpr uint8_t REG_PRPCOUNT      = 0x1D;
    constexpr uint8_t REG_CONTROL1      = 0x1E;
    constexpr uint8_t REG_TIAGAIN       = 0x20;
    constexpr uint8_t REG_TIA_AMB_GAIN  = 0x21;
    constexpr uint8_t REG_LEDCNTRL      = 0x22;
    constexpr uint8_t REG_CONTROL2      = 0x23;
    constexpr uint8_t REG_ALARM         = 0x29;
    constexpr uint8_t REG_LED2VAL       = 0x2A;
    constexpr uint8_t REG_ALED2VAL      = 0x2B;
    constexpr uint8_t REG_LED1VAL       = 0x2C;
    constexpr uint8_t REG_ALED1VAL      = 0x2D;
    constexpr uint8_t REG_LED2_ALED2VAL = 0x2E;
    constexpr uint8_t REG_LED1_ALED1VAL = 0x2F;
    constexpr uint8_t REG_DIAG          = 0x30;

    // CONTROL0 bits
    constexpr uint32_t ctrl0_spi_read   = 0x000001UL;
    constexpr uint32_t ctrl0_sw_rst     = 0x000008UL;

    // CONTROL1 bits
    constexpr uint32_t ctrl1_timeren    = 0x000100UL;

    // AFE clock frequency
    constexpr uint32_t afeclk = 4000000UL;  // 4 MHz → 1 count = 0.25 µs

    // TIAGAIN / TIA_AMB_GAIN: RF bits [2:0]
    // enum order: RF_10K=0..RF_1M=6 → register codes (non-monotonic — see datasheet Table 17)
    constexpr uint32_t rf_code[7] = { 5, 4, 3, 2, 1, 0, 6 };
    // Physical RF values: use kAFE_RF_OHM[] from header (single source of truth)

    // TIAGAIN / TIA_AMB_GAIN: CF_LED[4:0] occupies bits D[7:3] — the code IS the value,
    // so it is written with a plain shift (no lookup table). See AFE4490CFCode in the header.
    constexpr uint32_t cf_shift = 3;
    // Physical CF values: use kAFE_CF_PF[] from header (single source of truth)

    // Auto-CF settling parameters
    // PROVENANCE (spec §7.2): none of these three comes from the AFE4490 datasheet. They are
    // kept deliberately — the combination errs conservative — but do NOT "align them with TI"
    // one at a time: they only make sense together (raising tia_n_tau 5→10 to match the
    // datasheet's implied count, while keeping the margin basis, makes the criterion twice as
    // STRICT, halving the allowed CF).
    constexpr float    tia_settle_fraction = 0.10f;  // 10% of LED-on window reserved as dead time.
                                                     // Probably Eq. 1's "/10" misapplied: it bounds τ
                                                     // against the SAMPLE WINDOW, not a dead-time fraction.
    constexpr uint32_t tia_settle_min      = 50;     // floor: 12.5 µs @ 4 MHz. Matches TI's Table 2
                                                     // example (t1 = t3 + 50), not a stated requirement.
    constexpr float    tia_n_tau           = 5.0f;   // generic 5-time-constants rule of thumb.
                                                     // "<0.7% error" is e^-5 restated, not the reason.
                                                     // Datasheet's own implied count is 10τ (Eq. 1).

    // Ambient-phase settle margin: LED OFF decay before ambient sampling starts (t5/t11).
    // No datasheet-cited value; empirical, raised 200->400 for the 50 mA default (v0.47, see
    // spec §7.2). Named so _apply_timing_regs() and _compute_switched_rc_settling_samples() share one source.
    constexpr uint32_t afe_ambient_margin_counts = 400;  // counts (100 µs)

    // Datasheet §7.7, row t5 (p.17): "> 3 ms of cumulative sampling time in each phase" after any
    // SPI command changing the signal chain — footnote (1) names "TIA gain" explicitly, so this
    // is the datasheet's OWN figure for how long to distrust data after an HGAC RF step. Replaces
    // the untraceable afe_settle_time_s (0.15 s, no derivation — see spec §5.8.4 history).
    constexpr float    afe_t5_cumulative_sample_time_s = 0.003f;  // 3 ms, datasheet minimum

    // TIAGAIN: STG2GAIN bits D[10:8] only — STAGE2EN (D14) is set separately via _afe_stg2_en_led1/2
    constexpr uint32_t stg2_gain_code[5] = {
        0x000000UL,   // RG_100K:   STG2=0
        0x000100UL,   // RG_150K: STG2=1
        0x000200UL,   // RG_200K:   STG2=2
        0x000300UL,   // RG_300K: STG2=3
        0x000400UL    // RG_400K:  STG2=4
    };
    // Physical Stage 2 gains: use kAFE_RG_GAIN[] / kAFE_RG_OHM[] from header (single source of truth)

    // Internal queue item used when debug mode is active (begin(debug=true)).
    // Carries AFE4490Data and AFE4490DebugData in a single queue slot for true atomicity.
    struct DebugQueueItem {
        AFE4490Data     data;
        AFE4490DebugData dbg;
    };
}

// ── Static member ─────────────────────────────────────────────────────────────
// Singleton pointer used by the static ISR trampoline (_drdy_isr_static) to reach
// the class instance. Static members must be defined exactly once in a .cpp file;
// the declaration in the header only reserves the name.
INCUNEST_AFE4490* INCUNEST_AFE4490::_g_instance = nullptr;

// ── Constructor / destructor ──────────────────────────────────────────────────
INCUNEST_AFE4490::INCUNEST_AFE4490()
    : _pin_cs(-1), _pin_drdy(-1),
      _drdy_sem(nullptr), _spi_mutex(nullptr), _state_mutex(nullptr),
      _data_queue(nullptr), _task_handle(nullptr),
      _initialized(false),
      _diag_active(false),
      _diag_holdoff_samples(0),
      _last_valid_led1(0), _last_valid_led2(0),
      _last_valid_aled1(0), _last_valid_aled2(0),
      _last_valid_led1_sub(0), _last_valid_led2_sub(0),
      _last_valid_analog_state{},
      _hr2_compute_sem(nullptr), _hr3_compute_sem(nullptr),
      _hr2_task_handle(nullptr), _hr3_task_handle(nullptr),
      _diag_task_handle(nullptr),
      _hr2_computing(false), _hr3_computing(false),
      _hr2_result(0.0f), _hr2_result_sqi(0.0f),
      _hr3_result(0.0f), _hr3_result_sqi(0.0f),
      _afe_sample_rate_hz(500), _afe_adc_averages(8),
      _afe_led1_current_mA(50.0f), _afe_led2_current_mA(50.0f), _afe_led_range_mA(150),
      _afe_sep_tia_en(true),
      _afe_tia_rf_led1(AFE4490RF::RF_100K),
      _afe_tia_cf_led1(kAFE_CF_CODE_DEFAULT),
      _afe_stg2_rg_led1(AFE4490RG::RG_100K),
      _afe_stg2_en_led1(true),
      _afe_tia_rf_led2(AFE4490RF::RF_100K),
      _afe_tia_cf_led2(kAFE_CF_CODE_DEFAULT),
      _afe_stg2_rg_led2(AFE4490RG::RG_100K),
      _afe_stg2_en_led2(true),
      _afe_ambdac_uA(0),
      _ppgdisp_channel(AFE4490Channel::LED1),
      _ppgdisp_bpf(),
      _hr1_ma_len(0), _hr1_ma_idx(0), _hr1_ma_sum(0.0f),
      _hr1_refractory_samples(0),
      _hr1_dc_alpha(0.0f),
      _spo2_ch_ir_ema{},
      _spo2_ch_red_ema{},
      _spo2_bp_ir(), _spo2_bp_red(),
      _spo2_reg_x_prev(0.0f), _spo2_reg_y_prev(0.0f), _spo2_reg_count(0),
      _spo2_block_clock(),
      _spo2_blk_sxy(0.0f), _spo2_blk_sxx(0.0f), _spo2_blk_syy(0.0f),
      _spo2_blk_var_ir(0.0f), _spo2_blk_var_red(0.0f),
      _spo2_sxy_sum(0.0f), _spo2_sxx_sum(0.0f), _spo2_syy_sum(0.0f),
      _spo2_var_ir_sum(0.0f), _spo2_var_red_sum(0.0f),
      _spo2_buf_n(1), _spo2_buf_idx(0), _spo2_buf_count(0),
      _spo2_r_curve_a(spo2_r_curve_a_default), _spo2_r_curve_b(spo2_r_curve_b_default),
      _pi_bpf(), _pi_block_clock(),
      _pi_ac_sum(0.0f), _pi_dc_sum(0.0f), _pi_ac_block_sum(0.0f), _pi_dc_block_sum(0.0f),
      _pi_buf_n(1), _pi_buf_idx(0), _pi_buf_count(0),
      _hr1_dc(0.0f),
      _hr1_running_max(0.0f), _hr1_ppg_above_thresh(false),
      _hr1_last_peak_idx(0), _hr1_sample_idx(0),
      _hr1_interval_count(0),
      _hr2_bpf(),
      _hr2_buf_idx(0), _hr2_buf_count(0), _hr2_update_counter(0),
      _hr3_lp(),
      _hr3_buf_idx(0), _hr3_buf_count(0), _hr3_update_counter(0),
      _spo2_ema_mean_tau_s(spo2_ema_mean_tau_s),
      _spo2_window_s(spo2_window_s),
      _pi_window_s(pi_window_s),
      _spo2_min(spo2_min),
      _spo2_max(spo2_max),
      _spo2_pi_sqi_lo(spo2_pi_sqi_lo),
      _spo2_pi_sqi_hi(spo2_pi_sqi_hi),
      _spo2_pi_max(spo2_pi_max),
      _spo2_r_method(spo2_r_method_default),
      _spo2_r_method2_reg_corr_min(spo2_r_method2_reg_corr_min),
      _spo2_pulse_gate_enable(spo2_pulse_gate_enable),
      _hr1_dc_tau_s(hr1_dc_tau_s),
      _hr1_lp_cutoff_hz(hr1_lp_cutoff_hz), _hr1_max_decay_tau_s(hr1_max_decay_tau_s),
      _hr1_threshold_fraction(hr1_threshold_fraction),
      _hr1_max_decay_alpha(0.0f),
      _hr1_sqi_cv_max(hr1_sqi_cv_max),
      _hr2_min_corr(hr2_min_corr),
      _hr2_update_interval_s(0.5f), _hr2_update_interval(25u),
      _hr3_update_interval_s(0.5f), _hr3_update_interval(25u),
      _hr_min_bpm(hr_min_bpm),
      _hr_max_bpm(hr_max_bpm),
      _rsqm_probe_state(ProbeState::PROBE_DISCONNECTED),
      _switched_rc_settling_countdown(0),
      _hgac_rf_change_count(0), _hgac_rf_change_ts_us(0),
      _debug_enabled(false)
{
    // Quantize initial LED currents to the DAC grid (range already set in init list)
    _afe_led1_current_mA = _quantize_led_mA(_afe_led1_current_mA);
    _afe_led2_current_mA = _quantize_led_mA(_afe_led2_current_mA);
    memset(_hr1_ma_buf, 0, sizeof(_hr1_ma_buf));
    _hr1_ma_idx = 0; _hr1_ma_sum = 0.0f;
    memset(_hr1_intervals, 0, sizeof(_hr1_intervals));
    memset(_hr2_buf, 0, sizeof(_hr2_buf));
    memset(_hr3_buf, 0, sizeof(_hr3_buf));
    memset(_pi_ac_buf, 0, sizeof(_pi_ac_buf));
    memset(_pi_dc_buf, 0, sizeof(_pi_dc_buf));
    _current_data = AFE4490Data{};
    _ppgdisp_bpf.f_low = 0.5f;  _ppgdisp_bpf.f_high = 20.0f;
    _hr2_bpf.f_low     = 0.5f;  _hr2_bpf.f_high     =  5.0f;
    _pi_bpf.f_low      = pi_bp_f_low_hz;  _pi_bpf.f_high = pi_bp_f_high_hz;
    _hr3_lp.f_high     = 15.0f;   // low-pass only (v0.83); 4th order since v0.97
    _copy_spo2_r_id(_spo2_r_curve_id, spo2_r_curve_id_default);
    _copy_spo2_r_id(_spo2_r_curve_r_method_id, spo2_r_curve_r_method_id_default);
    _recalc_rate_params();
    _reset_algorithms();
}

// Copies an R method / R curve id into a SPO2_R_ID_LEN buffer. Returns false, leaving dst
// untouched, if src is null, empty or would not fit with its terminator.
bool INCUNEST_AFE4490::_copy_spo2_r_id(char* dst, const char* src) {
    if (src == nullptr) return false;
    size_t n = strnlen(src, SPO2_R_ID_LEN);
    if (n == 0 || n >= SPO2_R_ID_LEN) return false;
    memcpy(dst, src, n + 1);
    return true;
}

INCUNEST_AFE4490::~INCUNEST_AFE4490() {
    if (_task_handle)      { vTaskDelete(_task_handle);      _task_handle      = nullptr; }
    if (_hr2_task_handle)  { vTaskDelete(_hr2_task_handle);  _hr2_task_handle  = nullptr; }
    if (_hr3_task_handle)  { vTaskDelete(_hr3_task_handle);  _hr3_task_handle  = nullptr; }
    if (_data_queue)       vQueueDelete(_data_queue);
    if (_drdy_sem)         vSemaphoreDelete(_drdy_sem);
    if (_hr2_compute_sem)     vSemaphoreDelete(_hr2_compute_sem);
    if (_hr3_compute_sem)     vSemaphoreDelete(_hr3_compute_sem);
    if (_spi_mutex)        vSemaphoreDelete(_spi_mutex);
    if (_state_mutex)      vSemaphoreDelete(_state_mutex);
    if (_g_instance == this) _g_instance = nullptr;
}

// ── begin() ───────────────────────────────────────────────────────────────────
// Requires the SPI bus to be initialised beforehand by the application (SPI.begin() under
// Arduino, spi_bus_initialize() under ESP-IDF). This library intentionally never initialises
// the bus, to avoid reinitialising it under other devices; it only runs its own register
// transactions through incunest_afe4490_hal_spi_transfer().
void INCUNEST_AFE4490::begin(int pin_cs, int pin_drdy, bool debug) {
    _pin_cs        = pin_cs;
    _pin_drdy      = pin_drdy;
    _debug_enabled = debug;
    _g_instance    = this;

    incunest_afe4490_hal_gpio_init_output(_pin_cs, true);   // CS idles high

    const size_t queue_item_size = debug ? sizeof(DebugQueueItem) : sizeof(AFE4490Data);
    _drdy_sem    = xSemaphoreCreateBinary();
    _spi_mutex   = xSemaphoreCreateMutex();
    _state_mutex = xSemaphoreCreateMutex();
    _data_queue  = xQueueCreate(INCUNEST_AFE4490_QUEUE_SIZE, queue_item_size);
    _hr2_compute_sem = xSemaphoreCreateBinary();
    _hr3_compute_sem = xSemaphoreCreateBinary();

    if (!_drdy_sem || !_spi_mutex || !_state_mutex || !_data_queue ||
        !_hr2_compute_sem || !_hr3_compute_sem) {
        ESP_LOGE(TAG, "FreeRTOS object creation failed");
        return;
    }

    if (!incunest_afe4490_hal_spi_attach()) {
        ESP_LOGE(TAG, "SPI attach failed");
        return;
    }

    xSemaphoreTake(_spi_mutex, portMAX_DELAY);
    _chip_init();
    xSemaphoreGive(_spi_mutex);

    _initialized = true;

    _spo2_warn_if_curve_mismatch();

    incunest_afe4490_hal_gpio_init_input_pullup(_pin_drdy);
    if (!incunest_afe4490_hal_gpio_attach_isr_rising(_pin_drdy, _drdy_isr_static)) {
        ESP_LOGE(TAG, "DRDY interrupt attach failed");   // Arduino: cannot happen; ESP-IDF: ISR service error
    }

    xTaskCreatePinnedToCore(
        _task_trampoline,     "incunest_afe4490",
        INCUNEST_AFE4490_TASK_STACK, this,
        INCUNEST_AFE4490_TASK_PRIORITY, &_task_handle, 1);

    xTaskCreatePinnedToCore(
        _hr2_task_trampoline, "incunest_hr2",
        INCUNEST_AFE4490_HR2_TASK_STACK, this,
        INCUNEST_AFE4490_HR23_TASK_PRIORITY, &_hr2_task_handle, 1);

    xTaskCreatePinnedToCore(
        _hr3_task_trampoline, "incunest_hr3",
        INCUNEST_AFE4490_HR3_TASK_STACK, this,
        INCUNEST_AFE4490_HR23_TASK_PRIORITY, &_hr3_task_handle, 1);

    xTaskCreatePinnedToCore(
        _diag_task_trampoline, "incunest_diag",
        INCUNEST_AFE4490_DIAG_TASK_STACK, this,
        INCUNEST_AFE4490_DIAG_TASK_PRIORITY, &_diag_task_handle, 1);

    ESP_LOGI(TAG, "Started: PRF=%u Hz, NUMAV=%u", _afe_sample_rate_hz, _afe_adc_averages);
}

// ── Configuration setters ─────────────────────────────────────────────────────
// Acceptance policy for sample rates — the ONLY place it lives. setSampleRate(uint16_t) defers
// here, so widening the catalogue into a range (e.g. once a FractionalDecimator removes the
// integer-decimation-factor requirement) is a change to this predicate and to the table, not to
// any public signature.
bool INCUNEST_AFE4490::isValidSampleRateHz(uint16_t hz, AFE4490SampleRate* out) {
    for (uint8_t i = 0; i < kAFE_SAMPLE_RATE_COUNT; i++) {
        if (kAFE_SAMPLE_RATE_HZ[i] == hz) {
            if (out) *out = (AFE4490SampleRate)i;
            return true;
        }
    }
    return false;
}

// Numeric form: rejects anything outside the validated catalogue. Previously this accepted
// 63-5000 Hz while checking only the numeric bounds, which let a legal call write inverted
// timer-compare pairs into the chip above ~2487 Hz (see spec §7.4).
void INCUNEST_AFE4490::setSampleRate(uint16_t hz) {
    AFE4490SampleRate rate;
    if (!isValidSampleRateHz(hz, &rate)) {
        ESP_LOGE(TAG, "setSampleRate: %u Hz is not a validated rate; supported: "
                      "500, 800, 1000, 1250, 1600 Hz (rate unchanged)", hz);
        return;
    }
    setSampleRate(rate);
}

// Changing the PRF is not just a constant recalculation: it invalidates every piece of state
// that was accumulated at the previous rate, and it rewrites all four sampling windows in the
// chip. Hence the full teardown below. See spec §7.4 "Changing the rate at runtime".
//
// LOCK ORDER: _spi_mutex, then _state_mutex. This is the only place in the library that holds
// both at once (getConfig() and _task_body() take them sequentially, never nested), so this
// establishes the order — do not invert it anywhere. Holding _state_mutex across the whole
// mutation is what makes the sequence atomic against _process_sample(), which in turn is why
// settling can be armed AFTER _reset_algorithms() here instead of before the mutation as the
// other setters must do (§5.8.4): with no reader able to observe an intermediate state, the
// cross-thread race the arm-before-mutate contract guards against cannot occur.
void INCUNEST_AFE4490::setSampleRate(AFE4490SampleRate rate) {
    const uint16_t hz = sampleRateHz(rate);
    if (hz == _afe_sample_rate_hz) return;   // no-op: never pay a teardown for a no-change call

    if (_initialized) xSemaphoreTake(_spi_mutex, portMAX_DELAY);
    if (_initialized) xSemaphoreTake(_state_mutex, portMAX_DELAY);

    _afe_sample_rate_hz = hz;

    // Maximum averages = floor(T_conv_window / T_conv_min) = floor(PRP/4 / 50µs) where PRP (Pulse Repetition Period)
    // With PRF 500 Hz then PRP is 2000 µs and max_averages = 10 (NUMAV = 9 = 10-1)
    // Hardware field limit: NUMAV ≤ 15 (16 averages max, datasheet CONTROL1 bits [7:0])
    uint8_t numav_max = (uint8_t)((5000u / hz) - 1u);
    if (numav_max > 15) numav_max = 15;

    if ((_afe_adc_averages - 1u) > numav_max) {
        uint8_t clamped = numav_max + 1u;
        ESP_LOGE(TAG, "setSampleRate: num_averages clamped %u→%u at %u Hz",
                 _afe_adc_averages, clamped, hz);
        _afe_adc_averages = clamped;
    }

    _recalc_rate_params();

    // Everything accumulated at the old rate is now wrong, and recalculating coefficients does
    // not fix it:
    //   * the biquads get new coefficients on top of _v1/_v2 from the old rate — init_bp() and
    //     init_lp() explicitly leave state untouched;
    //   * EmaChannel::init() recomputes α but keeps the mean/var accumulated with the old one;
    //   * _hr2_buf / _hr3_buf hold samples decimated at the old rate, so the 8 s / 10.24 s
    //     analysis windows would mix two rates until they refill;
    //   * the decimation phase counters are mid-cycle for a factor that just changed.
    // _reset_algorithms() clears all of it (it is the same teardown begin() performs). It also
    // recomputes the Hann window, which does not depend on the rate — a few hundred µs of
    // redundant work, paid once per configuration change and cheaper than maintaining a
    // partial-reset variant that could drift out of step with this list.
    _reset_algorithms();

    // AFTER the reset, which ends by clearing the countdown. Changing the PRF rewrites all four
    // sampling windows via _apply_timing_regs(), a signal-chain change in the sense of the
    // datasheet's §7.7 t5 — the same reason RF, CF, ILED, AMBDAC, RG and Stage2En arm it.
    _arm_switched_rc_settling();

    if (_initialized) {
        _apply_timing_regs();
        _apply_control_regs();
        xSemaphoreGive(_state_mutex);
        xSemaphoreGive(_spi_mutex);
    }
}

void INCUNEST_AFE4490::setAdcAverages(uint8_t num) {
    if (num == 0) num = 1;

    uint8_t numav_max = (uint8_t)((5000u / _afe_sample_rate_hz) - 1u);
    if (numav_max > 15) numav_max = 15;

    if ((uint8_t)(num - 1u) > numav_max) {
        uint8_t clamped = numav_max + 1u;
        ESP_LOGE(TAG, "setAdcAverages: %u clamped to %u (max at %u Hz)",
                 num, clamped, _afe_sample_rate_hz);
        num = clamped;
    }

    if (_initialized) xSemaphoreTake(_spi_mutex, portMAX_DELAY);
    _afe_adc_averages = num;
    if (_initialized) {
        _apply_control_regs();
        xSemaphoreGive(_spi_mutex);
    }
}

float INCUNEST_AFE4490::_quantize_led_mA(float mA) const {
    float full_scale_mA = (float)_afe_led_range_mA;
    uint8_t code = (uint8_t)clampf(roundf((mA / full_scale_mA) * 256.0f), 0.0f, 255.0f);
    return (code / 256.0f) * full_scale_mA;
}

void INCUNEST_AFE4490::setLED1Current(float mA) {
    if (_initialized) xSemaphoreTake(_spi_mutex, portMAX_DELAY);
    float new_val = _quantize_led_mA(clampf(mA, 0.0f, (float)_afe_led_range_mA));
    bool changed = (new_val != _afe_led1_current_mA);
    // Arm BEFORE mutating: closes the cross-thread race with _process_sample() (spec §5.8.4) —
    // otherwise a reader on another core could observe the new value with the countdown still 0.
    if (changed) _arm_switched_rc_settling();  // datasheet §7.7 t5 footnote (1) names "LED current setting" explicitly
    _afe_led1_current_mA = new_val;
    if (_initialized) {
        _apply_analog_regs();
        xSemaphoreGive(_spi_mutex);
    }
}

void INCUNEST_AFE4490::setLED2Current(float mA) {
    if (_initialized) xSemaphoreTake(_spi_mutex, portMAX_DELAY);
    float new_val = _quantize_led_mA(clampf(mA, 0.0f, (float)_afe_led_range_mA));
    bool changed = (new_val != _afe_led2_current_mA);
    // Arm before mutating — closes the cross-thread race with _process_sample(), spec §5.8.4.
    if (changed) _arm_switched_rc_settling();  // datasheet §7.7 t5 footnote (1) names "LED current setting" explicitly
    _afe_led2_current_mA = new_val;
    if (_initialized) {
        _apply_analog_regs();
        xSemaphoreGive(_spi_mutex);
    }
}

void INCUNEST_AFE4490::setLEDRange(uint8_t mA) {
    if (mA != 75 && mA != 150) {
        ESP_LOGE(TAG, "setLEDRange: must be 75 or 150 mA");
        return;
    }
    if (_initialized) xSemaphoreTake(_spi_mutex, portMAX_DELAY);
    _afe_led_range_mA = mA;
    // Re-quantize LED currents to the new range's DAC grid
    _afe_led1_current_mA = _quantize_led_mA(_afe_led1_current_mA);
    _afe_led2_current_mA = _quantize_led_mA(_afe_led2_current_mA);
    if (_initialized) {
        _apply_analog_regs();
        xSemaphoreGive(_spi_mutex);
    }
}

// Joint setters — apply the same value to both LED1 and LED2 channels
void INCUNEST_AFE4490::setTIAGain(AFE4490RF gain) {
    if (_initialized) xSemaphoreTake(_spi_mutex, portMAX_DELAY);
    bool changed = (gain != _afe_tia_rf_led1) || (gain != _afe_tia_rf_led2);
    // Arm before mutating — closes the cross-thread race with _process_sample(), spec §5.8.4.
    if (changed) _arm_switched_rc_settling();
    _afe_tia_rf_led1 = _afe_tia_rf_led2 = gain;
    _recalc_afe_tia_cf_led1();
    _recalc_afe_tia_cf_led2();
    if (_initialized) {
        _apply_analog_regs();
        xSemaphoreGive(_spi_mutex);
    }
}

void INCUNEST_AFE4490::setTIACF(float pF) {
    if (_initialized) xSemaphoreTake(_spi_mutex, portMAX_DELAY);
    _afe_tia_cf_led1 = _afe_tia_cf_led2 = afeCFPFToCode(pF);
    if (_initialized) {
        _apply_analog_regs();
        xSemaphoreGive(_spi_mutex);
    }
}

void INCUNEST_AFE4490::setStage2Gain(AFE4490RG gain) {
    if (_initialized) xSemaphoreTake(_spi_mutex, portMAX_DELAY);
    bool changed = (gain != _afe_stg2_rg_led1) || (gain != _afe_stg2_rg_led2);
    // Arm before mutating — closes the cross-thread race with _process_sample(), spec §5.8.4.
    if (changed) _arm_switched_rc_settling();  // RG is part of the same feedback path as RF
    _afe_stg2_rg_led1 = _afe_stg2_rg_led2 = gain;
    if (_initialized) {
        _apply_analog_regs();
        xSemaphoreGive(_spi_mutex);
    }
}

// Separate gain mode
void INCUNEST_AFE4490::setEnSepGain(bool enable) {
    if (_initialized) xSemaphoreTake(_spi_mutex, portMAX_DELAY);
    _afe_sep_tia_en = enable;
    if (_initialized) {
        _apply_analog_regs();
        xSemaphoreGive(_spi_mutex);
    }
}

// Per-channel setters — LED1 (IR)
void INCUNEST_AFE4490::setTIAGainLED1(AFE4490RF gain) {
    if (_initialized) xSemaphoreTake(_spi_mutex, portMAX_DELAY);
    bool changed = (gain != _afe_tia_rf_led1);
    // Arm before mutating — closes the cross-thread race with _process_sample(), spec §5.8.4.
    if (changed) _arm_switched_rc_settling();
    _afe_tia_rf_led1 = gain;
    _recalc_afe_tia_cf_led1();
    if (_initialized) {
        _apply_analog_regs();
        xSemaphoreGive(_spi_mutex);
    }
}

void INCUNEST_AFE4490::setTIACFLED1(float pF) {
    if (_initialized) xSemaphoreTake(_spi_mutex, portMAX_DELAY);
    _afe_tia_cf_led1 = afeCFPFToCode(pF);
    if (_initialized) {
        _apply_analog_regs();
        xSemaphoreGive(_spi_mutex);
    }
}

void INCUNEST_AFE4490::setStage2GainLED1(AFE4490RG gain) {
    if (_initialized) xSemaphoreTake(_spi_mutex, portMAX_DELAY);
    bool changed = (gain != _afe_stg2_rg_led1);
    // Arm before mutating — closes the cross-thread race with _process_sample(), spec §5.8.4.
    if (changed) _arm_switched_rc_settling();
    _afe_stg2_rg_led1 = gain;
    if (_initialized) {
        _apply_analog_regs();
        xSemaphoreGive(_spi_mutex);
    }
}

// Per-channel setters — LED2 (RED)
void INCUNEST_AFE4490::setTIAGainLED2(AFE4490RF gain) {
    if (_initialized) xSemaphoreTake(_spi_mutex, portMAX_DELAY);
    bool changed = (gain != _afe_tia_rf_led2);
    // Arm before mutating — closes the cross-thread race with _process_sample(), spec §5.8.4.
    if (changed) _arm_switched_rc_settling();
    _afe_tia_rf_led2 = gain;
    _recalc_afe_tia_cf_led2();
    if (_initialized) {
        _apply_analog_regs();
        xSemaphoreGive(_spi_mutex);
    }
}

void INCUNEST_AFE4490::setTIACFLED2(float pF) {
    if (_initialized) xSemaphoreTake(_spi_mutex, portMAX_DELAY);
    _afe_tia_cf_led2 = afeCFPFToCode(pF);
    if (_initialized) {
        _apply_analog_regs();
        xSemaphoreGive(_spi_mutex);
    }
}

void INCUNEST_AFE4490::setStage2GainLED2(AFE4490RG gain) {
    if (_initialized) xSemaphoreTake(_spi_mutex, portMAX_DELAY);
    bool changed = (gain != _afe_stg2_rg_led2);
    // Arm before mutating — closes the cross-thread race with _process_sample(), spec §5.8.4.
    if (changed) _arm_switched_rc_settling();
    _afe_stg2_rg_led2 = gain;
    if (_initialized) {
        _apply_analog_regs();
        xSemaphoreGive(_spi_mutex);
    }
}

void INCUNEST_AFE4490::setStage2En1(bool en) {
    if (_initialized) xSemaphoreTake(_spi_mutex, portMAX_DELAY);
    bool changed = (en != _afe_stg2_en_led1);
    // Bypassing Stage 2 steps the effective gain between unity and RG, same as setStage2Gain().
    // Arm before mutating — closes the cross-thread race with _process_sample(), spec §5.8.4.
    if (changed) _arm_switched_rc_settling();
    _afe_stg2_en_led1 = en;
    if (_initialized) {
        _apply_analog_regs();
        xSemaphoreGive(_spi_mutex);
    }
}

void INCUNEST_AFE4490::setStage2En2(bool en) {
    if (_initialized) xSemaphoreTake(_spi_mutex, portMAX_DELAY);
    bool changed = (en != _afe_stg2_en_led2);
    // Bypassing Stage 2 steps the effective gain between unity and RG, same as setStage2Gain().
    // Arm before mutating — closes the cross-thread race with _process_sample(), spec §5.8.4.
    if (changed) _arm_switched_rc_settling();
    _afe_stg2_en_led2 = en;
    if (_initialized) {
        _apply_analog_regs();
        xSemaphoreGive(_spi_mutex);
    }
}

void INCUNEST_AFE4490::setAmbDac(uint8_t uA) {
    if (uA > 10) uA = 10;
    if (_initialized) xSemaphoreTake(_spi_mutex, portMAX_DELAY);
    bool changed = (uA != _afe_ambdac_uA);
    // Arm before mutating — closes the cross-thread race with _process_sample(), spec §5.8.4.
    if (changed) _arm_switched_rc_settling();  // same feedback path as RF/CF — datasheet's "and so forth"
    _afe_ambdac_uA = uA;
    if (_initialized) {
        _apply_analog_regs();
        xSemaphoreGive(_spi_mutex);
    }
}

void INCUNEST_AFE4490::setPPGChannel(AFE4490Channel channel) {
    if (_initialized) xSemaphoreTake(_state_mutex, portMAX_DELAY);
    _ppgdisp_channel = channel;
    // Reset filter state: changing channel means a different signal enters the filter
    _ppgdisp_bpf.reset();
    if (_initialized) xSemaphoreGive(_state_mutex);
}

void INCUNEST_AFE4490::setPPGDispFilter(float f_low_hz, float f_high_hz) {
    if (_initialized) xSemaphoreTake(_state_mutex, portMAX_DELAY);
    _ppgdisp_bpf.init_bp(f_low_hz, f_high_hz, (float)_afe_sample_rate_hz);
    _ppgdisp_bpf.reset();
    if (_initialized) xSemaphoreGive(_state_mutex);
}

void INCUNEST_AFE4490::setHR2Filter(float f_low_hz, float f_high_hz) {
    if (_initialized) xSemaphoreTake(_state_mutex, portMAX_DELAY);
    _hr2_bpf.init_bp(f_low_hz, f_high_hz, (float)_afe_sample_rate_hz);
    _hr2_bpf.reset();
    if (_initialized) xSemaphoreGive(_state_mutex);
}

void INCUNEST_AFE4490::setPIFilter(float f_low_hz, float f_high_hz) {
    if (_initialized) xSemaphoreTake(_state_mutex, portMAX_DELAY);
    _pi_bpf.init_bp(f_low_hz, f_high_hz, (float)_afe_sample_rate_hz);
    _pi_bpf.reset();
    if (_initialized) xSemaphoreGive(_state_mutex);
}

void INCUNEST_AFE4490::setHR3Filter(float f_high_hz) {
    if (_initialized) xSemaphoreTake(_state_mutex, portMAX_DELAY);
    _hr3_lp.init(f_high_hz, (float)_afe_sample_rate_hz);
    _hr3_lp.reset();
    if (_initialized) xSemaphoreGive(_state_mutex);
}

bool INCUNEST_AFE4490::setSpO2RCurve(const char* id, const char* r_method_id, float a, float b) {
    char new_id[SPO2_R_ID_LEN];
    char new_method[SPO2_R_ID_LEN];
    if (!_copy_spo2_r_id(new_id, id) || !_copy_spo2_r_id(new_method, r_method_id)
        || !std::isfinite(a) || !std::isfinite(b)) {
        ESP_LOGE(TAG, "setSpO2RCurve rejected (empty/oversized id or non-finite coefficient); "
                 "curve unchanged");
        return false;
    }
    if (_initialized) xSemaphoreTake(_state_mutex, portMAX_DELAY);
    memcpy(_spo2_r_curve_id, new_id, sizeof(new_id));
    memcpy(_spo2_r_curve_r_method_id, new_method, sizeof(new_method));
    _spo2_r_curve_a = a;
    _spo2_r_curve_b = b;
    if (_initialized) xSemaphoreGive(_state_mutex);
    _spo2_warn_if_curve_mismatch();
    return true;
}

void INCUNEST_AFE4490::_spo2_warn_if_curve_mismatch() const {
    if (strcmp(_spo2_r_curve_r_method_id, _spo2_r_method_id()) != 0) {
        ESP_LOGW(TAG, "R curve %s was fitted with %s, this library computes R as %s: SpO2 is "
                 "shown with a curve for another R method", _spo2_r_curve_id,
                 _spo2_r_curve_r_method_id, _spo2_r_method_id());
    }
}

void INCUNEST_AFE4490::setSpO2RMethod(uint8_t method) {
    if (method != 1 && method != 2) {
        ESP_LOGE(TAG, "setSpO2RMethod(%u) rejected: 1 or 2; method unchanged", (unsigned)method);
        return;
    }
    if (_initialized) xSemaphoreTake(_state_mutex, portMAX_DELAY);
    _spo2_r_method = method;
    if (_initialized) xSemaphoreGive(_state_mutex);
    _spo2_warn_if_curve_mismatch();
}

void INCUNEST_AFE4490::setSpO2RMethod2RegCorrMin(float r) {
    if (_initialized) xSemaphoreTake(_state_mutex, portMAX_DELAY);
    _spo2_r_method2_reg_corr_min = r;
    if (_initialized) xSemaphoreGive(_state_mutex);
}

void INCUNEST_AFE4490::setSpO2PulseGateEnable(bool on) {
    if (_initialized) xSemaphoreTake(_state_mutex, portMAX_DELAY);
    _spo2_pulse_gate_enable = on;
    if (_initialized) xSemaphoreGive(_state_mutex);
}

void INCUNEST_AFE4490::setSpO2EmaMeanTauS(float tau_s) {
    if (_initialized) xSemaphoreTake(_state_mutex, portMAX_DELAY);
    _spo2_ema_mean_tau_s = tau_s;
    _recalc_rate_params();
    if (_initialized) xSemaphoreGive(_state_mutex);
}

// Window lengths (v0.102): clamped to [alg_window_s_min, alg_window_s_max]; the buffer length
// follows in _recalc_rate_params(), which empties the window when it changes (a half-filled
// window of another length is not a valid estimate — NaN for one window length instead).
void INCUNEST_AFE4490::setSpO2WindowS(float window_s) {
    if (_initialized) xSemaphoreTake(_state_mutex, portMAX_DELAY);
    _spo2_window_s = clampf(window_s, alg_window_s_min, alg_window_s_max);
    _recalc_rate_params();
    if (_initialized) xSemaphoreGive(_state_mutex);
}

void INCUNEST_AFE4490::setPIWindowS(float window_s) {
    if (_initialized) xSemaphoreTake(_state_mutex, portMAX_DELAY);
    _pi_window_s = clampf(window_s, alg_window_s_min, alg_window_s_max);
    _recalc_rate_params();
    if (_initialized) xSemaphoreGive(_state_mutex);
}

void INCUNEST_AFE4490::setSpO2Range(float min_pct, float max_pct) {
    if (_initialized) xSemaphoreTake(_state_mutex, portMAX_DELAY);
    _spo2_min = min_pct;
    _spo2_max = max_pct;
    if (_initialized) xSemaphoreGive(_state_mutex);
}

void INCUNEST_AFE4490::setSpO2PiSqiThresholds(float lo_pct, float hi_pct, float max_pct) {
    if (_initialized) xSemaphoreTake(_state_mutex, portMAX_DELAY);
    _spo2_pi_sqi_lo = lo_pct;
    _spo2_pi_sqi_hi = hi_pct;
    _spo2_pi_max    = max_pct;
    if (_initialized) xSemaphoreGive(_state_mutex);
}

void INCUNEST_AFE4490::setHR1DcTauS(float tau_s) {
    if (_initialized) xSemaphoreTake(_state_mutex, portMAX_DELAY);
    _hr1_dc_tau_s = tau_s;
    _recalc_rate_params();
    if (_initialized) xSemaphoreGive(_state_mutex);
}

void INCUNEST_AFE4490::setHR1LpCutoffHz(float hz) {
    if (_initialized) xSemaphoreTake(_state_mutex, portMAX_DELAY);
    _hr1_lp_cutoff_hz = hz;
    _recalc_rate_params();
    if (_initialized) xSemaphoreGive(_state_mutex);
}

void INCUNEST_AFE4490::setHR1SqiCvMax(float cv) {
    if (_initialized) xSemaphoreTake(_state_mutex, portMAX_DELAY);
    _hr1_sqi_cv_max = cv;
    if (_initialized) xSemaphoreGive(_state_mutex);
}

void INCUNEST_AFE4490::setHR2MinCorr(float corr) {
    if (_initialized) xSemaphoreTake(_state_mutex, portMAX_DELAY);
    _hr2_min_corr = corr;
    if (_initialized) xSemaphoreGive(_state_mutex);
}

void INCUNEST_AFE4490::setHR2UpdateIntervalS(float s) {
    if (_initialized) xSemaphoreTake(_state_mutex, portMAX_DELAY);
    _hr2_update_interval_s = s;
    _recalc_rate_params();
    if (_initialized) xSemaphoreGive(_state_mutex);
}

// Detection threshold as a fraction of the running max. Clamped to (0, 1]: at 0 every sample
// crosses and at >1 nothing ever does, so both extremes silence HR1 rather than degrade it.
void INCUNEST_AFE4490::setHR1ThresholdFraction(float frac) {
    if (frac <= 0.0f || frac > 1.0f) {
        ESP_LOGE(TAG, "setHR1ThresholdFraction: %.3f out of (0, 1] — ignored", frac);
        return;
    }
    if (_initialized) xSemaphoreTake(_state_mutex, portMAX_DELAY);
    _hr1_threshold_fraction = frac;
    if (_initialized) xSemaphoreGive(_state_mutex);
}

void INCUNEST_AFE4490::setHR1MaxDecayTauS(float tau_s) {
    if (_initialized) xSemaphoreTake(_state_mutex, portMAX_DELAY);
    _hr1_max_decay_tau_s = tau_s;
    _recalc_rate_params();
    if (_initialized) xSemaphoreGive(_state_mutex);
}

void INCUNEST_AFE4490::setHR3UpdateIntervalS(float s) {
    if (_initialized) xSemaphoreTake(_state_mutex, portMAX_DELAY);
    _hr3_update_interval_s = s;
    _recalc_rate_params();
    if (_initialized) xSemaphoreGive(_state_mutex);
}

void INCUNEST_AFE4490::setHRValidRange(float min_bpm, float max_bpm) {
    if (_initialized) xSemaphoreTake(_state_mutex, portMAX_DELAY);
    _hr_min_bpm = min_bpm;
    _hr_max_bpm = max_bpm;
    if (_initialized) xSemaphoreGive(_state_mutex);
}

// ── RSQM algorithm setters ────────────────────────────────────────────────────
void INCUNEST_AFE4490::setRsqmOtThr(float v) {
    if (_initialized) xSemaphoreTake(_state_mutex, portMAX_DELAY);
    rsqm_ot_thr = v;
    if (_initialized) xSemaphoreGive(_state_mutex);
}

void INCUNEST_AFE4490::setRsqmDisconnLedSubThr(float v) {
    if (_initialized) xSemaphoreTake(_state_mutex, portMAX_DELAY);
    rsqm_disconn_led_sub_thr = v;
    if (_initialized) xSemaphoreGive(_state_mutex);
}

void INCUNEST_AFE4490::setRsqmDisconnIPdThr(float v) {
    if (_initialized) xSemaphoreTake(_state_mutex, portMAX_DELAY);
    rsqm_disconn_i_pd_thr = v;
    if (_initialized) xSemaphoreGive(_state_mutex);
}

void INCUNEST_AFE4490::setRsqmProbeStateMinS(float s) {
    if (_initialized) xSemaphoreTake(_state_mutex, portMAX_DELAY);
    rsqm_probe_state_min_s = s;
    _recalc_rate_params();
    if (_initialized) xSemaphoreGive(_state_mutex);
}

// ── HGAC setters (Phase 1: RF-only descent) ───────────────────────────────────
void INCUNEST_AFE4490::setHgacEnable(bool en) {
    if (_initialized) xSemaphoreTake(_state_mutex, portMAX_DELAY);
    hgac_enable = en;
    if (_initialized) xSemaphoreGive(_state_mutex);
}

void INCUNEST_AFE4490::setSettleFreezeEnable(bool en) {
    if (_initialized) xSemaphoreTake(_state_mutex, portMAX_DELAY);
    afe_settle_freeze_enable = en;
    if (_initialized) xSemaphoreGive(_state_mutex);
}

void INCUNEST_AFE4490::setHgacVTiaHigh2(float v) {
    if (_initialized) xSemaphoreTake(_state_mutex, portMAX_DELAY);
    // Keep the guard on the operating axis: it must trip BELOW the TIA full-scale
    // (tia_axis::FS_V) so HGAC acts before the reading turns unreliable — never above it.
    hgac_v_tia_high2 = clampf(v, 0.0f, tia_axis::FS_V);
    if (_initialized) xSemaphoreGive(_state_mutex);
}

void INCUNEST_AFE4490::setHgacVTiaHigh1(float v) {
    if (_initialized) xSemaphoreTake(_state_mutex, portMAX_DELAY);
    hgac_v_tia_high1 = v;
    if (_initialized) xSemaphoreGive(_state_mutex);
}

void INCUNEST_AFE4490::setHgacVTiaLow1(float v) {
    if (_initialized) xSemaphoreTake(_state_mutex, portMAX_DELAY);
    hgac_v_tia_low1 = v;
    if (_initialized) xSemaphoreGive(_state_mutex);
}

void INCUNEST_AFE4490::setHgacEmaFastTauS(float s) {
    if (_initialized) xSemaphoreTake(_state_mutex, portMAX_DELAY);
    hgac_ema_fast_tau_s = s;
    _recalc_rate_params();
    if (_initialized) xSemaphoreGive(_state_mutex);
}

void INCUNEST_AFE4490::setHgacEmaSlowTauS(float s) {
    if (_initialized) xSemaphoreTake(_state_mutex, portMAX_DELAY);
    hgac_ema_slow_tau_s = s;
    _recalc_rate_params();
    if (_initialized) xSemaphoreGive(_state_mutex);
}

void INCUNEST_AFE4490::setHgacEmaAmbientTauS(float s) {
    if (_initialized) xSemaphoreTake(_state_mutex, portMAX_DELAY);
    hgac_ema_ambient_tau_s = s;
    _recalc_rate_params();
    if (_initialized) xSemaphoreGive(_state_mutex);
}

// ── getConfig() ───────────────────────────────────────────────────────────────
AFE4490Config INCUNEST_AFE4490::getConfig() {
    AFE4490Config cfg;

    // SPI-mutex fields: sample rate, averages, LED currents, analog front-end settings
    if (_initialized) xSemaphoreTake(_spi_mutex, portMAX_DELAY);
    cfg.afe_sample_rate_hz  = _afe_sample_rate_hz;
    cfg.afe_adc_averages    = _afe_adc_averages;
    cfg.afe_led1_current_mA = _afe_led1_current_mA;
    cfg.afe_led2_current_mA = _afe_led2_current_mA;
    cfg.afe_led_range_mA    = _afe_led_range_mA;
    cfg.afe_sep_tia_en        = _afe_sep_tia_en;
    cfg.afe_tia_rf_led1    = _afe_tia_rf_led1;
    cfg.afe_tia_cf_led1_code = _afe_tia_cf_led1;
    cfg.afe_tia_cf_led1_pF   = afeCFCodeToPF(_afe_tia_cf_led1);
    cfg.afe_stg2_rg_led1 = _afe_stg2_rg_led1;
    cfg.afe_stg2_en_led1       = _afe_stg2_en_led1;
    cfg.afe_tia_rf_led2    = _afe_tia_rf_led2;
    cfg.afe_tia_cf_led2_code = _afe_tia_cf_led2;
    cfg.afe_tia_cf_led2_pF   = afeCFCodeToPF(_afe_tia_cf_led2);
    cfg.afe_stg2_rg_led2 = _afe_stg2_rg_led2;
    cfg.afe_stg2_en_led2       = _afe_stg2_en_led2;
    cfg.afe_ambdac_uA        = _afe_ambdac_uA;
    if (_initialized) xSemaphoreGive(_spi_mutex);

    // state-mutex fields: PPG channel, filters, SpO2 calibration
    if (_initialized) xSemaphoreTake(_state_mutex, portMAX_DELAY);
    cfg.ppgdisp_channel      = _ppgdisp_channel;
    cfg.ppgdisp_f_low_hz     = _ppgdisp_bpf.f_low;
    cfg.ppgdisp_f_high_hz    = _ppgdisp_bpf.f_high;
    cfg.hr2_f_low_hz     = _hr2_bpf.f_low;
    cfg.hr2_f_high_hz    = _hr2_bpf.f_high;
    cfg.hr3_f_high_hz    = _hr3_lp.f_high;
    _copy_spo2_r_id(cfg.spo2_r_method_id, _spo2_r_method_id());
    memcpy(cfg.spo2_r_curve_id, _spo2_r_curve_id, sizeof(_spo2_r_curve_id));
    memcpy(cfg.spo2_r_curve_r_method_id, _spo2_r_curve_r_method_id, sizeof(_spo2_r_curve_r_method_id));
    cfg.spo2_r_curve_method_match = strcmp(_spo2_r_curve_r_method_id, _spo2_r_method_id()) == 0;
    cfg.spo2_r_method               = _spo2_r_method;
    cfg.spo2_r_method2_reg_corr_min = _spo2_r_method2_reg_corr_min;
    cfg.spo2_r_curve_a              = _spo2_r_curve_a;
    cfg.spo2_r_curve_b              = _spo2_r_curve_b;
    cfg.spo2_ema_mean_tau_s     = _spo2_ema_mean_tau_s;
    cfg.spo2_window_s         = _spo2_window_s;
    cfg.pi_window_s           = _pi_window_s;
    cfg.spo2_min              = _spo2_min;
    cfg.spo2_max              = _spo2_max;
    cfg.spo2_pi_sqi_lo        = _spo2_pi_sqi_lo;
    cfg.spo2_pi_sqi_hi        = _spo2_pi_sqi_hi;
    cfg.spo2_pi_max           = _spo2_pi_max;
    cfg.spo2_pulse_gate_enable = _spo2_pulse_gate_enable;
    _copy_spo2_r_id(cfg.pi_method_id, PI_METHOD_ID);
    cfg.pi_bp_f_low_hz        = _pi_bpf.f_low;
    cfg.pi_bp_f_high_hz       = _pi_bpf.f_high;
    cfg.hr1_dc_tau_s          = _hr1_dc_tau_s;
    cfg.hr1_lp_cutoff_hz      = _hr1_lp_cutoff_hz;
    cfg.hr1_max_decay_tau_s   = _hr1_max_decay_tau_s;
    cfg.hr1_threshold_fraction = _hr1_threshold_fraction;
    cfg.hr1_sqi_cv_max        = _hr1_sqi_cv_max;
    cfg.hr2_min_corr          = _hr2_min_corr;
    cfg.hr2_update_interval_s = _hr2_update_interval_s;
    cfg.hr3_update_interval_s = _hr3_update_interval_s;
    cfg.hr_min_bpm            = _hr_min_bpm;
    cfg.hr_max_bpm            = _hr_max_bpm;
    cfg.rsqm_ot_thr              = rsqm_ot_thr;
    cfg.rsqm_disconn_led_sub_thr = rsqm_disconn_led_sub_thr;
    cfg.rsqm_disconn_i_pd_thr    = rsqm_disconn_i_pd_thr;
    cfg.rsqm_probe_state_min_s   = rsqm_probe_state_min_s;
    cfg.hgac_enable          = hgac_enable;
    cfg.afe_settle_freeze_enable = afe_settle_freeze_enable;
    cfg.hgac_v_tia_high2     = hgac_v_tia_high2;
    cfg.hgac_v_tia_high1     = hgac_v_tia_high1;
    cfg.hgac_v_tia_low1      = hgac_v_tia_low1;
    cfg.hgac_ema_fast_tau_s  = hgac_ema_fast_tau_s;
    cfg.hgac_ema_slow_tau_s  = hgac_ema_slow_tau_s;
    cfg.hgac_ema_ambient_tau_s = hgac_ema_ambient_tau_s;
    if (_initialized) xSemaphoreGive(_state_mutex);

    return cfg;
}

// ── getTimingConfig() — reads all timing registers from chip via SPI ──────────
AFE4490TimingConfig INCUNEST_AFE4490::getTimingConfig() {
    AFE4490TimingConfig t = {};
    if (!_initialized) return t;
    xSemaphoreTake(_spi_mutex, portMAX_DELAY);
    // Enable SPI_READ once, read all 28 registers sequentially, then disable.
    _write_reg(REG_CONTROL0, ctrl0_spi_read);
    t.t1  = _read_spi_raw(0x01); t.t2  = _read_spi_raw(0x02);
    t.t3  = _read_spi_raw(0x03); t.t4  = _read_spi_raw(0x04);
    t.t5  = _read_spi_raw(0x05); t.t6  = _read_spi_raw(0x06);
    t.t7  = _read_spi_raw(0x07); t.t8  = _read_spi_raw(0x08);
    t.t9  = _read_spi_raw(0x09); t.t10 = _read_spi_raw(0x0A);
    t.t11 = _read_spi_raw(0x0B); t.t12 = _read_spi_raw(0x0C);
    t.t13 = _read_spi_raw(0x0D); t.t14 = _read_spi_raw(0x0E);
    t.t15 = _read_spi_raw(0x0F); t.t16 = _read_spi_raw(0x10);
    t.t17 = _read_spi_raw(0x11); t.t18 = _read_spi_raw(0x12);
    t.t19 = _read_spi_raw(0x13); t.t20 = _read_spi_raw(0x14);
    t.t21 = _read_spi_raw(0x15); t.t22 = _read_spi_raw(0x16);
    t.t23 = _read_spi_raw(0x17); t.t24 = _read_spi_raw(0x18);
    t.t25 = _read_spi_raw(0x19); t.t26 = _read_spi_raw(0x1A);
    t.t27 = _read_spi_raw(0x1B); t.t28 = _read_spi_raw(0x1C);
    _write_reg(REG_CONTROL0, 0x000000UL);
    xSemaphoreGive(_spi_mutex);
    return t;
}

// ── setTimingReg() — writes one timing register directly to chip ──────────────
void INCUNEST_AFE4490::setTimingReg(uint8_t addr, uint32_t value) {
    if (!_initialized) return;
    xSemaphoreTake(_spi_mutex, portMAX_DELAY);
    _write_reg(addr, value);
    xSemaphoreGive(_spi_mutex);
}

// ── getData() ─────────────────────────────────────────────────────────────────
bool INCUNEST_AFE4490::getData(AFE4490Data& data, AFE4490DebugData* dbg) {
    if (_debug_enabled) {
        DebugQueueItem item;
        const bool ok = xQueueReceive(_data_queue, &item, 0) == pdTRUE;
        if (ok) {
            data = item.data;
            if (dbg) *dbg = item.dbg;
        }
        return ok;
    }
    return xQueueReceive(_data_queue, &data, 0) == pdTRUE;
}

// ── stop() ────────────────────────────────────────────────────────────────────
void INCUNEST_AFE4490::stop() {
    if (!_initialized) return;

    incunest_afe4490_hal_gpio_detach_isr(_pin_drdy);

    // Take mutex to wait for any in-progress SPI transaction to finish
    if (_spi_mutex) xSemaphoreTake(_spi_mutex, portMAX_DELAY);

    if (_task_handle)      { vTaskDelete(_task_handle);      _task_handle      = nullptr; }
    if (_hr2_task_handle)  { vTaskDelete(_hr2_task_handle);  _hr2_task_handle  = nullptr; }
    if (_hr3_task_handle)  { vTaskDelete(_hr3_task_handle);  _hr3_task_handle  = nullptr; }
    if (_diag_task_handle) { vTaskDelete(_diag_task_handle); _diag_task_handle = nullptr; }

    incunest_afe4490_hal_spi_detach();

    // Delete FreeRTOS objects (mutexes last since we may hold _spi_mutex)
    if (_data_queue)   { vQueueDelete(_data_queue);          _data_queue   = nullptr; }
    if (_drdy_sem)     { vSemaphoreDelete(_drdy_sem);        _drdy_sem     = nullptr; }
    if (_hr2_compute_sem) { vSemaphoreDelete(_hr2_compute_sem);    _hr2_compute_sem = nullptr; }
    if (_hr3_compute_sem) { vSemaphoreDelete(_hr3_compute_sem);    _hr3_compute_sem = nullptr; }
    if (_spi_mutex)    { vSemaphoreDelete(_spi_mutex);       _spi_mutex    = nullptr; }
    if (_state_mutex)  { vSemaphoreDelete(_state_mutex);     _state_mutex  = nullptr; }

    _initialized = false;
    _reset_algorithms();

    ESP_LOGI(TAG, "Stopped");
}

// ── _diag_task_body() ─────────────────────────────────────────────────────────
// Runs at low priority. Calls runAfeDiagnostics() every rsqm_diag_period_ms, but ONLY in
// states with no patient confirmed in the optical path: PROBE_DISCONNECTED and
// the other absent states (isProbeAbsent(): OT_HIGH, ONLY_LED_SATURATING). The diagnostic's
// ~10 ms blackout would disrupt HR/SpO2 acquisition in PROBE_APPLIED, and in PROBE_AMB_SATURATING
// it would also disrupt HGAC's attempt to clear the saturation with a real patient possibly
// connected — so both are excluded.
// Whitelist (not blacklist) on purpose, via isProbeAbsent(): a state not listed there counts as
// present and defaults to NOT diagnosing, so a new state can never silently interrupt acquisition.
void INCUNEST_AFE4490::_diag_task_body() {
    const TickType_t period = pdMS_TO_TICKS(rsqm_diag_period_ms);
    while (true) {
        vTaskDelay(period);
        if (isProbeAbsent(_rsqm_probe_state))
            runAfeDiagnostics();
    }
}

void INCUNEST_AFE4490::_diag_task_trampoline(void* pv) {
    static_cast<INCUNEST_AFE4490*>(pv)->_diag_task_body();
    vTaskDelete(nullptr);
}

// ── _reset_algorithms() ───────────────────────────────────────────────────────
void INCUNEST_AFE4490::_reset_algorithms() {
    _spo2_reset_state();
    _hr1_dc                    = 0.0f;
    _hr1_running_max           = 0.0f;
    _hr1_ppg_above_thresh   = false;
    _hr1_last_peak_idx  = 0;
    _hr1_sample_idx     = 0;
    _hr1_interval_count = 0;
    memset(_hr1_intervals, 0, sizeof(_hr1_intervals));
    _ppgdisp_bpf.reset();
    _hr2_bpf.reset();
    _hr2_buf_idx = 0; _hr2_buf_count = 0;
    _hr2_decim.reset(); _hr2_update_counter = 0;
    memset(_hr2_buf, 0, sizeof(_hr2_buf));
    _hr3_lp.reset();
    _hr3_buf_idx = 0; _hr3_buf_count = 0;
    _hr3_decim.reset(); _hr3_update_counter = 0;
    memset(_hr3_buf, 0, sizeof(_hr3_buf));
    _pi_bpf.reset();
    _pi_window_reset();
    _hr2_computing = false;  _hr3_computing = false;
    _hr2_result = 0.0f; _hr2_result_sqi = 0.0f;
    _hr3_result = 0.0f; _hr3_result_sqi = 0.0f;
    _rsqm_probe_state        = ProbeState::PROBE_DISCONNECTED;
    _switched_rc_settling_countdown = 0;
    _hgac_ema_fast_led1.reset(); _hgac_ema_slow_led1.reset(); _hgac_ema_ambient_led1.reset();
    _hgac_ema_fast_led2.reset(); _hgac_ema_slow_led2.reset(); _hgac_ema_ambient_led2.reset();
    _current_data = AFE4490Data{};
    // Precompute Hann window — needed for HR3 FFT (lives here, not in begin(), so host builds compute it too)
    for (int i = 0; i < hr3_buf_len; i++)
        _hr3_hann[i] = 0.5f * (1.0f - cosf(2.0f * kPi * (float)i / (float)(hr3_buf_len - 1)));
}

// ── RSQM ──────────────────────────────────────────────────────────────────────

// _rsqm_request_probe_state — debounce gate for probe state transitions.
//
// CHECK TO PERFORM (v0.90 — recorded, not yet observed; Alex doubts it occurs): the debounce
// counts consecutive IDENTICAL requests. Until v0.90 both "no tissue" causes voted the same
// PROBE_NOT_APPLIED and accumulated together; PROBE_OT_HIGH and PROBE_ONLY_LED_SATURATING are now
// distinct requests. If samples alternate between them near the saturation boundary (finger being
// removed, RF at the edge), each alternation resets the count, neither commits, and the state stays
// where it was — possibly APPLIED with the finger gone. If that is ever observed, debounce by class
// (isProbeAbsent) rather than by exact state.
// Commits the requested state to _rsqm_probe_state only after
// _rsqm_probe_state_min_samples consecutive calls with the same value.
// Any change in the requested state resets the counter.
void INCUNEST_AFE4490::_rsqm_request_probe_state(ProbeState requested) {
    if (requested == _rsqm_probe_state_pending) {
        if (++_rsqm_probe_state_count >= _rsqm_probe_state_min_samples)
            _rsqm_probe_state = requested;
    } else {
        _rsqm_probe_state_pending = requested;
        _rsqm_probe_state_count   = 1;
    }
}

// Analog state reconstruction — converts raw ADC codes to physical signal-chain quantities.
// Datasheet Eq.2 (p.30): V_DIFF = 2 × (I_PD × RF/Ri − I_CANCEL) × RG
// Inverted:
//   V_ADC      = code × adc::SCALE   (= code / adc::FS_CODE × adc::FSR)
//   V_TIA = 2 × (V_ADC / (2 × RG) + I_CANCEL_A) × Ri   [= 2 × I_PD × RF]
//   I_PD       = V_TIA / (2 × RF)
// V_TIA is always the DIFFERENTIAL TIA output (datasheet §9.2.2); the code has no branch value.
// When STAGE2EN=0: RG = Ri = kAFE_RI_OHM (unity gain).
// ALED channels share the RF and RG of the corresponding LED channel.
// Also computes ot_led1/ot_led2 and the validity masks (value-always policy: every field
// is computed regardless of saturation; masks tell measurement vs bound — see header).
AFE4490AnalogState INCUNEST_AFE4490::_compute_analog_state(
    int32_t led1, int32_t led2, int32_t aled1, int32_t aled2) const {

    AFE4490AnalogState as;

    // ── ADC code → volts ────────────────────────────────────────────────────
    // Datasheet p.11 (Electrical Characteristics, ADC): 22-bit resolution,
    // ADC full-scale voltage ±1.2 V. Datasheet p.43 (§8.4.1): output format
    // is 22-bit twos complement. Standard bipolar twos-complement transfer
    // function → 1 LSB = FSR / 2^21, code range [−2^21, +2^21−1] (asymmetric:
    // negative rail −1.2 V ↔ −2^21 exact; positive code +2^21−1 ↔ +1.2 V − 1 LSB).
    // Matches Table 7 (p.88) rows code=±1 ↔ ±1.2/2^21 V. See namespace adc in .h.
    // V_ADC = code × adc::SCALE
    const float scale = adc::SCALE;
    as.v_adc_led1  = (float)led1  * scale;
    as.v_adc_led2  = (float)led2  * scale;
    as.v_adc_aled1 = (float)aled1 * scale;
    as.v_adc_aled2 = (float)aled2 * scale;

    // ── Circuit parameters ──────────────────────────────────────────────────
    // RF: TIA feedback resistor — TIAGAIN register bits [2:0] / [10:8], datasheet p.27.
    //     Values: 10K…1M Ω (non-monotonic bit encoding, see rf_code[] in .cpp).
    // RG: Stage 2 gain resistor — TIAGAIN register bits [10:8] / TIA_AMB_GAIN [10:8], datasheet p.30.
    //     When STAGE2EN=0 (bits [14]): Stage 2 is bypassed → gain = 1 → RG = Ri (Eq.2 holds with RG=Ri).
    // Ri: fixed input resistor of Stage 2, 100 kΩ — datasheet p.28.
    // I_CANCEL: ambient DAC current — TIA_AMB_GAIN register bits [19:16], datasheet p.27.
    //     1 µA per LSB, range 0–15 µA.
    const float rf1      = kAFE_RF_OHM[(int)_afe_tia_rf_led1];
    const float rg1      = _afe_stg2_en_led1 ? kAFE_RG_OHM[(int)_afe_stg2_rg_led1] : kAFE_RI_OHM;
    const float rf2      = kAFE_RF_OHM[(int)_afe_tia_rf_led2];
    const float rg2      = _afe_stg2_en_led2 ? kAFE_RG_OHM[(int)_afe_stg2_rg_led2] : kAFE_RI_OHM;
    const float ri       = kAFE_RI_OHM;
    const float i_cancel = (float)_afe_ambdac_uA * 1e-6f;  // A

    // ── V_TIA = 2 × (V_ADC / (2 × RG) + I_CANCEL) × Ri ─────────────────
    // Inverted from datasheet Eq.2 (p.30): V_DIFF = 2 × (I_PD × RF/Ri − I_CANCEL) × RG
    // Differential TIA output voltage before Stage 2 (= 2 × I_PD × RF).
    // LED1/ALED1 share rf1, rg1; LED2/ALED2 share rf2, rg2 (same TIA path).
    as.v_tia_led1  = 2.0f * (as.v_adc_led1  / (2.0f * rg1) + i_cancel) * ri;
    as.v_tia_led2  = 2.0f * (as.v_adc_led2  / (2.0f * rg2) + i_cancel) * ri;
    as.v_tia_aled1 = 2.0f * (as.v_adc_aled1 / (2.0f * rg1) + i_cancel) * ri;
    as.v_tia_aled2 = 2.0f * (as.v_adc_aled2 / (2.0f * rg2) + i_cancel) * ri;

    // ── I_PD = V_TIA / (2 × RF) ─────────────────────────────────────────
    // From V_TIA = 2 × I_PD × RF (TIA transimpedance, differential).
    as.i_pd_led1  = as.v_tia_led1  / (2.0f * rf1);
    as.i_pd_led2  = as.v_tia_led2  / (2.0f * rf2);
    as.i_pd_aled1 = as.v_tia_aled1 / (2.0f * rf1);
    as.i_pd_aled2 = as.v_tia_aled2 / (2.0f * rf2);

    // ── OT = (I_PD_LED − I_PD_ALED) / I_LED  [A/A, dimensionless] ────────────
    // Ambient-subtracted: isolates the modulated component driven by the LED.
    // I_LED from LEDCNTRL register (datasheet p.28): 8-bit DAC, range 0–_afe_led_range_mA.
    // Full electro-optical system ratio (LED wall-plug efficiency × tissue transmittance
    // × photodiode responsivity) — not the tissue optical transmittance alone.
    // APPLIED (finger present): ot ≈ 1.4e-5; NOT_APPLIED (no finger): ot ≈ 8e-4.
    // Value-always: computed even under saturation — validity via otLedxValid().
    as.ot_led1 = (as.i_pd_led1 - as.i_pd_aled1) / (_afe_led1_current_mA * 1e-3f);
    as.ot_led2 = (as.i_pd_led2 - as.i_pd_aled2) / (_afe_led2_current_mA * 1e-3f);

    // ── Validity masks (bit = channel, order per AFE4490Ch) ──────────────────
    // adc_sat: code at/beyond ≈95% of rail — stored values are bounds, not measurements.
    // tia_over_fs: outside TI-guaranteed linearity (OFF_SPEC band up to tia_axis::LIN_V).
    // tia_over_lin: beyond empirical linearity — TIA compresses, value is a bound.
    // No lower TIA bound: photocurrent is unipolar (v_tia ≥ 0 physically).
    const int32_t code[4] = { led1, aled1, led2, aled2 };
    const float   vtd[4]  = { as.v_tia_led1, as.v_tia_aled1,
                              as.v_tia_led2, as.v_tia_aled2 };
    for (int ch = 0; ch < 4; ++ch) {
        const uint8_t bit = (uint8_t)(1u << ch);
        if (code[ch] >= adc::SAT_POS)  as.adc_sat_pos  |= bit;
        if (code[ch] <= adc::SAT_NEG)  as.adc_sat_neg  |= bit;
        if (vtd[ch]  >  tia_axis::FS_V)  as.tia_over_fs  |= bit;
        if (vtd[ch]  >  tia_axis::LIN_V) as.tia_over_lin |= bit;
    }

    return as;
}

// Raw Signal Quality Monitor — runs once per sample at 500 Hz.
// Updates EMA estimators, classifies ProbeState, computes DiagCode and RSQI.
// Called at the top of _process_sample() after _compute_analog_state().
void INCUNEST_AFE4490::_rsqm_update(int32_t led1_sub, int32_t led2_sub, const AFE4490AnalogState& as) {
    // ── Probe state detection ──────────────────────────────────────────────────
    // PROBE_DISCONNECTED: instantaneous criterion — all four i_pd below threshold AND
    // |led1_sub|/|led2_sub| near zero on both channels.  When cable is disconnected,
    // the photodiode is not connected; i_pd reflects only TIA input bias current (~nA).
    // The sub criterion confirms the four channels are equal (floating TIA input, no differential signal).
    // Responds in one sample; no EMA warmup required.
    const bool disconnected =
        fabsf(as.i_pd_led1)  < rsqm_disconn_i_pd_thr &&
        fabsf(as.i_pd_led2)  < rsqm_disconn_i_pd_thr &&
        fabsf(as.i_pd_aled1) < rsqm_disconn_i_pd_thr &&
        fabsf(as.i_pd_aled2) < rsqm_disconn_i_pd_thr &&
        fabsf((float)led1_sub) < rsqm_disconn_led_sub_thr &&
        fabsf((float)led2_sub) < rsqm_disconn_led_sub_thr;

    // Classification ladder (DISCONNECTED highest priority, computed above). OT is computed in
    // _compute_analog_state() (as.ot_led1/ot_led2, A/A); thresholds need empirical calibration.
    //
    // The presence question ("is there tissue?") is answered by OT = (i_pd_led − i_pd_aled)/ILED,
    // which is INVARIANT to ambient light (it subtracts the ambient/ALED phase) — so it separates
    // "tissue" (OT ≈ 1e-4) from "no tissue" (OT ≈ 1) even under a lamp. Two regimes:
    //
    //  • NOT saturated → OT is a clean measurement → classify presence directly:
    //      BOTH channels above thr → PROBE_OT_HIGH (no tissue) ; otherwise → APPLIED.
    //    AND, not OR (v0.89): with OR, whichever channel happened to read higher decided
    //    alone, so one channel drifting over the threshold declared the probe absent while
    //    the other still saw tissue — and a false absent verdict resets SpO2/HR1/HR2/HR3 every
    //    sample. AND requires two independent channels to agree. In air neither channel is
    //    attenuated by tissue, so both are high and OT_HIGH still holds.
    //    Cost: one channel reading persistently low (weak LED, obstructed PD) now forces
    //    APPLIED; the LED-only-saturation branch below still catches probe-in-air
    //    independently of OT, and DISCONNECTED still checks all four i_pd channels.
    //    NOTE the channel ordering is NOT predictable from tissue spectra: OT is the whole
    //    electro-optical ratio (LED efficiency × tissue × PD responsivity × geometry), not
    //    tissue transmittance. Measured on the CONTEC MS100, probe applied (2026-09-07):
    //    ot_led1 (IR) = 91 ppm, ot_led2 (RED) = 112 ppm — RED higher, and both within 12 %
    //    of rsqm_ot_thr. The AND holds here only because IR sits 9 ppm under the threshold.
    //  • Saturated (anyPositiveSaturation: ADC at rail or v_tia > FS) → OT is clipped/unreliable,
    //    so discriminate by WHICH phase clips (v0.61 — replaces the old "saturation ⇒ SATURATING"
    //    priority, which drove RF to the floor whenever the finger was removed):
    //      – an AMBIENT phase (ALED1/ALED2) also clips → external light (lamp/phototherapy) is
    //        saturating with tissue present → PROBE_AMB_SATURATING (HGAC should correct it).
    //      – only LED phases clip, ambient phases do NOT → the LED's own light reaches the PD
    //        directly (no tissue in the path) → probe in air → PROBE_ONLY_LED_SATURATING, which
    //        isProbeAbsent() classifies as absent, so HGAC does NOT chase RF to the floor on
    //        finger removal.
    //    Limitation: when ONLY the LED phase clips, the masks can't tell "probe in air" from
    //    "tissue so thin that it saturates without ambient" — we assume air. v0.90 gave that case
    //    its own state so the assumption is visible in the data; the suspected consequence
    //    (thin tissue at high RF → absent → HGAC gated off → RF never lowered → stays absent) is an
    //    open check, see spec §5.6.2. Photocurrent is unipolar in v1 (adc_sat_neg needs AMBDAC).
    // Debounce (same for all transitions) lives in _rsqm_request_probe_state().
    ProbeState requested;
    if (disconnected) {
        requested = ProbeState::PROBE_DISCONNECTED;
    } else if (!as.anyPositiveSaturation()) {
        requested = (as.ot_led1 > rsqm_ot_thr && as.ot_led2 > rsqm_ot_thr)
                    ? ProbeState::PROBE_OT_HIGH
                    : ProbeState::PROBE_APPLIED;
    } else {
        const bool ambient_sat =
            as.adcSatPos(AFE4490Ch::ALED1) || as.tiaOverFs(AFE4490Ch::ALED1) ||
            as.adcSatPos(AFE4490Ch::ALED2) || as.tiaOverFs(AFE4490Ch::ALED2);
        requested = ambient_sat ? ProbeState::PROBE_AMB_SATURATING
                                : ProbeState::PROBE_ONLY_LED_SATURATING;
    }
    _rsqm_request_probe_state(requested);

    // ── DiagCode ───────────────────────────────────────────────────────────────
    uint32_t diag = 0;

    // RSQM_DIAG_AMB_SAT: either ambient channel at or beyond positive rail
    if (as.adcSatPos(AFE4490Ch::ALED1) || as.adcSatPos(AFE4490Ch::ALED2))
        diag |= RSQM_DIAG_AMB_SAT;

    // RSQM_DIAG_SWITCHED_RC_SETTLING: set externally by HGAC or config change; counts down to 0
    if (_switched_rc_settling_countdown > 0) {
        diag |= RSQM_DIAG_SWITCHED_RC_SETTLING;
        // `x = x - 1`, not `--`: compound assignment on a volatile is deprecated in C++20
        // (-Wvolatile). Same non-atomic read-modify-write as before.
        _switched_rc_settling_countdown = _switched_rc_settling_countdown - 1;
    }

    // ── Merge RSQM flags into _diag_code (bits 13+), preserving AFE DIAG bits (0-12) ──
    // diag holds only the RSQM bits computed above; shift them to their assigned positions.
    _diag_code = (_diag_code & AFE_DIAG_MASK) | diag;

    // ── RSQI ───────────────────────────────────────────────────────────────────
    // 1 only when: probe applied and no active diagnostic flags (any bit)
    _current_data.rsqi        = (_rsqm_probe_state == ProbeState::PROBE_APPLIED && _diag_code == 0) ? 1 : 0;
    _current_data.diag_code   = _diag_code;
    _current_data.probe_state = _rsqm_probe_state;
}

// ── HGAC (Hardware Gain Control) ──────────────────────────────────────────────
// Phase 1: TIA guard (A3) + RF-only descent + mandatory SpO2 EMA rescale.
// No ascent yet (recovery is manual) — documented limitation of this phase, not an
// oversight. See incunest_afe4490_spec.md §5.8 for the full design.

// Atomic "change RF + reset this domain's EMAs + arm settling" primitive.
// CONTRACT: only ever called from _hgac_update() (i.e. from inside _process_sample(),
// under _state_mutex) — never call this directly from application code or another task.
// The reset() is essential: a gain change scales v_tia by k, so prior measurements are
// meaningless — the domain's EMAs re-warm at the new RF (warmup → valid()==false → no action,
// which doubles as a cooldown between consecutive steps; see spec §5.8).
void INCUNEST_AFE4490::_hgac_change_rf(HgacColor color, AFE4490RF new_rf) {
    const AFE4490RF old_rf = (color == HgacColor::IR) ? _afe_tia_rf_led1 : _afe_tia_rf_led2;
    if (new_rf == old_rf) return;
    // CF/RF atomicity (2026-08-20 review, no bug found — recorded so it isn't re-litigated):
    // setTIAGainLED1/2() recalculates CF for the NEW rf (_recalc_afe_tia_cf_led1/2()) BEFORE
    // _apply_analog_regs() writes it — and RF/CF for one color share the SAME 24-bit register
    // (TIAGAIN / TIA_AMB_GAIN, datasheet p.57), written in a single SPI transaction. The chip can
    // therefore never observe a transient (new RF, old CF) or (old RF, new CF) combination; the
    // 5τ auto-CF criterion holds for whichever RF the chip currently has. getConfig() reads both
    // fields under the same _spi_mutex, so no torn read either. See test_hgac_change_rf_recalculates_cf
    // (PulseNest test_hgac.cpp) and spec §5.8.4.
    if (color == HgacColor::IR) {
        setTIAGainLED1(new_rf);   // takes _spi_mutex internally; safe nested under caller's _state_mutex
        _hgac_ema_fast_led1.reset();
        _hgac_ema_slow_led1.reset();
        _hgac_ema_ambient_led1.reset();
    } else {
        setTIAGainLED2(new_rf);
        _hgac_ema_fast_led2.reset();
        _hgac_ema_slow_led2.reset();
        _hgac_ema_ambient_led2.reset();
    }
    // HW_SETTLING is now armed by setTIAGainLED1/2() itself (v0.74) — any RF change gets the
    // same protection, HGAC-driven or manual ($SET). No separate arm needed here.

    // Tell the host (v0.94). Timestamp first, count second: a reader that sees count N therefore
    // sees a timestamp at least as new as move N (hgacRfChangeCount() re-reads the count around it).
    _hgac_rf_change_ts_us = (uint64_t)esp_timer_get_time();
    _hgac_rf_change_count = _hgac_rf_change_count + 1;   // not ++: volatile, C++20
}

uint32_t INCUNEST_AFE4490::hgacRfChangeCount(uint64_t* last_ts_us) const {
    for (;;) {
        const uint32_t c1 = _hgac_rf_change_count;
        const uint64_t ts = _hgac_rf_change_ts_us;   // two 32-bit loads on this core: may tear
        const uint32_t c2 = _hgac_rf_change_count;
        if (c1 == c2) {                              // no move landed between the two reads
            if (last_ts_us) *last_ts_us = ts;
            return c2;
        }
    }
}

// One gain domain (color) — pure actuation. fast EMA drives the HIGH2 guard (urgent, integrates
// severity×time); slow EMA drives the HIGH1/LOW1 leveling dead-band on the clean DC. valid()
// short-circuits each check so nothing acts during an EMA's warmup (see spec §5.8). RF steps are
// one LUT level. The ambient alarm is a diagnostic, computed separately (_hgac_ambient_high).
void INCUNEST_AFE4490::_hgac_track(HgacColor color, EmaChannel& fast, EmaChannel& slow,
                                   bool slow_step_down_only) {
    const AFE4490RF rf = (color == HgacColor::IR) ? _afe_tia_rf_led1 : _afe_tia_rf_led2;

    // Guard (fast EMA): reduce RF urgently above HIGH2, then bail (guard has priority).
    // Off in slow_step_down_only mode (ONLY_LED_SATURATING, v0.105): the slow branch below is the
    // only actuator there, so a brief probe removal moves nothing.
    if (!slow_step_down_only && fast.valid() && fast.mean >= hgac_v_tia_high2) {
        if (rf > AFE4490RF::RF_10K) _hgac_change_rf(color, (AFE4490RF)((int)rf - 1));
        return;
    }
    // Leveling (slow EMA): dead-band [LOW1, HIGH1]. Above → reduce RF; below → raise RF.
    if (!slow.valid()) return;
    if (slow.mean >= hgac_v_tia_high1) {
        if (rf > AFE4490RF::RF_10K) _hgac_change_rf(color, (AFE4490RF)((int)rf - 1));
    } else if (!slow_step_down_only && slow.mean < hgac_v_tia_low1) {
        if (rf < AFE4490RF::RF_1M)  _hgac_change_rf(color, (AFE4490RF)((int)rf + 1));
    }
}

// Ambient-light alarm condition for one domain (diagnostic, not actuation): RF at the floor AND
// the ambient (ALED) EMA ≥ HIGH2 → ambient light alone saturates, so lowering RF/ILED can't help
// (v_tia_led ≥ v_tia_aled, LED adds only ~0.1 V at the floor). Independent of the LED guard on
// purpose: ALED is the direct ambient indicator. Produced by HGAC because it depends on RF state.
bool INCUNEST_AFE4490::_hgac_ambient_high(HgacColor color, const EmaChannel& ambient) const {
    const AFE4490RF rf = (color == HgacColor::IR) ? _afe_tia_rf_led1 : _afe_tia_rf_led2;
    return rf == AFE4490RF::RF_10K && ambient.valid() && ambient.mean >= hgac_v_tia_high2;
}

void INCUNEST_AFE4490::_hgac_update(const AFE4490AnalogState& as) {
    if (!hgac_enable) return;

    // HGAC gate (spec §5.8; the design's "Gate G0", from the RSQM/HGAC separation axiom A6):
    // HGAC acts only while the probe is active (APPLIED or AMB_SATURATING) and the front-end is
    // not re-settling after a gain change. Gate off in the absent states — isProbeAbsent():
    // DISCONNECTED / OT_HIGH / ONLY_LED_SATURATING — so HGAC doesn't drive RF on an unrelated
    // optical event (e.g. a phototherapy lamp) with nobody connected. AMB_SATURATING is
    // deliberately NOT absent: it has its own state since 2026-07-24 so a guard event can't flip
    // to an absent state and deadlock the step-down.
    // NOTE: the absent list lives in isProbeAbsent() (single source of truth, shared with the
    // diagnostics task). A ProbeState missing from it is NOT gated off here and lets HGAC act.
    //
    // v0.105 — ONLY_LED_SATURATING is half-open: step DOWN only, and only by the slow EMA.
    // The LED-only clip signature has two causes RSQM cannot tell apart (spec §5.6.2): the probe
    // in air (finger removed: the LED light reaches the PD directly) and tissue so thin that it
    // saturates at the current RF. Gating fully off (v0.61–v0.104) served the first — RF stays
    // put for the reattach — and deadlocked the second: saturated → absent → nobody lowers RF →
    // still saturated, a stable false negative that only a power cycle cleared (reproduced on
    // the bench 2026-10-17: finger on, RF 500K, both LEDs clipping, APPLIED never reached). So in
    // this state the leveling branch may lower RF, one LUT step per slow-EMA warm-up (~6 s at
    // τ = 2 s), and nothing else: the fast guard is off (an urgent response is for real light on
    // a patient, not for a hand that may come back in a second) and the raise branch is off
    // (saturation never asks for more gain). A brief removal therefore moves nothing; a persistent
    // saturation walks down until the clip clears and RSQM can classify the signal again. The
    // cost in the probe-in-air case is bounded: a long removal walks RF to the floor at one step
    // per ~6 s, and the ordinary leveling climbs back at the same pace once the probe is
    // reapplied (rationale §13).
    const bool led_sat_only = (_rsqm_probe_state == ProbeState::PROBE_ONLY_LED_SATURATING);
    if ((isProbeAbsent(_rsqm_probe_state) && !led_sat_only)
     || (_diag_code & RSQM_DIAG_SWITCHED_RC_SETTLING)) {
        // Reset the EMAs while gated off so they re-warm cleanly when the probe is re-applied
        // (otherwise they'd carry stale probe-in-air values into the first decisions).
        _hgac_ema_fast_led1.reset(); _hgac_ema_slow_led1.reset(); _hgac_ema_ambient_led1.reset();
        _hgac_ema_fast_led2.reset(); _hgac_ema_slow_led2.reset(); _hgac_ema_ambient_led2.reset();
        return;
    }

    // Feed the EMAs (only while the gate is open): fast/slow on the LED phase, ambient on ALED.
    _hgac_ema_fast_led1.update(as.v_tia_led1); _hgac_ema_slow_led1.update(as.v_tia_led1);
    _hgac_ema_fast_led2.update(as.v_tia_led2); _hgac_ema_slow_led2.update(as.v_tia_led2);
    _hgac_ema_ambient_led1.update(as.v_tia_aled1);
    _hgac_ema_ambient_led2.update(as.v_tia_aled2);

    // Actuate per domain (RF only); half-open (slow step-down only) under LED-only saturation.
    _hgac_track(HgacColor::IR,  _hgac_ema_fast_led1, _hgac_ema_slow_led1, led_sat_only);
    _hgac_track(HgacColor::RED, _hgac_ema_fast_led2, _hgac_ema_slow_led2, led_sat_only);

    // Ambient alarm (diagnostic, separate from actuation): global OR across domains. Evaluated
    // AFTER _hgac_track — if a track just stepped to the floor it reset the ambient EMA, so
    // valid()==false holds off the alarm until it re-warms (correct).
    const bool ambient_alarm = _hgac_ambient_high(HgacColor::IR,  _hgac_ema_ambient_led1)
                            || _hgac_ambient_high(HgacColor::RED, _hgac_ema_ambient_led2);

    // Publish AMBIENT_HIGH (bit 14). _rsqm_update() (runs before) already cleared bits 13+ and
    // wrote _current_data.diag_code without this bit, so set/clear it here and refresh the mirror.
    if (ambient_alarm) _diag_code |=  RSQM_DIAG_AMBIENT_HIGH;
    else               _diag_code &= ~RSQM_DIAG_AMBIENT_HIGH;
    _current_data.diag_code = _diag_code;
}

// ── SPI primitives ────────────────────────────────────────────────────────────
void INCUNEST_AFE4490::_write_reg(uint8_t addr, uint32_t data) {
    // 1 address byte + 24 data bits, MSB first, one chip-select frame (datasheet §8.5.1).
    const uint8_t tx[4] = { addr,
                            (uint8_t)((data >> 16) & 0xFF),
                            (uint8_t)((data >>  8) & 0xFF),
                            (uint8_t)( data        & 0xFF) };
    incunest_afe4490_hal_spi_transfer(_pin_cs, tx, nullptr, sizeof(tx));
}

// Raw read — caller must have enabled SPI_READ in CONTROL0 beforehand
uint32_t INCUNEST_AFE4490::_read_spi_raw(uint8_t addr) {
    const uint8_t tx[4] = { addr, 0x00, 0x00, 0x00 };
    uint8_t       rx[4] = { 0x00, 0x00, 0x00, 0x00 };
    incunest_afe4490_hal_spi_transfer(_pin_cs, tx, rx, sizeof(tx));
    return ((uint32_t)rx[1] << 16) | ((uint32_t)rx[2] << 8) | (uint32_t)rx[3];
}

uint32_t INCUNEST_AFE4490::_read_reg(uint8_t addr) {
    _write_reg(REG_CONTROL0, ctrl0_spi_read);
    uint32_t val = _read_spi_raw(addr);
    _write_reg(REG_CONTROL0, 0x000000UL);
    return val;
}

uint32_t INCUNEST_AFE4490::runAfeDiagnostics(uint32_t diag_holdoff_ms) {
    if (!_initialized || !_spi_mutex) return 0;
    // Signal _task_body to skip reads during the diagnostic window.
    // Mutex is held only for brief SPI transactions, not for the full 10 ms sleep.
    _diag_active = true;
    xSemaphoreTake(_spi_mutex, portMAX_DELAY);  // wait for any in-progress ADC read to finish
    _write_reg(REG_CONTROL0, 0x000004UL);       // DIAG_EN=1, SPI_READ=0
    xSemaphoreGive(_spi_mutex);                 // release — _task_body skips reads while _diag_active
    vTaskDelay(pdMS_TO_TICKS(10));              // t_DIAG = 8 ms (fast) + 2 ms margin
    xSemaphoreTake(_spi_mutex, portMAX_DELAY);
    _write_reg(REG_CONTROL0, ctrl0_spi_read);   // SPI_READ=1 to read DIAG register
    uint32_t result = _read_spi_raw(REG_DIAG);
    _write_reg(REG_CONTROL0, 0x000000UL);       // restore CONTROL0 to idle
    xSemaphoreGive(_spi_mutex);
    // Set holdoff before clearing _diag_active so _task_body() cannot slip through
    // with a real SPI read between the two assignments.
    _diag_holdoff_samples = (uint32_t)((uint64_t)diag_holdoff_ms * _afe_sample_rate_hz / 1000);
    _diag_active = false;                       // resume; holdoff takes over if diag_holdoff_ms > 0
    // Merge AFE DIAG bits (0-12) into _diag_code, preserving RSQM bits (13+)
    _diag_code = (_diag_code & ~AFE_DIAG_MASK) | (result & AFE_DIAG_MASK);
    return result;
}

int32_t INCUNEST_AFE4490::_sign_extend_22(uint32_t raw) {
    // AFE4490 ADC output is 22-bit two's complement in bits [21:0]
    return ((int32_t)(raw << 10)) >> 10;
}

// ── Chip init ─────────────────────────────────────────────────────────────────
void INCUNEST_AFE4490::_chip_init() {
    // Step 2: Set SPI write mode
    _write_reg(REG_CONTROL0, 0x000000UL);

    // Step 3: Software reset
    _write_reg(REG_CONTROL0, ctrl0_sw_rst);
    vTaskDelay(pdMS_TO_TICKS(10));

    // Step 4: Analog front-end
    _apply_analog_regs();

    // Step 5: Timing registers
    _apply_timing_regs();

    // Step 6: CONTROL1 (enables timer — must be last)
    _apply_control_regs();

    // Step 7: Stabilization
    vTaskDelay(pdMS_TO_TICKS(1000));
}

void INCUNEST_AFE4490::_apply_timing_regs() {
    // Datasheet Table 2 formulas, PRF = _afe_sample_rate_hz
    // AFECLK = 4 MHz → 1 count = 0.25 µs  (afeclk is in the anonymous namespace)
    const uint32_t tia_margin = _compute_led_on_to_sample_margin_counts();  // dead time after LED ON
    const uint32_t ambient_margin  = afe_ambient_margin_counts;  // LED OFF decay before ambient
                                           // sampling. Raised from 200 (50 µs) for the 50 mA drive
                                           // default (v0.47): higher current → longer LED extinction
                                           // / CF discharge tail (see leakage note). Valid for
                                           // PRF ≤ ~2 kHz (at 500 Hz the ambient sample window is
                                           // 400..q-2 = 400..1998, still ample).
    const uint32_t adc_reset       = 3;    // counts (0.75 µs → -60 dB crosstalk)

    uint32_t phase = afeclk / _afe_sample_rate_hz;
    uint32_t prp   = phase - 1;
    uint32_t q     = phase / 4;        // quarter period

    // LED drive windows (25% duty cycle)
    _write_reg(REG_LED2LEDSTC,   3*q);          // t3
    _write_reg(REG_LED2LEDENDC,  prp);           // t4
    _write_reg(REG_LED2STC,      3*q + tia_margin); // t1
    _write_reg(REG_LED2ENDC,     prp - 1);       // t2
    _write_reg(REG_ALED2STC,     ambient_margin);  // t5 — starts 50 µs after LED2 OFF
    _write_reg(REG_ALED2ENDC,    q - 2);           // t6
    _write_reg(REG_LED1LEDSTC,   q);             // t9
    _write_reg(REG_LED1LEDENDC,  2*q - 1);       // t10
    _write_reg(REG_LED1STC,      q + tia_margin); // t7
    _write_reg(REG_LED1ENDC,     2*q - 2);       // t8
    _write_reg(REG_ALED1STC,     2*q + ambient_margin); // t11 — starts 50 µs after LED1 OFF
    _write_reg(REG_ALED1ENDC,    3*q - 2);              // t12

    // ADC reset pulses (3 counts at each phase boundary)
    _write_reg(REG_ADCRSTSTCT0,  0);             // t21
    _write_reg(REG_ADCRSTENDCT0, adc_reset);     // t22
    _write_reg(REG_ADCRSTSTCT1,  q);             // t23
    _write_reg(REG_ADCRSTENDCT1, q  + adc_reset); // t24
    _write_reg(REG_ADCRSTSTCT2,  2*q);           // t25
    _write_reg(REG_ADCRSTENDCT2, 2*q + adc_reset); // t26
    _write_reg(REG_ADCRSTSTCT3,  3*q);           // t27
    _write_reg(REG_ADCRSTENDCT3, 3*q + adc_reset); // t28

    // ADC conversion windows (CONVST = adc_reset_end + 1, CONVEND = next_reset_start - 1)
    _write_reg(REG_LED2CONVST,   adc_reset + 1);            // t13
    _write_reg(REG_LED2CONVEND,  q - 1);                    // t14
    _write_reg(REG_ALED2CONVST,  q  + adc_reset + 1);       // t15
    _write_reg(REG_ALED2CONVEND, 2*q - 1);                  // t16
    _write_reg(REG_LED1CONVST,   2*q + adc_reset + 1);      // t17
    _write_reg(REG_LED1CONVEND,  3*q - 1);                  // t18
    _write_reg(REG_ALED1CONVST,  3*q + adc_reset + 1);      // t19
    _write_reg(REG_ALED1CONVEND, prp);                      // t20

    _write_reg(REG_PRPCOUNT,     prp);                      // t29
}

uint32_t INCUNEST_AFE4490::_build_tiagain_led1() {
    // TIAGAIN register (0x20): RF_LED1[2:0] | CF_LED1[4:0] | STG2GAIN1[2:0] | STAGE2EN1 | ENSEPGAIN
    uint32_t reg = rf_code[(int)_afe_tia_rf_led1];           // bits D[2:0]
    reg         |= ((uint32_t)(_afe_tia_cf_led1 & kAFE_CF_CODE_MAX)) << cf_shift;  // bits D[7:3]
    reg         |= stg2_gain_code[(int)_afe_stg2_rg_led1]; // bits D[10:8]: gain only
    if (_afe_stg2_en_led1)  reg |= 0x004000UL;                   // D14: STAGE2EN1 (explicit)
    if (_afe_sep_tia_en)   reg |= 0x008000UL;                   // D15: ENSEPGAIN
    return reg;
}

uint32_t INCUNEST_AFE4490::_build_tia_amb_gain_led2() {
    // TIA_AMB_GAIN register (0x21): RF_LED2[2:0] | CF_LED2[4:0] | STG2GAIN2[2:0] | STAGE2EN2 | FLTRCNRSEL | AMBDAC[3:0]
    // RF/CF/Stage2 fields control LED2 (RED) when ENSEPGAIN=1, or both channels when ENSEPGAIN=0.
    uint32_t reg = rf_code[(int)_afe_tia_rf_led2];           // bits D[2:0]
    reg         |= ((uint32_t)(_afe_tia_cf_led2 & kAFE_CF_CODE_MAX)) << cf_shift;  // bits D[7:3]
    reg         |= stg2_gain_code[(int)_afe_stg2_rg_led2]; // bits D[10:8]: gain only
    reg         |= ((uint32_t)_afe_ambdac_uA << 16);           // bits D[19:16]: AMBDAC[3:0]
    // STAGE2EN2 (D14): set when explicitly enabled OR when AMBDAC > 0.
    // When AMBDAC > 0, Stage 2 must be active — the cancellation current has no effect without it.
    if (_afe_stg2_en_led2 || _afe_ambdac_uA > 0) reg |= 0x004000UL;
    // FLTRCNRSEL (D15): low-pass filter corner — always written explicitly from library constant.
    if (kAFE_FLTRCNRSEL) reg |= 0x008000UL;
    return reg;
}

// Rebuilds and writes all four analog registers from current in-memory state — unconditionally,
// even from a setter whose value didn't change. Several fields share one register (e.g. TIAGAIN
// packs RF_LED1+CF_LED1+STG2GAIN1+STAGE2EN1+ENSEPGAIN), so any SPI write already retransmits the
// unchanged neighbours; writing the same bits back has no physical effect on the chip (no voltage
// step, nothing re-settles) and doubles as a forced resync if hardware and software ever drift.
// Unlike this, arming switched-RC settling (_arm_switched_rc_settling(), §5.8.4) IS guarded by
// "did the value actually change" — that one has a real cost (freezes live data ~16 ms).
void INCUNEST_AFE4490::_apply_analog_regs() {
    // TIAGAIN (0x20): LED1 gain fields + ENSEPGAIN flag.
    // When ENSEPGAIN=0: chip uses TIA_AMB_GAIN for both channels; LED1 fields in TIAGAIN are ignored.
    // When ENSEPGAIN=1: TIAGAIN → LED1 (IR); TIA_AMB_GAIN → LED2 (RED).
    _write_reg(REG_TIAGAIN, _build_tiagain_led1());
    // TIA_AMB_GAIN (0x21): LED2 gain fields + AMBDAC.
    // Controls LED2 (RED) independently when ENSEPGAIN=1; controls both channels when ENSEPGAIN=0.
    _write_reg(REG_TIA_AMB_GAIN, _build_tia_amb_gain_led2());

    // LEDCNTRL: LED_RANGE | (code_led1 << 8) | code_led2
    // I (mA) = (code / 256) * full_scale_mA
    float full_scale_mA = (float)_afe_led_range_mA;
    uint8_t code1 = (uint8_t)clampf(roundf((_afe_led1_current_mA / full_scale_mA) * 256.0f), 0.0f, 255.0f);
    uint8_t code2 = (uint8_t)clampf(roundf((_afe_led2_current_mA / full_scale_mA) * 256.0f), 0.0f, 255.0f);
    uint32_t range_bit = (_afe_led_range_mA == 75) ? 0x010000UL : 0x000000UL;
    _write_reg(REG_LEDCNTRL, range_bit | ((uint32_t)code1 << 8) | code2);

    // CONTROL2: TX_REF=0x00 (0.75 V), all subsystems powered on, H-bridge, crystal enabled
    _write_reg(REG_CONTROL2, 0x000000UL);
}

void INCUNEST_AFE4490::_apply_control_regs() {
    // CONTROL1: TIMEREN | NUMAV
    uint8_t numav = (_afe_adc_averages > 0) ? (_afe_adc_averages - 1u) : 0u;
    _write_reg(REG_CONTROL1, ctrl1_timeren | numav);
}

void INCUNEST_AFE4490::_recalc_rate_params() {
    float sample_rate_hz              = (float)_afe_sample_rate_hz;
    // One decimation chain per filtered path. Each derives its factor from its own target
    // rate, so the constants calibrated against a chain's rate (hr2_buf_len = 8 s,
    // hr3_buf_len = 10.24 s, the update intervals, hr2_acorr_lag_cap) keep meaning the same
    // number of SECONDS whatever the AFE rate. Consumers read chain.rate_hz — the rate really
    // achieved — never the nominal target.
    _hr2_decim.configure(sample_rate_hz, hr2_decim_target_rate_hz);
    _hr3_decim.configure(sample_rate_hz, hr3_decim_target_rate_hz);
    _hr1_refractory_samples      = (uint32_t)(hr1_refractory_s       * sample_rate_hz);
    _rsqm_probe_state_min_samples = (uint32_t)roundf(rsqm_probe_state_min_s * sample_rate_hz);
    if (_rsqm_probe_state_min_samples < 1) _rsqm_probe_state_min_samples = 1;
    // SpO2 EMAs: only the mean (DC) is read since v0.102, so var_tau = mean_tau (as HGAC's).
    _spo2_ch_ir_ema.init(_spo2_ema_mean_tau_s, _spo2_ema_mean_tau_s, sample_rate_hz);
    _spo2_ch_red_ema.init(_spo2_ema_mean_tau_s, _spo2_ema_mean_tau_s, sample_rate_hz);
    _spo2_bp_ir.init_bp(spo2_bp_f_low_hz, spo2_bp_f_high_hz, sample_rate_hz);
    _spo2_bp_red.init_bp(spo2_bp_f_low_hz, spo2_bp_f_high_hz, sample_rate_hz);
    // Sliding windows (v0.102): blocks of alg_window_block_s (100 ms, v0.103) of the full-rate
    // summands; the live length is the window in SECONDS at the block rate really achieved,
    // capped by the buffer. A length change (window setter or PRF change) empties the window:
    // its content would mix two lengths.
    _spo2_block_clock.configure(sample_rate_hz, alg_window_block_s);
    _pi_block_clock.configure(sample_rate_hz, alg_window_block_s);
    const int spo2_n = _window_len(_spo2_window_s, _spo2_block_clock.rate_hz);
    if (spo2_n != _spo2_buf_n) { _spo2_buf_n = spo2_n; _spo2_window_reset(); }
    const int pi_n = _window_len(_pi_window_s, _pi_block_clock.rate_hz);
    if (pi_n != _pi_buf_n) { _pi_buf_n = pi_n; _pi_window_reset(); }
    _pi_bpf.init_bp(_pi_bpf.f_low, _pi_bpf.f_high, sample_rate_hz);
    // HGAC EMAs: var unused (only mean is read), so var_tau = mean_tau; warmup = 3·τ (fail-safe
    // via valid() → NaN-free: _hgac_track() skips checks until the EMA has matured after a reset).
    _hgac_ema_fast_led1.init(hgac_ema_fast_tau_s, hgac_ema_fast_tau_s, sample_rate_hz, 3.0f * hgac_ema_fast_tau_s);
    _hgac_ema_fast_led2.init(hgac_ema_fast_tau_s, hgac_ema_fast_tau_s, sample_rate_hz, 3.0f * hgac_ema_fast_tau_s);
    _hgac_ema_slow_led1.init(hgac_ema_slow_tau_s, hgac_ema_slow_tau_s, sample_rate_hz, 3.0f * hgac_ema_slow_tau_s);
    _hgac_ema_slow_led2.init(hgac_ema_slow_tau_s, hgac_ema_slow_tau_s, sample_rate_hz, 3.0f * hgac_ema_slow_tau_s);
    _hgac_ema_ambient_led1.init(hgac_ema_ambient_tau_s, hgac_ema_ambient_tau_s, sample_rate_hz, 3.0f * hgac_ema_ambient_tau_s);
    _hgac_ema_ambient_led2.init(hgac_ema_ambient_tau_s, hgac_ema_ambient_tau_s, sample_rate_hz, 3.0f * hgac_ema_ambient_tau_s);
    _hr1_dc_alpha           = expf(-1.0f / (_hr1_dc_tau_s * sample_rate_hz));
    _hr1_max_decay_alpha    = expf(-1.0f / (_hr1_max_decay_tau_s * sample_rate_hz));
    // Recompute periods: configured in seconds, applied in DECIMATED samples, so each uses its
    // own chain's achieved rate (§5.3.1) rather than the AFE rate.
    _hr2_update_interval = (uint32_t)roundf(_hr2_update_interval_s * _hr2_decim.rate_hz);
    if (_hr2_update_interval < 1u) _hr2_update_interval = 1u;
    _hr3_update_interval = (uint32_t)roundf(_hr3_update_interval_s * _hr3_decim.rate_hz);
    if (_hr3_update_interval < 1u) _hr3_update_interval = 1u;
#if INCUNEST_TIMING_STATS
    _ts_emit_interval = (uint32_t)roundf(ts_emit_period_s * sample_rate_hz);
    if (_ts_emit_interval < 1u) _ts_emit_interval = 1u;
#endif
    _hr1_ma_len             = (uint32_t)roundf(sample_rate_hz / (2.0f * _hr1_lp_cutoff_hz));
    if (_hr1_ma_len < 1) _hr1_ma_len = 1;
    if (_hr1_ma_len > (uint32_t)hr1_ma_max_len) {
        // Clamped: the filter no longer cuts where it was configured to, which lets the dicrotic
        // notch through to HR1's threshold detector and risks double-counting (a doubled HR).
        // Never silent — before v0.86b this clamp was mute and hit four of the five catalogue
        // rates. See hr1_ma_max_len for why no fixed capacity can cover every cutoff.
        ESP_LOGE(TAG, "HR1 moving average clamped: %u samples needed at %.0f Hz / %.1f Hz cutoff, "
                      "%d available -> effective cutoff %.1f Hz",
                 (unsigned)_hr1_ma_len, sample_rate_hz, _hr1_lp_cutoff_hz,
                 hr1_ma_max_len, sample_rate_hz / (2.0f * (float)hr1_ma_max_len));
        _hr1_ma_len = (uint32_t)hr1_ma_max_len;
    }
    _ppgdisp_bpf.init_bp(_ppgdisp_bpf.f_low, _ppgdisp_bpf.f_high, sample_rate_hz);
    _hr2_bpf.init_bp(_hr2_bpf.f_low, _hr2_bpf.f_high, sample_rate_hz);
    _hr3_lp.init(_hr3_lp.f_high, sample_rate_hz);
    _recalc_afe_tia_cf_led1();
    _recalc_afe_tia_cf_led2();
}

uint32_t INCUNEST_AFE4490::_compute_led_on_to_sample_margin_counts() const {
    // Dead time between LED turn-on and the opening of the Rx sample window (positions
    // LEDxSTC = LEDxLEDSTC + this margin, see _apply_timing_regs()).
    //
    // NOTE ON PURPOSE (see spec §7.2): the datasheet's stated reason for this delay is the
    // settling of the LED and CABLE, not of the TIA — §8.3.1.3: "To avoid settling effects
    // resulting from the LED or cable, program S_LED2 to start after the LED turns on."
    // That transient is a property of the LED/wiring, essentially independent of RF and CF.
    // The library nonetheless also reuses this margin as the auto-CF budget (5τ ≤ margin),
    // which is a SEPARATE and much stricter constraint than the datasheet's own Equation 1
    // (see getCFMaxEq1PF()) — deliberately named without asserting "TIA" or "LED/cable" alone,
    // since it serves both roles (see the two call sites below and _recalc_afe_tia_cf_led1/2()).
    // Kept deliberately conservative: the margin costs nothing measurable here and shrinking it
    // risks sampling before the LED has stabilised.
    //
    // Margin = max(tia_settle_min, tia_settle_fraction × LED-on window)
    // LED-on window = quarter period = afeclk / (4 × _afe_sample_rate_hz) counts
    uint32_t q      = (afeclk / _afe_sample_rate_hz) / 4u;
    uint32_t margin = (uint32_t)((float)q * tia_settle_fraction);
    return (margin < tia_settle_min) ? tia_settle_min : margin;
}

uint32_t INCUNEST_AFE4490::_window_counts(uint32_t margin) const {
    // Shared clamp: a phase window = quarter-period − 2 (ADC reset) − its own settle margin.
    // Never negative — degenerate configs (margin close to q) return 0.
    uint32_t q = (afeclk / _afe_sample_rate_hz) / 4u;
    return (q > margin + 2u) ? (q - 2u - margin) : 0u;
}

uint32_t INCUNEST_AFE4490::_compute_led_sample_window_counts() const {
    // Rx sample window = LEDxENDC − LEDxSTC, mirroring _apply_timing_regs():
    //   LED1: t8 − t7 = (2q − 2) − (q + tia_margin) = q − 2 − tia_margin
    // LED1 == LED2 (symmetric register layout, see _apply_timing_regs()) — one figure for both.
    return _window_counts(_compute_led_on_to_sample_margin_counts());
}

uint32_t INCUNEST_AFE4490::_compute_ambient_sample_window_counts() const {
    // Ambient sample window = ALEDxENDC − ALEDxSTC, mirroring _apply_timing_regs():
    //   ALED1: t12 − t11 = (3q − 2) − (2q + afe_ambient_margin_counts) = q − 2 − afe_ambient_margin_counts
    // ALED1 == ALED2 (symmetric) — one figure for both.
    return _window_counts(afe_ambient_margin_counts);
}

uint32_t INCUNEST_AFE4490::_compute_switched_rc_settling_samples() const {
    // Datasheet §7.7 t5 (p.17) + footnote (1): after any change to a signal-chain control
    // ("LED current setting, TIA gain, and so forth"), each of the four switched-RC phases
    // (LED1, LED2, ALED1, ALED2) needs >= 3 ms of CUMULATIVE sampling time before its data is
    // valid. Every phase delivers exactly one sample per PRP cycle, so the number of cycles
    // (= samples, = what _switched_rc_settling_countdown counts down in) to discard is
    // ceil(3 ms / phase_window_s) — evaluated on the SHORTEST of the four windows, since that
    // one is the last to reach 3 ms. Neither window depends on RF (only on _afe_sample_rate_hz),
    // so this single figure covers an RF step on either HGAC color domain — see spec §5.8.4.
    uint32_t led_window   = _compute_led_sample_window_counts();
    uint32_t amb_window   = _compute_ambient_sample_window_counts();
    uint32_t worst_window = (led_window < amb_window) ? led_window : amb_window;
    if (worst_window == 0u) return 1u;  // degenerate config guard — never divide by zero
    float worst_window_s = (float)worst_window / (float)afeclk;
    uint32_t samples     = (uint32_t)ceilf(afe_t5_cumulative_sample_time_s / worst_window_s);
    return (samples < 1u) ? 1u : samples;
}

float INCUNEST_AFE4490::getCFMaxEq1PF(AFE4490RF rf) const {
    // Datasheet Equation 1 (§8.3.1.1): RF × CF ≤ Rx Sample Time / 10. "Rx Sample Time" is the
    // datasheet's own term for the LED phase window (Fig. 58) — the ambient window is a separate,
    // library-only margin (§7.2), not part of Eq. 1.
    // This is the CHIP's own upper bound on CF. The library's auto-CF criterion is stricter
    // (≈4.5×), so auto-selection can never exceed this — but a manual setTIACF*() can, which
    // is what this accessor exists to let the caller check.
    float window_s = (float)_compute_led_sample_window_counts() / (float)afeclk;
    return ((window_s / 10.0f) / kAFE_RF_OHM[(int)rf]) * 1e12f;
}

void INCUNEST_AFE4490::_recalc_afe_tia_cf_led1() {
    // Constraint: 5τ ≤ settle_time  →  CF ≤ settle_time / (5 × RF_LED1)
    float settle_s  = (float)_compute_led_on_to_sample_margin_counts() / (float)afeclk;
    float tau_max   = settle_s / tia_n_tau;
    float cf_max_pF = (tau_max / kAFE_RF_OHM[(int)_afe_tia_rf_led1]) * 1e12f;
    // Same quantise-DOWN rule as the public setter: largest of the 32 achievable CF ≤ cf_max_pF.
    _afe_tia_cf_led1 = afeCFPFToCode(cf_max_pF);
}

void INCUNEST_AFE4490::_recalc_afe_tia_cf_led2() {
    // Constraint: 5τ ≤ settle_time  →  CF ≤ settle_time / (5 × RF_LED2)
    float settle_s  = (float)_compute_led_on_to_sample_margin_counts() / (float)afeclk;
    float tau_max   = settle_s / tia_n_tau;
    float cf_max_pF = (tau_max / kAFE_RF_OHM[(int)_afe_tia_rf_led2]) * 1e12f;
    // Same quantise-DOWN rule as the public setter: largest of the 32 achievable CF ≤ cf_max_pF.
    _afe_tia_cf_led2 = afeCFPFToCode(cf_max_pF);
}

// ── BiquadFilter methods ──────────────────────────────────────────────────────

void INCUNEST_AFE4490::BiquadFilter::init_bp(float f_low_hz, float f_high_hz, float sample_rate_hz) {
    // 2nd-order Butterworth bandpass via bilinear transform.
    // Analog prototype: H(s) = BW·s / (s² + BW·s + Ω₀²)
    // State unchanged — call reset() if needed.
    f_low  = f_low_hz;
    f_high = f_high_hz;
    float k     = 2.0f * sample_rate_hz;
    float o_low = k * tanf(3.14159265358979f * f_low_hz  / sample_rate_hz);
    float o_hi  = k * tanf(3.14159265358979f * f_high_hz / sample_rate_hz);
    float o0sq  = o_low * o_hi;
    float bw    = o_hi - o_low;
    float d     = k*k + bw*k + o0sq;
    _b0 =  bw * k / d;
    _b1 =  0.0f;
    _b2 = -bw * k / d;
    _a1 =  2.0f * (o0sq - k*k) / d;
    _a2 =  (k*k - bw*k + o0sq) / d;
}

void INCUNEST_AFE4490::BiquadFilter::init_lp(float f_high_hz, float sample_rate_hz, float q) {
    // 2nd-order low-pass via bilinear transform, prewarped at f_high_hz.
    // Analog prototype: H(s) = Ω² / (s² + (Ω/q)·s + Ω²); q = 1/√2 is Butterworth (-3 dB at
    // f_high_hz). Other q values are the sections of higher-order Butterworths (LowPass4).
    // State unchanged — call reset() if needed.
    f_high = f_high_hz;
    float Ohm  = tanf(3.14159265358979f * f_high_hz / sample_rate_hz);
    float Ohm2 = Ohm * Ohm;
    float inv_q = 1.0f / q;
    float d    = 1.0f + inv_q * Ohm + Ohm2;
    _b0 =  Ohm2 / d;
    _b1 =  2.0f * _b0;
    _b2 =  _b0;
    _a1 =  2.0f * (Ohm2 - 1.0f) / d;
    _a2 = (1.0f - inv_q * Ohm + Ohm2) / d;
}

// ── FreeRTOS task ─────────────────────────────────────────────────────────────
// FreeRTOS requires the task entry point to be a plain C function (static or free function).
// _task_trampoline satisfies that requirement: it receives the INCUNEST_AFE4490 instance pointer
// via the pvParameters argument and immediately forwards execution to _task_body(), which is
// the actual member function with full access to private state. This pattern (trampoline +
// member body) is the standard idiom for running a C++ method as a FreeRTOS task.
void INCUNEST_AFE4490::_task_trampoline(void* pv) {
    static_cast<INCUNEST_AFE4490*>(pv)->_task_body();
    vTaskDelete(nullptr); // should never reach here
}

void INCUNEST_AFE4490::_task_body() {
    for (;;) {
        // Block until DRDY fires (100 ms watchdog — warns if chip stops outputting)
        if (xSemaphoreTake(_drdy_sem, pdMS_TO_TICKS(100)) != pdTRUE) {
            ESP_LOGW(TAG, "DRDY timeout: no sample in 100 ms");
            continue;
        }

        if (_should_freeze_input()) {
            // During diagnostics the chip produces no valid ADC data (~10 ms, ≈5 samples).
            // During post-diagnostic holdoff, OR after an HGAC RF change (_switched_rc_settling_countdown,
            // RSQM_DIAG_SWITCHED_RC_SETTLING, spec §5.8.4), the analog front-end is re-settling — same
            // physical phenomenon, two different triggers. In all cases: feed frozen last-valid
            // raw values into _process_sample() so filters advance continuously. A constant input
            // is absorbed by IIR/BPF/LP filters with near-zero AC contribution — far less
            // disruptive than a transient outlier. This is what actually enforces
            // RSQM_DIAG_SWITCHED_RC_SETTLING as "don't trust this window", not just a label on data that
            // still flows through unfiltered — before this, SpO2/HR1/HR2/HR3/ppg_disp all kept
            // processing the real (transient) values during an RF-change settling window, since
            // none of them gate on this bit (only probe_state). Found by Alex (2026-08-21) from a
            // visible spike in ppg_disp; SpO2/HR1-3 were silently exposed to the same transient.
            // _switched_rc_settling_countdown is decremented inside _process_sample() (_rsqm_update()),
            // unconditionally, whether fed frozen or fresh inputs — no separate decrement needed
            // here, unlike _diag_holdoff_samples (which only exists in this loop).
            if (_diag_holdoff_samples > 0) _diag_holdoff_samples = _diag_holdoff_samples - 1;  // not `--`: volatile, C++20
            xSemaphoreTake(_state_mutex, portMAX_DELAY);
            _process_sample(_last_valid_led1, _last_valid_led2,
                            _last_valid_aled1, _last_valid_aled2,
                            _last_valid_led1_sub, _last_valid_led2_sub);
            xSemaphoreGive(_state_mutex);
            continue;
        }

        // _spi_mutex: protects the SPI bus while reading all 6 channels.
#if INCUNEST_TIMING_STATS
        uint64_t _t_cycle = esp_timer_get_time();
#endif
        xSemaphoreTake(_spi_mutex, portMAX_DELAY);
        // Enable SPI read mode once, burst-read the 4 raw channels, disable.
        // REG_LED2_ALED2VAL and REG_LED1_ALED1VAL (hardware-subtracted registers) are
        // intentionally NOT read: the chip performs the subtraction in 22-bit arithmetic,
        // which overflows when led and aled values have opposite signs near the rail.
        // The software subtraction below uses int32_t (32-bit), eliminating the overflow.
        // Removed reads (kept for reference):
        //   int32_t led2_diff = _sign_extend_22(_read_spi_raw(REG_LED2_ALED2VAL));  // 22-bit overflow risk
        //   int32_t led1_diff = _sign_extend_22(_read_spi_raw(REG_LED1_ALED1VAL));  // 22-bit overflow risk
#if INCUNEST_TIMING_STATS
        uint64_t _t_spi = esp_timer_get_time();
#endif
        _write_reg(REG_CONTROL0, ctrl0_spi_read);
        int32_t led2      = _sign_extend_22(_read_spi_raw(REG_LED2VAL));
        int32_t aled2     = _sign_extend_22(_read_spi_raw(REG_ALED2VAL));
        int32_t led1      = _sign_extend_22(_read_spi_raw(REG_LED1VAL));
        int32_t aled1     = _sign_extend_22(_read_spi_raw(REG_ALED1VAL));
        int32_t led2_diff = led2 - aled2;  // SW subtraction in int32_t — no overflow
        int32_t led1_diff = led1 - aled1;  // SW subtraction in int32_t — no overflow
        _write_reg(REG_CONTROL0, 0x000000UL);
#if INCUNEST_TIMING_STATS
        _ts_spi.update(esp_timer_get_time() - _t_spi);   // the six frames, nothing else
#endif
        xSemaphoreGive(_spi_mutex);

        // Save raw values for use during any subsequent diagnostic holdoff.
        // Written and read exclusively from _task_body() — no mutex required.
        _last_valid_led1       = led1;
        _last_valid_led2       = led2;
        _last_valid_aled1      = aled1;
        _last_valid_aled2      = aled2;
        _last_valid_led1_sub = led1_diff;
        _last_valid_led2_sub = led2_diff;

        // _state_mutex: protects internal processing state (_ppgdisp_channel, filter
        // buffers, SpO2/HR accumulators) against concurrent config setter calls.
        xSemaphoreTake(_state_mutex, portMAX_DELAY);
        _process_sample(led1, led2, aled1, aled2, led1_diff, led2_diff);
        xSemaphoreGive(_state_mutex);
#if INCUNEST_TIMING_STATS
        _ts_cycle.update(esp_timer_get_time() - _t_cycle);
        if (++_ts_emit_counter >= _ts_emit_interval) {
            _ts_emit_counter = 0;
            _emit_timing();
        }
#endif
    }
}

// ── ISR ───────────────────────────────────────────────────────────────────────
// Trampoline required because the HAL's ISR contract (like Arduino's attachInterrupt() and
// ESP-IDF's gpio_isr_handler_add()) takes a plain function pointer; C++ member functions
// are not compatible. _drdy_isr_static is what incunest_afe4490_hal_gpio_attach_isr_rising()
// registers; it forwards the call to the actual member ISR (_drdy_isr) via
// the singleton pointer _g_instance. The null-check guards against a spurious interrupt
// arriving after stop() has cleared _g_instance.
void IRAM_ATTR INCUNEST_AFE4490::_drdy_isr_static() {
    if (_g_instance) _g_instance->_drdy_isr();
}

void IRAM_ATTR INCUNEST_AFE4490::_drdy_isr() {
    BaseType_t woken = pdFALSE;
    xSemaphoreGiveFromISR(_drdy_sem, &woken);
    portYIELD_FROM_ISR(woken);
}

// ── Signal processing ─────────────────────────────────────────────────────────

void INCUNEST_AFE4490::_process_sample(int32_t led1, int32_t led2, int32_t aled1, int32_t aled2,
                                   int32_t led1_sub, int32_t led2_sub) {
    // Analog reconstruction — converts ADC codes to physical quantities (V_ADC, V_TIA, I_PD, OT).
    // While the front-end is re-settling (_should_freeze_input()), REUSE the last fully-computed
    // state instead of recomputing it from the frozen raw codes above: recomputing would combine
    // an OLD raw ADC code with the NEW (already-changed) circuit parameters (RF/RG/AMBDAC/ILED —
    // live members, updated by the very setter that armed this freeze), producing an artificially
    // STEPPED v_tia/i_pd/ot rather than a flat one (found by Alex, 2026-08-22: an abrupt pulse in
    // ppg_disp instead of a benign constant). Reusing the whole struct sidesteps the mismatch by
    // construction — see spec §5.8.4 and the _last_valid_analog_state comment in the header.
    AFE4490AnalogState as;
    if (_should_freeze_input()) {
        as = _last_valid_analog_state;
    } else {
        as = _compute_analog_state(led1, led2, aled1, aled2);
        _last_valid_analog_state = as;
    }

    // RSQM runs next — computes probe state, DiagCode, RSQI for this sample
    _rsqm_update(led1_sub, led2_sub, as);

    // HGAC runs next — Phase 1: RF-only saturation guard + descent (see incunest_afe4490_spec.md §5.8)
    _hgac_update(as);

    // Select PPG source — OT domain (v0.69, gain-invariant, same rationale as SpO2/HR1/HR2/HR3
    // in §5.1/§5.2: an RF change from HGAC no longer creates a scaling discontinuity here).
    float raw_ppg = (_ppgdisp_channel == AFE4490Channel::LED2) ? as.ot_led2 : as.ot_led1;

    float filtered = _ppgdisp_bpf.process(raw_ppg);

    _current_data.ppg_disp        = -filtered;  // negated: AFE raw falls on systole; invert for conventional PPG polarity (peaks up)
    _current_data.led1       = led1;
    _current_data.led2       = led2;
    _current_data.aled1      = aled1;
    _current_data.aled2      = aled2;
    _current_data.led1_sub = led1_sub;
    _current_data.led2_sub = led2_sub;

    // SpO2/HR1/HR2/HR3 all use OT (gain-invariant, EXPERIMENT — see spec §5.1/§5.8) instead of
    // the raw ambient-corrected adc_code (led1_sub) used before this experiment.
    // HR1 runs fully in this task. HR2/HR3 fast paths run here; slow computation in Task B/C.
#if INCUNEST_TIMING_STATS
    // PI (v0.101): no dedicated TimingStat yet — see rationale §11 for the scope note.
    _pi_update_sample(as.ot_led1, _rsqm_probe_state);
    { uint64_t _t = esp_timer_get_time(); _spo2_update(as.ot_led1, as.ot_led2, _rsqm_probe_state); _ts_spo2.update(esp_timer_get_time() - _t); }
    { uint64_t _t = esp_timer_get_time(); _hr1_update(as.ot_led1, _rsqm_probe_state);            _ts_hr1.update(esp_timer_get_time() - _t); }
    uint64_t _t_hr2 = esp_timer_get_time();
#else
    _pi_update_sample(as.ot_led1, _rsqm_probe_state);
    _spo2_update(as.ot_led1, as.ot_led2, _rsqm_probe_state);
    _hr1_update(as.ot_led1, _rsqm_probe_state);
#endif

    // HR2 fast path: filter + decimate + buffer; trigger Task B when interval fires
    if (_hr2_update_sample(as.ot_led1, _rsqm_probe_state)) {
        if (!_hr2_computing) {
            _hr2_take_snapshot();
            _hr2_computing = true;
            xSemaphoreGive(_hr2_compute_sem);
        }
    }

#if INCUNEST_TIMING_STATS
    _ts_hr2.update(esp_timer_get_time() - _t_hr2);
    uint64_t _t_hr3 = esp_timer_get_time();
#endif

    // HR3 fast path: LP filter + decimate + buffer; trigger Task C when interval fires
    if (_hr3_update_sample(as.ot_led1, _rsqm_probe_state)) {
        if (!_hr3_computing) {
            _hr3_prepare_fft_input();
            _hr3_computing = true;
            xSemaphoreGive(_hr3_compute_sem);
        }
    }

#if INCUNEST_TIMING_STATS
    _ts_hr3.update(esp_timer_get_time() - _t_hr3);
#endif

    // Update debug snapshot — always called under _state_mutex (held by _task_body() caller).
    _debug_data.analog                = as;
    _debug_data.rf_led1               = _afe_tia_rf_led1;
    _debug_data.rf_led2               = _afe_tia_rf_led2;

    // Push to queue; if full, drop oldest to keep most recent.
    // Debug mode: push combined DebugQueueItem for atomic getData(data,dbg) access.
    if (_debug_enabled) {
        DebugQueueItem item;
        item.data = _current_data;
        item.dbg  = _debug_data;
        if (xQueueSend(_data_queue, &item, 0) != pdTRUE) {
            DebugQueueItem dummy;
            xQueueReceive(_data_queue, &dummy, 0);
            xQueueSend(_data_queue, &item, 0);
        }
    } else {
        if (xQueueSend(_data_queue, &_current_data, 0) != pdTRUE) {
            AFE4490Data dummy;
            xQueueReceive(_data_queue, &dummy, 0);
            xQueueSend(_data_queue, &_current_data, 0);
        }
    }
}

// ── Timing report ─────────────────────────────────────────────────────────────
// Emits a $TIMING serial frame with per-algorithm mean/max execution times (µs)
// and remaining task stack (words). Called every _ts_emit_interval samples (ts_emit_period_s).
// Frame: $TIMING,hr1_mean,hr1_max,hr2fp_mean,hr2fp_max,hr3fp_mean,hr3fp_max,
//                spo2_mean,spo2_max,cycle_mean,cycle_max,
//                hr2cmp_mean,hr2cmp_max,hr3cmp_mean,hr3cmp_max,stack_free,
//                spi_mean,spi_max*XX
// Task A fast-path (first 10 values): budget reference = 2000 µs (1/500 Hz).
// Task B/C compute (values 11-14): CPU load % = mean / 500000 µs (0.5 s period).
// SPI (values 16-17, v0.92): the six 4-byte frames of one sample — appended LAST so a parser
// written for the 15-value frame keeps working.
#if INCUNEST_TIMING_STATS
void INCUNEST_AFE4490::_emit_timing() {
    UBaseType_t stack_free = uxTaskGetStackHighWaterMark(nullptr);
    char buf[256];
    int n = snprintf(buf, sizeof(buf),
        "TIMING,%llu,%llu,%llu,%llu,%llu,%llu,%llu,%llu,%llu,%llu,%llu,%llu,%llu,%llu,%u,%llu,%llu",
        (unsigned long long)_ts_hr1.mean_us(),          (unsigned long long)_ts_hr1.max_us,
        (unsigned long long)_ts_hr2.mean_us(),          (unsigned long long)_ts_hr2.max_us,
        (unsigned long long)_ts_hr3.mean_us(),          (unsigned long long)_ts_hr3.max_us,
        (unsigned long long)_ts_spo2.mean_us(),         (unsigned long long)_ts_spo2.max_us,
        (unsigned long long)_ts_cycle.mean_us(),        (unsigned long long)_ts_cycle.max_us,
        (unsigned long long)_ts_hr2_compute.mean_us(),  (unsigned long long)_ts_hr2_compute.max_us,
        (unsigned long long)_ts_hr3_compute.mean_us(),  (unsigned long long)_ts_hr3_compute.max_us,
        (unsigned)stack_free,
        (unsigned long long)_ts_spi.mean_us(),          (unsigned long long)_ts_spi.max_us);
    uint8_t chk = 0;
    for (int i = 0; i < n; i++) chk ^= (uint8_t)buf[i];
    incunest_afe4490_hal_console_printf("$%s*%02X\r\n", buf, chk);
    // Emit task CPU stats BEFORE resetting accumulators (needs sum_us values)
    _emit_tasks();
    _ts_spo2.reset(); _ts_hr1.reset(); _ts_hr2.reset(); _ts_hr3.reset(); _ts_cycle.reset(); _ts_spi.reset();
    _ts_hr2_compute.reset(); _ts_hr3_compute.reset();
}

// Emits one $TASK frame per library task + $TASKS_END.
// CPU% is computed from sum_us over the emit window (current interval, not cumulative).
// Window = _ts_emit_interval samples / sample_rate_hz = ts_emit_period_s.
// stack_words = uxTaskGetStackHighWaterMark() for each task handle.
// Does NOT use uxTaskGetSystemState() — avoids requiring configUSE_TRACE_FACILITY=1, which
// the precompiled Arduino-ESP32 FreeRTOS does not set (ESP-IDF builds could, via menuconfig).
void INCUNEST_AFE4490::_emit_tasks() {
    uint64_t window_us = (uint64_t)_ts_emit_interval * 1000000u / _afe_sample_rate_hz;

    struct LibTask { const char* name; uint64_t sum_us; TaskHandle_t handle; };
    LibTask lib_tasks[] = {
        { "incunest_afe4490", _ts_cycle.sum_us,        _task_handle      },
        { "incunest_hr2",     _ts_hr2_compute.sum_us,   _hr2_task_handle  },
        { "incunest_hr3",     _ts_hr3_compute.sum_us,   _hr3_task_handle  },
    };
    for (const auto& t : lib_tasks) {
        uint32_t pct_x10 = (window_us > 0) ?
            (uint32_t)(t.sum_us * 1000u / window_us) : 0u;
        uint32_t stack = t.handle ? (uint32_t)uxTaskGetStackHighWaterMark(t.handle) : 0u;
        char buf[96];
        int len = snprintf(buf, sizeof(buf), "TASK,%s,%lu,%lu",
            t.name, (unsigned long)pct_x10, (unsigned long)stack);
        uint8_t chk = 0;
        for (int j = 0; j < len; j++) chk ^= (uint8_t)buf[j];
        incunest_afe4490_hal_console_printf("$%s*%02X\r\n", buf, chk);
    }
    const char* end_str = "TASKS_END";
    uint8_t end_chk = 0;
    for (int i = 0; end_str[i]; i++) end_chk ^= (uint8_t)end_str[i];
    incunest_afe4490_hal_console_printf("$%s*%02X\r\n", end_str, end_chk);
}
#endif

// ── SpO2 algorithm ────────────────────────────────────────────────────────────
// R = (AC_rms_RED / DC_RED) / (AC_rms_IR / DC_IR)
// SpO2 = a - b * R
// EXPERIMENT (OT-domain input, branch experiment/ot-domain-inputs — see spec §5.1/§5.8):
// ot_ir/ot_red replace the raw ambient-corrected adc_code (led1_sub/led2_sub) used before this
// experiment. OT = (i_pd_led − i_pd_aled) / I_LED is gain-invariant by construction (a linear
// rescaling of led_sub by 1/(2·RF·I_LED)) — an RF change no longer creates a discontinuity in
// this input, so an RF change no longer needs to rescale this EmaChannel by k.
//
// probe_state is RSQM's classification, consumed here — this function never computes its own
// presence detection (see spo2_div_eps rationale above §). Encapsulation: only this function
// reads/writes _spo2_ch_ir_ema/_spo2_ch_red_ema; nothing outside resets or feeds them.
//
// Output contract: sqi==0.0f always implies pi==spo2==spo2_r==NAN (invalid, never stale) —
// unified across warmup, not-applied, and the division-safety guard below. NAN (not a
// sentinel like -1.0f) fails safe: any consumer that forgets to check sqi and instead
// compares/averages a NAN value gets a comparison that is always false / a propagated NAN,
// rather than a plausible-looking-but-wrong number that could trigger a false alarm.
// Window length in blocks for a window in seconds at the block rate really achieved, capped by
// the buffer (alg_window_s_max at 10 Hz; a block clock running faster than 10 Hz because the
// PRF is not a multiple of it would otherwise overflow — then the longest window that fits).
int INCUNEST_AFE4490::_window_len(float window_s, float rate_hz) {
    long n = lroundf(window_s * rate_hz);
    if (n < 1) n = 1;
    if (n > alg_window_buf_len_max) n = alg_window_buf_len_max;
    return (int)n;
}

void INCUNEST_AFE4490::_spo2_window_reset() {
    _spo2_block_clock.reset();
    _spo2_blk_sxy = 0.0f; _spo2_blk_sxx = 0.0f; _spo2_blk_syy = 0.0f;
    _spo2_blk_var_ir = 0.0f; _spo2_blk_var_red = 0.0f;
    _spo2_sxy_sum = 0.0f; _spo2_sxx_sum = 0.0f; _spo2_syy_sum = 0.0f;
    _spo2_var_ir_sum = 0.0f; _spo2_var_red_sum = 0.0f;
    _spo2_buf_idx = 0; _spo2_buf_count = 0;
    memset(_spo2_sxy_buf, 0, sizeof(_spo2_sxy_buf));
    memset(_spo2_sxx_buf, 0, sizeof(_spo2_sxx_buf));
    memset(_spo2_syy_buf, 0, sizeof(_spo2_syy_buf));
    memset(_spo2_var_ir_buf, 0, sizeof(_spo2_var_ir_buf));
    memset(_spo2_var_red_buf, 0, sizeof(_spo2_var_red_buf));
}

void INCUNEST_AFE4490::_spo2_reset_state() {
    _spo2_ch_ir_ema.reset();
    _spo2_ch_red_ema.reset();
    _spo2_bp_ir.reset();
    _spo2_bp_red.reset();
    _spo2_reg_x_prev = 0.0f; _spo2_reg_y_prev = 0.0f;
    _spo2_reg_count = 0;
    _spo2_window_reset();
}

void INCUNEST_AFE4490::_spo2_update(float ot_ir, float ot_red, ProbeState probe_state) {
    if (probe_state != ProbeState::PROBE_APPLIED) {
        // Reset every sample while not applied (idempotent — cheap, no stored "previous
        // probe_state" needed): the state is already clean the instant probe_state returns
        // to APPLIED, so warmup restarts naturally via ir_ema.count below.
        _spo2_reset_state();
        _current_data.spo2        = NAN;
        _current_data.spo2_r      = NAN;
        _current_data.spo2_r_method2_reg_corr = NAN;
        _current_data.spo2_sqi    = 0.0f;
        _current_data.spo2_hold_reason = SpO2HoldReason::PROBE;
        return;
    }

    // DC via EmaChannel (one per channel): mean += α_mean × (x − mean), slow IIR low-pass,
    // τ = spo2_ema_mean_tau_s (2.0 s) — the DC of both R methods. PI is computed independently by
    // _pi_update_sample() since v0.101 (rationale §11) — it no longer reads this EMA.
    _spo2_ch_ir_ema.update(ot_ir);
    _spo2_ch_red_ema.update(ot_red);
    const float dc_ir  = _spo2_ch_ir_ema.mean;
    const float dc_red = _spo2_ch_red_ema.mean;

    // Both methods' state advances on every applied sample whichever method is selected, so a
    // switch costs no warm-up. Full rate: R-METHOD-1's (x − mean)² per channel; R-METHOD-2's x, y
    // (band-passed OT normalised by its DC), their first differences and the three summands dx·dy, dx², dy². All
    // five are summed over the current block (v0.102, header SpO2 state block).
    {
        const float d_ir = ot_ir - dc_ir, d_red = ot_red - dc_red;
        _spo2_blk_var_ir  += d_ir  * d_ir;
        _spo2_blk_var_red += d_red * d_red;
        const float x = (dc_ir  > spo2_div_eps) ? _spo2_bp_ir.process(ot_ir)   / dc_ir  : 0.0f;
        const float y = (dc_red > spo2_div_eps) ? _spo2_bp_red.process(ot_red) / dc_red : 0.0f;
        if (_spo2_reg_count > 0) {
            const float dx = x - _spo2_reg_x_prev, dy = y - _spo2_reg_y_prev;
            _spo2_blk_sxy += dx * dy;
            _spo2_blk_sxx += dx * dx;
            _spo2_blk_syy += dy * dy;
        }
        _spo2_reg_x_prev = x; _spo2_reg_y_prev = y;
        _spo2_reg_count++;
    }

    // Window update at the block cadence: the block sums enter the circular buffers and the
    // fixed-cost running totals; between blocks every output keeps its last value (as PI) —
    // except while the window is still filling, where "last value" must be NaN, never a stale or
    // constructed 0 (output contract: sqi == 0 ⇒ spo2, R, r are NaN).
    if (!_spo2_block_clock.tick()) {
        if (_spo2_buf_count < (uint32_t)_spo2_buf_n) {
            _current_data.spo2        = NAN;
            _current_data.spo2_r      = NAN;
            _current_data.spo2_r_method2_reg_corr = NAN;
            _current_data.spo2_sqi    = 0.0f;
            _current_data.spo2_hold_reason = SpO2HoldReason::WINDOW;
        }
        return;
    }
    _spo2_sxy_sum    += _spo2_blk_sxy    - _spo2_sxy_buf[_spo2_buf_idx];
    _spo2_sxx_sum    += _spo2_blk_sxx    - _spo2_sxx_buf[_spo2_buf_idx];
    _spo2_syy_sum    += _spo2_blk_syy    - _spo2_syy_buf[_spo2_buf_idx];
    _spo2_var_ir_sum += _spo2_blk_var_ir - _spo2_var_ir_buf[_spo2_buf_idx];
    _spo2_var_red_sum+= _spo2_blk_var_red- _spo2_var_red_buf[_spo2_buf_idx];
    _spo2_sxy_buf[_spo2_buf_idx]     = _spo2_blk_sxy;
    _spo2_sxx_buf[_spo2_buf_idx]     = _spo2_blk_sxx;
    _spo2_syy_buf[_spo2_buf_idx]     = _spo2_blk_syy;
    _spo2_var_ir_buf[_spo2_buf_idx]  = _spo2_blk_var_ir;
    _spo2_var_red_buf[_spo2_buf_idx] = _spo2_blk_var_red;
    _spo2_blk_sxy = 0.0f; _spo2_blk_sxx = 0.0f; _spo2_blk_syy = 0.0f;
    _spo2_blk_var_ir = 0.0f; _spo2_blk_var_red = 0.0f;
    if (++_spo2_buf_idx >= _spo2_buf_n) {
        // Wrap: rebuild the totals from the buffers so float32 add/subtract drift never
        // accumulates (5·n adds once per window length — negligible at the block cadence).
        _spo2_buf_idx = 0;
        float sxy = 0.0f, sxx = 0.0f, syy = 0.0f, vir = 0.0f, vred = 0.0f;
        for (int i = 0; i < _spo2_buf_n; i++) {
            sxy += _spo2_sxy_buf[i]; sxx += _spo2_sxx_buf[i]; syy += _spo2_syy_buf[i];
            vir += _spo2_var_ir_buf[i]; vred += _spo2_var_red_buf[i];
        }
        _spo2_sxy_sum = sxy; _spo2_sxx_sum = sxx; _spo2_syy_sum = syy;
        _spo2_var_ir_sum = vir; _spo2_var_red_sum = vred;
    }
    // The window IS the warm-up: no estimate until it is full (replaces spo2_warmup_s = 3·τ).
    if (_spo2_buf_count < (uint32_t)_spo2_buf_n) {
        _spo2_buf_count++;
        _current_data.spo2        = NAN;
        _current_data.spo2_r      = NAN;
        _current_data.spo2_r_method2_reg_corr = NAN;
        _current_data.spo2_sqi    = 0.0f;
        _current_data.spo2_hold_reason = SpO2HoldReason::WINDOW;
        return;
    }

    // Locals (renamed v0.110, Alex): R the ratio; reg_corr the correlation of R-METHOD-2's
    // regression (never a bare r); eps_valid = the window sums clear their epsilons, i.e. the
    // divisions below are safe — not "SpO2 is valid", which the gates at the end decide.
    float R, reg_corr = NAN;
    bool  eps_valid;
    if (_spo2_r_method == 2) {
        // R = Σ(dx·dy)/Σ(dx²) over the window: the least-squares slope of dy on dx through the
        // origin, i.e. of ln(I_red) on ln(I_ir) over the pulse — the Beer-Lambert ratio R-METHOD-1
        // approximates. r = Σ(dx·dy)/√(Σ(dx²)·Σ(dy²)) is that fit's correlation: ~0.99 with a pulse.
        // (Sums, not means: the common 1/N cancels in both ratios.)
        eps_valid = _spo2_sxx_sum > spo2_reg_eps && _spo2_syy_sum > spo2_reg_eps;
        R        = eps_valid ? _spo2_sxy_sum / _spo2_sxx_sum : NAN;
        reg_corr = eps_valid ? _spo2_sxy_sum / sqrtf(_spo2_sxx_sum * _spo2_syy_sum) : NAN;
    } else {
        // R = (AC_red/DC_red)/(AC_ir/DC_ir) — ISO 80601-2-61:2026 "ratio of ratios"; each term is
        // the channel's AC/DC, AC = RMS of (x − mean) over the window (the common 1/N cancels).
        // Guard is division-safety only (presence is probe_state's job).
        const float ac_ir  = sqrtf(_spo2_var_ir_sum);
        const float ac_red = sqrtf(_spo2_var_red_sum);
        eps_valid = dc_red >= spo2_div_eps && ac_ir >= spo2_div_eps;
        R = eps_valid ? (ac_red / dc_red) / (ac_ir / dc_ir) : NAN;
    }
    _current_data.spo2_r      = R;
    _current_data.spo2_r_method2_reg_corr = reg_corr;
    if (!eps_valid) {
        _current_data.spo2     = NAN;
        _current_data.spo2_sqi = 0.0f;
        _current_data.spo2_hold_reason = SpO2HoldReason::EPS;
        return;
    }

    const float spo2 = _spo2_r_curve_a - _spo2_r_curve_b * R;    // the R curve (setSpO2RCurve)
    // R-METHOD-2's validity gate: a fit that does not describe the data gives no SpO2. R and r
    // stay visible so the reason is on record.
    const bool reg_corr_ok = (_spo2_r_method != 2) || (reg_corr >= _spo2_r_method2_reg_corr_min);
    // PI is computed independently by _pi_update_sample() (v0.101, rationale §11) — NaN while its
    // own window is still filling; since v0.102 the two windows are independent lengths, so a PI
    // window longer than the SpO2 window makes this gate bind for the difference.
    // Checked explicitly rather than relied upon via fmaxf/fminf's NaN pass-through (which would
    // read a not-yet-valid PI as SQI=1, the opposite of correct) — defensive for any caller that
    // feeds SpO2 without PI, now that the two are independent functions. PI above spo2_pi_max is
    // a DC transient, not perfusion: it used to map to SQI = 1 (output clamped, input not) and
    // publish a plausible-looking SpO2 while the probe was coming off.
    const bool pi_ok = !isnan(_current_data.pi) && _current_data.pi <= _spo2_pi_max;
    const bool range_ok = spo2 >= _spo2_min && spo2 <= _spo2_max + spo2_clamp_margin;
    // Pulse gate (v0.110): HR3's harmonic test is the artefact criterion the red-IR correlation
    // lacks (SUBJ10: r 0.96 with −16 points). _current_data.hr3 is NaN while HR3 has no valid
    // result — committed by Task C under _state_mutex, which Task A holds here. The window above
    // keeps running meanwhile, so when HR3 returns SpO2 is published in that same block: HR3
    // valid means 10.24 s of pulse, which covers this 6 s window.
    // Validity is hr3_sqi > 0 (Task C writes hr3 = sqi > 0 ? result : NAN), not !isnan(hr3): before
    // Task C's first commit after boot _current_data.hr3 is 0.0, which would open the gate.
    const bool pulse_ok = !_spo2_pulse_gate_enable || (_current_data.hr3_sqi > 0.0f && !isnan(_current_data.hr3));
    _current_data.spo2_hold_reason =
        !reg_corr_ok ? SpO2HoldReason::REG_CORR :
        !pi_ok       ? SpO2HoldReason::PI :
        !range_ok    ? SpO2HoldReason::RANGE :
        !pulse_ok    ? SpO2HoldReason::PULSE : SpO2HoldReason::NONE;
    if (_current_data.spo2_hold_reason == SpO2HoldReason::NONE) {
        _current_data.spo2 = fminf(spo2, _spo2_max);
        // SQI: Perfusion Index linearly mapped to [0, 1].
        // PI < spo2_pi_sqi_lo → SQI = 0 (very weak signal). PI ≥ spo2_pi_sqi_hi → SQI = 1
        // (full quality, per Nellcor/Masimo thresholds).
        float sqi = (_current_data.pi - _spo2_pi_sqi_lo) / (_spo2_pi_sqi_hi - _spo2_pi_sqi_lo);
        _current_data.spo2_sqi = fmaxf(0.0f, fminf(1.0f, sqi));
    } else {
        _current_data.spo2     = NAN;
        _current_data.spo2_sqi = 0.0f;
    }
}

// ── PI (Perfusion Index) algorithm — decoupled from SpO2 (v0.101, rationale §11) ─────────────
// Fast path only: no slow/async computation needed, unlike HR2's autocorrelation (N × lags multiplications) and HR3's FFT — the
// sliding-window RMS update below costs the same per block whatever the window length, cheap enough to commit
// _current_data.pi directly here. probe_state gate lives here exactly as in
// _hr2_update_sample()/_spo2_update(): while not applied, resets every sample (idempotent).
void INCUNEST_AFE4490::_pi_window_reset() {
    _pi_block_clock.reset();
    _pi_buf_idx = 0; _pi_buf_count = 0;
    _pi_ac_sum = 0.0f; _pi_dc_sum = 0.0f; _pi_ac_block_sum = 0.0f; _pi_dc_block_sum = 0.0f;
    memset(_pi_ac_buf, 0, sizeof(_pi_ac_buf));
    memset(_pi_dc_buf, 0, sizeof(_pi_dc_buf));
}

void INCUNEST_AFE4490::_pi_update_sample(float ot_ir, ProbeState probe_state) {
    if (probe_state != ProbeState::PROBE_APPLIED) {
        _pi_bpf.reset();
        _pi_window_reset();
        _current_data.pi = NAN;
        return;
    }

    // Both sums at the full rate over the current block (v0.103): the raw OT for the DC and the
    // squared band-passed sample for the AC. DC is a denominator only, never subtracted — it
    // carries none of the bradycardia-tracking risk an AC-extraction EMA would (header, PI
    // state block). Until v0.102 the AC took ONE band-passed sample per 20 ms block (a true
    // 50 Hz decimation); at 100 ms that would alias the 0.5–15 Hz band, and the full-rate sum
    // is the exact RMS anyway.
    _pi_dc_block_sum += ot_ir;
    const float filtered = _pi_bpf.process(ot_ir);
    _pi_ac_block_sum += filtered * filtered;

    if (!_pi_block_clock.tick()) {         // no block completed on this sample: pi keeps its last value —
        if (_pi_buf_count < (uint32_t)_pi_buf_n) _current_data.pi = NAN;   // which is NaN while filling
        return;
    }

    // Fixed-cost sliding windows, one cadence: add the incoming block sum, subtract the one falling
    // out of the pi_window_s window, for AC² and for DC alike. True boxcars, not EMAs — PI
    // settles EXACTLY at pi_window_s (contrast EMA's ~3·τ to reach ~95%) and its numerator and
    // denominator always span the same samples.
    const float ac_block = _pi_ac_block_sum, dc_block = _pi_dc_block_sum;
    _pi_ac_block_sum = 0.0f; _pi_dc_block_sum = 0.0f;
    _pi_ac_sum += ac_block - _pi_ac_buf[_pi_buf_idx];
    _pi_dc_sum += dc_block - _pi_dc_buf[_pi_buf_idx];
    _pi_ac_buf[_pi_buf_idx] = ac_block;
    _pi_dc_buf[_pi_buf_idx] = dc_block;
    if (++_pi_buf_idx >= _pi_buf_n) {
        // Wrap: rebuild the totals from the buffers (float32 drift guard, as the SpO2 window).
        _pi_buf_idx = 0;
        float ac = 0.0f, dc_acc = 0.0f;
        for (int i = 0; i < _pi_buf_n; i++) { ac += _pi_ac_buf[i]; dc_acc += _pi_dc_buf[i]; }
        _pi_ac_sum = ac; _pi_dc_sum = dc_acc;
    }
    if (_pi_buf_count < (uint32_t)_pi_buf_n) {
        _pi_buf_count++;
        _current_data.pi = NAN;   // window not full yet — no stale/partial estimate
        return;
    }

    // Peak-to-peak units (2√2·RMS, the industry's PI — see pi_rms_to_pp_ratio): AC_pp / DC × 100 [%].
    // Both sums span _pi_buf_n blocks of _pi_block_clock.len full-rate samples.
    const float n_raw = (float)_pi_buf_n * (float)_pi_block_clock.len;
    const float dc    = _pi_dc_sum / n_raw;
    const float ac_pp = pi_rms_to_pp_ratio * sqrtf(_pi_ac_sum / n_raw);
    _current_data.pi = (dc > pi_div_eps) ? (ac_pp / dc) * 100.0f : NAN;
}

// ── HR algorithm ──────────────────────────────────────────────────────────────
// Adaptive-threshold peak detection on filtered PPG.
// Threshold = 0.6 × running_max; refractory = _hr1_refractory_samples.
// HR reported from average of 5 consecutive RR intervals.
// probe_state: RSQM's classification, consumed only (mirrors SpO2 v0.41 — see spec §5.1/§5.2).
void INCUNEST_AFE4490::_hr1_update(float ir, ProbeState probe_state) {
    if (probe_state != ProbeState::PROBE_APPLIED) {
        // Reset every sample while not applied (idempotent — no stored "previous state"
        // needed, fully self-contained). Also resets the MA buffer, which the original
        // _reset_algorithms() omitted — a stale MA buffer would otherwise briefly bias the
        // peak-detection filter right after re-application.
        _hr1_dc                = 0.0f;
        _hr1_running_max       = 0.0f;
        _hr1_ppg_above_thresh  = false;
        _hr1_last_peak_idx     = 0;
        _hr1_sample_idx        = 0;
        _hr1_interval_count    = 0;
        memset(_hr1_intervals, 0, sizeof(_hr1_intervals));
        memset(_hr1_ma_buf, 0, sizeof(_hr1_ma_buf));
        _hr1_ma_sum = 0.0f;
        _hr1_ma_idx = 0;
        _current_data.hr1     = NAN;
        _current_data.hr1_sqi = 0.0f;
        return;
    }

    _hr1_sample_idx++;

    // DC removal: IIR estimator (tau = hr1_dc_tau_s), then negate for conventional PPG polarity (peaks up)
    _hr1_dc = _hr1_dc_alpha * _hr1_dc + (1.0f - _hr1_dc_alpha) * ir;
    // Apply dedicated MA low-pass filter (5 Hz cutoff, independent of PPG display filter)
    float raw = -(ir - _hr1_dc);
    _hr1_ma_sum -= _hr1_ma_buf[_hr1_ma_idx];
    _hr1_ma_buf[_hr1_ma_idx] = raw;
    _hr1_ma_sum += raw;
    _hr1_ma_idx = (_hr1_ma_idx + 1) % (int)_hr1_ma_len;
    float ppg_filtered = _hr1_ma_sum / (float)_hr1_ma_len;

    // Running max: slow exponential decay keeps it tracking signal amplitude
    _hr1_running_max = fmaxf(_hr1_running_max * _hr1_max_decay_alpha, ppg_filtered);

    float threshold = _hr1_threshold_fraction * _hr1_running_max;

    // Threshold crossing (rising edge only)
    if (ppg_filtered > threshold && !_hr1_ppg_above_thresh) {
        _hr1_ppg_above_thresh = true;

        uint32_t elapsed = _hr1_sample_idx - _hr1_last_peak_idx;
        if (_hr1_last_peak_idx > 0 && elapsed > _hr1_refractory_samples) {
            // Shift interval buffer and store new interval
            for (int i = 4; i > 0; i--) _hr1_intervals[i] = _hr1_intervals[i - 1];
            _hr1_intervals[0] = (int32_t)elapsed;
            if (_hr1_interval_count < 5) _hr1_interval_count++;
        }
        _hr1_last_peak_idx = _hr1_sample_idx;

    } else if (ppg_filtered <= threshold) {
        _hr1_ppg_above_thresh = false;
    }

    // Need 5 intervals for a stable estimate
    if (_hr1_interval_count < 5) {
        _current_data.hr1     = NAN;
        _current_data.hr1_sqi = 0.0f;
        return;
    }

    float sum = 0.0f;
    for (int i = 0; i < 5; i++) sum += (float)_hr1_intervals[i];
    float avg_interval = sum / 5.0f;

    float hr1 = ((float)_afe_sample_rate_hz * 60.0f) / avg_interval;

    if (hr1 >= _hr_min_bpm && hr1 <= _hr_max_bpm) {
        _current_data.hr1 = hr1;
        // SQI: coefficient of variation (CV = std / mean) of the 5 RR intervals.
        // Perfectly regular rhythm → CV = 0 → SQI = 1.
        // CV ≥ hr1_sqi_cv_max (15%) → SQI = 0 (arrhythmia or motion artefact).
        float var = 0.0f;
        for (int i = 0; i < 5; i++) {
            float diff = (float)_hr1_intervals[i] - avg_interval;
            var += diff * diff;
        }
        float cv  = (avg_interval > 0.0f) ? sqrtf(var / 5.0f) / avg_interval : 1.0f;
        float sqi = 1.0f - cv / _hr1_sqi_cv_max;
        _current_data.hr1_sqi = fmaxf(0.0f, fminf(1.0f, sqi));
    } else {
        _current_data.hr1     = NAN;
        _current_data.hr1_sqi = 0.0f;
    }
}

// ── HR2 algorithm — split into fast path + async computation ──────────────────

// Fast path: called every raw sample from Task A.
// Returns true when the computation window fires (buffer full, interval elapsed).
// probe_state gate lives here (shared by the production call site and the sync test
// wrapper _hr2_update_for_test()): while not applied, resets fast-path state every sample
// (idempotent) and never triggers the slow path (Task B never even gets signalled).
bool INCUNEST_AFE4490::_hr2_update_sample(float ir, ProbeState probe_state) {
    if (probe_state != ProbeState::PROBE_APPLIED) {
        _hr2_bpf.reset();
        _hr2_buf_idx = 0; _hr2_buf_count = 0;
        _hr2_decim.reset(); _hr2_update_counter = 0;
        memset(_hr2_buf, 0, sizeof(_hr2_buf));
        _current_data.hr2     = NAN;
        _current_data.hr2_sqi = 0.0f;
        return false;
    }

    float filtered = -_hr2_bpf.process(ir);

    if (!_hr2_decim.step()) return false;

    _hr2_buf[_hr2_buf_idx] = filtered;
    _hr2_buf_idx = (_hr2_buf_idx + 1) % hr2_buf_len;
    if (_hr2_buf_count < (uint32_t)hr2_buf_len) _hr2_buf_count++;

    _hr2_update_counter++;
    if (_hr2_update_counter < _hr2_update_interval) return false;
    _hr2_update_counter = 0;

    return _hr2_buf_count >= (uint32_t)hr2_buf_len;
}

// Freeze the circular buffer into _hr2_snapshot, linearised oldest → newest, so Task B can
// work on a coherent window while Task A keeps writing _hr2_buf.
// Called under _state_mutex by Task A immediately before signalling Task B.
void INCUNEST_AFE4490::_hr2_take_snapshot() {
    for (int i = 0; i < hr2_buf_len; i++)
        _hr2_snapshot[i] = _hr2_buf[(_hr2_buf_idx + i) % hr2_buf_len];
}

// Slow path: autocorrelation on _hr2_snapshot → _hr2_result / _hr2_result_sqi.
// Called by Task B. Reads only _hr2_snapshot (no other shared state written).
void INCUNEST_AFE4490::_hr2_compute() {
    float acorr0 = 0.0f;
    for (int i = 0; i < hr2_buf_len; i++) acorr0 += _hr2_snapshot[i] * _hr2_snapshot[i];
    if (acorr0 < hr2_ot_energy_eps) { _hr2_result_sqi = 0.0f; return; }

    int   min_lag = (int)(60.0f / (_hr_max_bpm + hr_guard_band_bpm) * _hr2_decim.rate_hz);
    if (min_lag < 1) min_lag = 1;

    // Upper bound of the lag sweep = slowest accepted heart rate, converted to lag at the
    // *current* decimated rate. Symmetric with min_lag above: both derive from the configured
    // HR range plus the same 3 BPM guard band, so the sweep follows setHrRange() and the
    // sample rate instead of assuming 50 Hz.
    //
    // Previously a hardcoded 137 samples ("22 BPM at 50 Hz"). Being fixed in *samples* while
    // HR2's decimated rate is not, it silently raised the lowest detectable HR
    // whenever the sample rate left 500 Hz: 43.8 BPM at 1000 Hz, 87.6 BPM at 2000 Hz — both
    // above the then-default 40 BPM _hr_min_bpm, so real bradycardia would have been reported as
    // "no periodicity" (sqi = 0) instead of as a low HR.
    float lag_min_bpm = fmaxf(_hr_min_bpm - hr_guard_band_bpm, 1.0f);  // setHrRange() could reach ≤3
    int   max_lag     = (int)(60.0f / lag_min_bpm * _hr2_decim.rate_hz);
    if (max_lag > hr2_acorr_lag_cap)   max_lag = hr2_acorr_lag_cap;    // scratch-array capacity
    if (max_lag > hr2_buf_len / 2 - 1) max_lag = hr2_buf_len / 2 - 1;  // unbiased-ACF noise limit

    // The peak search below inspects interior lags only, so it needs at least 3 of them.
    if (max_lag < min_lag + 2) { _hr2_result_sqi = 0.0f; return; }

    int   n_lags  = max_lag - min_lag + 1;

    // Unbiased normalised autocorrelation: divide by (acorr0 * (N-lag)/N) instead of acorr0.
    // The biased estimator (/ acorr0) systematically underestimates because its numerator
    // sums only (N-lag) terms while the denominator reflects N terms. For a perfectly
    // periodic signal this yields SQI = (N-lag)/N < 1 — e.g. ~0.875 at 60 BPM (lag=50,
    // N=400). The unbiased correction restores SQI ≈ 1.0 for a clean periodic signal.
    float acorr_buf[hr2_acorr_lag_cap + 1];
    for (int lag = min_lag; lag <= max_lag; lag++) {
        float sum   = 0.0f;
        int   n     = hr2_buf_len - lag;
        for (int i = 0; i < n; i++) sum += _hr2_snapshot[i] * _hr2_snapshot[i + lag];
        acorr_buf[lag - min_lag] = (n > 0) ? sum * (float)hr2_buf_len / (acorr0 * (float)n) : 0.0f;
    }

    int   peak_idx = -1;
    float y_prev = 0.0f, y_peak = 0.0f, y_next = 0.0f;
    for (int i = 1; i < n_lags - 1; i++) {
        if (acorr_buf[i] > acorr_buf[i - 1] &&
            acorr_buf[i] > acorr_buf[i + 1] &&
            acorr_buf[i] >= _hr2_min_corr) {
            peak_idx = i;
            y_prev   = acorr_buf[i - 1];
            y_peak   = acorr_buf[i];
            y_next   = acorr_buf[i + 1];
            break;
        }
    }

    if (peak_idx < 0) { _hr2_result_sqi = 0.0f; return; }

    float denom      = y_prev - 2.0f * y_peak + y_next;
    float delta      = (denom < 0.0f) ? 0.5f * (y_prev - y_next) / denom : 0.0f;
    float peak_lag_s = (float)(min_lag + peak_idx + delta) / _hr2_decim.rate_hz;

    if (peak_lag_s <= 0.0f) { _hr2_result_sqi = 0.0f; return; }

    float hr2 = 60.0f / peak_lag_s;
    if (hr2 >= _hr_min_bpm && hr2 <= _hr_max_bpm) {
        _hr2_result     = hr2;
        _hr2_result_sqi = y_peak;
    } else {
        _hr2_result_sqi = 0.0f;
    }
}

// Synchronous wrapper kept for unit-test compatibility.
void INCUNEST_AFE4490::_hr2_update_for_test(float ir, ProbeState probe_state) {
    if (!_hr2_update_sample(ir, probe_state)) return;
    _hr2_take_snapshot();
    _hr2_compute();
    _current_data.hr2     = (_hr2_result_sqi > 0.0f) ? _hr2_result : NAN;
    _current_data.hr2_sqi = _hr2_result_sqi;
}

// ── HR3 algorithm — split into fast path + async computation ──────────────────

// Fast path: called every raw sample from Task A.
// Returns true when the computation window fires (buffer full, interval elapsed).
// probe_state gate lives here (shared by the production call site and the sync test
// wrapper _hr3_update_for_test()): while not applied, resets fast-path state every sample
// (idempotent) and never triggers the slow path (Task C never even gets signalled).
bool INCUNEST_AFE4490::_hr3_update_sample(float ir, ProbeState probe_state) {
    if (probe_state != ProbeState::PROBE_APPLIED) {
        _hr3_lp.reset();
        _hr3_buf_idx = 0; _hr3_buf_count = 0;
        _hr3_decim.reset(); _hr3_update_counter = 0;
        memset(_hr3_buf, 0, sizeof(_hr3_buf));
        _current_data.hr3     = NAN;
        _current_data.hr3_sqi = 0.0f;
        return false;
    }

    float filtered = -_hr3_lp.process(ir);   // negate: peaks up

    if (!_hr3_decim.step()) return false;

    _hr3_buf[_hr3_buf_idx] = filtered;
    _hr3_buf_idx = (_hr3_buf_idx + 1) % hr3_buf_len;
    if (_hr3_buf_count < (uint32_t)hr3_buf_len) _hr3_buf_count++;

    _hr3_update_counter++;
    if (_hr3_update_counter < _hr3_update_interval) return false;
    _hr3_update_counter = 0;

    return _hr3_buf_count >= (uint32_t)hr3_buf_len;
}

// Build Task C's FFT input in _hr3_fft: snapshot the circular buffer (linearised oldest →
// newest), subtract the window mean (this is what removes DC — the 15 Hz low-pass does not,
// see spec §5.4) and apply the precomputed Hann window.
// Called under _state_mutex by Task A immediately before signalling Task C.
void INCUNEST_AFE4490::_hr3_prepare_fft_input() {
    float mean = 0.0f;
    for (int i = 0; i < hr3_buf_len; i++)
        mean += _hr3_buf[(_hr3_buf_idx + i) % hr3_buf_len];
    mean /= (float)hr3_buf_len;

    for (int i = 0; i < hr3_buf_len; i++) {
        float sample = _hr3_buf[(_hr3_buf_idx + i) % hr3_buf_len] - mean;
        _hr3_fft[2 * i]     = sample * _hr3_hann[i];  // real — uses precomputed Hann window
        _hr3_fft[2 * i + 1] = 0.0f;                   // imag
    }
}

// Slow path: FFT + HPS on _hr3_fft → _hr3_result / _hr3_result_sqi.
// Called by Task C. Reads/writes only _hr3_fft and result floats (no other shared state).
void INCUNEST_AFE4490::_hr3_compute() {
    // ── In-place radix-2 DIT FFT ──
    _fft_r2(_hr3_fft, hr3_buf_len);

    // ── Find HPS peak in guard-band search range ──
    float bin_res  = _hr3_decim.rate_hz / (float)hr3_buf_len;

    int search_min = (int)ceilf((_hr_min_bpm - hr_guard_band_bpm) / 60.0f / bin_res);
    int search_max = (int)floorf((_hr_max_bpm + hr_guard_band_bpm) / 60.0f / bin_res);
    int nyquist    = hr3_buf_len / 2;
    if (search_max >= nyquist)    search_max = nyquist - 2;
    if (search_max > nyquist / 3) search_max = nyquist / 3;  // 3rd harmonic must stay inside Nyquist
    if (search_min < 1)           search_min = 1;
    if (search_min >= search_max) { _hr3_result_sqi = 0.0f; return; }

    // HPS = P[k] * P[2k] * P[3k]  where P[k] = re[k]^2 + im[k]^2
    // _hr3_fft is stored interleaved: [re0, im0, re1, im1, ...], so bin k occupies
    // indices 2k (real) and 2k+1 (imaginary). P[k] = re[k]^2 + im[k]^2 is the power
    // of a single spectral bin — not two bins.
    auto hps_at = [&](int k) -> float {
        float p1 = _hr3_fft[2*k]   * _hr3_fft[2*k]   + _hr3_fft[2*k+1]   * _hr3_fft[2*k+1];  // P[k]  = |FFT[k]|²
        float p2 = _hr3_fft[4*k]   * _hr3_fft[4*k]   + _hr3_fft[4*k+1]   * _hr3_fft[4*k+1];  // P[2k] = |FFT[2k]|²
        float p3 = _hr3_fft[6*k]   * _hr3_fft[6*k]   + _hr3_fft[6*k+1]   * _hr3_fft[6*k+1];  // P[3k] = |FFT[3k]|²
        return p1 * p2 * p3;
    };

    int   peak_bin  = -1;
    float peak_hps  = 0.0f;
    for (int k = search_min; k <= search_max; k++) {
        float hps = hps_at(k);
        if (hps > peak_hps) { peak_hps = hps; peak_bin = k; }
    }

    // ── Gaussian interpolation: sub-bin interpolation on log|X|² ────────────
    // For a Hann-windowed DFT, adjacent bins around the peak carry a phase
    // factor e^{±jπ} = -1 relative to the peak bin.  This causes the Jacobsen
    // formula (Re{(X[k+1]-X[k-1])/(2X[k]-X[k+1]-X[k-1])}) to invert its sign
    // and return δ ≈ −0.5·δ_true.  Parabolic on linear |X|² underestimates by
    // ~50% because the Hann main lobe is not parabolic in power scale.
    // Gaussian interpolation (parabolic fit on log|X|²) gives δ ≈ 1.07·δ_true
    // for a pure tone, reducing HR3 error from ~0.5 BPM to < 0.1 BPM.
    float p_m = _hr3_fft[2*(peak_bin-1)]*_hr3_fft[2*(peak_bin-1)] + _hr3_fft[2*(peak_bin-1)+1]*_hr3_fft[2*(peak_bin-1)+1];
    float p_c = _hr3_fft[2* peak_bin  ]*_hr3_fft[2* peak_bin  ] + _hr3_fft[2* peak_bin  +1]*_hr3_fft[2* peak_bin  +1];
    float p_p = _hr3_fft[2*(peak_bin+1)]*_hr3_fft[2*(peak_bin+1)] + _hr3_fft[2*(peak_bin+1)+1]*_hr3_fft[2*(peak_bin+1)+1];
    float lm    = logf(p_m > 0.0f ? p_m : 1e-30f);
    float lc    = logf(p_c > 0.0f ? p_c : 1e-30f);
    float lp    = logf(p_p > 0.0f ? p_p : 1e-30f);
    float denom = lm - 2.0f*lc + lp;   // < 0 for a genuine power peak
    float delta = (denom != 0.0f) ? 0.5f*(lm - lp) / denom : 0.0f;
    float peak_freq = ((float)peak_bin + delta) * bin_res;  // Hz

    if (peak_freq <= 0.0f) { _hr3_result_sqi = 0.0f; return; }

    float hr3 = 60.0f * peak_freq;
    if (hr3 >= _hr_min_bpm && hr3 <= _hr_max_bpm) {
        _hr3_result = hr3;

        // ── SQI: two-bin HPS local SNR ───────────────────────────────────────────
        // Due to the Hann window, a real signal at a non-integer bin position
        // (peak_bin + delta) splits its energy across b1=floor(peak_bin+delta) and
        // b2=b1+1. A metric based on a single bin is therefore bin-position-dependent.
        //
        // Fix: use (hps(b1) + hps(b2)) as numerator, where b1=floor(peak_bin+delta)
        // and b2=b1+1. Since delta∈[-0.5, 0.5], b1 is always peak_bin or peak_bin-1,
        // so peak_bin is always in {b1,b2} → hps_num ≥ peak_hps always.
        //
        // Denominator: sum of HPS over local window [b1-W, b2+W] of width
        // 2·hr3_snr_local_w+2 bins. Local window excludes distant harmonics that
        // would inflate the denominator for high-HR signals.
        //
        // Normalised against a uniform-noise baseline (2 / n_window) so that:
        //   SQI = 0  when snr == baseline (flat noise floor)
        //   SQI = 1  when all HPS energy in the window sits in b1 and b2
        int   b1 = (int)floorf((float)peak_bin + delta);   // floor of fractional bin
        int   b2 = b1 + 1;                                 // ceil  of fractional bin
        if (b1 < 1)            b1 = 1;
        if (b2 > nyquist / 3)  b2 = nyquist / 3;
        float hps_b1  = (b1 == peak_bin) ? peak_hps : hps_at(b1);
        float hps_b2  = (b2 == peak_bin) ? peak_hps : hps_at(b2);
        float hps_num = hps_b1 + hps_b2;
        int   b_lo = b1 - hr3_snr_local_w;
        int   b_hi = b2 + hr3_snr_local_w;
        float hps_win = 0.0f;
        int   n_win   = 0;
        for (int k = b_lo; k <= b_hi; k++) {
            if (k < 1 || k > nyquist / 3) continue;
            float h = (k == b1) ? hps_b1 :
                      (k == b2) ? hps_b2 : hps_at(k);
            hps_win += h;
            n_win++;
        }
        if (hps_win > 0.0f && n_win > 2) {
            float snr      = hps_num / hps_win;
            float baseline = 2.0f / (float)n_win;  // expected snr under flat noise: numerator spans exactly 2 bins (b1+b2) out of n_win
            _hr3_result_sqi = fmaxf(0.0f, fminf(1.0f,
                (snr - baseline) / (1.0f - baseline)));
        } else {
            _hr3_result_sqi = 0.0f;  // insufficient window → conservative
        }
    } else {
        _hr3_result_sqi = 0.0f;
    }
}

// Synchronous wrapper kept for unit-test compatibility.
void INCUNEST_AFE4490::_hr3_update_for_test(float ir, ProbeState probe_state) {
    if (!_hr3_update_sample(ir, probe_state)) return;
    _hr3_prepare_fft_input();
    _hr3_compute();
    _current_data.hr3     = (_hr3_result_sqi > 0.0f) ? _hr3_result : NAN;
    _current_data.hr3_sqi = _hr3_result_sqi;
}

// ── HR2 async task (Task B) ────────────────────────────────────────────────────
void INCUNEST_AFE4490::_hr2_task_trampoline(void* pv) {
    static_cast<INCUNEST_AFE4490*>(pv)->_hr2_task_body();
    vTaskDelete(nullptr);
}

// Blocks on _hr2_compute_sem. When signalled by Task A:
//   1. Runs autocorrelation on the _hr2_snapshot taken by Task A.
//   2. Takes _state_mutex to write results into _current_data.
//   3. Clears _hr2_computing so Task A knows the slot is free.
void INCUNEST_AFE4490::_hr2_task_body() {
    for (;;) {
        xSemaphoreTake(_hr2_compute_sem, portMAX_DELAY);

#if INCUNEST_TIMING_STATS
        { uint64_t _t = esp_timer_get_time(); _hr2_compute(); _ts_hr2_compute.update(esp_timer_get_time() - _t); }
#else
        _hr2_compute();  // reads _hr2_snapshot only — no shared-state write
#endif

        xSemaphoreTake(_state_mutex, portMAX_DELAY);
        _current_data.hr2     = (_hr2_result_sqi > 0.0f) ? _hr2_result : NAN;
        _current_data.hr2_sqi = _hr2_result_sqi;
        xSemaphoreGive(_state_mutex);

        _hr2_computing = false;  // release slot after results committed
    }
}

// ── HR3 async task (Task C) ────────────────────────────────────────────────────
void INCUNEST_AFE4490::_hr3_task_trampoline(void* pv) {
    static_cast<INCUNEST_AFE4490*>(pv)->_hr3_task_body();
    vTaskDelete(nullptr);
}

// Blocks on _hr3_compute_sem. When signalled by Task A:
//   1. Runs FFT + HPS on the already-prepared _hr3_fft buffer.
//   2. Takes _state_mutex to write results into _current_data.
//   3. Clears _hr3_computing so Task A knows the slot is free.
void INCUNEST_AFE4490::_hr3_task_body() {
    for (;;) {
        xSemaphoreTake(_hr3_compute_sem, portMAX_DELAY);

#if INCUNEST_TIMING_STATS
        { uint64_t _t = esp_timer_get_time(); _hr3_compute(); _ts_hr3_compute.update(esp_timer_get_time() - _t); }
#else
        _hr3_compute();  // reads/writes _hr3_fft only — no other shared state
#endif

        xSemaphoreTake(_state_mutex, portMAX_DELAY);
        _current_data.hr3     = (_hr3_result_sqi > 0.0f) ? _hr3_result : NAN;
        _current_data.hr3_sqi = _hr3_result_sqi;
        xSemaphoreGive(_state_mutex);

        _hr3_computing = false;  // release slot after results committed
    }
}
