#include <unity.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "incunest_afe4490.h"

// EXPERIMENT (OT-domain input, branch experiment/ot-domain-inputs): HR1 now consumes OT
// (dimensionless A/A, ~1e-5 typical per incunest_afe4490_spec.md) instead of raw
// ambient-corrected ADC counts. HR1 is peak-timing based (not a ratio like SpO2's R), so a
// uniform gain scale does not change peak timing — this is a pure domain/type change, no
// threshold recalibration needed (confirmed: _hr1_running_max/threshold are self-relative).
//
// probe_state (RSQM's classification) is consumed, never computed here — mirrors SpO2 v0.41
// (see incunest_afe4490_spec.md §5.1/§5.2). While probe_state != PROBE_APPLIED, internal
// state resets every sample and hr1/hr1_sqi become NaN/0.

// Helper: feed N samples of a sine at freq_hz (with DC offset) into HR1, in OT-scale units.
static void feed_hr1_sine(INCUNEST_AFE4490& afe, float freq_hz, float fs, int n_samples,
                           ProbeState probe_state = ProbeState::PROBE_APPLIED) {
    const float ot_dc = 1.4e-5f;
    const float scale = 1.4e-10f;
    for (int i = 0; i < n_samples; i++) {
        float x = ot_dc + 50000.0f * scale * sinf(2.0f * (float)M_PI * freq_hz * i / fs);
        afe.test_feed_hr1(x, probe_state);
    }
}

// Helper: same as feed_hr1_sine but with uniform noise (~10% of amplitude, ~20 dB SNR).
// srand(42) called by the test before use for reproducibility.
static void feed_hr1_sine_noisy(INCUNEST_AFE4490& afe, float freq_hz, float fs, int n_samples) {
    const float ot_dc = 1.4e-5f;
    const float scale = 1.4e-10f;
    for (int i = 0; i < n_samples; i++) {
        float noise = 5000.0f * scale * (2.0f * (float)rand() / (float)RAND_MAX - 1.0f);
        float x = ot_dc + 50000.0f * scale * sinf(2.0f * (float)M_PI * freq_hz * i / fs) + noise;
        afe.test_feed_hr1(x, ProbeState::PROBE_APPLIED);
    }
}

void setUp() {}
void tearDown() {}

// ── Test 1: not valid until 5 intervals have been detected ────────────────────
// After only 1 second of signal (< 2 complete cycles at 1 Hz = 60 BPM),
// hr1_valid must be false.
void test_hr1_not_valid_too_soon() {
    INCUNEST_AFE4490 afe;
    feed_hr1_sine(afe, 1.0f, 500.0f, 500);  // 1 second — not enough intervals
    TEST_ASSERT_EQUAL_FLOAT(0.0f, afe.test_hr1_sqi());
    TEST_ASSERT_TRUE(isnan(afe.test_hr1()));
}

// ── Test 2: 60 BPM (1 Hz sine) ───────────────────────────────────────────────
// After enough samples, HR1 should converge to 60 BPM ± 1.
// SQI: a synthetic sine has near-zero RR jitter → SQI ≈ 1.0. Threshold: > 0.95.
void test_hr1_60bpm() {
    INCUNEST_AFE4490 afe;
    feed_hr1_sine(afe, 1.0f, 500.0f, 6000);  // 12 seconds — plenty of intervals
    TEST_ASSERT_GREATER_THAN_FLOAT(0.95f, afe.test_hr1_sqi());
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 60.0f, afe.test_hr1());
}

// ── Test 3: 120 BPM (2 Hz sine) ──────────────────────────────────────────────
// SQI continuous: synthetic sine → low jitter → SQI > 0.95.
void test_hr1_120bpm() {
    INCUNEST_AFE4490 afe;
    feed_hr1_sine(afe, 2.0f, 500.0f, 6000);
    TEST_ASSERT_GREATER_THAN_FLOAT(0.95f, afe.test_hr1_sqi());
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 120.0f, afe.test_hr1());
}

// ── Test 4: out-of-range signal → hr1_valid false ────────────────────────────
// A flat (DC only) signal has no peaks — hr1_valid must stay false.
void test_hr1_flat_signal_invalid() {
    INCUNEST_AFE4490 afe;
    for (int i = 0; i < 6000; i++)
        afe.test_feed_hr1(1.4e-5f, ProbeState::PROBE_APPLIED);  // constant DC (OT scale), no PPG pulses
    TEST_ASSERT_EQUAL_FLOAT(0.0f, afe.test_hr1_sqi());
    TEST_ASSERT_TRUE(isnan(afe.test_hr1()));
}

// ── Test 5: 60 BPM with noise (~20 dB SNR) ───────────────────────────────────
// With uniform noise ±10% of amplitude, HR1 must still converge to 60 BPM ± 2
// and SQI > 0.8. Noise raises RR jitter slightly, lowering SQI from 1.0 to ~0.99.
void test_hr1_60bpm_noisy() {
    INCUNEST_AFE4490 afe;
    srand(42);
    feed_hr1_sine_noisy(afe, 1.0f, 500.0f, 6000);
    TEST_ASSERT_GREATER_THAN_FLOAT(0.8f, afe.test_hr1_sqi());
    TEST_ASSERT_FLOAT_WITHIN(2.0f, 60.0f, afe.test_hr1());
}

// ── Test 6: 120 BPM with noise (~20 dB SNR) ──────────────────────────────────
void test_hr1_120bpm_noisy() {
    INCUNEST_AFE4490 afe;
    srand(42);
    feed_hr1_sine_noisy(afe, 2.0f, 500.0f, 6000);
    TEST_ASSERT_GREATER_THAN_FLOAT(0.8f, afe.test_hr1_sqi());
    TEST_ASSERT_FLOAT_WITHIN(2.0f, 120.0f, afe.test_hr1());
}

// ── Test 7 (new): PROBE_DISCONNECTED/NOT_APPLIED forces invalid + resets state ─
// HR1 never classifies presence itself — it only consumes probe_state. Feeding a converged
// signal, then switching to PROBE_DISCONNECTED must: force sqi=0/hr1=NaN, reset internal
// state (peak/interval tracking), and require 5 fresh intervals once PROBE_APPLIED resumes.
void test_hr1_not_applied_resets() {
    INCUNEST_AFE4490 afe;
    feed_hr1_sine(afe, 1.0f, 500.0f, 6000);
    TEST_ASSERT_GREATER_THAN_FLOAT(0.95f, afe.test_hr1_sqi());  // valid before disconnecting

    feed_hr1_sine(afe, 1.0f, 500.0f, 1000, ProbeState::PROBE_DISCONNECTED);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, afe.test_hr1_sqi());
    TEST_ASSERT_TRUE(isnan(afe.test_hr1()));

    // Re-applying must require fresh intervals, not resume instantly from stale state.
    feed_hr1_sine(afe, 1.0f, 500.0f, 500);  // 1 second — not enough intervals from scratch
    TEST_ASSERT_EQUAL_FLOAT(0.0f, afe.test_hr1_sqi());
    TEST_ASSERT_TRUE(isnan(afe.test_hr1()));

    // PROBE_OT_HIGH gets the same treatment as PROBE_DISCONNECTED.
    feed_hr1_sine(afe, 1.0f, 500.0f, 6000);
    TEST_ASSERT_GREATER_THAN_FLOAT(0.95f, afe.test_hr1_sqi());
    feed_hr1_sine(afe, 1.0f, 500.0f, 1000, ProbeState::PROBE_OT_HIGH);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, afe.test_hr1_sqi());
}

// ── Test 3b: 30 BPM — lower bound of the configured HR range (lib v0.99) ─────
// 5 RR intervals of 2 s: the first valid reading needs 6 crossings, ~10-11 s, the slowest
// first-report of the three estimators and the reason this feeds 20 s. Clean sine only: with a
// dicrotic-like harmonic bump and noise, HR1 double-counts at 30 BPM because the fixed 0.185 s
// refractory covers nothing of a 2 s period (rationale §5.3, §8) — a known limit. Since v0.122 the
// 0.5 Hz high-pass sits on this fundamental (-3 dB, phase lead): the same on every beat, so the
// RR does not change.
void test_hr1_30bpm_lower_bound() {
    INCUNEST_AFE4490 afe;
    feed_hr1_sine(afe, 30.0f / 60.0f, 500.0f, 10000);  // 20 s
    TEST_ASSERT_GREATER_THAN_FLOAT(0.95f, afe.test_hr1_sqi());
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 30.0f, afe.test_hr1());
}

// ── Test 9 (v0.122): no start-up transient ────────────────────────────────────
// A realistic OT (DC 1.4e-5, 1 % modulation) at 150 BPM from the moment the probe is applied.
// The v0.121 chain started its DC estimate from zero, so OT − DC stayed far below zero for about
// 1.6 s × ln(100) ≈ 7 s and the first HR1 came after ~10 s; the v0.122 filters precharge on their
// first sample, so the first value needs only 6 crossings (2 s) plus the filters' settling.
void test_hr1_starts_without_dc_transient() {
    INCUNEST_AFE4490 afe;
    const float fs = 500.0f;
    int first_valid = -1;
    for (int i = 0; i < 5000; i++) {
        float x = 1.4e-5f * (1.0f + 0.01f * sinf(2.0f * (float)M_PI * 2.5f * i / fs));
        afe.test_feed_hr1(x, ProbeState::PROBE_APPLIED);
        if (first_valid < 0 && afe.test_hr1_sqi() > 0.0f) first_valid = i;
    }
    TEST_ASSERT_TRUE_MESSAGE(first_valid >= 0, "HR1 never became valid");
    char msg[96];
    snprintf(msg, sizeof(msg), "first HR1 after %.2f s (v0.121: ~10 s)", first_valid / fs);
    TEST_ASSERT_LESS_THAN_FLOAT_MESSAGE(3.5f, first_valid / fs, msg);
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 150.0f, afe.test_hr1());
}

// ── Test 10 (v0.122): 240 BPM, the top of WHO-UNICEF's range ──────────────────
// A 4 Hz fundamental: the 15 Hz low-pass keeps it whole (the v0.121 moving average cut near 4.4 Hz).
void test_hr1_240bpm() {
    INCUNEST_AFE4490 afe;
    feed_hr1_sine(afe, 4.0f, 500.0f, 5000);   // 10 s
    TEST_ASSERT_GREATER_THAN_FLOAT(0.95f, afe.test_hr1_sqi());
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 240.0f, afe.test_hr1());
}

// ── Test 11 (v0.122): setHR1Filter validates its band ─────────────────────────
void test_hr1_filter_setter() {
    INCUNEST_AFE4490 afe;
    AFE4490Config c = afe.getConfig();
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.5f, c.hr1_f_low_hz);      // defaults (spec §10.4)
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 15.0f, c.hr1_f_high_hz);
    afe.setHR1Filter(0.0f, 15.0f);                              // f_low must be > 0: ignored
    afe.setHR1Filter(2.0f, 1.0f);                               // f_high must exceed f_low: ignored
    afe.setHR1Filter(0.5f, 60.0f);                              // above 0.45 × 100 Hz: ignored
    c = afe.getConfig();
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.5f, c.hr1_f_low_hz);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 15.0f, c.hr1_f_high_hz);
    afe.setHR1Filter(0.7f, 12.0f);                              // valid: applied
    c = afe.getConfig();
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.7f, c.hr1_f_low_hz);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 12.0f, c.hr1_f_high_hz);
    feed_hr1_sine(afe, 2.0f, 500.0f, 6000);
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 120.0f, afe.test_hr1());
}

// ── Test 12 (optional): replay a capture, for the comparison with the PulseNest bench ──
// Runs only when HR1_CAPTURE_BIN and HR1_OUT_CSV are set (PulseNest
// experiments/hr1_port_check_20261116.py sets them). The binary file: int32 n, float32 fs (a
// catalogue rate), n float32 OT_LED1, n int8 ProbeState. Every 0.5 s of each run of the probe applied
// (rows r0 + k·round(fs/2) - 1, as the bench's output_rows) it writes row, hr1, hr1_sqi.
void test_hr1_replay_capture() {
    const char* in_path  = getenv("HR1_CAPTURE_BIN");
    const char* out_path = getenv("HR1_OUT_CSV");
    if (!in_path || !out_path) TEST_IGNORE_MESSAGE("HR1_CAPTURE_BIN / HR1_OUT_CSV not set");
    FILE* f = fopen(in_path, "rb");
    TEST_ASSERT_NOT_NULL_MESSAGE(f, "cannot open HR1_CAPTURE_BIN");
    int32_t n = 0; float fs = 0.0f;
    TEST_ASSERT_EQUAL_INT(1, (int)fread(&n, sizeof(n), 1, f));
    TEST_ASSERT_EQUAL_INT(1, (int)fread(&fs, sizeof(fs), 1, f));
    float*  ot = (float*)malloc((size_t)n * sizeof(float));
    int8_t* ps = (int8_t*)malloc((size_t)n);
    TEST_ASSERT_NOT_NULL(ot); TEST_ASSERT_NOT_NULL(ps);
    TEST_ASSERT_EQUAL_INT(n, (int)fread(ot, sizeof(float), (size_t)n, f));
    TEST_ASSERT_EQUAL_INT(n, (int)fread(ps, 1, (size_t)n, f));
    fclose(f);
    INCUNEST_AFE4490 afe;
    afe.setSampleRate((uint16_t)lroundf(fs));
    FILE* o = fopen(out_path, "w");
    TEST_ASSERT_NOT_NULL_MESSAGE(o, "cannot open HR1_OUT_CSV");
    fprintf(o, "row,hr1,hr1_sqi\n");
    const int32_t step = (int32_t)lroundf(0.5f * fs);
    int32_t run_start = -1;
    for (int32_t r = 0; r < n; r++) {
        const ProbeState s = (ProbeState)ps[r];
        afe.test_feed_hr1(ot[r], s);
        if (s != ProbeState::PROBE_APPLIED) { run_start = -1; continue; }
        if (run_start < 0) run_start = r;
        if ((r - run_start) % step == step - 1)
            fprintf(o, "%ld,%.6f,%.6f\n", (long)r, (double)afe.test_hr1(), (double)afe.test_hr1_sqi());
    }
    fclose(o);
    free(ot); free(ps);
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_hr1_not_valid_too_soon);
    RUN_TEST(test_hr1_60bpm);
    RUN_TEST(test_hr1_120bpm);
    RUN_TEST(test_hr1_30bpm_lower_bound);
    RUN_TEST(test_hr1_flat_signal_invalid);
    RUN_TEST(test_hr1_60bpm_noisy);
    RUN_TEST(test_hr1_120bpm_noisy);
    RUN_TEST(test_hr1_not_applied_resets);
    RUN_TEST(test_hr1_starts_without_dc_transient);
    RUN_TEST(test_hr1_240bpm);
    RUN_TEST(test_hr1_filter_setter);
    RUN_TEST(test_hr1_replay_capture);
    return UNITY_END();
}
