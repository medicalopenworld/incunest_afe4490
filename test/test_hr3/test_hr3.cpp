#include <unity.h>
#include <math.h>
#include <stdlib.h>
#include "incunest_afe4490.h"

// EXPERIMENT (OT-domain input, branch experiment/ot-domain-inputs): HR3 now consumes OT
// (dimensionless A/A, ~1e-5 typical) instead of raw ambient-corrected ADC counts. SQI is a
// ratio of HPS values (same power units, cancels scale), so a uniform gain scale does not
// change the result — pure domain/type change, no threshold recalibration needed.
//
// probe_state (RSQM's classification) is consumed, never computed here — mirrors SpO2 v0.41
// (see incunest_afe4490_spec.md §5.1/§5.2). While probe_state != PROBE_APPLIED, the fast-path
// buffer resets every sample (never triggers the slow FFT/HPS path) and hr3/hr3_sqi
// become NaN/0.

// HR3 constants (mirror of incunest_afe4490.cpp namespace)
static constexpr int HR3_BUF_LEN      = 512;   // decimated samples
static constexpr int HR3_DECIM_FACTOR = 10;
static constexpr int HR3_BUF_RAW      = HR3_BUF_LEN * HR3_DECIM_FACTOR;  // 5120 raw samples

static constexpr float OT_DC = 1.4e-5f;
static constexpr float SCALE = 1.4e-10f;

// Helper: feed N raw samples of a PPG-like signal (fundamental + 2nd + 3rd harmonic) into HR3,
// in OT-scale units. HR3 uses the Harmonic Product Spectrum (HPS = P[k]*P[2k]*P[3k]), which
// requires energy at the harmonic frequencies to produce a clear peak. A pure sine would
// yield near-zero HPS at all bins and SQI ≈ 0. All harmonics stay below the 10 Hz LP filter
// cutoff for freq_hz ≤ 3 Hz.
static void feed_hr3_sine(INCUNEST_AFE4490& afe, float freq_hz, float fs, int n_samples,
                          ProbeState probe_state = ProbeState::PROBE_APPLIED) {
    for (int i = 0; i < n_samples; i++) {
        float t = (float)i / fs;
        float x = OT_DC
                + 40000.0f * SCALE * sinf(2.0f * (float)M_PI * freq_hz * t)   // fundamental
                + 20000.0f * SCALE * sinf(4.0f * (float)M_PI * freq_hz * t)   // 2nd harmonic
                +  8000.0f * SCALE * sinf(6.0f * (float)M_PI * freq_hz * t);  // 3rd harmonic
        afe.test_feed_hr3(x, probe_state);
    }
}

// Helper: same as feed_hr3_sine but with uniform noise (~10% of fundamental, ~20 dB SNR).
// srand(42) called by the test before use for reproducibility.
static void feed_hr3_sine_noisy(INCUNEST_AFE4490& afe, float freq_hz, float fs, int n_samples) {
    for (int i = 0; i < n_samples; i++) {
        float t = (float)i / fs;
        float noise = 4000.0f * SCALE * (2.0f * (float)rand() / (float)RAND_MAX - 1.0f);
        float x = OT_DC
                + 40000.0f * SCALE * sinf(2.0f * (float)M_PI * freq_hz * t)   // fundamental
                + 20000.0f * SCALE * sinf(4.0f * (float)M_PI * freq_hz * t)   // 2nd harmonic
                +  8000.0f * SCALE * sinf(6.0f * (float)M_PI * freq_hz * t)   // 3rd harmonic
                + noise;
        afe.test_feed_hr3(x, ProbeState::PROBE_APPLIED);
    }
}

void setUp() {}
void tearDown() {}

// ── Test 1: not valid until buffer is full ────────────────────────────────────
// HR3 needs HR3_BUF_LEN decimated samples before computing FFT. After half that,
// hr3_valid must be false.
void test_hr3_not_valid_until_buffer_full() {
    INCUNEST_AFE4490 afe;
    feed_hr3_sine(afe, 1.0f, 500.0f, HR3_BUF_RAW / 2);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, afe.test_hr3_sqi());
    // Note: unlike SpO2/HR1 (which run their full invalid-path logic every sample), HR3's
    // fast path simply returns early while the buffer is filling — no commit to
    // _current_data happens yet, so hr3 stays at its construction default (0.0f), not NaN.
}

// ── Test 2: 60 BPM (1 Hz sine) ───────────────────────────────────────────────
// At 50 Hz decimated rate, 1 Hz → bin index 1 in a 512-point FFT (resolution ~0.098 Hz).
// HR3 should converge to 60 BPM ± 2 via parabolic interpolation on the HPS peak.
// SQI: dominant HPS peak at 1 Hz → SQI = 1.0. Threshold: > 0.95.
void test_hr3_60bpm() {
    INCUNEST_AFE4490 afe;
    feed_hr3_sine(afe, 1.0f, 500.0f, HR3_BUF_RAW + 1000);  // fill + margin
    TEST_ASSERT_GREATER_THAN_FLOAT(0.95f, afe.test_hr3_sqi());
    TEST_ASSERT_FLOAT_WITHIN(2.0f, 60.0f, afe.test_hr3());
}

// ── Test 3: 120 BPM (2 Hz sine) ──────────────────────────────────────────────
// SQI: dominant HPS peak at 2 Hz → SQI ≈ 0.73. Threshold: > 0.65.
// HR precision limited by FFT bin width (~0.098 Hz = ~5.9 BPM at 2 Hz).
void test_hr3_120bpm() {
    INCUNEST_AFE4490 afe;
    feed_hr3_sine(afe, 2.0f, 500.0f, HR3_BUF_RAW + 1000);
    TEST_ASSERT_GREATER_THAN_FLOAT(0.65f, afe.test_hr3_sqi());
    TEST_ASSERT_FLOAT_WITHIN(2.0f, 120.0f, afe.test_hr3());
}

// ── Test 3b: 85 BPM (worst-case inter-bin frequency) ─────────────────────────
// 85 BPM = 1.4167 Hz → bin 14.506, exactly halfway between bins 14 and 15.
// Without HPS interpolation, the cubic product loss reduces SQI to ~0.5 even
// for a clean signal. With parabolic HPS interpolation SQI must stay > 0.80.
void test_hr3_85bpm() {
    INCUNEST_AFE4490 afe;
    feed_hr3_sine(afe, 85.0f / 60.0f, 500.0f, HR3_BUF_RAW + 1000);
    TEST_ASSERT_GREATER_THAN_FLOAT(0.80f, afe.test_hr3_sqi());
    TEST_ASSERT_FLOAT_WITHIN(2.0f, 85.0f, afe.test_hr3());
}

// ── Test 4: flat signal → hr3_valid false ────────────────────────────────────
// A constant DC signal has zero AC energy after the LP filter.
// The FFT output is flat → no dominant HPS peak → SQI must be 0.
void test_hr3_flat_signal_invalid() {
    INCUNEST_AFE4490 afe;
    for (int i = 0; i < HR3_BUF_RAW + 1000; i++)
        afe.test_feed_hr3(OT_DC, ProbeState::PROBE_APPLIED);  // constant DC (OT scale), no PPG pulses
    TEST_ASSERT_EQUAL_FLOAT(0.0f, afe.test_hr3_sqi());
    TEST_ASSERT_TRUE(isnan(afe.test_hr3()));
}

// ── Test 5: 60 BPM with noise (~20 dB SNR) ───────────────────────────────────
// With ±10% noise the HPS peak should remain dominant. HR3 must converge to
// 60 BPM ± 2 and SQI > 0.95.
void test_hr3_60bpm_noisy() {
    INCUNEST_AFE4490 afe;
    srand(42);
    feed_hr3_sine_noisy(afe, 1.0f, 500.0f, HR3_BUF_RAW + 1000);
    TEST_ASSERT_GREATER_THAN_FLOAT(0.95f, afe.test_hr3_sqi());
    TEST_ASSERT_FLOAT_WITHIN(2.0f, 60.0f, afe.test_hr3());
}

// ── Test 6: 120 BPM with noise (~20 dB SNR) ──────────────────────────────────
void test_hr3_120bpm_noisy() {
    INCUNEST_AFE4490 afe;
    srand(42);
    feed_hr3_sine_noisy(afe, 2.0f, 500.0f, HR3_BUF_RAW + 1000);
    TEST_ASSERT_GREATER_THAN_FLOAT(0.65f, afe.test_hr3_sqi());
    TEST_ASSERT_FLOAT_WITHIN(2.0f, 120.0f, afe.test_hr3());
}

// ── Test 7 (new): PROBE_DISCONNECTED/NOT_APPLIED forces invalid + resets state ─
// HR3 never classifies presence itself — it only consumes probe_state. Feeding a converged
// signal, then switching to PROBE_DISCONNECTED must: force sqi=0/hr3=NaN, reset the fast-path
// buffer, and require a full fresh buffer once PROBE_APPLIED resumes (never triggers the
// slow FFT/HPS path while not applied).
void test_hr3_not_applied_resets() {
    INCUNEST_AFE4490 afe;
    feed_hr3_sine(afe, 1.0f, 500.0f, HR3_BUF_RAW + 1000);
    TEST_ASSERT_GREATER_THAN_FLOAT(0.95f, afe.test_hr3_sqi());  // valid before disconnecting

    feed_hr3_sine(afe, 1.0f, 500.0f, 1000, ProbeState::PROBE_DISCONNECTED);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, afe.test_hr3_sqi());
    TEST_ASSERT_TRUE(isnan(afe.test_hr3()));

    // Re-applying must require a full fresh buffer, not resume instantly from stale state.
    feed_hr3_sine(afe, 1.0f, 500.0f, HR3_BUF_RAW / 2);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, afe.test_hr3_sqi());

    // PROBE_OT_HIGH gets the same treatment as PROBE_DISCONNECTED.
    feed_hr3_sine(afe, 1.0f, 500.0f, HR3_BUF_RAW + 1000);
    TEST_ASSERT_GREATER_THAN_FLOAT(0.95f, afe.test_hr3_sqi());
    feed_hr3_sine(afe, 1.0f, 500.0f, 1000, ProbeState::PROBE_OT_HIGH);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, afe.test_hr3_sqi());
}

// ── Test 3c: 30 BPM — lower bound of the configured HR range (lib v0.99) ─────
// 30 BPM = 0.5 Hz = bin 5.12 at 50 Hz / 512; search_min = ceil(27/60/0.0977) = 5, so the floor
// is searchable with no margin to spare. Clean PPG-like input: measured 30.06, SQI 1.00. Under
// 2× baseline wander HR3 loses 30–33 BPM (no high-pass; rationale §8) — not pinned, a known limit.
void test_hr3_30bpm_lower_bound() {
    INCUNEST_AFE4490 afe;
    feed_hr3_sine(afe, 30.0f / 60.0f, 500.0f, HR3_BUF_RAW + 1000);
    TEST_ASSERT_GREATER_THAN_FLOAT(0.95f, afe.test_hr3_sqi());
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 30.0f, afe.test_hr3());
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_hr3_not_valid_until_buffer_full);
    RUN_TEST(test_hr3_60bpm);
    RUN_TEST(test_hr3_120bpm);
    RUN_TEST(test_hr3_85bpm);
    RUN_TEST(test_hr3_30bpm_lower_bound);
    RUN_TEST(test_hr3_flat_signal_invalid);
    RUN_TEST(test_hr3_60bpm_noisy);
    RUN_TEST(test_hr3_120bpm_noisy);
    RUN_TEST(test_hr3_not_applied_resets);
    return UNITY_END();
}
