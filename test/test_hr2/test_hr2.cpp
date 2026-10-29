#include <unity.h>
#include <math.h>
#include <stdlib.h>
#include "incunest_afe4490.h"

// EXPERIMENT (OT-domain input, branch experiment/ot-domain-inputs): HR2 now consumes OT
// (dimensionless A/A, ~1e-5 typical) instead of raw ambient-corrected ADC counts.
// Autocorrelation is normalised (divides by its own acorr0), so a uniform gain scale does
// not change the result — pure domain/type change. The near-zero-energy guard (acorr0 <
// hr2_ot_energy_eps) was recalibrated for OT magnitudes in incunest_afe4490.cpp.
//
// probe_state (RSQM's classification) is consumed, never computed here — mirrors SpO2 v0.41
// (see incunest_afe4490_spec.md §5.1/§5.2). While probe_state != PROBE_APPLIED, the fast-path
// buffer resets every sample (never triggers the slow autocorrelation path) and
// hr2/hr2_sqi become NaN/0.

// HR2 constants (mirror of incunest_afe4490.cpp namespace)
static constexpr int HR2_BUF_LEN      = 400;   // decimated samples
static constexpr int HR2_DECIM_FACTOR = 10;
static constexpr int HR2_BUF_RAW      = HR2_BUF_LEN * HR2_DECIM_FACTOR;  // 4000 raw samples

static constexpr float OT_DC = 1.4e-5f;
static constexpr float SCALE = 1.4e-10f;

// Helper: feed N raw samples of a sine at freq_hz into HR2, in OT-scale units.
static void feed_hr2_sine(INCUNEST_AFE4490& afe, float freq_hz, float fs, int n_samples,
                           ProbeState probe_state = ProbeState::PROBE_APPLIED) {
    for (int i = 0; i < n_samples; i++) {
        float x = OT_DC + 50000.0f * SCALE * sinf(2.0f * (float)M_PI * freq_hz * i / fs);
        afe.test_feed_hr2(x, probe_state);
    }
}

// Helper: same as feed_hr2_sine but with uniform noise (~10% of amplitude, ~20 dB SNR).
// srand(42) called by the test before use for reproducibility.
static void feed_hr2_sine_noisy(INCUNEST_AFE4490& afe, float freq_hz, float fs, int n_samples) {
    for (int i = 0; i < n_samples; i++) {
        float noise = 5000.0f * SCALE * (2.0f * (float)rand() / (float)RAND_MAX - 1.0f);
        float x = OT_DC + 50000.0f * SCALE * sinf(2.0f * (float)M_PI * freq_hz * i / fs) + noise;
        afe.test_feed_hr2(x, ProbeState::PROBE_APPLIED);
    }
}

void setUp() {}
void tearDown() {}

// ── Test 1: not valid until buffer is full ────────────────────────────────────
// HR2 needs HR2_BUF_LEN decimated samples before reporting. After half that,
// hr2_valid must be false.
void test_hr2_not_valid_until_buffer_full() {
    INCUNEST_AFE4490 afe;
    feed_hr2_sine(afe, 1.0f, 500.0f, HR2_BUF_RAW / 2);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, afe.test_hr2_sqi());
    // Note: unlike SpO2/HR1 (which run their full invalid-path logic every sample), HR2's
    // fast path simply returns early while the buffer is filling — no commit to
    // _current_data happens yet, so hr2 stays at its construction default (0.0f), not NaN.
}

// ── Test 2: 60 BPM (1 Hz sine) ───────────────────────────────────────────────
// At 50 Hz decimated rate, 1 Hz → period = 50 samples lag.
// HR2 should converge to 60 BPM ± 1.
// SQI: unbiased normalised autocorrelation at fundamental lag → SQI ≈ 1.0 for
// clean periodic signal (finite-window bias corrected). Threshold: > 0.95.
void test_hr2_60bpm() {
    INCUNEST_AFE4490 afe;
    feed_hr2_sine(afe, 1.0f, 500.0f, HR2_BUF_RAW + 1000);  // fill + margin
    TEST_ASSERT_GREATER_THAN_FLOAT(0.95f, afe.test_hr2_sqi());
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 60.0f, afe.test_hr2());
}

// ── Test 3: 120 BPM (2 Hz sine) ──────────────────────────────────────────────
// At 50 Hz decimated rate, 2 Hz → period = 25 samples lag.
// SQI: unbiased normalised autocorrelation → SQI ≈ 1.0 for clean signal. Threshold: > 0.95.
void test_hr2_120bpm() {
    INCUNEST_AFE4490 afe;
    feed_hr2_sine(afe, 2.0f, 500.0f, HR2_BUF_RAW + 1000);
    TEST_ASSERT_GREATER_THAN_FLOAT(0.95f, afe.test_hr2_sqi());
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 120.0f, afe.test_hr2());
}

// ── Test 3b: 30 BPM — lower bound of the configured HR range ─────────────────
// Regression guard for the lag sweep bound. max_lag used to be a hardcoded 137 samples;
// it is now derived from _hr_min_bpm (30 BPM since lib v0.99, 40 before) plus the same 3 BPM
// guard band that min_lag uses, which at 50 Hz gives max_lag = 60/27*50 = 111 lags — and the
// peak search skips the endpoints, so the usable range ends at lag 110.
//
// 30 BPM = 0.5 Hz → lag 100, i.e. 10 lags of margin. If the derivation is ever tightened
// (smaller guard band, higher hr_min, shorter buffer), the slowest accepted heart rate
// silently stops being reachable and this test is what catches it. 0.5 Hz is exactly the
// bandpass corner (−3 dB on the fundamental): the normalised autocorrelation does not care,
// and this test is the proof — measured 2026-10-03, a 30.0 BPM sine reads 30.01 with SQI 1.00.
// The validity gate is hard at 30, so the estimate must land on or above it; it does.
void test_hr2_30bpm_lower_bound() {
    INCUNEST_AFE4490 afe;
    feed_hr2_sine(afe, 30.0f / 60.0f, 500.0f, HR2_BUF_RAW + 1000);
    TEST_ASSERT_GREATER_THAN_FLOAT(0.95f, afe.test_hr2_sqi());
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 30.0f, afe.test_hr2());
}

// ── Test 3b': 27 BPM — inside the search guard band, outside the valid range ──
// The sweep reaches 27 BPM (lag 111) so the peak is found, but the result is below
// hr_min_bpm: hr2 must be NaN and sqi 0. Pins the gate, not the search.
void test_hr2_27bpm_guard_band_is_invalid() {
    INCUNEST_AFE4490 afe;
    feed_hr2_sine(afe, 27.0f / 60.0f, 500.0f, HR2_BUF_RAW + 1000);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, afe.test_hr2_sqi());
    TEST_ASSERT_TRUE(isnan(afe.test_hr2()));
}

// ── Test 3c: the decimated rate is invariant to the AFE sample rate ──────────
// Pins the v0.83 inversion: _decim_factor is DERIVED from decim_target_rate_hz (50 Hz), so
// raising the AFE rate changes the factor and leaves the decimated rate — and with it the
// meaning of hr2_buf_len (8 s), the update interval (0.5 s) and hr2_acorr_lag_cap (22 BPM).
//
// The range floor at 1000 Hz is the case that DISCRIMINATES. With the old fixed factor of 10
// the decimated rate would have doubled to 100 Hz, so 30 BPM would need lag = 60/30*100 = 200,
// beyond hr2_acorr_lag_cap (137): the sweep would stop short and real bradycardia would be
// reported as "no periodicity" (sqi = 0). A 60 BPM check would NOT catch this — its lag
// stays inside the cap either way. With the derived factor (20 at 1000 Hz) the decimated
// rate stays at 50 Hz and the lag is 100, inside the 111-lag sweep. (Written at 40 BPM when
// the floor was 40; moved to 30 with lib v0.99.)
void test_hr2_decimated_rate_invariant_to_sample_rate() {
    INCUNEST_AFE4490 afe;
    afe.setSampleRate(1000);
    // Same ~10 s of signal as the 500 Hz tests, at twice the raw rate.
    feed_hr2_sine(afe, 30.0f / 60.0f, 1000.0f, HR2_BUF_RAW * 2 + 2000);
    TEST_ASSERT_GREATER_THAN_FLOAT(0.95f, afe.test_hr2_sqi());
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 30.0f, afe.test_hr2());
}

// ── Test 3d: the two decimation chains derive independently ─────────────────
// The pulse chain (BP 0.5-5 Hz → HR2 and its future MPM/YIN variants) and the harmonic chain
// (LP 15 Hz → HR3) share no filter, no buffer and no phase counter, so nothing forces them to
// run at one rate. Each derives its factor from its OWN target (v0.84). Both targets are 50 Hz
// today, so the factors match — this test pins that they are computed per chain and that both
// track the AFE rate, which is what makes it safe to lower one of them later.
void test_decim_chains_derive_independently() {
    INCUNEST_AFE4490 afe;
    TEST_ASSERT_EQUAL_UINT16(10, afe.test_hr2_decim_factor());
    TEST_ASSERT_EQUAL_UINT16(10, afe.test_hr3_decim_factor());
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 50.0f, afe.test_hr2_decim_rate_hz());
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 50.0f, afe.test_hr3_decim_rate_hz());

    afe.setSampleRate(1000);
    TEST_ASSERT_EQUAL_UINT16(20, afe.test_hr2_decim_factor());
    TEST_ASSERT_EQUAL_UINT16(20, afe.test_hr3_decim_factor());
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 50.0f, afe.test_hr2_decim_rate_hz());
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 50.0f, afe.test_hr3_decim_rate_hz());

    // Top of the catalogue, where the factors are largest.
    afe.setSampleRate(AFE4490SampleRate::HZ_1600);
    TEST_ASSERT_EQUAL_UINT16(32, afe.test_hr2_decim_factor());
    TEST_ASSERT_EQUAL_UINT16(32, afe.test_hr3_decim_factor());
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 50.0f, afe.test_hr2_decim_rate_hz());

    // NOTE (v0.85): the off-grid case this test used to cover (700 Hz -> factor 14, achieved
    // rate 50.02 Hz) is no longer reachable through the API — AFE4490SampleRate makes an
    // unvalidated rate inexpressible, and the numeric form rejects it. The rounding branch in
    // Decimator::configure() is kept deliberately: it is defensive, and it is what a future
    // FractionalDecimator would build on. Rejection itself is covered by
    // test_sample_rate::test_rejected_rate_leaves_configuration_untouched.
}

// ── Test 4: flat signal → hr2_valid false ────────────────────────────────────
// A constant DC signal has zero AC energy after the bandpass filter.
// The autocorrelation check (acorr0 < hr2_ot_energy_eps) must reject it.
void test_hr2_flat_signal_invalid() {
    INCUNEST_AFE4490 afe;
    for (int i = 0; i < HR2_BUF_RAW + 1000; i++)
        afe.test_feed_hr2(OT_DC, ProbeState::PROBE_APPLIED);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, afe.test_hr2_sqi());
    TEST_ASSERT_TRUE(isnan(afe.test_hr2()));
}

// ── Test 5: 60 BPM with noise (~20 dB SNR) ───────────────────────────────────
// Autocorrelation is robust to additive noise. With ±10% noise HR2 must still
// converge to 60 BPM ± 1 and SQI > 0.80 (unbiased; noise lowers peak but correction
// raises it slightly vs. biased). Noise barely changes autocorrelation shape.
void test_hr2_60bpm_noisy() {
    INCUNEST_AFE4490 afe;
    srand(42);
    feed_hr2_sine_noisy(afe, 1.0f, 500.0f, HR2_BUF_RAW + 1000);
    TEST_ASSERT_GREATER_THAN_FLOAT(0.80f, afe.test_hr2_sqi());
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 60.0f, afe.test_hr2());
}

// ── Test 6: 120 BPM with noise (~20 dB SNR) ──────────────────────────────────
void test_hr2_120bpm_noisy() {
    INCUNEST_AFE4490 afe;
    srand(42);
    feed_hr2_sine_noisy(afe, 2.0f, 500.0f, HR2_BUF_RAW + 1000);
    TEST_ASSERT_GREATER_THAN_FLOAT(0.80f, afe.test_hr2_sqi());
    TEST_ASSERT_FLOAT_WITHIN(2.0f, 120.0f, afe.test_hr2());
}

// ── Test 7 (new): PROBE_DISCONNECTED/NOT_APPLIED forces invalid + resets state ─
// HR2 never classifies presence itself — it only consumes probe_state. Feeding a converged
// signal, then switching to PROBE_DISCONNECTED must: force sqi=0/hr2=NaN, reset the fast-path
// buffer, and require a full fresh buffer once PROBE_APPLIED resumes (never triggers the
// slow autocorrelation path while not applied).
void test_hr2_not_applied_resets() {
    INCUNEST_AFE4490 afe;
    feed_hr2_sine(afe, 1.0f, 500.0f, HR2_BUF_RAW + 1000);
    TEST_ASSERT_GREATER_THAN_FLOAT(0.95f, afe.test_hr2_sqi());  // valid before disconnecting

    feed_hr2_sine(afe, 1.0f, 500.0f, 1000, ProbeState::PROBE_DISCONNECTED);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, afe.test_hr2_sqi());
    TEST_ASSERT_TRUE(isnan(afe.test_hr2()));

    // Re-applying must require a full fresh buffer, not resume instantly from stale state.
    feed_hr2_sine(afe, 1.0f, 500.0f, HR2_BUF_RAW / 2);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, afe.test_hr2_sqi());

    // PROBE_OT_HIGH gets the same treatment as PROBE_DISCONNECTED.
    feed_hr2_sine(afe, 1.0f, 500.0f, HR2_BUF_RAW + 1000);
    TEST_ASSERT_GREATER_THAN_FLOAT(0.95f, afe.test_hr2_sqi());
    feed_hr2_sine(afe, 1.0f, 500.0f, 1000, ProbeState::PROBE_OT_HIGH);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, afe.test_hr2_sqi());
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_hr2_not_valid_until_buffer_full);
    RUN_TEST(test_hr2_60bpm);
    RUN_TEST(test_hr2_120bpm);
    RUN_TEST(test_hr2_30bpm_lower_bound);
    RUN_TEST(test_hr2_27bpm_guard_band_is_invalid);
    RUN_TEST(test_hr2_decimated_rate_invariant_to_sample_rate);
    RUN_TEST(test_decim_chains_derive_independently);
    RUN_TEST(test_hr2_flat_signal_invalid);
    RUN_TEST(test_hr2_60bpm_noisy);
    RUN_TEST(test_hr2_120bpm_noisy);
    RUN_TEST(test_hr2_not_applied_resets);
    return UNITY_END();
}
