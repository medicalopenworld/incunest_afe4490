#include <unity.h>
#include <math.h>
#include "incunest_afe4490.h"

// PI (Perfusion Index) — decoupled from SpO2 (v0.101, incunest_afe4490_design_rationale.md §11).
// No slow/async path: _pi_update_sample() is both the fast AND the only path (O(1) sliding-window
// updates, unlike HR2/HR3's O(N·lags) autocorrelation/FFT), so test_feed_pi() calls it directly —
// mirrors test_hr2.cpp's style, minus the "_for_test" synchronous wrapper.
//
// PI is reported in PEAK-TO-PEAK units (2√2·RMS of the 0.5–15 Hz band-passed OT, over the mean of
// the same 6 s window): for a sine of amplitude A on a DC of D, PI = 2A/D × 100 exactly (up to
// the band-pass's in-band gain). probe_state (RSQM's classification) is consumed, never computed.

static constexpr int PI_BUF_LEN   = 60;    // 6 s of 100 ms blocks (alg_window_block_s, v0.103) — pi_window_s default
static constexpr int PI_BLOCK_RAW = 50;    // raw samples per block @ 500 Hz AFE rate
static constexpr int PI_BUF_RAW   = PI_BUF_LEN * PI_BLOCK_RAW;  // 3000 raw samples

static constexpr float OT_DC = 1.4e-5f;
static constexpr float SCALE = 1.4e-10f;
static constexpr float A_IR  = 2500.0f;       // amplitude 3.5e-7 → PI = 2·3.5e-7/1.4e-5 = 5.0 %

// Helper: feed N raw samples of a sine of given AC amplitude (OT-scale units, same convention as
// test_spo2.cpp/test_hr2.cpp) into PI.
static void feed_pi_sine(INCUNEST_AFE4490& afe, float a_ir, float freq_hz, float fs, int n_samples,
                          ProbeState probe_state = ProbeState::PROBE_APPLIED) {
    for (int i = 0; i < n_samples; i++) {
        float x = OT_DC + a_ir * SCALE * sinf(2.0f * (float)M_PI * freq_hz * i / fs);
        afe.test_feed_pi(x, probe_state);
    }
}

void setUp() {}
void tearDown() {}

// ── Test 1: not valid until the 6 s window is full ───────────────────────────
void test_pi_not_valid_until_window_full() {
    INCUNEST_AFE4490 afe;
    feed_pi_sine(afe, A_IR, 2.0f, 500.0f, PI_BUF_RAW / 2);
    TEST_ASSERT_TRUE(isnan(afe.test_pi()));
    TEST_ASSERT_TRUE(afe.test_pi_buf_count() < (uint32_t)PI_BUF_LEN);
}

// ── Test 2: peak-to-peak units — a 2 Hz (120 BPM) sine reads 2A/D ────────────
// 2 Hz sits well inside 0.5–15 Hz (first-order roll-off of the 0.5 Hz corner leaves ~0.97 of the
// amplitude there), so the value is pinned to within 6 %. At 1 Hz (60 BPM) the same corner would
// read ~0.9: that is the band-pass, not the estimator — test 3 pins the estimator's linearity.
void test_pi_peak_to_peak_units() {
    INCUNEST_AFE4490 afe;
    feed_pi_sine(afe, A_IR, 2.0f, 500.0f, PI_BUF_RAW + 1000);
    float pi = afe.test_pi();
    TEST_ASSERT_FALSE(isnan(pi));
    TEST_ASSERT_FLOAT_WITHIN(0.30f, 5.0f, pi);
}

// ── Test 3: PI scales linearly with AC amplitude ──────────────────────────────
// Doubling the AC amplitude must double PI (DC held fixed) — what a windowed RMS guarantees and
// a gain error would break. Independent of the band-pass's exact in-band gain.
void test_pi_scales_linearly_with_ac_amplitude() {
    INCUNEST_AFE4490 afe_1x, afe_2x;
    feed_pi_sine(afe_1x, A_IR, 2.0f, 500.0f, PI_BUF_RAW + 1000);
    feed_pi_sine(afe_2x, 2.0f * A_IR, 2.0f, 500.0f, PI_BUF_RAW + 1000);
    TEST_ASSERT_FLOAT_WITHIN(0.02f * afe_2x.test_pi(), 2.0f * afe_1x.test_pi(), afe_2x.test_pi());
}

// ── Test 4: flat signal (no AC) → PI ≈ 0, not NaN ─────────────────────────────
// A pure DC input has zero AC energy after the band-pass: valid division (DC > eps), near-zero
// numerator. Distinct from "invalid" (NaN only while the window is filling or probe is off).
void test_pi_flat_signal_is_zero() {
    INCUNEST_AFE4490 afe;
    feed_pi_sine(afe, 0.0f, 2.0f, 500.0f, PI_BUF_RAW + 1000);
    TEST_ASSERT_FALSE(isnan(afe.test_pi()));
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, afe.test_pi());
}

// ── Test 5: the block rate is invariant to the AFE sample rate ──────────────
// Same regression this guards for HR2's decimation (test_hr2_decimated_rate_invariant_to_sample_rate):
// _pi_block derives its length from the block DURATION (alg_window_block_s = 100 ms), so
// pi_buf_len (6 s) keeps meaning 6 seconds — and the window sums divide by the samples actually
// in the window — whatever the AFE rate does.
void test_pi_block_rate_invariant_to_sample_rate() {
    INCUNEST_AFE4490 afe;
    afe.setSampleRate(1000);
    TEST_ASSERT_EQUAL_UINT16(100, afe.test_pi_block_len());
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 10.0f, afe.test_pi_block_rate_hz());
    feed_pi_sine(afe, A_IR, 2.0f, 1000.0f, PI_BUF_RAW * 2 + 2000);
    TEST_ASSERT_FLOAT_WITHIN(0.30f, 5.0f, afe.test_pi());
}

// ── Test 6: PROBE_DISCONNECTED/NOT_APPLIED forces invalid + resets state ─────
// Mirrors test_hr2_not_applied_resets: PI never classifies presence itself, only consumes
// probe_state. Disconnecting must force pi=NaN, reset the sliding windows, and require a full
// fresh 6 s window once PROBE_APPLIED resumes (never an instant resume from stale state).
void test_pi_not_applied_resets() {
    INCUNEST_AFE4490 afe;
    feed_pi_sine(afe, A_IR, 2.0f, 500.0f, PI_BUF_RAW + 1000);
    TEST_ASSERT_FALSE(isnan(afe.test_pi()));   // valid before disconnecting

    feed_pi_sine(afe, A_IR, 2.0f, 500.0f, 1000, ProbeState::PROBE_DISCONNECTED);
    TEST_ASSERT_TRUE(isnan(afe.test_pi()));
    TEST_ASSERT_EQUAL_UINT32(0, afe.test_pi_buf_count());

    // Re-applying must require a full fresh window, not resume instantly from stale state.
    feed_pi_sine(afe, A_IR, 2.0f, 500.0f, PI_BUF_RAW / 2);
    TEST_ASSERT_TRUE(isnan(afe.test_pi()));

    feed_pi_sine(afe, A_IR, 2.0f, 500.0f, PI_BUF_RAW + 1000);
    TEST_ASSERT_FLOAT_WITHIN(0.30f, 5.0f, afe.test_pi());

    // PROBE_OT_HIGH gets the same treatment as PROBE_DISCONNECTED.
    feed_pi_sine(afe, A_IR, 2.0f, 500.0f, 1000, ProbeState::PROBE_OT_HIGH);
    TEST_ASSERT_TRUE(isnan(afe.test_pi()));
}

// ── Test 7: window setter (v0.102) — clamped to [2, 12] s, independent of SpO2's, empties ──
void test_pi_window_setter() {
    INCUNEST_AFE4490 afe;
    TEST_ASSERT_EQUAL_INT(PI_BUF_LEN, afe.test_pi_buf_n());
    afe.setPIWindowS(1.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 2.0f, afe.getConfig().pi_window_s);
    TEST_ASSERT_EQUAL_INT(20, afe.test_pi_buf_n());
    afe.setPIWindowS(20.0f);
    TEST_ASSERT_EQUAL_INT(120, afe.test_pi_buf_n());
    afe.setPIWindowS(3.0f);
    TEST_ASSERT_EQUAL_INT(30, afe.test_pi_buf_n());
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 6.0f, afe.getConfig().spo2_window_s);   // SpO2's untouched
    // 3 s window: NaN one block short, 2A/D one block after; same value as with 6 s.
    feed_pi_sine(afe, A_IR, 2.0f, 500.0f, 1500 - PI_BLOCK_RAW);
    TEST_ASSERT_TRUE(isnan(afe.test_pi()));
    feed_pi_sine(afe, A_IR, 2.0f, 500.0f, 2 * PI_BLOCK_RAW);
    TEST_ASSERT_FLOAT_WITHIN(0.30f, 5.0f, afe.test_pi());
    // Changing the length mid-stream empties the window.
    afe.setPIWindowS(6.0f);
    TEST_ASSERT_EQUAL_UINT32(0, afe.test_pi_buf_count());
    feed_pi_sine(afe, A_IR, 2.0f, 500.0f, 1000);
    TEST_ASSERT_TRUE(isnan(afe.test_pi()));
}

// ── Test 8: the PI definition is named (v0.106) — getConfig() carries PI_METHOD_ID ──
void test_pi_method_is_named() {
    INCUNEST_AFE4490 afe;
    TEST_ASSERT_EQUAL_STRING("PI-METHOD-1", PI_METHOD_ID);
    TEST_ASSERT_EQUAL_STRING(PI_METHOD_ID, afe.getConfig().pi_method_id);
    afe.setPIWindowS(3.0f);                                   // a parameter change is not a new method
    TEST_ASSERT_EQUAL_STRING(PI_METHOD_ID, afe.getConfig().pi_method_id);
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_pi_not_valid_until_window_full);
    RUN_TEST(test_pi_peak_to_peak_units);
    RUN_TEST(test_pi_scales_linearly_with_ac_amplitude);
    RUN_TEST(test_pi_flat_signal_is_zero);
    RUN_TEST(test_pi_block_rate_invariant_to_sample_rate);
    RUN_TEST(test_pi_not_applied_resets);
    RUN_TEST(test_pi_window_setter);
    RUN_TEST(test_pi_method_is_named);
    return UNITY_END();
}
