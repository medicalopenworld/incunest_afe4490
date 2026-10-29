#include <unity.h>
#include <math.h>
#include "incunest_afe4490.h"

// EXPERIMENT (OT-domain input, branch experiment/ot-domain-inputs): SpO2 now consumes OT
// (dimensionless A/A, ~1e-5 typical per incunest_afe4490_spec.md) instead of raw
// ambient-corrected ADC counts. Presence detection (finger/probe applied) is RSQM's
// responsibility alone (ProbeState, passed into _spo2_update()) — SpO2 never computes its own
// no-finger/no-signal classification, only a purely numerical division-safety guard
// (spo2_div_eps). See _spo2_update() rationale in incunest_afe4490.cpp.
//
// Output contract: sqi==0.0f always implies spo2==spo2_r==NaN (never a stale value), unified
// across warmup, PROBE_DISCONNECTED/PROBE_OT_HIGH, and the division-safety guard. PI (v0.101,
// rationale §11) is computed independently by _pi_update_sample(), not by _spo2_update() — the
// helper below feeds it in lockstep (same IR samples, same probe_state) because SpO2's SQI still
// reads AFE4490Data::pi (unchanged). A test that calls test_feed_spo2() alone, bypassing PI,
// gets pi==0.0f forever (never updated) rather than a real value — see test_pi/test_pi.cpp for
// PI's own tests.
//
// (SPO2_A/SPO2_B calibration coefficients and WARMUP_SAMPLES were removed 2026-08-19: both were
// dead code — hardcoded duplicates of incunest_afe4490.cpp's constants, defined but never
// referenced by any assertion. Derive from afe.getConfig() instead of re-hardcoding, per the
// same fix applied to test_hgac.cpp's WEAK_CODE/SAT_CODE — see PulseNest conversation_log.md.)
//
// Since v0.102 the averaging is a true sliding window (spo2_window_s, default 6 s in 100 ms
// blocks, v0.103): the first estimate appears when the window is full (3000 raw samples at
// 500 Hz, plus one block) and is exact from then on — the only remaining transient
// is the 2 s DC EMA, seeded on the first sample. 40000 samples (80 s) is kept as a generous
// "fully converged" budget for the accuracy tests; it is no longer a necessity.
static constexpr int CONVERGED_SAMPLES = 40000;
static constexpr int WINDOW_RAW        = 3000;    // 6 s × 500 Hz — spo2_window_s default
static constexpr int DECIM_RAW         = 50;      // raw samples per 100 ms block at 500 Hz (alg_window_block_s)

// Typical OT DC magnitude (per spec: APPLIED ~1.4e-5).
static constexpr float OT_DC = 1.4e-5f;

// Helper: feed N samples of dual-channel sines (same freq, different AC amplitude), scaled
// into OT units. IR: DC=OT_DC, AC=a_ir*scale. RED: DC=OT_DC, AC=a_red*scale.
// R = (AC_red/DC)/(AC_ir/DC) = a_red/a_ir (DC cancels, same as the original count-domain test).
static void feed_spo2_sine(INCUNEST_AFE4490& afe,
                            float a_ir, float a_red,
                            float freq_hz, int n_samples,
                            ProbeState probe_state = ProbeState::PROBE_APPLIED) {
    const float fs = 500.0f;
    const float scale = 1.4e-10f;  // brings a_ir~10000-scale amplitudes into OT-DC-scale AC
    // v0.110: SpO2 is gated on HR3 having a pulse. These tests exercise SpO2's own chain, so the
    // helper stands in for Task C with a valid HR3; the pulse-gate tests below set it themselves.
    afe.test_set_hr3(60.0f, 1.0f);
    for (int i = 0; i < n_samples; i++) {
        float phase = 2.0f * (float)M_PI * freq_hz * i / fs;
        float ot_ir  = OT_DC + a_ir  * scale * sinf(phase);
        float ot_red = OT_DC + a_red * scale * sinf(phase);
        afe.test_feed_pi(ot_ir, probe_state);   // SpO2's SQI reads AFE4490Data::pi — see note above;
        afe.test_feed_spo2(ot_ir, ot_red, probe_state);   // PI first, as the firmware's task body does
    }
}

void setUp() {}
void tearDown() {}

// ── Test 1: not valid while the window fills — outputs are NaN, not stale ────
void test_spo2_not_valid_during_warmup() {
    INCUNEST_AFE4490 afe;
    feed_spo2_sine(afe, 5000.0f, 2769.0f, 1.0f, 1000);  // well short of the 6 s window
    TEST_ASSERT_EQUAL_FLOAT(0.0f, afe.test_spo2_sqi());
    TEST_ASSERT_TRUE(isnan(afe.test_spo2()));
    TEST_ASSERT_TRUE(isnan(afe.test_spo2_r()));
}

// ── Test 1b: the window IS the warm-up — valid exactly when full, not 3·τ later ──
// NaN one block before the window is full; R, r and SpO2 present one block after (PI's own 6 s
// window fills at the same time, so the SQI gate on PI does not bind here).
void test_spo2_valid_exactly_when_window_full() {
    INCUNEST_AFE4490 afe;
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 6.0f, afe.getConfig().spo2_window_s);
    feed_spo2_sine(afe, 5000.0f, 2769.0f, 1.0f, WINDOW_RAW - DECIM_RAW);
    TEST_ASSERT_TRUE(isnan(afe.test_spo2_r()));
    TEST_ASSERT_EQUAL_UINT32(afe.test_spo2_buf_n() - 1, afe.test_spo2_buf_count());
    feed_spo2_sine(afe, 5000.0f, 2769.0f, 1.0f, 2 * DECIM_RAW);
    TEST_ASSERT_FALSE(isnan(afe.test_spo2_r()));
    TEST_ASSERT_FALSE(isnan(afe.test_spo2()));
    TEST_ASSERT_EQUAL_FLOAT(1.0f, afe.test_spo2_sqi());
    // Two clean sines: the regression is exact from the first full window.
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 2769.0f / 5000.0f, afe.test_spo2_r());
}

// ── Test 1c: window setter — clamped to [2, 12] s, length in seconds, empties the window ──
void test_spo2_window_setter_clamps_and_resets() {
    INCUNEST_AFE4490 afe;
    afe.setSpO2WindowS(1.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 2.0f, afe.getConfig().spo2_window_s);
    afe.setSpO2WindowS(20.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 12.0f, afe.getConfig().spo2_window_s);
    TEST_ASSERT_EQUAL_INT(120, afe.test_spo2_buf_n());
    afe.setSpO2WindowS(4.0f);
    TEST_ASSERT_EQUAL_INT(40, afe.test_spo2_buf_n());
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 6.0f, afe.getConfig().pi_window_s);   // independent of PI's
    // 4 s window: valid after 2000 raw (+ one block), with the same R.
    feed_spo2_sine(afe, 5000.0f, 2769.0f, 1.0f, 2000 + 2 * DECIM_RAW);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 2769.0f / 5000.0f, afe.test_spo2_r());
    // Changing the length mid-stream empties the window: NaN until the new length has filled.
    afe.setSpO2WindowS(8.0f);
    TEST_ASSERT_EQUAL_UINT32(0, afe.test_spo2_buf_count());
    feed_spo2_sine(afe, 5000.0f, 2769.0f, 1.0f, 1000);
    TEST_ASSERT_TRUE(isnan(afe.test_spo2_r()));
    feed_spo2_sine(afe, 5000.0f, 2769.0f, 1.0f, 3000 + 2 * DECIM_RAW);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 2769.0f / 5000.0f, afe.test_spo2_r());
}

// ── Test 2: PROBE_DISCONNECTED/NOT_APPLIED forces invalid + resets state ─────
// SpO2 never classifies presence itself — it only consumes probe_state. Feeding a real,
// already-converged signal, then switching to PROBE_DISCONNECTED must: force sqi=0 and
// pi/spo2/spo2_r to NaN, reset the internal EMAs and windows, and require a fresh full window
// once probe_state returns to PROBE_APPLIED (no instant resume from stale pre-disconnect state).
void test_spo2_not_applied_resets() {
    INCUNEST_AFE4490 afe;
    feed_spo2_sine(afe, 5000.0f, 2769.0f, 1.0f, CONVERGED_SAMPLES);
    TEST_ASSERT_EQUAL_FLOAT(1.0f, afe.test_spo2_sqi());  // valid before disconnecting

    // Sustained disconnect: probe_state alone must force invalidity, regardless of what OT
    // values are fed (even a plausible-looking DC/AC pair must NOT produce a valid reading).
    for (int i = 0; i < 1000; i++)
        afe.test_feed_spo2(OT_DC, OT_DC, ProbeState::PROBE_DISCONNECTED);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, afe.test_spo2_sqi());
    TEST_ASSERT_TRUE(isnan(afe.test_spo2()));
    TEST_ASSERT_TRUE(isnan(afe.test_spo2_r()));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, afe.test_spo2_ir_ema_mean());  // EMA reset, not just gated
    TEST_ASSERT_EQUAL_FLOAT(0.0f, afe.test_spo2_ir_ema_var());

    // Re-applying (PROBE_APPLIED) must require a fresh full window, not resume instantly from
    // whatever the pre-disconnect state was.
    TEST_ASSERT_EQUAL_UINT32(0, afe.test_spo2_buf_count());
    feed_spo2_sine(afe, 5000.0f, 2769.0f, 1.0f, 1000);  // well short of the window
    TEST_ASSERT_EQUAL_FLOAT(0.0f, afe.test_spo2_sqi());
    TEST_ASSERT_TRUE(isnan(afe.test_spo2()));

    // PROBE_OT_HIGH gets the same treatment as PROBE_DISCONNECTED (both non-APPLIED).
    for (int i = 0; i < 1000; i++)
        afe.test_feed_spo2(OT_DC, OT_DC, ProbeState::PROBE_OT_HIGH);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, afe.test_spo2_sqi());
    TEST_ASSERT_EQUAL_FLOAT(0.0f, afe.test_spo2_ir_ema_mean());
}

// Tests 3–6 derive the red amplitude from the default curve's own coefficients (a, b), so they
// hold whichever curve the library ships: a_red = 10000 · R_target, R_target = (a − SpO2) / b.
static float a_red_for(float spo2_target) {
    AFE4490Config cfg = INCUNEST_AFE4490().getConfig();
    return 5000.0f * (cfg.spo2_r_curve_a - spo2_target) / cfg.spo2_r_curve_b;
}

// ── Test 3: SpO2 ≈ 98% ───────────────────────────────────────────────────────
void test_spo2_98_percent() {
    INCUNEST_AFE4490 afe;
    feed_spo2_sine(afe, 5000.0f, a_red_for(98.0f), 1.0f, CONVERGED_SAMPLES);
    TEST_ASSERT_EQUAL_FLOAT(1.0f, afe.test_spo2_sqi());
    TEST_ASSERT_FLOAT_WITHIN(2.0f, 98.0f, afe.test_spo2());
}

// ── Test 4: SpO2 ≈ 90% ───────────────────────────────────────────────────────
void test_spo2_90_percent() {
    INCUNEST_AFE4490 afe;
    feed_spo2_sine(afe, 5000.0f, a_red_for(90.0f), 1.0f, CONVERGED_SAMPLES);
    TEST_ASSERT_EQUAL_FLOAT(1.0f, afe.test_spo2_sqi());
    TEST_ASSERT_FLOAT_WITHIN(2.0f, 90.0f, afe.test_spo2());
}

// ── Test 5: SpO2 slightly above 100 → clamped to 100 and reported valid ──────
// raw SpO2 ≈ 101.2 → within the clamp margin → 100.0
void test_spo2_clamp_above_100() {
    INCUNEST_AFE4490 afe;
    feed_spo2_sine(afe, 5000.0f, a_red_for(101.2f), 1.0f, CONVERGED_SAMPLES);
    TEST_ASSERT_EQUAL_FLOAT(1.0f, afe.test_spo2_sqi());
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 100.0f, afe.test_spo2());
}

// ── Test 6: a curve offset above 100 → still clamped to 100 (v0.107, rationale §15) ──
// raw SpO2 ≈ 105.8 — what the neonatal-foot curve gives on an adult finger or the MS100 at
// setpoint 100 — was invalid with the 3-point margin; with 10 points it reads 100, valid.
void test_spo2_offset_above_100_clamped() {
    INCUNEST_AFE4490 afe;
    feed_spo2_sine(afe, 5000.0f, a_red_for(105.8f), 1.0f, CONVERGED_SAMPLES);
    TEST_ASSERT_EQUAL_FLOAT(1.0f, afe.test_spo2_sqi());
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 100.0f, afe.test_spo2());
}

// ── Test 6b: SpO2 far above 100 → invalid (outside the 10-point clamp margin) ──
// raw SpO2 ≈ 115 → exceeds spo2_max + spo2_clamp_margin → invalid
void test_spo2_too_high_invalid() {
    INCUNEST_AFE4490 afe;
    feed_spo2_sine(afe, 5000.0f, a_red_for(115.0f), 1.0f, CONVERGED_SAMPLES);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, afe.test_spo2_sqi());
    TEST_ASSERT_TRUE(isnan(afe.test_spo2()));
}

// ── Pulse gate (v0.110, rationale §17) ───────────────────────────────────────
// Converged, clean sines, then HR3 reports no pulse: SpO2 withheld with reason PULSE, R and r
// still visible; same signal with the gate off: published.
void test_spo2_pulse_gate_withholds_without_hr3() {
    INCUNEST_AFE4490 afe;
    feed_spo2_sine(afe, 5000.0f, a_red_for(96.0f), 1.0f, CONVERGED_SAMPLES);
    TEST_ASSERT_TRUE(afe.getConfig().spo2_pulse_gate_enable);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 96.0f, afe.test_spo2());
    TEST_ASSERT_EQUAL(SpO2HoldReason::NONE, afe.test_spo2_hold_reason());
    afe.test_set_hr3(0.0f, 0.0f);                        // Task C: no pulse (fed by hand: the helper re-arms HR3)
    for (int i = 0; i < 2 * DECIM_RAW; i++) {            // two blocks: the gate is evaluated per block
        afe.test_feed_pi(OT_DC, ProbeState::PROBE_APPLIED);
        afe.test_feed_spo2(OT_DC + 5000.0f * 1.4e-10f * sinf(2.0f * (float)M_PI * i / 500.0f),
                           OT_DC + a_red_for(96.0f) * 1.4e-10f * sinf(2.0f * (float)M_PI * i / 500.0f),
                           ProbeState::PROBE_APPLIED);
    }
    TEST_ASSERT_TRUE(isnan(afe.test_spo2()));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, afe.test_spo2_sqi());
    TEST_ASSERT_EQUAL(SpO2HoldReason::PULSE, afe.test_spo2_hold_reason());
    TEST_ASSERT_FALSE(isnan(afe.test_spo2_r()));          // R and r stay on record
    TEST_ASSERT_FALSE(isnan(afe.test_spo2_r_method2_reg_corr()));
}

void test_spo2_pulse_gate_off_publishes_without_hr3() {
    INCUNEST_AFE4490 afe;
    afe.setSpO2PulseGateEnable(false);
    TEST_ASSERT_FALSE(afe.getConfig().spo2_pulse_gate_enable);
    afe.test_set_hr3(0.0f, 0.0f);
    for (int i = 0; i < CONVERGED_SAMPLES; i++) {
        afe.test_feed_pi(OT_DC + 5000.0f * 1.4e-10f * sinf(2.0f * (float)M_PI * i / 500.0f), ProbeState::PROBE_APPLIED);
        afe.test_feed_spo2(OT_DC + 5000.0f * 1.4e-10f * sinf(2.0f * (float)M_PI * i / 500.0f),
                           OT_DC + a_red_for(96.0f) * 1.4e-10f * sinf(2.0f * (float)M_PI * i / 500.0f),
                           ProbeState::PROBE_APPLIED);
    }
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 96.0f, afe.test_spo2());
    TEST_ASSERT_EQUAL(SpO2HoldReason::NONE, afe.test_spo2_hold_reason());
}

// The SpO2 window keeps running while the gate is closed: when HR3 returns, SpO2 is back in the
// next 100 ms block — not after another 6 s window.
void test_spo2_pulse_gate_returns_in_the_next_block() {
    INCUNEST_AFE4490 afe;
    feed_spo2_sine(afe, 5000.0f, a_red_for(96.0f), 1.0f, CONVERGED_SAMPLES);
    afe.test_set_hr3(0.0f, 0.0f);
    for (int i = 0; i < 20 * DECIM_RAW; i++) {                       // 2 s without a pulse
        afe.test_feed_pi(OT_DC, ProbeState::PROBE_APPLIED);
        afe.test_feed_spo2(OT_DC + 5000.0f * 1.4e-10f * sinf(2.0f * (float)M_PI * i / 500.0f),
                           OT_DC + a_red_for(96.0f) * 1.4e-10f * sinf(2.0f * (float)M_PI * i / 500.0f),
                           ProbeState::PROBE_APPLIED);
    }
    TEST_ASSERT_EQUAL(SpO2HoldReason::PULSE, afe.test_spo2_hold_reason());
    afe.test_set_hr3(60.0f, 1.0f);                                   // Task C: pulse again
    for (int i = 0; i < DECIM_RAW; i++) {                            // one block
        afe.test_feed_pi(OT_DC, ProbeState::PROBE_APPLIED);
        afe.test_feed_spo2(OT_DC + 5000.0f * 1.4e-10f * sinf(2.0f * (float)M_PI * i / 500.0f),
                           OT_DC + a_red_for(96.0f) * 1.4e-10f * sinf(2.0f * (float)M_PI * i / 500.0f),
                           ProbeState::PROBE_APPLIED);
    }
    TEST_ASSERT_EQUAL(SpO2HoldReason::NONE, afe.test_spo2_hold_reason());
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 96.0f, afe.test_spo2());
}

// The reason is the FIRST failing gate in the library's order: probe, window, eps, r, PI, range, pulse.
void test_spo2_hold_reason_order() {
    INCUNEST_AFE4490 afe;
    feed_spo2_sine(afe, 5000.0f, 2000.0f, 1.0f, 10, ProbeState::PROBE_DISCONNECTED);
    TEST_ASSERT_EQUAL(SpO2HoldReason::PROBE, afe.test_spo2_hold_reason());
    feed_spo2_sine(afe, 5000.0f, 2000.0f, 1.0f, WINDOW_RAW / 2);
    TEST_ASSERT_EQUAL(SpO2HoldReason::WINDOW, afe.test_spo2_hold_reason());
    feed_spo2_sine(afe, 5000.0f, a_red_for(115.0f), 1.0f, CONVERGED_SAMPLES);   // > 110: range
    TEST_ASSERT_EQUAL(SpO2HoldReason::RANGE, afe.test_spo2_hold_reason());
    afe.test_set_hr3(0.0f, 0.0f);                                    // range fails before pulse
    for (int i = 0; i < 2 * DECIM_RAW; i++) {
        afe.test_feed_pi(OT_DC, ProbeState::PROBE_APPLIED);
        afe.test_feed_spo2(OT_DC + 5000.0f * 1.4e-10f * sinf(2.0f * (float)M_PI * i / 500.0f),
                           OT_DC + a_red_for(115.0f) * 1.4e-10f * sinf(2.0f * (float)M_PI * i / 500.0f),
                           ProbeState::PROBE_APPLIED);
    }
    TEST_ASSERT_EQUAL(SpO2HoldReason::RANGE, afe.test_spo2_hold_reason());
}

// ── Test 7 (new, OT-domain-specific): R is invariant to a fixed gain scale ───
// If both ot_ir and ot_red were scaled by the same constant k (as an RF change would do to
// the raw-ADC-domain led1_sub/led2_sub, but NOT to OT — this test proves OT already cancels
// it), R and SpO2 must be identical. Demonstrates the OT-domain migration's core claim.
void test_spo2_r_invariant_to_uniform_scale() {
    INCUNEST_AFE4490 afe_a, afe_b;
    feed_spo2_sine(afe_a, 5000.0f, 2769.0f, 1.0f, CONVERGED_SAMPLES);
    // afe_b fed with the SAME OT values scaled by an arbitrary constant (simulating what a
    // gain change would have done in the OLD raw-count domain) — R must match afe_a exactly,
    // because in OT domain a uniform scale is exactly what a real gain change looks like:
    // nothing, since OT is already gain-invariant. This test feeds pre-scaled OT directly
    // (no HGAC involved) to isolate the claim to the SpO2 math itself.
    const float k = 0.37f;  // arbitrary scale factor, would be kAFE_RF_OHM ratio in reality
    const float fs = 500.0f;
    const float scale = 1.4e-10f;
    afe_b.test_set_hr3(60.0f, 1.0f);      // fed by hand: stand in for Task C as the helper does (v0.110 gate)
    for (int i = 0; i < CONVERGED_SAMPLES; i++) {
        float phase = 2.0f * (float)M_PI * 1.0f * i / fs;
        float ot_ir  = k * (OT_DC + 5000.0f * scale * sinf(phase));
        float ot_red = k * (OT_DC + 2769.0f  * scale * sinf(phase));
        afe_b.test_feed_spo2(ot_ir, ot_red, ProbeState::PROBE_APPLIED);
        afe_b.test_feed_pi(ot_ir, ProbeState::PROBE_APPLIED);
    }
    TEST_ASSERT_EQUAL_FLOAT(1.0f, afe_b.test_spo2_sqi());
    TEST_ASSERT_FLOAT_WITHIN(0.05f, afe_a.test_spo2(), afe_b.test_spo2());
    TEST_ASSERT_FLOAT_WITHIN(0.001f, afe_a.test_spo2_r(), afe_b.test_spo2_r());
}

// ── R method / R curve identifiers (v0.98; default curve of v0.100) ─────────
// The default curve is the HOSPNAV one, fitted with R-METHOD-2 — the method in force — so the
// pair must match. Switching to R-METHOD-1 makes it a mismatch, reported as a warning only.
void test_spo2_default_r_curve_is_labelled_and_matches() {
    INCUNEST_AFE4490 afe;
    AFE4490Config cfg = afe.getConfig();
    TEST_ASSERT_EQUAL_STRING("R-METHOD-2", cfg.spo2_r_method_id);
    TEST_ASSERT_EQUAL_UINT8(2, cfg.spo2_r_method);
    TEST_ASSERT_EQUAL_STRING("R-CURVE-STS0163-HOSPNAV-20261004", cfg.spo2_r_curve_id);
    TEST_ASSERT_EQUAL_STRING("R-METHOD-2", cfg.spo2_r_curve_r_method_id);
    TEST_ASSERT_TRUE(cfg.spo2_r_curve_method_match);
    TEST_ASSERT_EQUAL_FLOAT(123.98f, cfg.spo2_r_curve_a);
    TEST_ASSERT_EQUAL_FLOAT(39.13f, cfg.spo2_r_curve_b);
    TEST_ASSERT_EQUAL_FLOAT(0.8f, cfg.spo2_r_method2_reg_corr_min);
    afe.setSpO2RMethod(1);
    TEST_ASSERT_FALSE(afe.getConfig().spo2_r_curve_method_match);
    feed_spo2_sine(afe, 5000.0f, a_red_for(98.0f), 1.0f, CONVERGED_SAMPLES);
    TEST_ASSERT_EQUAL_FLOAT(1.0f, afe.test_spo2_sqi());          // warning only: SpO2 still valid
    TEST_ASSERT_FLOAT_WITHIN(2.0f, 98.0f, afe.test_spo2());
}

// A curve for the library's own R method clears the warning, and SpO2 follows its coefficients.
void test_spo2_set_r_curve_applies_and_matches() {
    INCUNEST_AFE4490 afe;
    TEST_ASSERT_TRUE(afe.setSpO2RCurve("R-CURVE-TEST-BENCH-20261002", "R-METHOD-2", 110.0f, 20.0f));
    AFE4490Config cfg = afe.getConfig();
    TEST_ASSERT_EQUAL_STRING("R-CURVE-TEST-BENCH-20261002", cfg.spo2_r_curve_id);
    TEST_ASSERT_TRUE(cfg.spo2_r_curve_method_match);
    afe.setSpO2RMethod(1);                                    // the match follows the method in force
    TEST_ASSERT_FALSE(afe.getConfig().spo2_r_curve_method_match);
    TEST_ASSERT_EQUAL_STRING("R-METHOD-1", afe.getConfig().spo2_r_method_id);
    afe.setSpO2RMethod(2);
    feed_spo2_sine(afe, 5000.0f, 2769.0f, 1.0f, CONVERGED_SAMPLES);
    TEST_ASSERT_FLOAT_WITHIN(0.05f, 110.0f - 20.0f * afe.test_spo2_r(), afe.test_spo2());
}

// Bad input never half-applies: the previous curve stays, whole.
void test_spo2_set_r_curve_rejects_bad_input() {
    INCUNEST_AFE4490 afe;
    char too_long[SPO2_R_ID_LEN + 4];
    memset(too_long, 'X', sizeof(too_long) - 1);
    too_long[sizeof(too_long) - 1] = '\0';
    TEST_ASSERT_FALSE(afe.setSpO2RCurve("", "R-METHOD-1", 110.0f, 20.0f));
    TEST_ASSERT_FALSE(afe.setSpO2RCurve(nullptr, "R-METHOD-1", 110.0f, 20.0f));
    TEST_ASSERT_FALSE(afe.setSpO2RCurve(too_long, "R-METHOD-1", 110.0f, 20.0f));
    TEST_ASSERT_FALSE(afe.setSpO2RCurve("R-CURVE-X", "", 110.0f, 20.0f));
    TEST_ASSERT_FALSE(afe.setSpO2RCurve("R-CURVE-X", "R-METHOD-1", NAN, 20.0f));
    TEST_ASSERT_FALSE(afe.setSpO2RCurve("R-CURVE-X", "R-METHOD-1", 110.0f, INFINITY));
    AFE4490Config cfg = afe.getConfig();
    TEST_ASSERT_EQUAL_STRING("R-CURVE-STS0163-HOSPNAV-20261004", cfg.spo2_r_curve_id);
    TEST_ASSERT_EQUAL_STRING("R-METHOD-2", cfg.spo2_r_curve_r_method_id);
    TEST_ASSERT_EQUAL_FLOAT(123.98f, cfg.spo2_r_curve_a);
    TEST_ASSERT_EQUAL_FLOAT(39.13f, cfg.spo2_r_curve_b);
}

// ── R-METHOD-2 (lib v0.100) ──────────────────────────────────────────────────
// On a clean pair of in-phase sines the regression slope equals the amplitude ratio, so both
// methods must give the same R; r must read as a near-perfect fit.
void test_spo2_method2_equals_method1_on_clean_sines() {
    INCUNEST_AFE4490 m2, m1;
    m1.setSpO2RMethod(1);
    feed_spo2_sine(m2, 5000.0f, 2769.0f, 1.0f, CONVERGED_SAMPLES);
    feed_spo2_sine(m1, 5000.0f, 2769.0f, 1.0f, CONVERGED_SAMPLES);
    TEST_ASSERT_FLOAT_WITHIN(0.005f, 0.5538f, m2.test_spo2_r());
    TEST_ASSERT_FLOAT_WITHIN(0.005f, m1.test_spo2_r(), m2.test_spo2_r());
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.0f, m2.test_spo2_r_method2_reg_corr());
    TEST_ASSERT_TRUE(isnan(m1.test_spo2_r_method2_reg_corr()));            // r belongs to method 2 only
    TEST_ASSERT_EQUAL_FLOAT(1.0f, m2.test_spo2_sqi());
}

// Red uncorrelated with IR (no common pulse): r collapses, SpO2 is withheld, R and r stay on record.
void test_spo2_method2_low_r_invalidates() {
    INCUNEST_AFE4490 afe;
    const float fs = 500.0f, scale = 1.4e-10f;
    uint32_t seed = 12345u;
    for (int i = 0; i < CONVERGED_SAMPLES; i++) {
        float phase = 2.0f * (float)M_PI * 1.0f * i / fs;
        seed = seed * 1664525u + 1013904223u;
        float noise = ((float)(seed >> 8) / 16777216.0f - 0.5f) * 2.0f;   // white, ±1
        afe.test_feed_spo2(OT_DC + 5000.0f * scale * sinf(phase),
                           OT_DC + 5000.0f * scale * noise, ProbeState::PROBE_APPLIED);
    }
    TEST_ASSERT_TRUE(afe.test_spo2_r_method2_reg_corr() < 0.3f);
    TEST_ASSERT_FALSE(isnan(afe.test_spo2_r()));
    TEST_ASSERT_TRUE(isnan(afe.test_spo2()));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, afe.test_spo2_sqi());
    afe.setSpO2RMethod2RegCorrMin(-1.0f);                                 // gate off: SpO2 comes back (if in range)
    feed_spo2_sine(afe, 5000.0f, 2769.0f, 1.0f, CONVERGED_SAMPLES);
    TEST_ASSERT_FALSE(isnan(afe.test_spo2()));
}

// Switching method takes effect at the next window update (one decimation block, 20 ms at
// 500 Hz — since v0.102 the outputs refresh at the block cadence), without a new warm-up: both
// methods' windows run all the time.
void test_spo2_method_switch_is_immediate() {
    INCUNEST_AFE4490 afe;
    feed_spo2_sine(afe, 5000.0f, 2769.0f, 1.0f, CONVERGED_SAMPLES);
    float r2 = afe.test_spo2_r();
    afe.setSpO2RMethod(1);
    feed_spo2_sine(afe, 5000.0f, 2769.0f, 1.0f, DECIM_RAW);
    TEST_ASSERT_FALSE(isnan(afe.test_spo2()));
    TEST_ASSERT_FLOAT_WITHIN(0.005f, r2, afe.test_spo2_r());
    afe.setSpO2RMethod(7);                                      // rejected: stays on 1
    TEST_ASSERT_EQUAL_UINT8(1, afe.getConfig().spo2_r_method);
}

// ── PI upper bound (v0.101): an impossible PI withholds SpO2 instead of scoring SQI = 1 ────
// a_ir 15000 → PI = 2·15000·1.4e-10/1.4e-5 × 100 = 30 % (×0.9 band-pass gain at 1 Hz ≈ 27 %),
// above spo2_pi_max (20 %): the pre-v0.101 code clamped the OUTPUT of the SQI ramp and reported
// SQI = 1 here. Raising the bound through the third parameter of setSpO2PiSqiThresholds() restores it.
void test_spo2_pi_above_max_invalidates() {
    INCUNEST_AFE4490 afe;
    feed_spo2_sine(afe, 15000.0f, 15000.0f * 0.5538f, 1.0f, CONVERGED_SAMPLES);
    TEST_ASSERT_GREATER_THAN_FLOAT(20.0f, afe.test_pi());
    TEST_ASSERT_TRUE(isnan(afe.test_spo2()));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, afe.test_spo2_sqi());
    TEST_ASSERT_EQUAL_FLOAT(20.0f, afe.getConfig().spo2_pi_max);
    afe.setSpO2PiSqiThresholds(0.5f, 2.0f, 40.0f);
    feed_spo2_sine(afe, 15000.0f, 15000.0f * 0.5538f, 1.0f, DECIM_RAW);   // next window update
    TEST_ASSERT_FALSE(isnan(afe.test_spo2()));
    TEST_ASSERT_EQUAL_FLOAT(1.0f, afe.test_spo2_sqi());
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_spo2_pi_above_max_invalidates);
    RUN_TEST(test_spo2_method2_equals_method1_on_clean_sines);
    RUN_TEST(test_spo2_method2_low_r_invalidates);
    RUN_TEST(test_spo2_method_switch_is_immediate);
    RUN_TEST(test_spo2_not_valid_during_warmup);
    RUN_TEST(test_spo2_valid_exactly_when_window_full);
    RUN_TEST(test_spo2_window_setter_clamps_and_resets);
    RUN_TEST(test_spo2_not_applied_resets);
    RUN_TEST(test_spo2_98_percent);
    RUN_TEST(test_spo2_90_percent);
    RUN_TEST(test_spo2_clamp_above_100);
    RUN_TEST(test_spo2_offset_above_100_clamped);
    RUN_TEST(test_spo2_too_high_invalid);
    RUN_TEST(test_spo2_pulse_gate_withholds_without_hr3);
    RUN_TEST(test_spo2_pulse_gate_off_publishes_without_hr3);
    RUN_TEST(test_spo2_pulse_gate_returns_in_the_next_block);
    RUN_TEST(test_spo2_hold_reason_order);
    RUN_TEST(test_spo2_r_invariant_to_uniform_scale);
    RUN_TEST(test_spo2_default_r_curve_is_labelled_and_matches);
    RUN_TEST(test_spo2_set_r_curve_applies_and_matches);
    RUN_TEST(test_spo2_set_r_curve_rejects_bad_input);
    return UNITY_END();
}
