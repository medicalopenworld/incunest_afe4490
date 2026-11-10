#include <unity.h>
#include <math.h>
#include "incunest_afe4490.h"

// ppg_disp display modes (v0.116, incunest_afe4490_spec.md §5.9; rationale §21).
// NO_NORM is the band-passed, negated OT as it was from v0.69 to v0.115. NORM centres it and divides
// it by the SLEW envelope tracker (targets = max/min of the newest window of 100 ms block extrema;
// rise slew-limited to rise_per_s × frame per second; fall exponential with tau_s; headroom H maps
// the frame's extrema to ±1/H), clamped to [−1, +1]. NORM_PI multiplies by A(PI) ∈ [0.5, 1] and
// clamps at ±1 — never at ±A. test_feed_ppgdisp() runs the production path from the OT of the
// display channel; test_set_pi() plants the PI that NORM_PI reads. Since v0.118 the band-pass is
// reset while the probe is not applied (as PI's), in every mode.

static constexpr float FS   = 500.0f;
static constexpr float H    = 1.15f;            // ppgdisp_norm_headroom default
static constexpr float FULL = 1.0f / H;         // what a settled sine peaks at in NORM

// Feeds a sine of amplitude amp (OT units) on dc at f_hz, keeping the phase across calls, and
// returns the max and min of ppg_disp over the LAST `measure_s` seconds fed (0 = none).
struct Feeder {
    INCUNEST_AFE4490& afe;
    int   n = 0;
    float dc = 1.4e-5f, f_hz = 2.0f;
    float out_max = 0.0f, out_min = 0.0f;
    bool  all_zero = true;
    explicit Feeder(INCUNEST_AFE4490& a) : afe(a) {}
    void run(float amp, float seconds, float measure_s = 0.0f,
             ProbeState ps = ProbeState::PROBE_APPLIED, float extra = 0.0f) {
        const int total = (int)lroundf(seconds * FS), from = total - (int)lroundf(measure_s * FS);
        out_max = -2.0f; out_min = 2.0f; all_zero = true;
        for (int i = 0; i < total; i++, n++) {
            const float x = dc + amp * sinf(2.0f * (float)M_PI * f_hz * (float)n / FS) + extra;
            afe.test_feed_ppgdisp(x, ps);
            const float y = afe.test_ppg_disp();
            if (y != 0.0f) all_zero = false;
            if (i >= from) { if (y > out_max) out_max = y; if (y < out_min) out_min = y; }
        }
    }
};

void setUp() {}
void tearDown() {}

// ── 1. NO_NORM (default) is −LP4(BPF(x)) (v0.119), sample by sample, exactly, with both filters
//       reset while the probe is not applied (v0.118): 0 there, the same chain from the first applied
//       sample on ─────────────────────────────────────────────────────────────────────────────────
void test_ot_ac_is_minus_bandpass_bit_exact() {
    INCUNEST_AFE4490 afe;
    INCUNEST_AFE4490::TestBiquadFilter ref;
    INCUNEST_AFE4490::TestLowPass4     ref_lp;
    ref.init_bp(0.5f, 20.0f, FS);
    ref.reset();
    ref_lp.init(20.0f, FS);
    ref_lp.reset();
    TEST_ASSERT_EQUAL(AFE4490PPGDispMode::NO_NORM, afe.getConfig().ppgdisp_mode);
    for (int n = 0; n < 3000; n++) {
        const bool applied = (n < 1000) || (n >= 2000);
        const float x = 1.4e-5f + 1e-6f * sinf(2.0f * (float)M_PI * 2.0f * (float)n / FS) + (n == 700 ? 5e-6f : 0.0f);
        afe.test_feed_ppgdisp(x, applied ? ProbeState::PROBE_APPLIED : ProbeState::PROBE_DISCONNECTED);
        if (!applied) { ref.reset(); ref_lp.reset(); }
        const float expect = -ref_lp.process(ref.process(x));
        TEST_ASSERT_TRUE_MESSAGE(afe.test_ppg_disp() == expect, "NO_NORM must be bit-identical to -LP4(BPF(x)) with the reset of the probe gate");
        if (!applied) TEST_ASSERT_TRUE_MESSAGE(afe.test_ppg_disp() == 0.0f, "NO_NORM is exactly 0 while the probe is not applied");
    }
}

// ── 2. NORM: three amplitudes × two DC levels all peak at ±1/H after one window ────────────────
void test_norm_fills_frame_for_any_amplitude_and_dc() {
    const float amps[3] = {2e-7f, 1e-6f, 5e-6f};
    const float dcs[2]  = {1e-5f, 1e-4f};
    for (int a = 0; a < 3; a++) for (int d = 0; d < 2; d++) {
        INCUNEST_AFE4490 afe;
        afe.setPPGDispMode(AFE4490PPGDispMode::NORM);
        Feeder f(afe); f.dc = dcs[d];
        f.run(amps[a], 0.9f);                      // the first window: no output yet
        TEST_ASSERT_TRUE_MESSAGE(f.all_zero, "NORM is 0 during the warm-up window");
        f.run(amps[a], 3.0f, 1.0f);
        TEST_ASSERT_FLOAT_WITHIN(0.02f * FULL,  FULL, f.out_max);
        TEST_ASSERT_FLOAT_WITHIN(0.02f * FULL, -FULL, f.out_min);
    }
}

// ── 3. NORM: an RF-like ×2 step clips first (the rise is slew-limited) and settles in 3 s; ×0.5 recovers in 4 s
void test_norm_step_up_is_slew_limited_and_step_down_recovers() {
    INCUNEST_AFE4490 afe;
    afe.setPPGDispMode(AFE4490PPGDispMode::NORM);
    Feeder f(afe);
    f.run(1e-6f, 4.0f, 1.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.02f * FULL, FULL, f.out_max);
    f.run(2e-6f, 0.4f, 0.3f);                      // right after the step: the frame has not caught up
    TEST_ASSERT_TRUE_MESSAGE(f.out_max >= 0.999f, "a doubled beat clips while the frame grows");
    // The frame doubles within ln2/0.5 = 1.4 s, but the step's own band-pass transient sits in the
    // window for 1 s and then leaves the frame with τ: settled to 2 % after 5 s.
    f.run(2e-6f, 5.0f, 1.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.02f * FULL,  FULL, f.out_max);
    TEST_ASSERT_FLOAT_WITHIN(0.02f * FULL, -FULL, f.out_min);
    // ×0.5: the old extremes stay in the 1 s window, then the frame falls with τ — 90 % of full
    // size after 1 s + 2.3τ, 95 % after 1 s + 3τ ≈ 4 s, 98 % after 5 s (measured: 0.824/0.870 at 4 s).
    f.run(1e-6f, 4.0f, 1.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.06f * FULL,  FULL, f.out_max);
    f.run(1e-6f, 1.0f, 1.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.03f * FULL,  FULL, f.out_max);
    TEST_ASSERT_FLOAT_WITHIN(0.03f * FULL, -FULL, f.out_min);
}

// ── 4. NORM_PI: A(PI) and the clamp at ±1, not at ±A ──────────────────────────────────────────
void test_norm_pi_amplitude_follows_pi_and_clamps_at_one() {
    INCUNEST_AFE4490 afe;
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.50f, afe.test_ppgdisp_amp_of_pi(0.5f));
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.75f, afe.test_ppgdisp_amp_of_pi(1.0f));
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 1.00f, afe.test_ppgdisp_amp_of_pi(2.0f));
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 1.00f, afe.test_ppgdisp_amp_of_pi(5.0f));
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.50f, afe.test_ppgdisp_amp_of_pi(0.2f));
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.50f, afe.test_ppgdisp_amp_of_pi(NAN));

    afe.setPPGDispMode(AFE4490PPGDispMode::NORM_PI);
    afe.test_set_pi(0.5f);                         // A = 0.5
    Feeder f(afe);
    f.run(1e-6f, 4.0f, 1.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.02f * FULL,  0.5f * FULL, f.out_max);
    TEST_ASSERT_FLOAT_WITHIN(0.02f * FULL, -0.5f * FULL, f.out_min);
    afe.test_set_pi(1.0f);                         // A = 0.75, applied on the next sample
    f.run(1e-6f, 2.0f, 1.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.02f * FULL, 0.75f * FULL, f.out_max);
    afe.test_set_pi(0.5f);
    f.run(1e-6f, 2.0f, 0.5f);
    TEST_ASSERT_FLOAT_WITHIN(0.02f * FULL, 0.5f * FULL, f.out_max);
    f.run(3e-6f, 0.6f, 0.6f);                      // one full cycle three times the frame: reaches ±1, not ±A
    TEST_ASSERT_TRUE_MESSAGE(f.out_max >= 0.999f, "NORM_PI clamps at +1, not at +A");
    TEST_ASSERT_TRUE_MESSAGE(f.out_min <= -0.999f, "NORM_PI clamps at -1, not at -A");
}

// ── 5. Probe not applied: 0 and a reset tracker; re-application warms up again ─────────────────
void test_not_applied_is_zero_and_resets_tracker() {
    INCUNEST_AFE4490 afe;
    afe.setPPGDispMode(AFE4490PPGDispMode::NORM);
    Feeder f(afe);
    f.run(1e-6f, 3.0f, 1.0f);
    TEST_ASSERT_TRUE(afe.test_ppgdisp_warmed());
    f.run(1e-6f, 1.0f, 1.0f, ProbeState::PROBE_DISCONNECTED);
    TEST_ASSERT_TRUE_MESSAGE(f.all_zero, "not applied -> exactly 0");
    TEST_ASSERT_FALSE(afe.test_ppgdisp_warmed());
    f.run(1e-6f, 0.9f);                            // re-applied: the warm-up window again
    TEST_ASSERT_TRUE(f.all_zero);
    f.run(1e-6f, 2.0f, 1.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.02f * FULL, FULL, f.out_max);
}

// ── 6. A 5× spike of 100 ms saturates briefly and does not re-scale the trace for long ─────────
// Measured on the same chain in PPGDISPLAB (2026-11-07): the frame grows while the spike sits in
// the 1 s window (×2.3 at most with rise 0.5/s), is back within 5 % about 4.5 s after the spike,
// and the trace saturates 0.25 s. This pins those numbers.
void test_spike_saturates_briefly_frame_bounded_and_recovers() {
    INCUNEST_AFE4490 afe;
    afe.setPPGDispMode(AFE4490PPGDispMode::NORM);
    Feeder f(afe);
    f.run(1e-6f, 4.0f, 1.0f);
    const float range0 = afe.test_ppgdisp_frame_hi() - afe.test_ppgdisp_frame_lo();
    f.run(1e-6f, 0.1f, 0.1f, ProbeState::PROBE_APPLIED, 5e-6f);   // +5× in OT = a dip after the negation
    TEST_ASSERT_TRUE_MESSAGE(f.out_min <= -0.999f, "the spike saturates the trace");
    float range_max = range0; int saturated = 0;
    for (int i = 0; i < (int)(2.0f * FS); i++) {
        f.run(1e-6f, 1.0f / FS, 1.0f / FS);
        const float r = afe.test_ppgdisp_frame_hi() - afe.test_ppgdisp_frame_lo();
        if (r > range_max) range_max = r;
        if (f.out_max >= 0.999f || f.out_min <= -0.999f) saturated++;
    }
    TEST_ASSERT_TRUE_MESSAGE(range_max <= 2.6f * range0, "frame growth is bounded by the slew limit");
    TEST_ASSERT_TRUE_MESSAGE(saturated <= (int)(0.35f * FS), "saturation lasts a fraction of a second");
    f.run(1e-6f, 3.0f, 1.0f);                      // 5 s after the spike
    TEST_ASSERT_FLOAT_WITHIN(0.05f * FULL, FULL, f.out_max);
    TEST_ASSERT_FLOAT_WITHIN(0.05f * range0, range0, afe.test_ppgdisp_frame_hi() - afe.test_ppgdisp_frame_lo());
}

// ── 7. A flat window does not trap the frame: the pulse re-acquires in one window ──────────────
void test_flat_signal_is_zero_and_pulse_reacquires_in_one_window() {
    INCUNEST_AFE4490 afe;
    afe.setPPGDispMode(AFE4490PPGDispMode::NORM);
    Feeder f(afe);
    f.run(0.0f, 3.0f, 2.0f);                       // probe applied, no pulse
    TEST_ASSERT_TRUE_MESSAGE(f.all_zero, "flat input -> 0, not amplified noise");
    f.run(1e-6f, 0.9f);                            // the warm-up window again
    TEST_ASSERT_TRUE(f.all_zero);
    f.run(1e-6f, 0.6f, 0.5f);                      // output within the next half second...
    TEST_ASSERT_TRUE_MESSAGE(f.out_max > 0.5f * FULL, "the pulse shows one window after it returns");
    f.run(1e-6f, 3.0f, 1.0f);                      // ...full size once the onset transient has left the frame
    TEST_ASSERT_FLOAT_WITHIN(0.02f * FULL, FULL, f.out_max);
}

// ── 8. Setter ranges, configuration read-back, codec round trip ────────────────────────────────
void test_tracker_setter_ranges_config_and_codec() {
    INCUNEST_AFE4490 afe;
    AFE4490Config c = afe.getConfig();
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.5f,  c.ppgdisp_norm_rise_per_s);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1.0f,  c.ppgdisp_norm_win_s);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1.0f,  c.ppgdisp_norm_tau_s);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1.15f, c.ppgdisp_norm_headroom);
    afe.setPPGDispNormTracker(10.0f, 1.0f, 1.0f, 1.15f);   // rise out of range: ignored
    afe.setPPGDispNormTracker(0.5f, 1.0f, 1.0f, 0.9f);     // headroom out of range: ignored
    c = afe.getConfig();
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.5f,  c.ppgdisp_norm_rise_per_s);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1.15f, c.ppgdisp_norm_headroom);
    afe.setPPGDispNormTracker(0.5f, 2.0f, 1.0f, 1.0f);     // window 2 s, no headroom
    afe.setPPGDispMode(AFE4490PPGDispMode::NORM);
    c = afe.getConfig();
    TEST_ASSERT_EQUAL(AFE4490PPGDispMode::NORM, c.ppgdisp_mode);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 2.0f, c.ppgdisp_norm_win_s);
    Feeder f(afe);
    f.run(1e-6f, 1.9f);                            // a 2 s window is a 2 s warm-up
    TEST_ASSERT_TRUE(f.all_zero);
    f.run(1e-6f, 3.0f, 1.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.02f, 1.0f, f.out_max);   // H = 1: the frame's extrema map to ±1

    AFE4490PPGDispMode m;
    TEST_ASSERT_EQUAL_STRING("no_norm",   afePPGDispModeToStr(AFE4490PPGDispMode::NO_NORM));
    TEST_ASSERT_EQUAL_STRING("norm",    afePPGDispModeToStr(AFE4490PPGDispMode::NORM));
    TEST_ASSERT_EQUAL_STRING("norm_pi", afePPGDispModeToStr(AFE4490PPGDispMode::NORM_PI));
    TEST_ASSERT_TRUE(afeStrToPPGDispMode("norm_pi", m)); TEST_ASSERT_EQUAL(AFE4490PPGDispMode::NORM_PI, m);
    TEST_ASSERT_TRUE(afeStrToPPGDispMode("no_norm", m));   TEST_ASSERT_EQUAL(AFE4490PPGDispMode::NO_NORM, m);
    TEST_ASSERT_FALSE(afeStrToPPGDispMode("scaled", m));
}

// ── 9. NORM_PI thresholds are their own parameters, not the SQI ramp's (v0.117) ────────────────
void test_norm_pi_thresholds_are_decoupled_from_sqi() {
    INCUNEST_AFE4490 afe;
    AFE4490Config c = afe.getConfig();
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.5f, c.ppgdisp_norm_pi_lo);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 2.0f, c.ppgdisp_norm_pi_hi);
    afe.setSpO2SqiPiThresholds(0.2f, 1.0f);                // the SQI ramp moves...
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.50f, afe.test_ppgdisp_amp_of_pi(0.5f));   // ...A(PI) does not
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.75f, afe.test_ppgdisp_amp_of_pi(1.0f));
    afe.setPPGDispNormPI(2.0f, 1.0f);                       // lo ≥ hi: ignored
    afe.setPPGDispNormPI(0.5f, 25.0f);                      // above the displayed range: ignored
    afe.setPPGDispNormPI(0.0f, 2.0f);                       // lo must be positive: ignored
    c = afe.getConfig();
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.5f, c.ppgdisp_norm_pi_lo);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 2.0f, c.ppgdisp_norm_pi_hi);
    afe.setPPGDispNormPI(1.0f, 4.0f);                       // own pair: A(2 %) = 0.75, A(1 %) = 0.5
    c = afe.getConfig();
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1.0f, c.ppgdisp_norm_pi_lo);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 4.0f, c.ppgdisp_norm_pi_hi);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.75f, afe.test_ppgdisp_amp_of_pi(2.0f));
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.50f, afe.test_ppgdisp_amp_of_pi(1.0f));
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 1.00f, afe.test_ppgdisp_amp_of_pi(4.0f));
    afe.setPPGDispMode(AFE4490PPGDispMode::NORM_PI);      // and the trace follows the new pair
    afe.test_set_pi(2.0f);
    Feeder f(afe);
    f.run(1e-6f, 4.0f, 1.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.02f * FULL, 0.75f * FULL, f.out_max);
}

// ── 10. An application: OT 5.9e-4 in air → 1.2e-4 + a beat of PI 1 %, PROBE_APPLIED confirmed 0.2 s
//        after the step (rsqm_probe_state_min_s). The band-pass is reset until then (v0.118), so the
//        step never enters the SLEW warm-up: NORM is full size in the first second after the warm-up
//        window. With the v0.117 chain (filter running through the step) the same input needed ~8 s
//        (rationale §21.9). NO_NORM: 0 while not applied, and only the beat afterwards. ────────────
void test_application_step_never_enters_the_frame() {
    for (int mode = 0; mode < 2; mode++) {
        INCUNEST_AFE4490 afe;
        afe.setPPGDispMode(mode == 0 ? AFE4490PPGDispMode::NO_NORM : AFE4490PPGDispMode::NORM);
        Feeder f(afe);
        f.f_hz = 2.5f;
        f.dc = 5.9e-4f;
        f.run(0.0f, 3.0f, 1.0f, ProbeState::PROBE_OT_HIGH);                 // in air, no pulse
        TEST_ASSERT_TRUE_MESSAGE(f.all_zero, "not applied -> 0 in every mode");
        f.dc = 1.2e-4f;
        f.run(0.6e-6f, 0.2f, 0.2f, ProbeState::PROBE_OT_HIGH);             // tissue, state not yet confirmed
        TEST_ASSERT_TRUE(f.all_zero);
        if (mode == 0) {
            f.run(0.6e-6f, 0.5f, 0.5f);                                     // NO_NORM: the beat, not the step
            TEST_ASSERT_TRUE_MESSAGE(f.out_max < 2.0f * 0.6e-6f && f.out_min > -2.0f * 0.6e-6f,
                                     "NO_NORM after application must be beat-sized, not the 4.7e-4 step");
        } else {
            f.run(0.6e-6f, 0.9f, 0.9f);                                     // the warm-up window (as test 5)
            TEST_ASSERT_TRUE_MESSAGE(f.all_zero, "NORM: 0 during the warm-up window");
            f.run(0.6e-6f, 1.1f, 1.0f);                                     // the first second of output
            TEST_ASSERT_FLOAT_WITHIN(0.05f * FULL, FULL, f.out_max);
            TEST_ASSERT_FLOAT_WITHIN(0.05f * FULL, -FULL, f.out_min);
        }
    }
}

// ── 11. The high side is 4th order (v0.119): with the default corners (0.5–20 Hz) a tone at 25 Hz
//        (the Nyquist of a 50 Hz consumer) is attenuated ≥ 12 dB, one at 50 Hz (mains, folded onto
//        the baseline by that consumer) ≥ 40 dB, and the band (2.5 Hz) passes within 1 dB. The
//        v0.118 chain (band-pass alone) gave −4 and −9 dB. Steady-state peak after 3 s, NO_NORM. ──
static float chain_gain_at(float f_hz) {
    INCUNEST_AFE4490 afe;
    Feeder f(afe);
    f.f_hz = f_hz;
    f.run(1e-6f, 3.0f, 1.0f);
    const float peak = (f.out_max > -f.out_min) ? f.out_max : -f.out_min;
    return peak / 1e-6f;
}

void test_high_side_is_fourth_order() {
    TEST_ASSERT_TRUE_MESSAGE(chain_gain_at(2.5f) > 0.89f, "2.5 Hz must pass within 1 dB");
    TEST_ASSERT_TRUE_MESSAGE(chain_gain_at(25.0f) < 0.251f, "25 Hz must be attenuated >= 12 dB (default f_high 20 Hz)");
    TEST_ASSERT_TRUE_MESSAGE(chain_gain_at(50.0f) < 0.010f, "50 Hz must be attenuated >= 40 dB (default f_high 20 Hz)");
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_ot_ac_is_minus_bandpass_bit_exact);
    RUN_TEST(test_norm_fills_frame_for_any_amplitude_and_dc);
    RUN_TEST(test_norm_step_up_is_slew_limited_and_step_down_recovers);
    RUN_TEST(test_norm_pi_amplitude_follows_pi_and_clamps_at_one);
    RUN_TEST(test_not_applied_is_zero_and_resets_tracker);
    RUN_TEST(test_spike_saturates_briefly_frame_bounded_and_recovers);
    RUN_TEST(test_flat_signal_is_zero_and_pulse_reacquires_in_one_window);
    RUN_TEST(test_tracker_setter_ranges_config_and_codec);
    RUN_TEST(test_norm_pi_thresholds_are_decoupled_from_sqi);
    RUN_TEST(test_application_step_never_enters_the_frame);
    RUN_TEST(test_high_side_is_fourth_order);
    return UNITY_END();
}
