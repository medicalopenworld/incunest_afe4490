#include <unity.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "incunest_afe4490.h"

// Display quantities (v0.124, spec §2.1.2 and §5.10, rationale §26): spo2_disp, hr_disp and
// pi_disp, what a monitor shows, each with the state that says why.
//
// _disp_update() only reads the clinical output quantities the algorithms leave in the sample,
// so these tests set them (test_set_hr2/hr3/spo2/pi) and call it once per sample
// (test_feed_disp), at the default 500 Hz. Times are taken from the library
// (test_disp_*_samples), never restated here.

static constexpr ProbeState ON  = ProbeState::PROBE_APPLIED;
static constexpr ProbeState OFF = ProbeState::PROBE_OT_HIGH;

void setUp() {}
void tearDown() {}

static void feed(INCUNEST_AFE4490& afe, uint32_t n, ProbeState p = ON) {
    for (uint32_t i = 0; i < n; i++) afe.test_feed_disp(p);
}

// HR2 and HR3 as two resolvers that agree: 140 and 142 bpm, SQI 0.9 and 0.7.
static void set_agreeing(INCUNEST_AFE4490& afe) {
    afe.test_set_hr2(140.0f, 0.9f);
    afe.test_set_hr3(142.0f, 0.7f);
}
static constexpr float AGREEING_HR = (140.0f * 0.9f + 142.0f * 0.7f) / (0.9f + 0.7f);   // 140.875

// ── Test 1: nothing to show before any sample and with no probe ───────────────
void test_disp_nan_without_probe() {
    INCUNEST_AFE4490 afe;
    TEST_ASSERT_TRUE(isnan(afe.test_spo2_disp()));
    TEST_ASSERT_TRUE(isnan(afe.test_hr_disp()));
    TEST_ASSERT_TRUE(isnan(afe.test_pi_disp()));
    TEST_ASSERT_TRUE(afe.test_hr_disp_state() == AFE4490DispState::DISP_NO_PROBE);
    // The sources carry values, but the probe is not applied: still nothing.
    set_agreeing(afe); afe.test_set_spo2(97.0f); afe.test_set_pi(2.0f);
    feed(afe, 5000, OFF);
    TEST_ASSERT_TRUE(isnan(afe.test_spo2_disp()));
    TEST_ASSERT_TRUE(isnan(afe.test_hr_disp()));
    TEST_ASSERT_TRUE(isnan(afe.test_pi_disp()));
    TEST_ASSERT_TRUE(afe.test_spo2_disp_state() == AFE4490DispState::DISP_NO_PROBE);
    TEST_ASSERT_TRUE(afe.test_hr_disp_state()   == AFE4490DispState::DISP_NO_PROBE);
    TEST_ASSERT_TRUE(afe.test_pi_disp_state()   == AFE4490DispState::DISP_NO_PROBE);
}

// ── Test 2: hr_disp needs the agreement to last, then is the SQI-weighted mean ─
void test_disp_hr_shows_after_the_agreement_lasts() {
    INCUNEST_AFE4490 afe;
    const uint32_t on = afe.test_disp_hr_on_samples();
    set_agreeing(afe);
    feed(afe, on);                               // exactly disp_hr_on_s: not yet
    TEST_ASSERT_TRUE(isnan(afe.test_hr_disp()));
    TEST_ASSERT_TRUE(afe.test_hr_disp_state() == AFE4490DispState::DISP_ACQUIRING);
    feed(afe, 1);                                // longer than it
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, AGREEING_HR, afe.test_hr_disp());
    TEST_ASSERT_TRUE(afe.test_hr_disp_state() == AFE4490DispState::DISP_CURRENT);
}

// ── Test 3: every condition of the agreement ──────────────────────────────────
static bool shows(float h2, float s2, float h3, float s3) {
    INCUNEST_AFE4490 afe;
    afe.test_set_hr2(h2, s2);
    afe.test_set_hr3(h3, s3);
    feed(afe, 5 * afe.test_disp_hr_on_samples());
    return !isnan(afe.test_hr_disp());   // 5 s: well past the wait
}
void test_disp_hr_agreement_conditions() {
    TEST_ASSERT_TRUE(shows(140.0f, 0.9f, 142.0f, 0.7f));
    TEST_ASSERT_TRUE(shows(140.0f, 0.5f, 147.9f, 0.8f));     // just inside every limit
    TEST_ASSERT_FALSE(shows(140.0f, 0.9f, 148.0f, 0.9f));    // 8 bpm apart: not closer than 8
    TEST_ASSERT_FALSE(shows(140.0f, 0.79f, 142.0f, 0.79f));  // neither SQI reaches 0.8
    TEST_ASSERT_FALSE(shows(140.0f, 0.9f, 142.0f, 0.49f));   // the worse SQI under 0.5
    TEST_ASSERT_FALSE(shows(140.0f, 0.0f, 142.0f, 0.9f));    // HR2 has no value (NaN)
    TEST_ASSERT_FALSE(shows(140.0f, 0.9f, 142.0f, 0.0f));    // HR3 has no value (NaN)
}

// ── Test 4: one bad instant shows dashes; nothing is held ────────────────────
void test_disp_hr_nothing_is_held() {
    INCUNEST_AFE4490 afe;
    set_agreeing(afe);
    feed(afe, 3 * afe.test_disp_hr_on_samples());
    TEST_ASSERT_FALSE(isnan(afe.test_hr_disp()));
    afe.test_set_hr3(160.0f, 0.9f);              // the resolvers part
    feed(afe, 1);
    TEST_ASSERT_TRUE(isnan(afe.test_hr_disp()));
    // A value had been shown since the probe was applied: no longer "acquiring".
    TEST_ASSERT_TRUE(afe.test_hr_disp_state() == AFE4490DispState::DISP_INADEQUATE);
    feed(afe, 10000);
    TEST_ASSERT_TRUE(isnan(afe.test_hr_disp()));
    TEST_ASSERT_TRUE(afe.test_hr_disp_state() != AFE4490DispState::DISP_HELD);   // reserved
}

// ── Test 5: a short break shows again at once, a long one asks for the wait again ─
void test_disp_hr_short_and_long_breaks() {
    INCUNEST_AFE4490 afe;
    const uint32_t on = afe.test_disp_hr_on_samples(), off = afe.test_disp_hr_off_samples();
    set_agreeing(afe);
    feed(afe, 3 * on);
    afe.test_set_hr3(160.0f, 0.9f);
    feed(afe, off);                              // a break of exactly disp_hr_off_s
    set_agreeing(afe);
    feed(afe, 1);
    TEST_ASSERT_FALSE_MESSAGE(isnan(afe.test_hr_disp()), "a short break must show again at once");

    afe.test_set_hr3(160.0f, 0.9f);
    feed(afe, off + 1);                          // a break longer than it
    set_agreeing(afe);
    feed(afe, on);
    TEST_ASSERT_TRUE_MESSAGE(isnan(afe.test_hr_disp()), "after a long break the agreement must last again");
    feed(afe, 1);
    TEST_ASSERT_FALSE(isnan(afe.test_hr_disp()));
}

// ── Test 6: spo2_disp and pi_disp are spo2 and pi as they are ─────────────────
void test_disp_spo2_and_pi_follow_their_sources() {
    INCUNEST_AFE4490 afe;
    afe.test_set_spo2(NAN); afe.test_set_pi(NAN);
    feed(afe, 100);
    TEST_ASSERT_TRUE(isnan(afe.test_spo2_disp()));
    TEST_ASSERT_TRUE(isnan(afe.test_pi_disp()));
    TEST_ASSERT_TRUE(afe.test_spo2_disp_state() == AFE4490DispState::DISP_ACQUIRING);
    TEST_ASSERT_TRUE(afe.test_pi_disp_state()   == AFE4490DispState::DISP_ACQUIRING);

    afe.test_set_pi(0.37f);                      // PI arrives first: the three are independent
    feed(afe, 1);
    TEST_ASSERT_EQUAL_FLOAT(0.37f, afe.test_pi_disp());
    TEST_ASSERT_TRUE(afe.test_pi_disp_state()   == AFE4490DispState::DISP_CURRENT);
    TEST_ASSERT_TRUE(isnan(afe.test_spo2_disp()));
    TEST_ASSERT_TRUE(afe.test_spo2_disp_state() == AFE4490DispState::DISP_ACQUIRING);

    afe.test_set_spo2(97.2f);                    // a SpO2 at low perfusion is shown: no SQI gate
    feed(afe, 1);
    TEST_ASSERT_EQUAL_FLOAT(97.2f, afe.test_spo2_disp());
    TEST_ASSERT_TRUE(afe.test_spo2_disp_state() == AFE4490DispState::DISP_CURRENT);

    afe.test_set_spo2(NAN);                      // withheld by one of its gates
    feed(afe, 1);
    TEST_ASSERT_TRUE(isnan(afe.test_spo2_disp()));
    TEST_ASSERT_TRUE(afe.test_spo2_disp_state() == AFE4490DispState::DISP_INADEQUATE);
    TEST_ASSERT_EQUAL_FLOAT(0.37f, afe.test_pi_disp());
}

// ── Test 7: "acquiring" runs out ─────────────────────────────────────────────
void test_disp_acquiring_runs_out() {
    INCUNEST_AFE4490 afe;
    const uint32_t acq = afe.test_disp_acquiring_samples();
    afe.test_set_spo2(NAN); afe.test_set_pi(NAN);
    feed(afe, acq);
    TEST_ASSERT_TRUE(afe.test_spo2_disp_state() == AFE4490DispState::DISP_ACQUIRING);
    TEST_ASSERT_TRUE(afe.test_hr_disp_state()   == AFE4490DispState::DISP_ACQUIRING);
    TEST_ASSERT_TRUE(afe.test_pi_disp_state()   == AFE4490DispState::DISP_ACQUIRING);
    feed(afe, 1);
    TEST_ASSERT_TRUE(afe.test_spo2_disp_state() == AFE4490DispState::DISP_INADEQUATE);
    TEST_ASSERT_TRUE(afe.test_hr_disp_state()   == AFE4490DispState::DISP_INADEQUATE);
    TEST_ASSERT_TRUE(afe.test_pi_disp_state()   == AFE4490DispState::DISP_INADEQUATE);
    TEST_ASSERT_TRUE(isnan(afe.test_spo2_disp()));
}

// ── Test 8: taking the probe off starts everything again ─────────────────────
void test_disp_probe_off_resets() {
    INCUNEST_AFE4490 afe;
    set_agreeing(afe); afe.test_set_spo2(97.0f); afe.test_set_pi(2.0f);
    feed(afe, 3 * afe.test_disp_hr_on_samples());
    TEST_ASSERT_FALSE(isnan(afe.test_hr_disp()));
    feed(afe, 1, OFF);
    TEST_ASSERT_TRUE(isnan(afe.test_hr_disp()));
    TEST_ASSERT_TRUE(isnan(afe.test_spo2_disp()));
    TEST_ASSERT_TRUE(isnan(afe.test_pi_disp()));
    TEST_ASSERT_TRUE(afe.test_hr_disp_state() == AFE4490DispState::DISP_NO_PROBE);
    // Applied again: the agreement must last again, and "acquiring" starts over.
    afe.test_set_spo2(NAN);
    feed(afe, afe.test_disp_hr_on_samples());
    TEST_ASSERT_TRUE(isnan(afe.test_hr_disp()));
    TEST_ASSERT_TRUE(afe.test_hr_disp_state()   == AFE4490DispState::DISP_ACQUIRING);
    TEST_ASSERT_TRUE(afe.test_spo2_disp_state() == AFE4490DispState::DISP_ACQUIRING);
}

// ── Test 9: the same turn-by-turn result as IncuNest's CommTask ──────────────
// The rule being moved (IncuNest motherBoard 18.58.0, CommTask.cpp), evaluated once a second:
struct CommTaskRule {
    int  valid_streak = 0, bad_streak = 0;
    bool displaying = false;
    // Returns the HR CommTask would send; 0 = no value.
    float step(float h2, float s2, float h3, float s3) {
        const bool valid = (h2 > 0.0f) && (h3 > 0.0f) && (fabsf(h2 - h3) < 8.0f) &&
                           (fmaxf(s2, s3) >= 0.8f) && (fminf(s2, s3) >= 0.5f);
        if (valid) { bad_streak = 0; if (++valid_streak >= 2) displaying = true; }
        else       { valid_streak = 0; if (++bad_streak >= 3) displaying = false; }
        return (displaying && valid) ? (h2 * s2 + h3 * s3) / (s2 + s3) : 0.0f;
    }
};
// 600 s of resolver outputs that change once a second, drawn at random among agreeing, apart,
// weak and absent. The consumer reads half a second after each change, as a 1 s loop that is not
// in step with the resolvers does. The library's hr_disp must be CommTask's value on every turn.
void test_disp_hr_matches_commtask_turn_by_turn() {
    INCUNEST_AFE4490 afe;
    CommTaskRule ref;
    const uint32_t fs = afe.test_disp_hr_on_samples();    // disp_hr_on_s is 1 s: samples per second
    uint32_t rng = 12345u;
    int shown = 0, hidden = 0;
    for (int k = 0; k < 600; k++) {
        rng = rng * 1664525u + 1013904223u;
        const uint32_t pick = (rng >> 24) % 10u;
        float h2 = 140.0f, s2 = 0.9f, h3 = 143.0f, s3 = 0.6f;       // agreeing (6 in 10)
        if (pick == 6)      { h3 = 170.0f; }                          // apart
        else if (pick == 7) { s2 = 0.7f; }                            // no SQI reaches 0.8
        else if (pick == 8) { s3 = 0.0f; }                            // HR3 absent
        else if (pick == 9) { s2 = 0.0f; s3 = 0.0f; }                 // both absent
        afe.test_set_hr2(h2, s2);
        afe.test_set_hr3(h3, s3);
        feed(afe, fs / 2);
        const float expected = ref.step((s2 > 0.0f) ? h2 : NAN, s2, (s3 > 0.0f) ? h3 : NAN, s3);
        const float got = afe.test_hr_disp();
        char msg[64];
        snprintf(msg, sizeof(msg), "turn %d", k);
        if (expected > 0.0f) { TEST_ASSERT_FLOAT_WITHIN_MESSAGE(1e-3f, expected, got, msg); shown++; }
        else                 { TEST_ASSERT_TRUE_MESSAGE(isnan(got), msg); hidden++; }
        feed(afe, fs - fs / 2);
    }
    TEST_ASSERT_GREATER_THAN_INT(100, shown);      // the comparison covered both outcomes
    TEST_ASSERT_GREATER_THAN_INT(100, hidden);
}

// ── Test 10: the times are seconds at every sample rate ──────────────────────
void test_disp_times_follow_the_sample_rate() {
    INCUNEST_AFE4490 afe;
    TEST_ASSERT_EQUAL_UINT32(500, afe.test_disp_hr_on_samples());
    TEST_ASSERT_EQUAL_UINT32(1000, afe.test_disp_hr_off_samples());
    afe.setSampleRate(AFE4490SampleRate::HZ_1000);
    TEST_ASSERT_EQUAL_UINT32(1000, afe.test_disp_hr_on_samples());
    TEST_ASSERT_EQUAL_UINT32(2000, afe.test_disp_hr_off_samples());
    TEST_ASSERT_EQUAL_UINT32(2u * 500u * 15u, afe.test_disp_acquiring_samples());
}

// ── Test 11 (optional): replay a capture through HR2, HR3 and the display quantities ──
// Runs only when DISP_CAPTURE_BIN and DISP_OUT_CSV are set. The binary file is test_hr1's:
// int32 n, float32 fs (a catalogue rate), n float32 OT_LED1, n int8 ProbeState. Every 0.05 s it
// writes row, probe, hr2, hr2_sqi, hr3, hr3_sqi, hr_disp, hr_disp_state, so that a script can
// run CommTask's rule on the same resolver outputs at any phase and compare.
void test_disp_replay_capture() {
    const char* in_path  = getenv("DISP_CAPTURE_BIN");
    const char* out_path = getenv("DISP_OUT_CSV");
    if (!in_path || !out_path) TEST_IGNORE_MESSAGE("DISP_CAPTURE_BIN / DISP_OUT_CSV not set");
    FILE* f = fopen(in_path, "rb");
    TEST_ASSERT_NOT_NULL_MESSAGE(f, "cannot open DISP_CAPTURE_BIN");
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
    TEST_ASSERT_NOT_NULL_MESSAGE(o, "cannot open DISP_OUT_CSV");
    fprintf(o, "row,probe,hr2,hr2_sqi,hr3,hr3_sqi,hr_disp,hr_disp_state\n");
    const int32_t step = (int32_t)lroundf(0.05f * fs);
    for (int32_t r = 0; r < n; r++) {
        const ProbeState s = (ProbeState)ps[r];
        afe.test_feed_hr2(ot[r], s);
        afe.test_feed_hr3(ot[r], s);
        afe.test_feed_disp(s);
        if (r % step == 0)
            fprintf(o, "%ld,%d,%.4f,%.4f,%.4f,%.4f,%.4f,%d\n", (long)r, (int)ps[r],
                    (double)afe.test_hr2(), (double)afe.test_hr2_sqi(),
                    (double)afe.test_hr3(), (double)afe.test_hr3_sqi(),
                    (double)afe.test_hr_disp(), (int)afe.test_hr_disp_state());
    }
    fclose(o);
    free(ot); free(ps);
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_disp_nan_without_probe);
    RUN_TEST(test_disp_hr_shows_after_the_agreement_lasts);
    RUN_TEST(test_disp_hr_agreement_conditions);
    RUN_TEST(test_disp_hr_nothing_is_held);
    RUN_TEST(test_disp_hr_short_and_long_breaks);
    RUN_TEST(test_disp_spo2_and_pi_follow_their_sources);
    RUN_TEST(test_disp_acquiring_runs_out);
    RUN_TEST(test_disp_probe_off_resets);
    RUN_TEST(test_disp_hr_matches_commtask_turn_by_turn);
    RUN_TEST(test_disp_times_follow_the_sample_rate);
    RUN_TEST(test_disp_replay_capture);
    return UNITY_END();
}
