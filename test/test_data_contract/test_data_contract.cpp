#include <unity.h>
#include <math.h>
#include "incunest_afe4490.h"

// Data contract of the clinical output quantities (spec §2.1.1, rationale §25): spo2, pi, hr1, hr2
// and hr3 are a finite number inside their declared range, or NaN. These tests pin rules 1 and 2
// where they are easiest to break: before any sample, with no probe, and from the first sample of
// an application. Never a 0 standing for "no value". (The case of rule 4, HR1 expiring when beats
// stop, is in test_hr1.)

static constexpr float FS    = 500.0f;
static constexpr float OT_DC = 1.4e-5f;     // typical OT with the probe applied (spec)

void setUp() {}
void tearDown() {}

static void assert_all_nan(INCUNEST_AFE4490& afe, const char* when) {
    TEST_ASSERT_TRUE_MESSAGE(isnan(afe.test_spo2()), when);
    TEST_ASSERT_TRUE_MESSAGE(isnan(afe.test_pi()),   when);
    TEST_ASSERT_TRUE_MESSAGE(isnan(afe.test_hr1()),  when);
    TEST_ASSERT_TRUE_MESSAGE(isnan(afe.test_hr2()),  when);
    TEST_ASSERT_TRUE_MESSAGE(isnan(afe.test_hr3()),  when);
}

// Each algorithm fed as the task body feeds it, PI first.
static void feed_all(INCUNEST_AFE4490& afe, float ot_ir, float ot_red, ProbeState p) {
    afe.test_feed_pi(ot_ir, p);
    afe.test_feed_spo2(ot_ir, ot_red, p);
    afe.test_feed_hr1(ot_ir, p);
    afe.test_feed_hr2(ot_ir, p);
    afe.test_feed_hr3(ot_ir, p);
}

// A clean pulse at 120 BPM: 2 % modulation on IR, 1 % on RED.
static void feed_pulse(INCUNEST_AFE4490& afe, int i) {
    const float s = sinf(2.0f * (float)M_PI * 2.0f * i / FS);
    feed_all(afe, OT_DC * (1.0f + 0.02f * s), OT_DC * (1.0f + 0.01f * s), ProbeState::PROBE_APPLIED);
}

// ── Test 1: before any sample ────────────────────────────────────────────────
// Until v0.122 the published sample started value-initialised: five quantities at 0.0.
void test_contract_nan_before_any_sample() {
    INCUNEST_AFE4490 afe;
    assert_all_nan(afe, "before any sample");
}

// ── Test 2: the whole pipeline with no probe ─────────────────────────────────
// Raw codes of a dark photodiode through _process_sample(), the path of the firmware: every
// sample carries five NaN, from the first one.
void test_contract_nan_with_no_probe() {
    INCUNEST_AFE4490 afe;
    for (int i = 0; i < 1000; i++) {            // 2 s
        afe.test_feed_sample(0, 0, 0, 0);
        afe.test_run_pending_hr_computations();
        TEST_ASSERT_TRUE(afe.test_probe_state() != ProbeState::PROBE_APPLIED);
        assert_all_nan(afe, "no probe");
    }
}

// ── Test 3: from the first sample of an application ──────────────────────────
// The probe applied from sample 0. On every sample each quantity is NaN or a number inside its
// declared range, and each of them does get a value (the test is not vacuous).
void test_contract_nan_or_in_range_from_first_applied_sample() {
    INCUNEST_AFE4490 afe;
    const AFE4490Config c = afe.getConfig();
    bool got_spo2 = false, got_pi = false, got_hr1 = false, got_hr2 = false, got_hr3 = false;
    for (int i = 0; i < 15000; i++) {           // 30 s
        feed_pulse(afe, i);
        const float spo2 = afe.test_spo2(), pi = afe.test_pi();
        const float hr[3] = { afe.test_hr1(), afe.test_hr2(), afe.test_hr3() };
        if (!isnan(spo2)) {
            got_spo2 = true;
            TEST_ASSERT_TRUE(isfinite(spo2) && spo2 >= c.spo2_min && spo2 <= c.spo2_max);
        }
        if (!isnan(pi)) {
            got_pi = true;
            TEST_ASSERT_TRUE(isfinite(pi) && pi > 0.0f);    // a modulated signal: strictly positive
        }
        for (int k = 0; k < 3; k++) {
            if (isnan(hr[k])) continue;
            TEST_ASSERT_TRUE(isfinite(hr[k]) && hr[k] >= c.hr_min_bpm && hr[k] <= c.hr_max_bpm);
        }
        got_hr1 = got_hr1 || !isnan(hr[0]);
        got_hr2 = got_hr2 || !isnan(hr[1]);
        got_hr3 = got_hr3 || !isnan(hr[2]);
    }
    TEST_ASSERT_TRUE_MESSAGE(got_spo2, "spo2 never published");
    TEST_ASSERT_TRUE_MESSAGE(got_pi,   "pi never published");
    TEST_ASSERT_TRUE_MESSAGE(got_hr1,  "hr1 never published");
    TEST_ASSERT_TRUE_MESSAGE(got_hr2,  "hr2 never published");
    TEST_ASSERT_TRUE_MESSAGE(got_hr3,  "hr3 never published");
}

// ── Test 4: the probe comes off ──────────────────────────────────────────────
// After values have been published, the first sample with the probe not applied carries five NaN.
void test_contract_nan_when_probe_comes_off() {
    INCUNEST_AFE4490 afe;
    for (int i = 0; i < 15000; i++) feed_pulse(afe, i);
    TEST_ASSERT_FALSE(isnan(afe.test_hr1()));
    TEST_ASSERT_FALSE(isnan(afe.test_pi()));
    feed_all(afe, OT_DC, OT_DC, ProbeState::PROBE_OT_HIGH);
    assert_all_nan(afe, "probe off");
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_contract_nan_before_any_sample);
    RUN_TEST(test_contract_nan_with_no_probe);
    RUN_TEST(test_contract_nan_or_in_range_from_first_applied_sample);
    RUN_TEST(test_contract_nan_when_probe_comes_off);
    return UNITY_END();
}
