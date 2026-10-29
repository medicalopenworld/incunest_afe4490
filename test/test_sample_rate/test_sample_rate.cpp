#include <unity.h>
#include <math.h>
#include <stdio.h>
#include "incunest_afe4490.h"

// Exhaustive validation of the sample-rate catalogue (spec §7.4).
//
// This is the test that makes a closed catalogue worth having: because the set is finite and
// enumerable, EVERY supported rate is a test case, and the invariants below are exactly the ones
// a validation report would have to assert. A continuous range could not be covered this way.
//
// Four conditions per entry:
//   1. q = (afeclk/hz)/4 exact       -> the real PRF equals the requested one, so every tau and
//                                       alpha derived from the nominal rate is correct.
//   2. hz multiple of 50             -> both analysis chains get an INTEGER decimation factor
//                                       and hold their 50 Hz rate (spec 5.3.1).
//   3. both sampling windows >= 50us -> TI's minimum (datasheet 8.3.1.1). The ambient window is
//                                       the binding one and sets the ~1660 Hz ceiling.
//   4. NUMAV_max >= 1                -> at least one ADC conversion fits in the window.

static constexpr uint32_t AFECLK          = 4000000u;  // counts/s, 1 count = 0.25 us
static constexpr uint32_t TIA_SETTLE_MIN  = 50u;       // counts (12.5 us)
static constexpr float    TIA_SETTLE_FRAC = 0.10f;
static constexpr uint32_t AMB_MARGIN      = 400u;      // counts (100 us), raised in v0.47
static constexpr uint32_t TI_MIN_WINDOW   = 200u;      // counts = 50 us

void setUp() {}
void tearDown() {}

void test_catalogue_is_not_empty() {
    TEST_ASSERT_EQUAL_UINT8(5, INCUNEST_AFE4490::sampleRateCount());
    TEST_ASSERT_EQUAL_UINT16(500, INCUNEST_AFE4490::sampleRateHz(AFE4490SampleRate::HZ_500));
    TEST_ASSERT_EQUAL_UINT16(1600, INCUNEST_AFE4490::sampleRateHz(AFE4490SampleRate::HZ_1600));
}

void test_every_rate_satisfies_its_invariants() {
    for (uint8_t i = 0; i < INCUNEST_AFE4490::sampleRateCount(); i++) {
        const uint16_t hz = kAFE_SAMPLE_RATE_HZ[i];
        char msg[96];

        // 1. exact q
        snprintf(msg, sizeof(msg), "%u Hz: q not exact (AFECLK %% (4*hz) != 0)", hz);
        TEST_ASSERT_EQUAL_UINT32_MESSAGE(0, AFECLK % (4u * hz), msg);
        const uint32_t q = (AFECLK / hz) / 4u;

        // 2. integer decimation factor for the analysis chains
        snprintf(msg, sizeof(msg), "%u Hz: not a multiple of 50 -> fractional decimation", hz);
        TEST_ASSERT_EQUAL_UINT16_MESSAGE(0, hz % 50u, msg);

        // 3. both sampling windows within TI's 50 us minimum
        uint32_t led_margin = (uint32_t)((float)q * TIA_SETTLE_FRAC);
        if (led_margin < TIA_SETTLE_MIN) led_margin = TIA_SETTLE_MIN;
        const uint32_t win_led = (q > led_margin + 2u) ? (q - 2u - led_margin) : 0u;
        const uint32_t win_amb = (q > AMB_MARGIN  + 2u) ? (q - 2u - AMB_MARGIN)  : 0u;
        snprintf(msg, sizeof(msg), "%u Hz: LED window %u counts < 50 us", hz, (unsigned)win_led);
        TEST_ASSERT_TRUE_MESSAGE(win_led >= TI_MIN_WINDOW, msg);
        snprintf(msg, sizeof(msg), "%u Hz: ambient window %u counts < 50 us", hz, (unsigned)win_amb);
        TEST_ASSERT_TRUE_MESSAGE(win_amb >= TI_MIN_WINDOW, msg);

        // The ambient start must never be programmed after its end (the >2487 Hz failure).
        snprintf(msg, sizeof(msg), "%u Hz: ALEDxSTC would exceed ALEDxENDC", hz);
        TEST_ASSERT_TRUE_MESSAGE(q >= AMB_MARGIN + 2u, msg);

        // 4. at least one ADC conversion fits
        snprintf(msg, sizeof(msg), "%u Hz: no ADC conversion fits (5000/hz < 1)", hz);
        TEST_ASSERT_TRUE_MESSAGE((5000u / hz) >= 1u, msg);
    }
}

// Every catalogue entry must round-trip through the numeric form, and nothing else may pass.
void test_numeric_form_accepts_only_catalogue_values() {
    for (uint8_t i = 0; i < INCUNEST_AFE4490::sampleRateCount(); i++) {
        AFE4490SampleRate r;
        TEST_ASSERT_TRUE(INCUNEST_AFE4490::isValidSampleRateHz(kAFE_SAMPLE_RATE_HZ[i], &r));
        TEST_ASSERT_EQUAL_UINT16(kAFE_SAMPLE_RATE_HZ[i], INCUNEST_AFE4490::sampleRateHz(r));
    }
    // All of these used to be accepted. 3000 and 5000 Hz invert the ambient timer pair, 250 Hz
    // collapses the anti-alias margin, 625 Hz would need a fractional decimation factor.
    static const uint16_t rejected[] = {63, 100, 250, 400, 625, 750, 2000, 3000, 5000, 0, 65535};
    for (unsigned k = 0; k < sizeof(rejected) / sizeof(rejected[0]); k++) {
        char msg[64];
        snprintf(msg, sizeof(msg), "%u Hz should not be accepted", rejected[k]);
        TEST_ASSERT_FALSE_MESSAGE(INCUNEST_AFE4490::isValidSampleRateHz(rejected[k]), msg);
    }
}

// A rejected rate must leave the configuration untouched, not fall back to a default.
void test_rejected_rate_leaves_configuration_untouched() {
    INCUNEST_AFE4490 afe;
    afe.setSampleRate(AFE4490SampleRate::HZ_1000);
    TEST_ASSERT_EQUAL_UINT16(20, afe.test_hr2_decim_factor());    // 1000/50
    afe.setSampleRate((uint16_t)3000);                            // rejected
    TEST_ASSERT_EQUAL_UINT16(20, afe.test_hr2_decim_factor());    // unchanged
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 50.0f, afe.test_hr2_decim_rate_hz());
}

// Both chains must hold their 50 Hz target across the whole catalogue, with an exact factor.
void test_chains_hold_50hz_across_catalogue() {
    INCUNEST_AFE4490 afe;
    for (uint8_t i = 0; i < INCUNEST_AFE4490::sampleRateCount(); i++) {
        afe.setSampleRate((AFE4490SampleRate)i);
        char msg[72];
        snprintf(msg, sizeof(msg), "%u Hz: HR2 chain rate", kAFE_SAMPLE_RATE_HZ[i]);
        TEST_ASSERT_FLOAT_WITHIN_MESSAGE(0.01f, 50.0f, afe.test_hr2_decim_rate_hz(), msg);
        snprintf(msg, sizeof(msg), "%u Hz: HR3 chain rate", kAFE_SAMPLE_RATE_HZ[i]);
        TEST_ASSERT_FLOAT_WITHIN_MESSAGE(0.01f, 50.0f, afe.test_hr3_decim_rate_hz(), msg);
        snprintf(msg, sizeof(msg), "%u Hz: decimation factor must be integer-exact",
                 kAFE_SAMPLE_RATE_HZ[i]);
        TEST_ASSERT_EQUAL_UINT16_MESSAGE(kAFE_SAMPLE_RATE_HZ[i] / 50u,
                                         afe.test_hr2_decim_factor(), msg);
    }
}

// A rate change must tear down everything accumulated at the previous rate. Recalculating
// coefficients is not enough: biquad state, EMA state, the decimated buffers and the decimation
// phase all belong to the old rate, and the chip's four sampling windows have just been
// rewritten (which is why settling must be armed).
void test_rate_change_tears_down_old_state() {
    INCUNEST_AFE4490 afe;

    // Fill HR2's and HR3's buffers at 500 Hz.
    for (int i = 0; i < 6000; i++) {
        float x = 1.4e-5f + 7.0e-6f * sinf(2.0f * (float)M_PI * 1.0f * i / 500.0f);
        afe.test_feed_hr2(x, ProbeState::PROBE_APPLIED);
        afe.test_feed_hr3(x, ProbeState::PROBE_APPLIED);
    }
    TEST_ASSERT_TRUE_MESSAGE(afe.test_hr2_buf_count() > 0, "HR2 buffer should have filled");
    TEST_ASSERT_TRUE_MESSAGE(afe.test_hr3_buf_count() > 0, "HR3 buffer should have filled");

    afe.setSampleRate(AFE4490SampleRate::HZ_1000);

    // Buffers emptied: an 8 s window must not mix samples decimated at two different rates.
    TEST_ASSERT_EQUAL_UINT32_MESSAGE(0, afe.test_hr2_buf_count(),
                                     "HR2 buffer must be emptied on a rate change");
    TEST_ASSERT_EQUAL_UINT32_MESSAGE(0, afe.test_hr3_buf_count(),
                                     "HR3 buffer must be emptied on a rate change");
    // Decimation phase restarted for the new factor.
    TEST_ASSERT_EQUAL_UINT32_MESSAGE(0, afe.test_hr2_decim_phase(),
                                     "decimation phase must restart");
    // Settling armed: the four sampling windows were just rewritten (datasheet 7.7 t5).
    TEST_ASSERT_TRUE_MESSAGE(afe.test_settling_countdown() > 0,
                             "switched-RC settling must be armed on a rate change");
    TEST_ASSERT_EQUAL_UINT32_MESSAGE(afe.test_compute_switched_rc_settling_samples(),
                                     afe.test_settling_countdown(),
                                     "countdown must equal the computed settle length");
}

// Setting the same rate must not pay the teardown: it would drop a full analysis window and
// re-arm settling for nothing.
void test_same_rate_is_a_no_op() {
    INCUNEST_AFE4490 afe;
    for (int i = 0; i < 6000; i++) {
        float x = 1.4e-5f + 7.0e-6f * sinf(2.0f * (float)M_PI * 1.0f * i / 500.0f);
        afe.test_feed_hr2(x, ProbeState::PROBE_APPLIED);
    }
    const uint32_t before = afe.test_hr2_buf_count();
    TEST_ASSERT_TRUE(before > 0);
    afe.setSampleRate(AFE4490SampleRate::HZ_500);           // already 500 Hz
    TEST_ASSERT_EQUAL_UINT32_MESSAGE(before, afe.test_hr2_buf_count(),
                                     "a no-change call must not empty the buffer");
    TEST_ASSERT_EQUAL_UINT32_MESSAGE(0, afe.test_settling_countdown(),
                                     "a no-change call must not arm settling");
}

// HR1's moving average must actually cut where it is configured to, at EVERY catalogue rate.
// Until v0.86b hr1_ma_max_len was 64, so 800-1600 Hz ran clamped with an effective cutoff of
// 6.2-12.5 Hz instead of 5 Hz — silently. A too-wide filter lets the dicrotic notch reach the
// threshold detector, which double-counts and reports a doubled HR; the 0.185 s refractory
// mitigates that unevenly, and least of all in bradycardia, where the notch falls outside it.
void test_hr1_ma_not_clamped_across_catalogue() {
    INCUNEST_AFE4490 afe;
    for (uint8_t i = 0; i < INCUNEST_AFE4490::sampleRateCount(); i++) {
        const uint16_t hz = kAFE_SAMPLE_RATE_HZ[i];
        afe.setSampleRate((AFE4490SampleRate)i);
        const uint32_t want = (uint32_t)((hz / (2.0f * 5.0f)) + 0.5f);   // default 5 Hz cutoff
        char msg[112];
        snprintf(msg, sizeof(msg),
                 "%u Hz: HR1 MA clamped -> effective cutoff %.1f Hz instead of 5 Hz",
                 hz, hz / (2.0f * (float)afe.test_hr1_ma_len()));
        TEST_ASSERT_EQUAL_UINT32_MESSAGE(want, afe.test_hr1_ma_len(), msg);
    }
}

// Every parameter whose meaning is a DURATION must keep that duration across the whole catalogue.
//
// This is the net the rate-dependency audit was missing. Auditing by hand found two live defects
// (HR1's running-max decay written as a bare 0.9999 per-sample literal, and ts_emit_interval fixed
// at 2500 samples) and two latent ones (the HR2/HR3 recompute periods, immune to the AFE rate only
// because the decimated rate is the invariant). A third would have to be found by hand again -
// unless a test sweeps the catalogue and checks the seconds.
//
// The pattern to watch for: anything expressed in SAMPLES or as a PER-SAMPLE factor is a duration
// in disguise, and silently changes meaning when the rate changes.
//
// TWO RULES this test had to learn the hard way (the first version passed with both defects
// deliberately reintroduced):
//   * OBSERVE THE EFFECT, not the coefficient. Reading a derived alpha cannot tell a derived
//     parameter from a hardcoded literal the code uses instead of it - the coefficient stays
//     correct and simply goes unused. Hence the decay is measured on _hr1_running_max itself.
//   * EXERCISE THE SETTER with a non-default value. Checking only defaults cannot distinguish a
//     derived parameter from a constant equal to that default.
void test_time_parameters_hold_across_catalogue() {
    INCUNEST_AFE4490 afe;
    for (uint8_t i = 0; i < INCUNEST_AFE4490::sampleRateCount(); i++) {
        const uint16_t hz = kAFE_SAMPLE_RATE_HZ[i];
        const float fs = (float)hz;
        afe.setSampleRate((AFE4490SampleRate)i);
        char msg[128];

        // --- Sample counts derived from the AFE rate: samples / fs must equal the seconds asked.
        // SpO2 and PI windows (v0.102): length in blocks over the block rate really achieved
        // (100 ms blocks, v0.103) must equal the window in seconds — default 6 s, and a non-default 4 s
        // through the setter (exercised, per the second rule above).
        snprintf(msg, sizeof(msg), "%u Hz: SpO2 window drifted from 6 s", hz);
        TEST_ASSERT_FLOAT_WITHIN_MESSAGE(0.05f, 6.0f,
            (float)afe.test_spo2_buf_n() / afe.test_spo2_block_rate_hz(), msg);
        snprintf(msg, sizeof(msg), "%u Hz: PI window drifted from 6 s", hz);
        TEST_ASSERT_FLOAT_WITHIN_MESSAGE(0.05f, 6.0f,
            (float)afe.test_pi_buf_n() / afe.test_pi_block_rate_hz(), msg);
        afe.setSpO2WindowS(4.0f);
        snprintf(msg, sizeof(msg), "%u Hz: SpO2 window setter (4 s) not honoured", hz);
        TEST_ASSERT_FLOAT_WITHIN_MESSAGE(0.05f, 4.0f,
            (float)afe.test_spo2_buf_n() / afe.test_spo2_block_rate_hz(), msg);
        afe.setSpO2WindowS(6.0f);

        snprintf(msg, sizeof(msg), "%u Hz: HR1 refractory drifted from 0.185 s", hz);
        TEST_ASSERT_FLOAT_WITHIN_MESSAGE(0.005f, 0.185f,
            (float)afe.test_hr1_refractory_samples() / fs, msg);

        snprintf(msg, sizeof(msg), "%u Hz: RSQM probe debounce drifted", hz);
        TEST_ASSERT_FLOAT_WITHIN_MESSAGE(0.02f, 0.2f,
            (float)afe.test_rsqm_probe_min_samples() / fs, msg);

        snprintf(msg, sizeof(msg), "%u Hz: HR1 DC tau drifted from 1.6 s", hz);
        TEST_ASSERT_FLOAT_WITHIN_MESSAGE(0.05f, 1.6f,
            -1.0f / (logf(afe.test_hr1_dc_alpha()) * fs), msg);

        // --- Recompute periods: exercised with 1.0 s, NOT the 0.5 s default, so a constant equal
        // to the default cannot pass. They live in decimated samples, hence the chain rate.
        afe.setHR2UpdateIntervalS(1.0f);
        afe.setHR3UpdateIntervalS(1.0f);
        snprintf(msg, sizeof(msg), "%u Hz: HR2 recompute period is not the 1.0 s requested", hz);
        TEST_ASSERT_FLOAT_WITHIN_MESSAGE(0.02f, 1.0f,
            (float)afe.test_hr2_update_interval() / afe.test_hr2_decim_rate_hz(), msg);
        snprintf(msg, sizeof(msg), "%u Hz: HR3 recompute period is not the 1.0 s requested", hz);
        TEST_ASSERT_FLOAT_WITHIN_MESSAGE(0.02f, 1.0f,
            (float)afe.test_hr3_update_interval() / afe.test_hr3_decim_rate_hz(), msg);
        afe.setHR2UpdateIntervalS(0.5f);
        afe.setHR3UpdateIntervalS(0.5f);
    }
}

// The running-max decay, measured on the running max itself rather than on its coefficient.
//
// Feed one large sample to charge the tracker, then silence for a fixed number of SECONDS, and
// check what fraction survives. The decay is exponential, so after tau seconds the tracker must
// hold exp(-1) of the peak - the same fraction at every rate. The 0.9999 per-sample literal this
// replaces held exp(-t/20) at 500 Hz but exp(-t/6.25) at 1600 Hz.
//
// The test sets tau itself instead of relying on the default: what is under test is the
// rate invariance, not the value shipped. The default is asserted separately below, where a
// deliberate change is meant to fail and send whoever made it to spec §10.4.
void test_hr1_running_max_decay_is_time_based() {
    static const float TAU_S = 4.0f;                // long enough to hold across the catalogue
    static const float T_S   = TAU_S;               // one time constant
    const float expected     = expf(-1.0f);         // 0.368 of the peak, at every rate

    for (uint8_t i = 0; i < INCUNEST_AFE4490::sampleRateCount(); i++) {
        const uint16_t hz = kAFE_SAMPLE_RATE_HZ[i];
        INCUNEST_AFE4490 afe;
        afe.setSampleRate((AFE4490SampleRate)i);
        afe.setHR1MaxDecayTauS(TAU_S);

        // _hr1_update() removes DC and NEGATES (peaks up for conventional PPG polarity), so the
        // input has to go BELOW the DC estimate to produce a positive peak. And the moving average
        // must fill before the peak reaches full amplitude, hence 2x its length.
        const uint32_t charge = 2u * afe.test_hr1_ma_len();
        for (uint32_t k = 0; k < charge; k++)
            afe.test_feed_hr1(-1.0e-4f, ProbeState::PROBE_APPLIED);
        const float peak = afe.test_hr1_running_max();
        TEST_ASSERT_TRUE_MESSAGE(peak > 0.0f, "running max did not charge");

        const uint32_t n = (uint32_t)(T_S * (float)hz);
        for (uint32_t k = 0; k < n; k++)
            afe.test_feed_hr1(0.0f, ProbeState::PROBE_APPLIED);

        const float frac = afe.test_hr1_running_max() / peak;
        char msg[136];
        snprintf(msg, sizeof(msg),
                 "%u Hz: after %.0f s the running max holds %.3f of the peak, expected %.3f "
                 "(decay is per-sample, not per-second)", hz, T_S, frac, expected);
        TEST_ASSERT_FLOAT_WITHIN_MESSAGE(0.05f, expected, frac, msg);
    }
}

// The shipped default, pinned on purpose: it is a measured value (lib v0.93), not an arbitrary
// one, and the measurements live in spec §10.4. If you are here because this failed, the value
// moved - update the spec in the same commit.
void test_hr1_max_decay_default_is_the_measured_value() {
    INCUNEST_AFE4490 afe;
    TEST_ASSERT_FLOAT_WITHIN_MESSAGE(1e-6f, 1.5f, afe.getConfig().hr1_max_decay_tau_s,
        "hr1_max_decay_tau_s default moved away from the measured 1.5 s - see spec SS10.4");
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_catalogue_is_not_empty);
    RUN_TEST(test_every_rate_satisfies_its_invariants);
    RUN_TEST(test_numeric_form_accepts_only_catalogue_values);
    RUN_TEST(test_rejected_rate_leaves_configuration_untouched);
    RUN_TEST(test_chains_hold_50hz_across_catalogue);
    RUN_TEST(test_rate_change_tears_down_old_state);
    RUN_TEST(test_same_rate_is_a_no_op);
    RUN_TEST(test_hr1_ma_not_clamped_across_catalogue);
    RUN_TEST(test_time_parameters_hold_across_catalogue);
    RUN_TEST(test_hr1_running_max_decay_is_time_based);
    RUN_TEST(test_hr1_max_decay_default_is_the_measured_value);
    return UNITY_END();
}
