#include <unity.h>
#include "incunest_afe4490.h"

// TIA feedback capacitance (CF_LED[4:0]) — see incunest_afe4490_spec.md §7.2.
//
// The AFE4490 exposes five switchable capacitors in parallel over a fixed 5 pF base
// (datasheet Figure 114, p.36): bit0=5, bit1=15, bit2=25, bit3=50, bit4=150 pF.
// All 32 combinations are legal, so the register code IS the value:
//     CF(code) = 5 pF + Σ selected weights   → 5 … 250 pF
// Before v0.62 the library only exposed the six single-bit settings, which made auto-CF
// under-select (e.g. 55 pF where 100 pF was reachable and legal).

void setUp() {}
void tearDown() {}

// ── LUT matches the datasheet bit weights over the whole 32-code range ──
void test_cf_table_matches_datasheet_weights() {
    const float w[5] = { 5.0f, 15.0f, 25.0f, 50.0f, 150.0f };
    for (int code = 0; code <= (int)kAFE_CF_CODE_MAX; ++code) {
        float expected = 5.0f;                       // base capacitor, always present
        for (int b = 0; b < 5; ++b)
            if (code & (1 << b)) expected += w[b];
        TEST_ASSERT_EQUAL_FLOAT(expected, kAFE_CF_PF[code]);
    }
    // Datasheet worked example: "to obtain CF = 100 pF, set D[7:3] = 01111".
    TEST_ASSERT_EQUAL_FLOAT(100.0f, kAFE_CF_PF[0x0F]);
    TEST_ASSERT_EQUAL_FLOAT(5.0f,   kAFE_CF_PF[0]);
    TEST_ASSERT_EQUAL_FLOAT(250.0f, kAFE_CF_PF[kAFE_CF_CODE_MAX]);
}

// ── pF → code quantises DOWN, never up (overshooting CF would break the 5τ budget) ──
void test_cf_pf_to_code_quantises_down() {
    TEST_ASSERT_EQUAL(0x0F, afeCFPFToCode(100.0f));   // exact grid value
    TEST_ASSERT_EQUAL(0x0F, afeCFPFToCode(120.0f));   // between 100 and 155 → down to 100
    TEST_ASSERT_EQUAL(0x0E, afeCFPFToCode(99.0f));    // just below 100      → down to 95
    TEST_ASSERT_EQUAL(kAFE_CF_CODE_MAX, afeCFPFToCode(9999.0f));  // clamps at 250 pF
    TEST_ASSERT_EQUAL(0, afeCFPFToCode(4.0f));        // below the 5 pF base → code 0
    TEST_ASSERT_EQUAL(0, afeCFPFToCode(0.0f));
}

// ── Round-trip: every code survives code → pF → code ──
void test_cf_code_pf_round_trip() {
    for (int code = 0; code <= (int)kAFE_CF_CODE_MAX; ++code)
        TEST_ASSERT_EQUAL(code, afeCFPFToCode(afeCFCodeToPF((AFE4490CFCode)code)));
}

// ── String codecs cover all 32 values and round-trip ──
void test_cf_string_codec_round_trip() {
    for (int code = 0; code <= (int)kAFE_CF_CODE_MAX; ++code) {
        AFE4490CFCode back;
        TEST_ASSERT_TRUE(afeStrToCF(afeCFToStr((AFE4490CFCode)code), back));
        TEST_ASSERT_EQUAL(code, back);
    }
    AFE4490CFCode c;
    TEST_ASSERT_FALSE(afeStrToCF("42p", c));   // not on the grid
    TEST_ASSERT_FALSE(afeStrToCF("", c));
}

// ── The six pre-v0.62 strings still map to the same hardware setting ──
void test_cf_legacy_strings_still_valid() {
    const char*  legacy[6] = { "5p", "10p", "20p", "30p", "55p", "155p" };
    const float  pF[6]     = { 5.0f, 10.0f, 20.0f, 30.0f, 55.0f, 155.0f };
    for (int i = 0; i < 6; ++i) {
        AFE4490CFCode code;
        TEST_ASSERT_TRUE(afeStrToCF(legacy[i], code));
        TEST_ASSERT_EQUAL_FLOAT(pF[i], afeCFCodeToPF(code));
    }
}

// ── Auto-CF picks the exact grid value at the default operating point ──
// Regression guard: cf_max is computed as (settle_s / 5 / RF) × 1e12, which in float lands a
// few ULP below the exact value (99.9999924 at 500 Hz / RF_100K). Without kAFE_CF_MATCH_TOL
// this silently drops a whole step to 95 pF and negates the point of exposing all 32 codes.
void test_autocf_reaches_exact_grid_value_at_default() {
    INCUNEST_AFE4490 afe;
    afe.setSampleRate(500);
    afe.setTIAGain(AFE4490RF::RF_100K);
    AFE4490Config cfg = afe.getConfig();
    TEST_ASSERT_EQUAL_FLOAT(100.0f, cfg.afe_tia_cf_led1_pF);
    TEST_ASSERT_EQUAL_FLOAT(100.0f, cfg.afe_tia_cf_led2_pF);
    TEST_ASSERT_EQUAL(0x0F, cfg.afe_tia_cf_led1_code);
}

// ── Auto-CF honours the 5τ ≤ settle budget for every RF at the default rate ──
void test_autocf_respects_settling_budget() {
    const AFE4490RF rfs[7] = { AFE4490RF::RF_10K,  AFE4490RF::RF_25K, AFE4490RF::RF_50K,
                               AFE4490RF::RF_100K, AFE4490RF::RF_250K, AFE4490RF::RF_500K,
                               AFE4490RF::RF_1M };
    // 500 Hz → LED window 2000 counts → margin = 10% = 200 counts = 50 µs at 4 MHz.
    const float settle_s = 200.0f / 4000000.0f;
    for (int i = 0; i < 7; ++i) {
        INCUNEST_AFE4490 afe;
        afe.setSampleRate(500);
        afe.setTIAGain(rfs[i]);
        AFE4490Config cfg = afe.getConfig();
        float tau_s = kAFE_RF_OHM[(int)rfs[i]] * cfg.afe_tia_cf_led1_pF * 1e-12f;
        // Allow the same relative slack the selector uses, no more.
        TEST_ASSERT_TRUE(5.0f * tau_s <= settle_s * (1.0f + kAFE_CF_MATCH_TOL));
    }
}

// ── Lower sample rate → wider settle window → larger CF ──
void test_autocf_grows_when_rate_drops() {
    INCUNEST_AFE4490 fast, slow;
    fast.setSampleRate(1000);
    fast.setTIAGain(AFE4490RF::RF_100K);
    slow.setSampleRate(125);
    slow.setTIAGain(AFE4490RF::RF_100K);
    TEST_ASSERT_TRUE(slow.getConfig().afe_tia_cf_led1_pF > fast.getConfig().afe_tia_cf_led1_pF);
}

// ── Manual setter overrides auto-CF and quantises down ──
void test_set_tiacf_overrides_and_quantises() {
    INCUNEST_AFE4490 afe;
    afe.setSampleRate(500);
    afe.setTIAGain(AFE4490RF::RF_100K);   // auto → 100 pF
    afe.setTIACF(60.0f);                  // exact grid value
    TEST_ASSERT_EQUAL_FLOAT(60.0f, afe.getConfig().afe_tia_cf_led1_pF);
    afe.setTIACF(69.0f);                  // between 60 and 70 → down to 60
    TEST_ASSERT_EQUAL_FLOAT(60.0f, afe.getConfig().afe_tia_cf_led1_pF);
    // Per-channel setters are independent once ENSEPGAIN is on.
    afe.setEnSepGain(true);
    afe.setTIACFLED1(155.0f);
    afe.setTIACFLED2(25.0f);
    AFE4490Config cfg = afe.getConfig();
    TEST_ASSERT_EQUAL_FLOAT(155.0f, cfg.afe_tia_cf_led1_pF);
    TEST_ASSERT_EQUAL_FLOAT(25.0f,  cfg.afe_tia_cf_led2_pF);
}

// ── Datasheet Equation 1 (§8.3.1.1): RF × CF ≤ Rx Sample Time / 10 ──
// The library's auto-CF criterion is stricter (≈4.5×), so auto-selection must always sit
// inside Eq. 1 — but a manual override can exceed it, which is what getCFMaxEq1PF() lets
// the caller detect. Eq. 1 bounds CF; it says nothing about when the sample window opens
// (that margin is for LED/cable settling per §8.3.1.3) — the two are not interchangeable.
void test_eq1_limit_matches_sample_window() {
    INCUNEST_AFE4490 afe;
    afe.setSampleRate(500);
    // 500 Hz: q = 2000, margin = 200 → Rx sample window = q − 2 − margin = 1798 counts = 449.5 µs.
    // Eq. 1 → τ ≤ 44.95 µs → at RF_100K, CF ≤ 449.5 pF.
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 449.5f, afe.getCFMaxEq1PF(AFE4490RF::RF_100K));
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 179.8f, afe.getCFMaxEq1PF(AFE4490RF::RF_250K));
    TEST_ASSERT_FLOAT_WITHIN(1.0f,  44.95f, afe.getCFMaxEq1PF(AFE4490RF::RF_1M));
}

// ── Auto-CF always stays inside Eq. 1, for every RF and every rate ──
void test_autocf_always_within_eq1() {
    const AFE4490RF rfs[7] = { AFE4490RF::RF_10K,  AFE4490RF::RF_25K, AFE4490RF::RF_50K,
                               AFE4490RF::RF_100K, AFE4490RF::RF_250K, AFE4490RF::RF_500K,
                               AFE4490RF::RF_1M };
    const uint16_t rates[4] = { 125, 250, 500, 1000 };
    for (int r = 0; r < 4; ++r) {
        for (int i = 0; i < 7; ++i) {
            INCUNEST_AFE4490 afe;
            afe.setSampleRate(rates[r]);
            afe.setTIAGain(rfs[i]);
            AFE4490Config cfg = afe.getConfig();
            TEST_ASSERT_TRUE(cfg.afe_tia_cf_led1_pF <= afe.getCFMaxEq1PF(rfs[i]));
        }
    }
}

// ── A manual override CAN exceed Eq. 1 — the library applies it and does not block ──
void test_manual_cf_can_exceed_eq1() {
    INCUNEST_AFE4490 afe;
    afe.setSampleRate(500);
    afe.setTIAGain(AFE4490RF::RF_250K);       // Eq. 1 limit ≈ 179.8 pF
    afe.setTIACF(250.0f);                      // deliberately above it
    AFE4490Config cfg = afe.getConfig();
    TEST_ASSERT_EQUAL_FLOAT(250.0f, cfg.afe_tia_cf_led1_pF);          // applied, not clamped
    TEST_ASSERT_TRUE(cfg.afe_tia_cf_led1_pF > afe.getCFMaxEq1PF(AFE4490RF::RF_250K));
}

// ── Eq. 1 limit scales with the sample window: halve the rate → roughly double the window ──
void test_eq1_limit_scales_with_rate() {
    INCUNEST_AFE4490 fast, slow;
    fast.setSampleRate(1000);
    slow.setSampleRate(500);
    TEST_ASSERT_TRUE(slow.getCFMaxEq1PF(AFE4490RF::RF_100K) >
                     fast.getCFMaxEq1PF(AFE4490RF::RF_100K));
}

// ── The RF-change settle window (datasheet t5) covers the 500 Hz post-stage2 filter's own 5tau ──
// Reviewed 2026-08-21 (spec S5.8.4 "500 Hz post-stage2 filter pole vs. the settle window"): the
// FLTRCNRSEL low-pass (corner always 500 Hz in this library) settles in 5/(2*pi*500) ~= 1.59 ms,
// independent of RF/CF. At the only rate this project actually configures (500 Hz default), the
// t5-derived settle window must dominate that filter pole with margin, or a gain change could
// re-enable HGAC/RSQM before the analog front end has actually settled.
void test_afe_settle_window_covers_500hz_filter_pole() {
    INCUNEST_AFE4490 afe;                       // 500 Hz default
    uint32_t samples = afe.test_compute_switched_rc_settling_samples();
    float settle_s = (float)samples / 500.0f;
    const float five_tau_500hz_s = 5.0f / (2.0f * 3.14159265f * 500.0f);  // ~1.59 ms
    TEST_ASSERT_TRUE(settle_s >= five_tau_500hz_s);
    // Not just barely — expect at least an order of magnitude of margin at the default rate
    // (measured ~10x on 2026-08-21); a future change eroding this to <2x would be worth a look.
    TEST_ASSERT_TRUE(settle_s >= 2.0f * five_tau_500hz_s);
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_cf_table_matches_datasheet_weights);
    RUN_TEST(test_cf_pf_to_code_quantises_down);
    RUN_TEST(test_cf_code_pf_round_trip);
    RUN_TEST(test_cf_string_codec_round_trip);
    RUN_TEST(test_cf_legacy_strings_still_valid);
    RUN_TEST(test_autocf_reaches_exact_grid_value_at_default);
    RUN_TEST(test_autocf_respects_settling_budget);
    RUN_TEST(test_autocf_grows_when_rate_drops);
    RUN_TEST(test_set_tiacf_overrides_and_quantises);
    RUN_TEST(test_eq1_limit_matches_sample_window);
    RUN_TEST(test_autocf_always_within_eq1);
    RUN_TEST(test_manual_cf_can_exceed_eq1);
    RUN_TEST(test_eq1_limit_scales_with_rate);
    RUN_TEST(test_afe_settle_window_covers_500hz_filter_pole);
    return UNITY_END();
}
