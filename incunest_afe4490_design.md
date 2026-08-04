# incunest_afe4490 — Design Rationale (PRELIMINARY)

> **Status: PRELIMINARY / work-in-progress snapshot (2026-08-04).**
> This document captures the **WHY** — the analysis, debates, and decisions behind the library.
> The normative **WHAT** lives in [`incunest_afe4490_spec.md`](incunest_afe4490_spec.md), which is
> the distilled *result* of this rationale.
>
> This is an initial dump to preserve design context; it will be consolidated and edited at the
> **HGAC v1 freeze** milestone. Sections vary in polish. The HGAC section (developed most recently)
> is the most complete; the rest are summaries with pointers to the spec.

---

## 1. Scope & relationship to the spec

- **`incunest_afe4490_design.md` (this file)** = process / rationale / trade-offs / discarded options.
- **`incunest_afe4490_spec.md`** = normative result (register maps, formulas, algorithm as built).

The design doc is the *delta* over the spec: it explains decisions the spec states without justifying.
For a medical device the rationale (and clinical basis) is worth keeping traceable and versioned.

---

## 2. HGAC — Hardware Gain Control

HGAC keeps the PPG operating point in a usable range by adjusting hardware gain. It lives **inside**
the library (per-sample access), gated so it acts only with a patient present.

### 2.1 Chain & actuators

`V_TIA ≈ ILED × RF × tissue_transmittance` (in v1, proportional — AMBDAC=0, Stage 2 ×1).
Two actuators, both gains, very different costs:
- **RF** — TIA transimpedance. Discrete LUT (10 kΩ…1 MΩ, 7 levels, steps ×2–2.5). Cheap: no power,
  heat, or patient-exposure cost. **The primary (and, in v1, only) control knob.**
- **ILED** — LED drive current (10–50 mA/channel). Costs power + skin heating + patient light
  exposure. Bounded by a **thermal envelope** (§4.1). In v1 **pinned at its thermal maximum** as a
  policy constant, not a control knob (see 2.2).

Two independent loops: IR (LED1) and RED (LED2). Each color shares one RF between its LED and ambient
(ALED) phases (hardware: only RF_LED1/RF_LED2 exist; subtraction LED−ALED requires equal gain).
Recommended `ENSEPGAIN=1` (independent RF per color) — R is invariant to per-color RF/ILED in steady
state, so it does not corrupt SpO2; `ENSEPGAIN=0` penalizes the weaker color's SNR.

### 2.2 Why v1 is RF-only (ILED is a weak anti-saturation actuator)

In the LED phase `v_tia_led = 2·RF·(i_amb + i_led)`, with `i_led = OT·ILED`; only `i_led` scales with
ILED (ambient does not). At the RF floor (10 kΩ) the maximum voltage lowering ILED can shed is:

```
Δv = 2·RF·OT·(ILED_max − ILED_min) = 2·10⁴·OT·(0.050 − 0.010) = 800·OT   [V]
```

For an **applied neonatal probe** `OT ≈ 1e-5…1e-4` → **Δv ≈ 0.01…0.08 V**, negligible vs the ±1.2 V
ADC rail. Physically: at RF=10 kΩ the LED contribution to `v_tia` is itself tiny (~0.1 V); if a
channel saturates there, the bulk (~1.1 V) is **ambient**, which ILED cannot touch. So lowering ILED
is the wrong tool at the floor. → **v1 controls RF only; ILED stays at its thermal max.** ILED as an
actuator is deferred to **Phase 4** and is only meaningful in the rare LED-dominated corner
(high OT, very translucent tissue, ALED not saturating).

Priority rule (for when ILED does actuate, Phase 4+): **lower signal → RF first; raise signal → ILED
first** (mirror). The √-noise law (SNR ∝ √i_pd_led) makes ILED the last to drop and first to restore,
so it naturally stays high.

### 2.3 Conceptual model — a single RF axis with graded urgency

Not "two layers": the saturation guard is the **top (most urgent) rung of the same descent**.

Thresholds on `V_TIA` (per color): `LOW 0.25` · `IDEAL 0.6` · `HIGH 0.75` · `GUARD 0.9`
(the physical ceiling `adc::FSR` is 1.2 V; `tia_axis::FS_V` 1.0 V is the TIA linearity guarantee).

```
V (sustained):
  ≥ 0.90  GUARD :  RF--   descend, faster the higher it is (short persistence, never instantaneous)
  ≥ 0.75  HIGH  :  RF--   descend, proactive (~500 ms; keeps headroom below the guard)
  0.25 – 0.75   :   —     dead band (no action)
  < 0.25  LOW   :  RF++   ascend, patient (~5×τ)
  RF at floor & still high  → alarm "excessive ambient light"   (do NOT lower ILED; see 2.2)
  RF at ceiling & still low → weak signal                        (no more gain available in v1)
```

Implemented today: only the descent (TIAGUARD, Phase 1). Ascent + dead band + alarm are Phase 2+.

### 2.4 The leveling band & why the guard is not redundant with it

The band `[LOW, HIGH]` is the operating-point target; the guard (0.9) is the ceiling above it. They
differ by **signal** (peak vs slow DC), **threshold** (0.9 vs 0.75), **direction** (down-only vs
bidirectional) and **speed** (urgent vs patient). The leveling keeps headroom so the guard rarely
fires; the guard catches fast transients the slow DC loop misses. In PPG the AC is ~1 % of DC
(DC ≈ peak), so the real difference is the **threshold and role**, not the smoothing.

### 2.5 Stability invariants

1. **Band ≥ one RF step:** `HIGH/LOW > largest_RF_step` (2.5×). With `[0.25, 0.75]` the ratio is 3.0.
   Otherwise a single step overshoots the opposite edge → hunting. Thresholds are currently
   eyeballed; tuning is pending (see 2.8) but this invariant is **hard** and must survive it.
2. **Asymmetric urgency:** descend fast (data at risk) / ascend slowly (~5×τ, not urgent) + hysteresis.
   **One-step ascent needs no prediction** — invariant 1 already prevents overshoot
   (`V_dc ≤ LOW → ×2.5 ≤ 0.625 < HIGH`). Prediction (`V(RF₊₁)=i_pd×RF₊₁ ≤ HIGH`) is only needed for
   multi-step jumps (feed-forward), which is an optional optimization. Urgent descent may use a large
   computed jump (a separate mechanism).

### 2.6 Descent ladder & the ambient alarm (step 3)

1. Start with **ILED = thermal max** and a typical RF.
2. Saturation → **lower RF** until it fits.
3. **RF at floor and saturation persists** → discriminate via the ALED phase:
   - **3.a — ALED saturated:** saturation is **ambient** → **alarm "excessive ambient light"**
     (the normal case).
   - **3.b — ALED far from saturation:** LED-dominated → lowering ILED *might* help, **but this is
     believed not to occur** in practice (per 2.2) → **not implemented for now**; ILED stays at max.

Landing rule: land **one RF step below** the largest that fits (headroom); with ×2–3 steps this puts
the DC near ~50 % FS as a *consequence*, not a chased setpoint.

### 2.7 What governs actuation

- **Sustained + patient-present only.** Every RF change requires (a) temporal **persistence** and
  (b) **enable: probe active** (`PROBE_APPLIED`/`SATURATING`; disabled in NOT_APPLIED/DISCONNECTED so
  HGAC does not chase a no-patient optical event, e.g. a phototherapy lamp). After each gain change,
  wait for the front-end to re-settle (~150 ms); if persistence ≥ settling, that wait is automatic.
- **Detection ≠ actuation.** A transient clip is flagged by the validity masks (`adc_sat` → RSQM
  quality drop), **not** corrected by moving RF.
- **KNOWN GAP (2026-08-04):** an anti-**motion** protection by *invalid-sample fraction* is in the
  design intent but **not in the code** — today the gate only checks probe-active + not-settling, so a
  sustained motion (> persistence) with the probe applied *would* move RF; only persistence damps it.
  To decide whether to implement.

### 2.8 Open points / pending

- **Thresholds HIGH/LOW eyeballed** (0.75 / 0.25) → tune with an optimization criterion (likely
  SNR vs headroom), preserving the ratio invariant.
- **0.9 rung:** keep GUARD (0.9) as a separate shorter-persistence descent, or use a single descent at
  0.75 (with 0.9/FS as the physical ceiling only)? Undecided.
- **Anti-motion invalid-fraction gate** (2.7) — implement or not.
- **`RSQM_DIAG_HW_SETTLING`:** it mirrors `_rsqm_settling_countdown` and is a self-imposed control
  state, not a diagnostic → candidate to read the countdown directly and drop the DiagCode bit
  (decision deferred).

### 2.9 Deferred (Phase 4+)

ILED as an actuator (computed decrements/increments; the LED-dominated corner), the AMBDAC off-spec
zone (accessing the 1.0–1.8 V TIA band by shifting the ADC window), and multi-step feed-forward.

---

## 3. Analog reconstruction & RSQM (summary — see spec §3, §5)

- **Single source of truth for thresholds:** `namespace tia_axis` (the ordered `v_tia` thresholds
  IDEAL/GUARD/FS/LIN, with a compile-time ascending `static_assert`) and `namespace adc` (ADC scale +
  code rails). No per-struct aliases (deliberately removed): the 1.2 V full-scale lives once in
  `adc::FSR`. ADC rails keep **NOMINAL** (datasheet 2^21) vs **ACTUAL** (empirically measured on
  IncuNest 16.A) side by side; the operative saturation flag is the guarded *actual* rail
  (`SAT_POS = POS_RAIL_ACTUAL − SAT_GUARD`), not the nominal FS.
- **Value-always policy:** every field is always computed from raw codes; per-channel validity masks
  (one bit per failure mode) tell the consumer whether a value is a *measurement* or a *bound*.
  Consumers must use the helper accessors, never the raw masks (packing may change).
- **ProbeState** (DISCONNECTED / NOT_APPLIED / SATURATING / APPLIED): SATURATING is a distinct state
  so a TIAGUARD event can't flip to NOT_APPLIED and deadlock HGAC's step-down.
- **Naming:** `v_tia` (bare) is always the DIFFERENTIAL TIA output; the `_diff` suffix was removed
  (no branch value exists in the code). See `incunest_afe4490_spec.md` and the nomenclature notes.

---

## 4. Clinical & hardware basis (medical-device rationale — MOST CRITICAL to preserve)

### 4.1 ILED thermal envelope
The limited quantity is **joint mean optical power** ∝ (ILED1+ILED2)×duty. Normative basis
ISO 80601-2-61:2026 (≤ 41 °C skin, Annex BB); worst case = incubator 37 °C + poor perfusion (the
IncuNest scenario). Mechanism (library hard cap `afe_iled_max_mA` per probe) vs policy (integrator).
Open-loop (no probe temperature sensor) → conservative caps.

### 4.2 Probe-dependent specs
Medle probe: I_F max 50 mA @25 °C (peak; incubator derating pending full datasheet). Defaults are
worst-case neonatal. LED current raised 20→50 mA preliminarily (v0.47): the real limit is
skin-thermal + TIA saturation, not the LED.

### 4.3 Preterm foot dimensions
Optical thickness ~5–10 mm (critical, **not published** in literature) — drives OT expectations and
the thermal/optical budget. See the source notes.

### 4.4 AFE selection
No modern AFE is a drop-in replacement (none has the H-bridge); best candidate for a redesign is
AFE4900 + external H-bridge. Protocentral rejected as a validation reference.

---

## 5. Nomenclature & conventions (pointer)

Naming rules, domain prefixes, ADC `adc_code`/`LSB` conventions, and the code↔ISO 80601-2-61↔AFE4490
term mapping (led1/led2 vs ir/red, etc.) are captured in the project's nomenclature notes; to be
folded here at consolidation.

---

## Appendix — source rationale (to distill at consolidation)

This preliminary doc is distilled from working notes (Claude local memory), primarily: the HGAC design
(`project_agc_design`, `project_agc_design_history`), closed decisions (ENSEPGAIN/O10, `namespace adc`,
value-always analog state, ProbeState), the clinical/HW basis (thermal envelope, probe specs, preterm
foot dimensions, AFE selection), and the nomenclature dictionary. Full consolidation (edit to
documentation tone, dedupe, drop task cruft) is planned for the HGAC v1 freeze.
