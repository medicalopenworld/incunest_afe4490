# incunest_afe4490 — Design Rationale

## 0. Why this document exists

This library takes decisions a patient depends on: when a pulse counts as present, how far the
SpO₂ value lags reality, how hot a LED may run on a preterm foot, when a signal is declared
unusable. Each decision ends up as a constant or a structure in the code and as a sentence in the
spec. **The reason for it ends up in neither** — and a reason that is not written down expires
silently. §5.1 is the canonical example: HR1's filter was chosen for a premise that stopped being
true in v0.6 and nobody noticed for ~80 versions, because the premise lived only in a conversation.
§5.4 is the mirror case: two analyses recommended changes to a measuring algorithm and both had to
be reversed within two days, once the whole chain was measured instead of one link.

So this document keeps, for every non-obvious choice:

1. **the evidence** — labelled as *measured* (our bench or captures), *literature* (with citation),
   *standard* (ISO 80601-2-61, WHO/UNICEF, datasheet) or *inference*, so a reader can tell how much
   to trust it;
2. **the alternatives** that were considered and why they were rejected;
3. **the reversals** — a decision that was undone is marked SUPERSEDED and kept, never deleted,
   because the mistake is as instructive as the fix.

It is written for two readers: whoever is about to change a constant (including ourselves, later),
who should find here *why* it has its value and what would have to be true for a different value to
be right; and whoever has to assess the device clinically or against a standard, for whom ISO
80601-2-61 expects exactly this kind of traceable rationale behind accuracy claims and disclosed
processing. Rules of use: write the entry **when the decision is taken**, not at consolidation;
one source per claim; the spec keeps the short "why" next to each number (§1), this document keeps
the long one.

> **Status: work in progress, not consolidated (last reviewed 2026-10-01).**
> Sections vary in polish and the document is still growing. It will be consolidated and edited at
> the **HGAC v1 freeze** milestone, which has not been reached: `hgac_enable` is still `false` by
> default, phase 1 is done and phases 2-7 are pending.
>
> Maturity per section, so a reader knows what to trust:
>
> | Section | State |
> |---|---|
> | §2 HGAC | most complete; developed alongside the implementation |
> | §3 Analog reconstruction & RSQM | summary only, with pointers to the spec |
> | §4 Clinical & hardware basis | the most critical to preserve; sourced |
> | §4.5 Averaging times — evidence base | complete, every claim sourced (2026-10-01) |
> | §5 HR1 filter & detector | complete, every claim sourced (2026-09-04) |
> | §5.4 SQI gate & the two reversals | complete, measured (2026-09-06) |
> | §6 Sample-rate dependency audit | complete; the tests that replace it are in place |
> | §12 Averaging: EMA → sliding window for R/SpO2 | complete, measured on HOSPNAV (2026-10-06/07) |
>
> ("PRELIMINARY" used to be in the title. It was moved here in favour of a status that says what is
> actually unfinished: a title that reads like a throwaway draft discourages the very thing this
> document needs, which is to be used and kept current.)

---

## 1. Scope & relationship to the spec

- **`incunest_afe4490_design_rationale.md` (this file)** = process / rationale / trade-offs / discarded options.
- **`incunest_afe4490_spec.md`** = normative result (register maps, formulas, algorithm as built).

For a medical device the rationale (and clinical basis) is worth keeping traceable and versioned.

**Where the boundary actually falls.** The split is not "all rationale here": the spec carries
**short justification inline, next to the constant it explains** — the reason a threshold is 0.6 or
a rate is 50 Hz belongs beside the number, because the spec has to be able to regenerate the code
*with judgement*, and stripping the reasons out would leave orphan magic numbers. What lives here
instead is what does not fit that: **long analyses, the alternatives that were rejected and why,
the clinical basis, and the history of decisions that were later reversed**.

So both documents contain "why", at different lengths and for different purposes. Examples of the
line in practice: spec §5.3.1 justifies the 50 Hz decimated rate where it is defined, while §5 of
this document holds the ten-strategy detector catalogue with sources — a comparison of options,
most of them not implemented, which has no place in a normative document.

*(This paragraph was added on 2026-09-04, after noticing that §7.3, §7.4 and §5.3.1 of the spec had
accumulated exactly the kind of reasoning §1 claimed belonged only here. The rule was rewritten to
match what the documents actually do, rather than moving the text and leaving unexplained numbers
behind.)*

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

Thresholds on `V_TIA` (per color): `LOW 0.25` · `HIGH 0.75` · `GUARD 0.9`
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
- **`RSQM_DIAG_SWITCHED_RC_SETTLING`:** it mirrors `_switched_rc_settling_countdown` and is a
  self-imposed control state, not a diagnostic → candidate to read the countdown directly and drop
  the DiagCode bit (decision deferred). (Renamed from `*_HW_SETTLING`/`_rsqm_*` v0.75 — see spec
  §5.8.4: "settling" alone was ambiguous among three unrelated phenomena in this library.)

### 2.9 Deferred (Phase 4+)

ILED as an actuator (computed decrements/increments; the LED-dominated corner), the AMBDAC off-spec
zone (accessing the 1.0–1.8 V TIA band by shifting the ADC window), and multi-step feed-forward.

---

## 3. Analog reconstruction & RSQM (summary — see spec §3, §5)

- **Single source of truth for thresholds:** `namespace tia_axis` (the ordered `v_tia` thresholds
  GUARD/FS/LIN, with a compile-time ascending `static_assert`) and `namespace adc` (ADC scale +
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

### 4.5 Averaging times for AC, DC, %mod/PI, R and SpO₂ — evidence base (2026-10-01)

What the library does today: `spo2_ema_mean_tau_s` = 2 s (DC), `spo2_ema_var_tau_s` = 6 s (AC²,
hence PI and R), `spo2_warmup_s` = 18 s (3·τ). The 6 s was taken from ISO Annex JJ. This section
records what the standards and the literature actually say, so that value can be re-decided with
the trade-off in view. Verified in the ISO 80601-2-61:2026 PDF unless marked otherwise.

**Standards — no averaging time is prescribed for the device itself.**

| Source | Statement | Figure |
|---|---|---|
| ISO 201.7.9.2.1.101 c), Table GG.3 | The IFU shall describe data averaging, *data update period*, *alarm condition delay* and *alarm signal generation delay*; "averaging time" is an operator setting | disclose, not fix |
| ISO 201.12.4.101 + AA.2.13 | Indication required when the *data update period* exceeds **30 s**; it "should be shorter than 30 s for continuous **neonatal** monitoring and diagnostic applications". Rationale: the delay between an event and the alarm | 30 s |
| ISO Annex JJ.2 d)–e) (informative) | A *transfer standard* (reference oximeter for development) shall average SpO₂ over **6–10 s** and output SpO₂, PI and PR at 1 Hz | 6–10 s |
| ISO Annex FF | Fidelity vs delay: δ (lag) and Δ (under-read nadir). Faster averaging tracks the real desaturation; slower averaging smooths noise and cuts false alarms (Figs. FF.2, FF.4). Cites Lynn & Curry 2011 [94] | qualitative |
| ISO Annex DD.2.3.2 | Desaturation study: ≥ 60 s after an FiO₂ change, ≥ 30 s plateau stabilisation, consecutive samples ≥ 20 s apart "considering the averaging time of the equipment" | 20–60 s |
| ISO Annex II.2.1; 201.12.4.102 | Typical %mod 1–8 %; signal inadequacy must use DC and %mod **per wavelength** | — |
| WHO/UNICEF 2019, Annex 1 A1.11 (cat. 6.5) | Valid data update | ≤ 10 s |
| AAP / BAPM / NICE / AARC / ILCOR 2010 | No guideline prescribes a window | — |

**Manufacturers (operator manuals / 510(k)).**

| Equipment | Averaging | Source |
|---|---|---|
| Masimo Rad-57 / Radical-7 | 2/4/8/10/12/14/16 s selectable, **default 8 s**; PI = AC/DC of IR; PI window not published | LAB-9204A |
| Nellcor N-600x / PM1000N | Normal **5–7 s**, Fast 2–4 s; extended dynamically above 7 s in low perfusion; > 20 s → "pulse search"; 1 s update; cardiac ensemble averaging (US 7039538) | Operator's Manual |
| Nonin OEM III / PureSAT | Per **beat**: 4-beat exponential (8 extended), 2-beat latency; "≤ 3 s response at ≥ 60 bpm, adult/paediatric/**neonate**" — the only explicit neonatal figure from a manufacturer | OEM III spec |
| Philips FAST-SpO₂ / picoSAT II | 5–20 s declared in 510(k) K120366; defaults *not confirmed* | FDA |
| Nihon Kohden / Mindray / GE | Fast/Normal/Slow, High/Med/Low; seconds *not confirmed* | manuals |
| Renesas OB1203 reference algorithm | 4 s window (512 @ 128 Hz) for AC/DC and R, 2 Hz update, 5-point moving average on SpO₂ and PI (≈ 6.5 s effective) | AN OB1203 |

**Literature — general.** Vagedes, Sobh, Al Islam, Poets 2024, *Nat Sci Sleep* 16:1131
(10.2147/NSS.S460231): only 6 % of 151 papers state the averaging time; the most used are 3 s
(33 %) and 2 s (22 %), range beat-to-beat…10 s. Webster (ed.) 1997, *Design of Pulse Oximeters*:
R is computable beat-to-beat and stabilised with a weighted moving average. Baquero 2011, *Acta
Paediatr* 100:515: at maximum sensitivity + 2 s, time to a stable reading on newborns — Radical-7
20 s, Biox 3700 74 s, Nellcor N395 67 s.

**Literature — neonates and preterms.**

| Study | Population | Windows | Finding |
|---|---|---|---|
| Ahmed, Rich, Finer 2010, *Pediatrics* 125:e115 (10.1542/peds.2008-1749) | preterm < 32 wk, respiratory support | 2 vs 16 s | desaturations < 30 s: **41 vs 16**; events > 300 s: 20 vs 40 (merging); nadirs < 70 % under-read at 16 s (p = 0.01) |
| Vagedes, Poets, Dietz 2013, *ADC F&N* 98:F265 (10.1136/archdischild-2012-302543) | 15 preterms, 168 h | 3 vs 16 s | desaturations < 80 %: **1958 vs 339**; deeper nadirs at 3 s, longer durations at 16 s |
| Vagedes et al. 2014, *PLoS One* (10.1371/journal.pone.0087280) | 12 preterms 24–27 wk, 170 h | raw red/IR reprocessed at 3/5/8/10/12/14/16 s | conversion **D₂ = D₁·(T₂/T₁)^c**; ≈ ×6 desaturations < 80 % at 3 s vs 16 s |
| Vagedes, Dietz, Poets 2019, *Acta Paediatr* 108:2246 | 49 children + 15 preterms | 3 → 16 s | count ×0.18, mean duration ×3.34; effect strongest in infants |
| McClure, Jang, Fairchild 2016, *J Neonatal Perinatal Med* (10.3233/NPM-16162) | 10 VLBW | 2 / 8 / 16 s | 2→16 s: alarms −53 % but duration ×2.15; **2 s + 15 s alarm delay: −67 %**. "NICU default is 8 s" |
| Janota 2023, *Front Pediatr* (10.3389/fped.2023.1240363) | 10 VLBW, closed-loop FiO₂ | 2–4 / 10 / 16 s | time in range equal (67 %); SpO₂ < 80 %: 6.5 / 5.3 / 4.5 % (p = 0.003) |
| NeOProM trials (Di Fiore 2012; Di Fiore & Martin) | SUPPORT, COT | — | SUPPORT **16 s**, sampled every 10 s (two centres at 2 s); COT 16 s; BOOST *not confirmed* |
| Martin 2023, *Acta Paediatr* (10.1111/apa.16766) | 20 extremely preterm | — | staff response median 20.5 s; SpO₂ falls **3.7 % per 10 s** of delay — monitor latency sits in the same chain |
| Delivery-room resuscitation (Baquero 2011; NCCC guideline) | newborns | — | maximum sensitivity + 2 s |
| Kemper 2011, *Pediatrics* 128:e1259 (CCHD screening) | — | — | no mention of averaging |

**De facto consensus:** 8 s in the NICU (Masimo default), 2–4 s in resuscitation and
intermittent-hypoxaemia research, 16 s in the oxygen-target trials. No manufacturer publishes the
PI window.

**What this means for the 6 s.** It is the *minimum* of an informative range written for a
reference instrument, not a clinical optimum; it coincides with Nellcor "Normal" and sits between
the 2–4 s of resuscitation/IH work and the 8 s NICU default. The cost of 6 s against 2–3 s is
quantified above (Vagedes 2014: fewer short desaturations counted by ≈ (6/3)^c). For IncuNest's
purpose — detecting apnoea-driven desaturation in an incubator — the neonatal literature points to
**short window + alarm delay** (McClure 2016), not to a longer window. The ISO obligations we do
meet by construction: %mod per wavelength (PI_red, PI_ir) and a declared window and update period.
Open: whether `spo2_ema_var_tau_s` should move towards 3–4 s once an alarm-delay layer exists in
the integrator (that layer is the integrator's, not the library's).

---

## 5. HR1 — the filter and the detector

Analysis 2026-09-04. **Nothing here is implemented yet** except where stated; the spec describes
what HR1 does today (§5.2). Each claim carries its **source**, so a reader can tell measurement
from literature from inference.

### 5.1 Why HR1 low-passes with a moving average (it is inherited, and the reason expired)

| Finding | Source |
|---|---|
| The MA entered as *"ring buffer de 8 muestras (simple, no necesita f_low/f_high)"*, while the biquad had *"coeficientes hardcodeados (mismos que Protocentral)"* and a pending TODO for dynamic coefficients | `conversation_log.md:276-277` |
| So the real premise was that **the biquad could not compute its own coefficients** — it only worked at 500 Hz / 0.5–20 Hz. The MA was the only filter that followed the sample rate | inference from the two lines above |
| When HR1 got its own filter, the decision being taken was **decoupling it from the display filter** (changing `f_high` to look at the signal altered HR), not choosing a filter type. The MA was justified by *"se adapta automáticamente si cambia `_sample_rate_hz`"* | `conversation_log.md:745-755` |
| The premise expired in **v0.6**: *"Coeficientes biquad dinámicos… `setFilter()` completamente funcional para cualquier fs/f_low/f_high"*. Nobody revisited HR1 in the ~80 versions since | `conversation_log.md:430` |
| A later review inventoried the filters and **noted the asymmetry without acting** (HR1 with MA "sin biquad"; HR2/HR3 with Butterworth biquads) | `conversation_log.md:5644` |
| Neither the log nor the code ever mentions linear phase or fixed cost per sample — the two arguments that *would* justify an MA today. They are real but accidental | search of log + code |

### 5.2 Measured: MA vs biquad (own measurement, `tools/hr1_filter_experiment.py`)

Method: both arms share the DC removal and the detector, so any difference is the filter's; the
biquad uses the same algebra as `BiquadFilter::init_lp()`. Two method corrections were decisive —
a 5 s warmup for the running max, and **pairing beats** instead of comparing each arm's RR spread
(an SD of 25-29 ms on a resting adult finger is physiological HRV, identical in both arms, and it
swamps the filter's contribution).

| Result | Value | Source |
|---|---|---|
| Differential jitter, clean signal (4 captures, 3 subjects) | **SD 1.0-3.7 ms** ~ 0.2 BPM at 60 BPM | own measurement |
| Reference scale: those subjects' own HRV | SD 25-29 ms ~ 1.5 BPM | own measurement |
| Reference scale: ISO 80601-2-61 HR accuracy | ±3 BPM | ISO 80601-2-61 |
| Differential group delay | −0.9…−2.4 ms, systematic, does not affect RR | own measurement |
| Under phototherapy interference, true rate 60 BPM | MA **75.5 BPM (+26 %)**, biquad **66.1 (+10 %)** | own measurement |
| Why: MA first sidelobe at −13 dB decaying 6 dB/octave, vs 40 dB/decade monotonic for a 2nd-order Butterworth | standard DSP result |
| Collateral: a biquad has no rate-dependent length, so it removes the `hr1_ma_max_len` clamp (v0.86b) and returns 640 B | inference |
| Nominal cutoff is not the real one: `N = fs/(2·fc)` puts −3 dB at `fs/2N`, but an N-sample MA is −3 dB at ≈ `0.443·fs/N` (so "5 Hz" is ~4.4 Hz) | standard DSP result |

**Conclusion (SUPERSEDED 2026-09-06 — see §5.4).** This read "no argument survives for keeping the
MA", on the strength of the phototherapy figures above. Those were measured on the RAW detector
output, without HR1's SQI gate. With the gate applied — which is what the firmware actually
publishes — the difference disappears: both filters report 60.0–60.4 BPM. The comparison below
stands as a measurement of the filter in isolation; it no longer supports a change.
**Caveat:** the captures are adults and a simulator, **not neonates**, whose dicrotic notch is
sharper and closer to the systolic peak — precisely the condition that decides double-counting.

### 5.3 The detector is the weaker link

The interference figures above are the argument: moving from +26 % to +10 % error is still a
failing HR monitor. What fails there is the **detector** — a rising-edge crossing of 0.6 × a
decaying running max, with a fixed refractory and no waveform validation. A better filter only
masks it.

#### Strategy catalogue

| Strategy | How it works | Cost in Task A | Robustness to interference | Demands on the filter | Source |
|---|---|---|---|---|---|
| **Adaptive threshold** (current) | 0.6 × decaying running max + fixed refractory | ~0 | **Poor** (+26 % measured) | aggressive low-pass | as built, spec §5.2 |
| **TERMA** | Two event-related moving averages — ~111 ms (peak) and ~667 ms (beat) — whose crossing generates blocks of interest; the peak is sought inside the block. The published design rule `2·W1 <= W2 <= 8·W1` is what separates the systolic peak from the dicrotic notch. Reported 99.89 % sensitivity / 99.84 % PPV over 5 071 beats, **on heat-stressed PPG** | two incremental MAs, fixed cost per sample | **Very good** | soft low-pass + squaring | Elgendi, PLOS One 2013 (10.1371/journal.pone.0076585); Biosensors 2016 (10.3390/bios6040055) |
| **SSF (slope sum function)** | Slope sum over a window enhances the upslope and suppresses the rest: low-pass + SSF + decision rule | O(W), W ~ 100 ms | Good | soft low-pass | Zong et al., open-source ABP onset algorithm |
| **First derivative / local extrema** | Zero crossing of dy with the sign of d²y; variants cube the derivative to highlight search zones | ~0 | Medium | low cutoff (the derivative amplifies HF noise) | Billauer; Li; FDPPG variants |
| **`y × dy` delayed** | Product amplifies edges and suppresses flat regions | ~0 | Medium | low cutoff | own idea — `project_hr1_improvements_task` option A |
| **Template matching** | Cross-correlation with an expected pulse shape | O(L), L ~ pulse length | Good at low SNR | **linear phase** (distortion misaligns the template) | own idea — option B |
| **Hilbert / envelope (WEPD)** | Analytic envelope locates the pulse; designed to cut both false positives and negatives | High (FFT or long filter) | Good | indifferent | Hilbert-transform PPG onset literature |
| **Waveform validation** *(complement)* | Accept a candidate only if width, area and rise/fall ratio are plausible | ~0 | Complementary | indifferent | own idea |
| **Adaptive refractory** *(complement)* | Refractory = f(estimated HR) instead of a fixed 0.185 s | ~0 | Complementary | indifferent | own idea |
| **Cross-check against HR2** *(complement)* | Accept an interval only if it agrees with the autocorrelation period already computed for HR2 | ~0 (already computed) | **Complementary, strong** | indifferent | own idea — specific to this library |

#### Recommendation

**TERMA**, with **SSF** as the simpler fallback. Reasons: fixed cost per sample (two incremental averages, like
the one already there); the strongest external validation among the candidates, and obtained on
*heat-stressed* signals — a poor-quality regime comparable to ours; and it attacks the actual cause,
since the ~667 ms average sets the threshold from the **local beat level** instead of from a global
decaying maximum that interference inflates.

Two complements are worth adding regardless, both ~free: **cross-check against HR2** (a doubled HR1
is exactly what disagrees with the autocorrelation period — this one is ours, not from the
literature, because no published detector has a second estimator sitting next to it) and **adaptive
refractory** (0.185 s covers up to 263 BPM, but in bradycardia it leaves the notch unprotected,
which is where the measurement shows the failure).

**Note on moving averages.** TERMA *reintroduces* them — as event detectors over the squared signal,
not as pre-threshold smoothing. The MA is not wrong as a tool; it was wrong in that role. So
replacing HR1's low-pass with a biquad stays correct under TERMA, SSF and the derivative variants;
only template matching would prefer the MA, and it is the weakest candidate on cost and on the
difficulty of defining a neonatal template.

**Pending before implementing a detector:** an offline comparison (TERMA vs SSF vs current) over
the recorded captures, and ideally neonatal captures — same approach as §5.2.

---

### 5.4 What the SQI gate changes — and why two analyses had to be reversed

Measured 2026-09-06, after the detector comparison revealed the same effect.

HR1 does not publish the rate of every beat it detects. Over a sliding window of five intervals it
computes their coefficient of variation, and while that exceeds `hr1_sqi_cv_max` = 0.15 the value
is marked invalid. It does not repair a bad detection: it **discards the stretch containing it**,
paying in availability.

That gate absorbs errors of precisely the size that both earlier analyses were built on:

| Capture (true 60 BPM) | MA raw | biquad raw | **MA gated** | **biquad gated** | kept, MA | kept, biquad |
|---|---|---|---|---|---|---|
| PHOTOTHERAPY_500HZ | 75.5 (**+15.5**) | 66.1 (+6.1) | **60.4** | **60.3** | 24 % | **45 %** |
| PHOTOTHERAPY_ONOFF | 62.2 | 62.6 | **60.0** | **60.2** | **56 %** | 48 % |
| PHOTOTHERAPY_400HZ | 59.8 | 59.9 | **60.0** | **60.0** | 70 % | 70 % |

**Both reversals have the same cause.** §5.2 concluded the moving average was the worse filter, and
§5.3 recommended replacing the detector with TERMA. Both rested on raw-output differences of 3 to
15 BPM that the gate removes entirely. Measured over the whole chain — the way the device actually
reports — neither change is justified:

* **Filter:** with the gate, MA and biquad publish within 0.4 BPM of each other and of the truth.
  What remains in the biquad's favour is structural, not performance: no rate-dependent length
  (removing the class of defect that `hr1_ma_max_len` cost 384 B to fix in v0.86b), 640 B returned,
  and consistency with HR2/HR3. Weighed against revalidating a measuring algorithm in a medical
  device for no measurable gain, that does not carry it.
* **Detector:** all three are accurate across the whole declared range on the MS100 series
  (≤ 0.2 BPM, 40–250 BPM), so there is nothing to gain there either.

**The methodological lesson, which is the durable part:** measuring one link in isolation produces
false recommendations when a quality gate downstream absorbs exactly that class of error. It
happened twice in two days, in both directions, and would have led to two changes to a medical
device that improve nothing a patient would see.

**What the numbers do expose is availability, not accuracy.** Under phototherapy only 24–70 % of
intervals survive the gate: the monitor is right when it speaks and can stay silent for three
intervals out of four. That is the real deficiency, it belongs to the gate's own criterion
(`hr1_sqi_cv_max` = 0.15, one of the parameters with no documented provenance), and neither a
different filter nor a different detector addresses it.

---

## 6. Sample-rate dependency audit (2026-09-04)

Alex's concern after v0.86b: *"me preocupa encontrar más dependencias del sampling rate del AFE sin
resolver"*. A parameter whose meaning is a **duration** but whose value is stored in **samples** —
or as a **per-sample factor** — silently changes meaning when the rate changes. Four fronts were
swept: parameters in samples, per-sample coefficients, the defaults namespace, and everything
touching `_afe_sample_rate_hz`.

| Item | Verdict | Source |
|---|---|---|
| `_hr1_running_max *= 0.9999f` | ⚠️ **live defect** — per-sample literal, tau 10 000 samples = 20 s at 500 Hz but **6.25 s at 1600 Hz**; relaxed HR1's detection threshold 3.2× faster at the top of the catalogue. Fixed in v0.87 (`hr1_max_decay_tau_s`) | own audit |
| `ts_emit_interval = 2500` samples | ⚠️ live defect (telemetry only) — its own comment admitted *"5 s at 500 Hz"*, i.e. 1.56 s at 1600 Hz. Fixed in v0.87 (`ts_emit_period_s`) | own audit |
| `hr2/hr3_update_interval` (decimated samples) | ⚠️ latent — immune to the AFE rate only because the decimated rate is the invariant (§5.3.1); lowering a chain's target would have changed the period silently. Fixed in v0.87 (seconds) | own audit |
| `_diag_holdoff_samples` | ✅ derived from ms × fs | own audit |
| `_spo2_warmup_samples`, `_hr1_refractory_samples`, `_rsqm_probe_state_min_samples`, `_hr1_dc_alpha`, `_hr1_ma_len`, the 8 EMAs, the 3 biquads | ✅ derived in `_recalc_rate_params()` | own audit |
| `_compute_switched_rc_settling_samples()` | ✅ derived from the datasheet's t₅ and the phase windows | own audit |
| `hr2_buf_len`, `hr3_buf_len`, `hr2_acorr_lag_cap` | ✅ in decimated samples, immune by the 50 Hz invariant — documented as such | own audit |
| `hr1_sqi_cv_max`, `hr2_min_corr`, `hr3_snr_local_w`, `hr_min/max_bpm`, PI thresholds | ✅ dimensionless or in BPM/Hz — no rate dependency | own audit |

**The lasting outcome is not the two fixes, it is the net.** A hand audit finds what it finds; a
third instance would need another hand audit. `test_time_parameters_hold_across_catalogue` and
`test_hr1_running_max_decay_is_time_based` sweep the whole PRF catalogue asserting that each
duration is preserved **in seconds**.

**Two rules those tests had to learn the hard way** — the first version passed with both defects
deliberately reintroduced, which is worth recording because both mistakes are easy to repeat:

1. **Observe the effect, not the coefficient.** Reading the derived `alpha` could not tell a derived
   parameter from a hardcoded literal used *instead* of it: the coefficient stayed correct and
   simply went unused. The decay is therefore measured on `_hr1_running_max` itself — charge it,
   wait a fixed number of *seconds*, and check the surviving fraction is `exp(-t/tau)` at every rate.
2. **Exercise the setter with a non-default value.** Checking only defaults cannot distinguish a
   derived parameter from a constant equal to that default: hardcoding `_hr2_update_interval = 25`
   passes "25 samples / 50 Hz == 0.5 s" because 25 *is* the default. The test now asks for 1.0 s.

A detail worth remembering: the decay regression fails at **800 Hz and above, never at 500 Hz** —
the `0.9999` literal is exactly right at the default rate, which is why it survived so long.

**Not covered by the native tests:** `_emit_tasks()` and the rest of the `$TIMING` telemetry live
behind `INCUNEST_TIMING_STATS`, which `pio test -e native` does not define, so the tests never
compile them — the `ts_emit_interval` rename broke the firmware build while all tests passed.
(Since v0.91 the host HAL *could* compile that block — `esp_timer_get_time()` and the console
exist on the host — if a test build defined the flag.)
Any change touching that block needs `pio run -e incunest_V16` to be verified.

---

## 7. Nomenclature & conventions (pointer)

Naming rules, domain prefixes, ADC `adc_code`/`LSB` conventions, and the code↔ISO 80601-2-61↔AFE4490
term mapping (led1/led2 vs ir/red, etc.) are captured in the project's nomenclature notes; to be
folded here at consolidation.

---

## 8. HR valid range — provenance of 40–260 and the floor move to 30 BPM (2026-10-03)

**Question (Alex):** why 40–260 BPM, and what documents say so.

**Provenance — none normative.** `conversation_log.md`: 40–240 (initial) → 30–250 (2026-03-31, v0.9,
"range extended", no reason for either bound) → 25–300 (v0.11, "25 = ISO minimum; 300 = neonatal",
no section cited) → 25→30 (2026-05-02, "user decision") → 300→260 (2026-05-02b, "user decision") →
30→40 (2026-05-27, v0.25, "ISO 80601-2-61", no section). Until 2026-10-03 the spec claimed the range
"derives from ISO neonatal criteria". It does not: ISO 80601-2-61:2026 §201.12 only requires the
manufacturer to *disclose* "the displayed ranges of pulse rate". The citable requirement is
**WHO-UNICEF 2019**, *Technical specifications and guidance for oxygen therapy devices*, §6.5 (tabletop,
continuous NICU monitoring — IncuNest's category) and Annex 1 Table A1.11: pulse-rate detection range
**to include 30–240 bpm**, resolution ≤ 1 bpm, accuracy ±3 bpm, valid-data update ≤ 10 s. "To include"
is minimum coverage, so 260 complies; 40 did not.

**Ceiling 260, kept.** Lowering to 240 would blank the reading during neonatal SVT: PALS (AHA, 2020)
defines infant SVT as > 220 bpm; reviews give 200–300 bpm in neonates (Turk Arch Pediatr 2022;
PMC12812894, 2025), 220–280 bpm typical. Nothing in the project pins 260 specifically — it is a
ceiling above the SVT band with HR2's lag resolution (~22 BPM per lag at 260) already marginal there;
HR3's uniform 5.9 BPM bins carry that end.

**Floor 40 → 30 (v0.99).** Clinical reason first: with 40, a bradycardia at 35 BPM was reported as
*no valid signal* (SQI 0), i.e. a signal-quality alarm where a bradycardia alarm was due. The 2026-09-08
analysis listed four costs; measured 2026-10-03 on the host build (`setHRValidRange(30, 260)` on
v0.98, PPG-like input = fundamental + 2nd/3rd harmonic at 50/25/10 %, 500 Hz, 10 s), they came out:

| Estimator | 30 BPM clean | 30 BPM + 20 dB noise | 30 BPM + 2× 0.1 Hz drift | 30 BPM + breathing 45 rpm at 1× |
|---|---|---|---|---|
| HR2, corner 0.5 Hz | 30.0 / SQI 1.00 | 30.0 / 1.00 | 30.2 / 0.89 | invalid |
| HR2, corner 0.35 Hz | **29.91 → invalid** | invalid | 30.2 / 0.84 | invalid |
| HR3 | 30.1 / 1.00 | 30.1 / 1.00 | **invalid** (also at 33; 35 OK at 0.94) | 30.0 / 0.98 |
| HR1 | 30.0 / 1.00, first valid at 11.1 s | invalid (harmonic bump double-counted) | invalid | invalid |

- **HR2 band-pass corner: 0.35 Hz rejected, 0.5 Hz kept.** The 2026-09-08 worry — "30 BPM = 0.5 Hz =
  the corner, −3 dB on the fundamental" — was wrong in its conclusion: the autocorrelation is normalised
  by its own energy, so uniform attenuation of a periodic input changes nothing. A lower corner only
  admits more baseline wander (SQI under drift 0.89 → 0.84) and, incidentally, shifted the 30.0 BPM
  estimate from 30.01 to 29.91, i.e. across the hard gate. Band-pass attenuation (single 2-pole section,
  computed): at 0.5 Hz corner, 30 BPM −3.0 dB, 27 BPM −3.6 dB, 40 BPM −1.7 dB.
- **The gate is a knife edge at the bound**, for any bound: ±0.1 BPM of estimation error decides
  validity at exactly 30.0 BPM. Inherent to a hard range gate; clinically irrelevant (alarm thresholds
  are far from the bounds). Unit tests pin the bound with a signal *at* it and the side it measured.
- **HR3 is the weak one at the bottom under drift.** It has no high-pass (LP-only since v0.83, mean
  subtraction before the FFT); `search_min` = bin 5 (0.49 Hz) sits next to the drift bins 1–3 and Hann
  leakage of a 2× wander reaches it. Clean and noisy 30 BPM are fine. Not fixed here: a high-pass before
  HR3 trades drift for the very band the floor just opened; left as a known limit, candidate for the
  MS100 campaign at 30/35 BPM.
- **HR1** reaches 30 BPM in 11.1 s (5 intervals) on a clean input; with 20 dB noise the harmonic bump
  gets double-counted at 30 BPM (the fixed 0.185 s refractory covers nothing of a 2 s period — §5.3's
  adaptive-refractory argument, now with a number). HR1 failing under drift/breathing at every rate
  tested is pre-existing (§5.3, §5.4), not a floor effect.
- **Breathing overlaps the band.** Neonatal 30–60 rpm = 0.5–1 Hz = 30–60 BPM. A respiratory component
  of pulse-like amplitude corrupts HR1/HR2 at 30–40 BPM and HR3 at 33–35; at 1/3 amplitude (more
  typical of respiratory PPG modulation) not measured yet. No range bound fixes this; it argues for a
  conservative SQI below 40 BPM and for validating with simulated breathing, which the MS100 does not do.

**Not done, deliberately:** `hr2_acorr_lag_cap` stays 137 (sweep now 111 lags at 50 Hz; the cap binds
only above ~700 Hz AFE rate with the derived decimation, which keeps 50 Hz anyway). HR2 `f_low`,
HR3 filtering and the HR1 refractory are unchanged. The capture campaign still lacks 30 and 35 BPM
points (`CAPTURE_SET_SPEC.md`), which is where the HR3-under-drift limit should be confirmed on hardware.

---

## 9. SpO2 R method — choosing R-METHOD-2 (2026-09-29 → 2026-10-03)

**Problem.** R-METHOD-1 (lib since v0.38) is `R = (σ_red/μ_red)/(σ_ir/μ_ir)` with μ an EMA (τ 2 s)
and σ the RMS of x − μ by EMA (τ 6 s), on OT. σ takes everything that is not DC — noise, motion,
venous and respiratory components — and on HOSPNAV SUBJ08/09 (2026-09-23, Medle probe, reference =
the NICU monitor read by VideoNest) the R→SpO2 scatter was 5.1 points with Spearman ρ −0.44.

**Options listed (Alex, 2026-10-02):** (1) band-pass + RMS ratio; (2) regression red-on-IR, OLS;
(3) the same, total least squares; (4) regression on derivatives ("reg_dols"); (5) spectral
amplitude at f_HR (+harmonics); (6) autocorrelation at the pulse lag; (7) literature — Masimo DST
(expired US 5,632,272), Philips FAST (expired US 6,631,281), per-beat peak-to-trough. All are
replayable offline: the captures keep raw OT.

**Measured** (3471 stationary pairs, 8 s windows; scripts and summaries in
`PulseNest/captures/sessions/20260923_HOSPNAV_SUBJ08_ANALYSIS/`):

| R estimator | ρ | scatter [SpO2 pts] | note |
|---|---|---|---|
| R-METHOD-1 (lib) | −0.44 | 5.07 | baseline |
| (1) band-pass + RMS, zero-phase | −0.68 | 2.7 | most of the gain is the band-pass |
| (2) OLS / (3) TLS, zero-phase | −0.71 / −0.69 | 2.4 / 2.6 | r² median 0.988, so OLS ≈ TLS |
| (4) derivative regression, zero-phase | −0.82 | 1.61 | best |
| (4) causal: Butterworth 4-pole forward + EMA(dx·dy)/EMA(dx²) | −0.80 | 1.62 | causality costs nothing here (it costs ~0.5 pt to (1)/(2)) |
| (4) causal with the library's **one-biquad** band-pass | −0.80 | **1.63** | what the firmware can run at today's cost |

Insensitive to band edges (0.3/0.7–4/8 Hz), filter order, derivative lag k 1–25 and τ_dc 1–6 s.
Leave-one-subject-out median |error| 0.95–1.37 points (R-METHOD-1: 3.1–3.3 with bias ±2.6).
Why the band-pass matters (step 1d): power below 0.5 Hz, ~2× larger relative to the pulse in red
than in IR, inflates R-METHOD-1's R by a near-constant ×1.2; it does not correlate with SpO2, so
the HOSPNAV slope change (21.5 → ~38) is regression dilution by R-METHOD-1's noise times that
inflation — not motion coinciding with desaturations. Methods (5)–(6) not yet run (step 1e).

**Decisions taken 2026-10-03 (dev session, with Alex):**
- **Band-pass = the library's one-biquad `BiquadFilter::init_bp`** (2 poles, −6 dB/octave per skirt),
  not a 4-pole design: 1.63 vs 1.62, same fits and same leave-one-out. No new filter type enters the
  library for this.
- **τ_ac stays 6 s.** τ_ac 2 s scored better (1.39), but the averaging time is a separate decision
  with its own evidence base (§4.5: ISO sets no window, neonatal practice 8 s); the method change
  must not carry a hidden averaging change. Revisit there, not here.
- **Step 2 done: the candidate gets worse nowhere it was tested.** `tools/offline_runner` v0.23
  computes it as `R_CAND` beside the library (C++ float = Python float64 to 2·10⁻⁴ in R; PILAB's
  [R CANDIDATE] preset = the runner to 2·10⁻⁴) and replayed every capture on disk: 8 HOSPNAV
  sessions (25 parts), the v0.4 corpus (3 sessions) and 16 MS100 captures. Coverage identical to the
  library's in every file (same gating and warm-up). MS100 clean, 98 %: library 0.554 flat, candidate
  0.545–0.551 across 40–250 BPM (a ≤ 0.2-point HR dependence). MS100 with the probe perturbed:
  library 0.82–0.87, candidate 0.66–0.73 (unperturbed 0.554) — less bias. HOSPNAV, with the replay following the HGAC records (runner v0.24, see the correction below): the
  library's 99th percentile reaches 1.4–1.8 in the SUBJ09 parts, the candidate's 1.0–1.2 (the
  "15–50" first reported here was a replay artefact).
- **Correction (2026-10-03, same day): the "new failure signature" first reported here — R ≈ 0 or
  negative with r ≈ 0 in 3 SUBJ09 episodes — was a replay artefact, not a property of the candidate.**
  The runner (≤ v0.23) kept the header's `$CFG` for the whole session and ignored the capture's
  `# @row N afe: cause=hgac` records, so after the board's HGAC moved RF2 (100k ↔ 250k, 2–6 times per
  SUBJ09 session) the replay divided the raw codes by the wrong RF: replayed OT_LED2 stepped +150 % /
  −60 % where the board's moved 1–4 %, for 82–89 % of those sessions; the library's ProbeState, SQIs
  and R, and the candidate's R and r, were wrong from there on. Runner v0.24 follows the records, and
  aligns each RF change to the raw code's own jump (the record's row is where the host learnt of it,
  −14…+6 rows off): applied at the record's row, one sample of old-RF-on-new-codes still went
  through the filters and the settling freeze held it for 10 more — a 20 ms ×2.5 spike that alone
  produced r ≈ 0.3 for 30 s after every HGAC move. With both fixed: no negative R anywhere (candidate
  p1 ≥ 0.62 in every part), and the r distribution is what the next bullet says. r stays the SQI
  *candidate* of R-METHOD-2 on first principles (the regression's own goodness of fit), but the
  corpus no longer supplies evidence for it or for any threshold.
- **What r actually does on the corpus (replays corrected, 2.66 h valid, 8 HOSPNAV sessions + 16
  MS100 captures).** r never drops below 0.83: HOSPNAV p0.1 / p1 / p5 / p50 = 0.906 / 0.949 / 0.973 /
  0.994; r < 0.8 = 0 s, r < 0.9 = 6 s (0.1 %). MS100 clean 0.975–0.996; MS100 with the probe perturbed
  0.94–0.97 — while R there is biased +0.1 to +0.18 (0.55 → 0.66–0.73). So the "bimodal 0.99 / 0.3"
  picture, the 3.8 % rejected at 0.8, the 10 SUBJ09 episodes and the desaturation-coverage figures
  reported earlier in this section were all the artefact: **a gate at 0.8 rejects nothing in this
  corpus, and would not flag the one bias the corpus does show (probe perturbation, r ≈ 0.95).**
  69 reference desaturation events are kept 100 % whatever the threshold. The value of r as an SQI
  has to be shown on signals that lack a pulse (probe off, motion) with the corrected replay — not
  yet done.

- **r threshold = 0.8 (Alex, 2026-10-03) — decided on the figures above before they were found to be
  an artefact; to be reconfirmed.** With the corrected replays the choice costs nothing on this corpus
  (0 s rejected) and protects against nothing the corpus shows; it remains a plausible guard for
  no-pulse input until measured there. Band-pass order and band confirmed the same day (Alex asked): 2nd order
  because 4 poles measured equal for R and add nothing the data ask for yet (they buy drift rejection,
  −30 vs −15 dB at 0.1 Hz, not neonatal respiration rejection — 40–60 breaths/min is inside the pulse
  band whatever the order); 5 Hz because R does not depend on the cut (both channels share the filter:
  MS100 0.545 at 40 BPM → 0.551 at 250 BPM; HOSPNAV 0.5–8 Hz 1.59, 0.5–4 Hz 1.67, insensitive).

**Pending:** step 1e (spectral/autocorrelation), 1f (MS100 R stability — partly answered above),
then step 3 = R-METHOD-2 in the library (r ≥ 0.8 as its SQI) with an R curve re-fitted with it
(decision (ii) of 2026-10-02: from MS100 and/or SUBJ08/09 replays). The DC estimator it divides by is
§10's question.

**What R is, and what the shipping devices estimate (prior art, 2026-10-04; consult session
pulsenest-31, citations verified on the PDF).** ISO 80601-2-61:2026 defines R abstractly — 201.3.239
*modulation ratio, ratio of ratios, R*: "basic quantity derived by pulse oximeter equipment from
time-dependent light intensity measurements", with the empirical curve R → SpO2 as note 1 and one
curve per range of emitter wavelengths as note 2 — and gives the two-point formula only in the
informative Annex EE.4, as something R "can be approximated as" (EE.1:
log10(max/min)_red / log10(max/min)_IR ≈ (AC/DC)_red / (AC/DC)_IR ≈ %mod_red / %mod_IR). So
(AC/DC)_red/(AC/DC)_IR — R-METHOD-1 — is an *estimator* of R, the original one of the analogue
instruments, not R's definition; the physical quantity is the ratio of the changes in logarithmic
absorbance at the two wavelengths over the pulse. The public patents describe other estimators.
**The direct antecedent of R-METHOD-2 is Ohmeda's US 5,503,148** ("System for pulse oximetry SpO2
determination", filed 1994-11-01, granted 1996-04-02, expired 2014-11-01 — public domain; verified
on Google Patents 2026-10-05, found by consult session pulsenest-4f): it takes the sample-to-sample
differences of the red and IR signals ("or a logarithmic difference circuit"), fits the (dRed, dIR)
pairs by a least-squares regression "that accounts for noise present in both data channels"
(a two-variable fit — our offline `reg_tls`, which measured the same as `reg_ols` because r² is
0.99), reads R as the slope of that line, and uses the fit's "goodness-of-fit correlation
coefficient … a measure of linearity" to detect motion artefact and poor data — our r gate, thirty
years earlier. Ohmeda's lineage (Biox → Ohmeda → Datex-Ohmeda 1998 → GE HealthCare 2003) is today's
TruSignal; GE's white paper "Accuracy of TruSignal SpO2" (Hyle, 2014) discloses no internals, only
adult Arms 1.4 % still / 1.9 % with motion / up to 2.7 % at low perfusion (manufacturer data). Then
Nellcor (Baker & Yorkey, 1998 on) takes ln and band-passes each channel,
differentiates, and fits the slope of red on IR **through the origin** over a window of several
seconds — R = Σ(dRed·dIR)/Σ(dIR²), exactly R-METHOD-2's regression — then feeds the fit's residual
variance to a Kalman filter on saturation and averages beats aligned to the pulse under motion;
Masimo SET (Goldman et al. 2000, J Clin Monit Comput; Diab/Kiani patents 1996–97) does not regress
at all: it sweeps candidate values of R from 100 % down, builds for each the reference
N(R) = Red − R·IR for an adaptive noise canceller with IR as primary, and reads the arterial R as the
higher-saturation peak of the output power — separation of the arterial and venous components
(the venous one, moved by motion, pulses with its own R_v > R_a and is correlated across channels,
so averaging cannot remove it). Implications: R-METHOD-2 is Nellcor's second step, reached here by
measurement rather than by reading; its r gate is a crude form of Nellcor's residual weighting;
beat-aligned averaging and arterial/venous separation remain open. **The lesson recorded with it:**
R-METHOD-1 was designed from the textbook formula without a prior-art pass, and the regression sat
as a half-line of a task list from 2026-09-29 until 2026-10-02; the HOSPNAV curve of 2026-09-24
(21.5 slope) was mostly that estimator's noise. From now on, before any measurement block is designed
or refined (R, HR, PI, SQI, averaging), the first step is a note of what Nellcor, Masimo and Philips
do and which part of ours is the textbook simplification (memory `feedback_prior_art_before_design`).

**Step 3 done (2026-10-04, lib v0.100).** R-METHOD-2 is in the library, as the default, built from
the pieces the candidate used (one-biquad `init_bp` 0.5–5 Hz, the `EmaChannel` mean as DC, three
EMAs of τ_var on dx·dy, dx², dy²); `offline_runner` v0.25 reports its R bit-identical to `R_CAND`
on every sample of the 24 HOSPNAV parts. Decisions taken with it:
- **R-METHOD-1 stays selectable** (`setSpO2RMethod(1)`), both methods computed on every sample.
  PI needs R-METHOD-1's EMAs anyway, so the cost is one more variance EMA; what it buys is an on-board
  A/B in the next hospital campaign and the replay of older behaviour. Not two outputs: `spo2_r` is
  the method in force, `$CFG` names it.
- **r is a validity gate, not the SQI value.** SQI keeps its PI ramp (signal strength); under
  R-METHOD-2, r < `spo2_r_method2_reg_corr_min` (0.8) sets SpO2 = NaN, SQI = 0 and leaves R and r reported, so
  the reason is on record. Mapping r itself onto [0, 1] was rejected: r = 0.9 is a sound fit on this
  corpus (p1 of the window medians 0.944) and would have read as "half quality". The 0.8 stays
  provisional (§9 above): it costs nothing on the corpus and has not been measured on pulse-less
  input. r travels in `AFE4490Data::spo2_r_method2_reg_corr`; whether the `$M4` frame carries it is the
  firmware's call (its 288-byte slot has ~10 B of margin after the 7-figure OT).
- **The curve is fitted on the library's own R** (`fit_method2.py`, same pairs, stationarity rule
  and bin fit as the step-1 table): SpO2 = 123.98 − 39.13·R, pooled over 3455 windows of the two
  subjects — the same numbers §10 obtained through the runner's candidate (124.0 − 39.1·R), by a
  different path. ρ −0.80, scatter 1.65, median |residual| 0.91; per subject 129.2 − 46.7·R and
  118.8 − 32.1·R (leave-one-out median |error| 0.95 / 1.38). Below a reference of 85 % (31 windows,
  all dynamic) it reads −3.4 points: the low range is unmeasured, not fitted. It is a hospital-monitor-
  referenced curve for one probe model (Medle ST-S0163), not an ISO Annex DD desaturation study; the
  reference monitor's make and averaging were not recorded. Its id reused
  `R-CURVE-STS0163-HOSPNAV-20260923`: the 2026-09-24 proposal under that id (115.17 − 21.54·R,
  R-METHOD-1) was never applied, and the pair (id, method) was taken to identify a curve. Reversed
  on 2026-10-20 (v0.106): the label's date is the date of the fit, `…-20261004`, so that one label
  is one (a, b) without needing the method to disambiguate — see §14.
- **Why not the MS100 curve any more:** with R-METHOD-1 it read a median −9.9 points against the
  ward monitor on these campaigns (p10 −17.2), and a curve fitted with one method applied to the R of
  another is worse still (−9.7 for the step-1 `reg_dols` fit on R-METHOD-1's R — the 2026-10-03
  "v0.100" that was reverted the next day). The simulator curve stays in the registry for the UpnMed
  probe and for replaying older captures.
- **Not changed:** τ_var 6 s (the averaging time, §4.5), the warm-up (18 s), the DC estimator (§10),
  PI (still RMS/DC from the EMAs), HR1–HR3.


## 10. The DC estimator under the R method — EMA vs 2nd-order low-pass (2026-10-03)

**Question (Alex).** Replace the EMA that estimates DC — everywhere in the library — by a 2nd-order
low-pass, and make band-passes 4th order by default. Two reasons given: the EMA is slow (τ 2 s), and
its selectivity cannot be set with the clarity of a biquad.

**Analysis before measuring.** The slowness is the cutoff's, not the form's: an EMA of τ 2 s is a
one-pole low-pass at 0.08 Hz, and any filter with that cutoff settles in about the same time. The
constraint is leakage: the DC must not follow the slowest pulse accepted, 30 BPM = 0.5 Hz. The
selectivity point is right (−6 vs −12 dB/octave) and converts into speed at equal leakage. Filter
facts at 500 Hz:

| DC estimator | \|H\| at 0.5 Hz | at 1 Hz | 63 % | 95 % | 98 % | overshoot |
|---|---|---|---|---|---|---|
| EMA τ 2 s (library) | −16.1 dB | −22.0 dB | 2.00 s | 5.99 s | 7.82 s | none |
| 2-pole LP 0.2 Hz, Q 0.5 (one biquad) | −17.2 dB | −28.3 dB | 1.71 s | 3.78 s | 4.64 s | none |
| two EMAs of 0.796 s in cascade (same poles) | −17.2 dB | −28.3 dB | 1.71 s | 3.77 s | 4.64 s | none |
| 2-pole LP 0.2 Hz Butterworth | −16.0 dB | −28.0 dB | 1.39 s | 2.33 s | 2.50 s | 4.3 % |
| EMA τ 0.5 s (the "just shorten τ" option) | −5.4 dB | −10.4 dB | 0.50 s | 1.50 s | 1.96 s | none |

EMA τ 0.5 s is out: it lets a 30–60 BPM pulse into the DC at −5 to −10 dB, so PI and R would be
modulated by the beat. The three 2-pole forms were chosen to leak like the current EMA (−16/−17 dB at
0.5 Hz) so that the comparison isolates speed and arithmetic.

**Measured** — `tools/offline_runner` v0.24 runs the candidate's regression four times on the same
samples, differing only in the DC each channel's band-passed signal is divided by (`R_CAND`,
`R_CAND_CD`, `R_CAND_CE`, `R_CAND_BW`), each DC in float as the firmware would, with a double twin;
8 HOSPNAV sessions with the HGAC records followed, 16 MS100 captures:

| | EMA 2 s | LP2 0.2 Hz Q 0.5 biquad | 2×EMA 0.796 s | LP2 0.2 Hz Butterworth |
|---|---|---|---|---|
| SpO2 scatter vs hospital reference, 3471 stationary pairs | **1.64** | 1.89 | **1.63** | 1.80 |
| Spearman ρ / joint fit | −0.80 / 124.0 − 39.1·R | −0.80 / 123.7 − 38.7·R | −0.80 / 124.0 − 39.2·R | −0.80 / 123.6 − 38.5·R |
| Leave-one-subject-out, median \|err\| (08→09 / 09→08) | 1.38 / 0.95 | 1.34 / 0.98 | 1.35 / 0.95 | 1.36 / 0.98 |
| R excursion after an HGAC RF2 move (4 events ×, median / p90) | 0.076 / 0.134 | 0.071 / 0.137 | 0.077 / 0.134 | 0.071 / 0.136 |
| recovery to within 0.02 (median / p90) | 2.0 / 3.7 s | 1.7 / 3.9 s | 1.9 / 3.8 s | 1.6 / 4.0 s |
| MS100 clean → probe perturbed, R bias at 60…250 BPM | +0.015…+0.18 | same ±0.004 | same | same |
| float vs double, max relative DC deviation | 3·10⁻⁶ | **2–5 %** | 3·10⁻⁶ | **2–5 %** |

**Findings.**
1. **No measurable gain in R from a faster or steeper DC.** The two-EMA cascade (twice as fast at
   equal leakage) scores the same as the EMA to the second decimal; the Butterworth, the same within
   noise. Perturbation bias and HGAC-move transients are identical across the four: those are set by
   the band-pass and the regression's 6 s memory, not by the DC.
2. **A float32 biquad at 0.2 Hz / 500 Hz is not usable as a DC estimator.** Its DC gain is
   (b0+b1+b2)/(1+a1+a2) with 1+a1+a2 ≈ 6·10⁻⁶, a cancellation of numbers of order 1 held to 10⁻⁷ —
   2–5 % error, state-dependent, and it shows: the biquad forms are the two worst scatters (1.89,
   1.80) while the same double pole computed as two EMAs scores 1.63 (the cascade lacks only the
   bilinear biquad's zeros at Nyquist, irrelevant below a few Hz). The EMA's
   `dc += α·(x − dc)` has exact unity DC gain by construction. A low-pass this slow would have to run
   on the decimated 50 Hz stream, in double, or as cascaded EMAs.
3. The only benefit left is settling after a reset — 95 % in 3.8 s instead of 6.0 s — and the
   regression's own warm-up (3·τ_ac = 18 s) hides it.
4. **Band-pass order**: §9 already measured 2 vs 4 poles equal for R (1.63 vs 1.62). 4 poles buy drift
   rejection (−30 vs −15 dB at 0.1 Hz) and adult-respiration rejection (−8.9 vs −5.6 dB at 0.33 Hz),
   not neonatal-respiration rejection (40–60 breaths/min is inside the pulse band). Not the data's
   ask yet.

**Decision (proposed; Alex to confirm).** R-METHOD-2 keeps the EMA DC of τ 2 s (`EmaChannel`,
spec §4.5). Averagers of products (EMA(dx·dy), EMA(dx²), EMA(AC²)), the probe/HGAC state detectors
and PILAB's option lists are untouched in any case: an averager must keep its weights positive (a
2-pole low-pass can ring EMA(dx²) negative), and detectors are calibrated on the EMA's dynamics. If a
faster DC is ever wanted, the form is two cascaded EMAs, never a float biquad at that cutoff. The
band-pass stays one biquad, 0.5–5 Hz, unless Alex prefers 4th order for drift rejection — measured
neutral for R, so that is a preference, not a correction.

**Side finding — what an HGAC move does on OT, measured.** On the board OT_LED2 steps 1–4 % at an RF2
change (the ±7 % RF tolerance; up to 30 % when the move coincides with a signal change), i.e. 1–30×
the pulse amplitude. Through the candidate: R excursion 0.07–0.10 for ~2 s, r never below 0.83. The
"state compensation at RF change" task (×k on the EMAs) was written for v_tia-based estimators
before SpO2 moved to OT; on OT it is moot for R — the residual step is small and short-lived.

**Method note — the replay artefacts.** Two were found and fixed on the way (runner v0.24, spec §9):
the replay did not follow the HGAC records, and then applied them a few rows off. They corrupted the
SUBJ09 half of every runner-based conclusion of §9 (corrected there). Rule drawn: before concluding
anything from a replay, compare its OT, ProbeState and SQI against the board's recorded columns
around every `afe:` record; the board's columns are the truth the replay must reproduce.
**Since runner v0.25 (2026-10-04) that rule is code:** every raw-code replay compares its OT with the
recorded OT row by row and prints the result per part. Doing so found a third, smaller artefact:
v0.24 applied each RF change at the raw code's jump, but the board changes RF 8 rows earlier — its
settling freeze repeats the last codes into the capture, so the jump follows a run of 8 held copies
— and the replay held the old OT 16 ms too long (~3 % on those rows). Fixed (applied at the first
held copy); 29 of the 30 parts with OT columns now reproduce the board within 10⁻⁴ on every value.
The R figures of this section were not re-measured; the change is expected in their last digit only, because the board's own OT steps by the
same ~3 % (RF tolerance), only 16 ms earlier.

---

## 11. PI (Perfusion Index) — decoupling from SpO2 and the sliding-window AC estimator (2026-10-05)

### Why decouple

PI lived inside `_spo2_update()`, sharing `_spo2_ch_ir_ema` — the same `EmaChannel` (mean τ 2 s,
var τ 6 s) R-METHOD-1 used for its own AC/DC. Alex noticed, comparing PILAB's STEP1 options, that
a 0.5–2.0 Hz band-pass looks like "EMA subtract" at τ 0.5 s but not at τ 2 s — correct: a
first-order EMA subtraction is a high-pass with f_c = 1/(2π·τ), 0.08 Hz at τ 2 s vs 0.32 Hz at
τ 0.5 s, and only the latter sits close to the band-pass's 0.5 Hz edge. That observation reopened
a question settled for a *different* reason on 2026-10-03 (§10): that session kept the DC EMA at
τ 2 s for R-METHOD-2's **DC_R** role (normalising denominator only), where a runner experiment
found no accuracy gain from a faster estimator and a real cost (biquad float32 DC-gain loss at
that corner). It said nothing about **DC_sub** — the role of subtracting that same EMA to
*extract* the AC waveform, which R-METHOD-1 and PI both still used. These are different jobs with
different constraints (tracked as a pending DC_sub/DC_R split task, previously only in Claude's
working memory, not in this repo), currently sharing one number only because the split was never
implemented. Shortening DC_sub's τ to make STEP1 "look like" the
band-pass would have fixed a cosmetic mismatch while reintroducing a real risk: at τ 0.5 s
(f_c ≈ 0.32 Hz), the EMA sits less than an octave below the slowest accepted heart rate (30 BPM =
0.5 Hz), so during bradycardia the "DC" would start tracking the pulse itself, and subtracting it
would partially cancel the very AC amplitude PI exists to measure — in exactly the clinical
scenario (bradycardia, often with desaturation) where an accurate PI matters most.

R-METHOD-1 is deprioritised (kept only for comparative analysis, per Alex), so the AC/DC split
task's R-METHOD-1 motivation no longer applies to it specifically — but PI is very much still
live, and used independently of either R method. So: implement PI as its own function with its
own state, free to pick an AC-extraction mechanism that does not carry the bradycardia risk at
all, instead of just re-tuning the shared EMA's τ.

### Why a private band-pass instead of an EMA (STEP1)

Two options were on the table: shorten DC_sub's τ, or replace the EMA subtraction with a
band-pass (Alex proposed 0.5–20 Hz). Neither "repair the EMA" option was taken. A band-pass
(biquad, 2nd order) has a steeper roll-off than a first-order EMA at an equivalent nominal
corner, so it can sit at 0.5 Hz and still pass 30 BPM faithfully without the EMA's "starts
tracking the pulse near its own corner" failure mode — the problem above is specific to using a
single real pole as a high-pass, not to the 0.5 Hz number itself.

**Upper corner: 15 Hz, not 5 Hz — a correction.** The first cut (morning of 2026-10-05) kept
R-METHOD-2's 0.5–5 Hz and argued that widening to 20 Hz "would only admit noise, not signal". The
measurement below proved that wrong for PI: on the MS100 at constant modulation, the 0.5–5 Hz
PI read 1.70 % at 60 BPM and 1.27 % at 220 BPM (−25 %), because the second harmonic (7.3 Hz)
leaves the band; with 0.5–10 Hz the drop is −6 %, with 0.5–15 Hz it is gone; on SUBJ08 (HR 166)
widening from 5 to 10 Hz adds +4 % and nothing beyond, and the noise admitted above 10 Hz was
unmeasurable even in the lowest-PI decile. The distinction that matters: R is a *ratio* with the
same filter on both channels, so a harmonic the filter drops is dropped in both and cancels; PI is
a *magnitude*, so a dropped harmonic is missing amplitude. 15 Hz is HR3's ceiling and its
derivation (`hr_max_bpm` = 260 → 4.33 Hz → 3rd harmonic 13 Hz, 1.15× margin) applies unchanged.
The floor stays the HR floor, 0.5 Hz, as HR2/R-METHOD-2 — reusing the *number*, not the *instance*:
PI gets its own private `BiquadFilter`
(`_pi_bpf`), following the library's existing "paths share nothing" rule for HR2 vs HR3 (spec
§5.3.1: separate filter, buffer and phase counter per analysis path, specifically so one path's
reset/reconfiguration can never perturb another's). Reusing `_spo2_bp_ir`'s instance directly was
considered and rejected: that filter's internal state (`_v1`/`_v2`) is only valid if `process()`
is called exactly once per sample from exactly one call site: sharing it between `_spo2_update()`
and a separately-timed PI function would mean either double-processing the same filter state
(corrupting it) or an ordering dependency between two functions that the whole point of
decoupling PI was to remove. A second, independent biquad instance costs a handful of floats and
a few multiply-adds per sample — the library already pays this exact price twice over for HR2 and
HR3 (same corner family, each with its own instance), so a third instance for PI is consistent,
not novel.

### The DC: the same window as the AC, not an EMA (STEP3)

The first cut gave PI a private 2 s DC EMA (denominator only, never subtracted, so none of the
bradycardia risk; the DC_R role of §10, where a faster estimator bought nothing). Alex asked why
the DC was not simply the mean of a buffer like the AC's, without the squaring — and asked for a
study of both the DC and the AC method against the manufacturers, the standard and the
literature. The study (2026-10-05, scripts in the session scratchpad, data below) settled it:

**DC, measured.** On HOSPNAV SUBJ08 (56 min, HR ≈ 166, 489 windows of 6 s inside PROBE_APPLIED)
the 2 s EMA differs from the 6 s window mean by a median of 0 % (IQR ±0.25 %, p5/p95 ±1.5 %);
Nellcor's per-beat (Max+Min)/2 by +0.2 %; SUBJ09 and the MS100 at 60–220 BPM agree (±0.2 %).
**The DC estimator is numerically irrelevant to PI** — the AC definition (×2.8–3.2, next section)
and the window-to-window variability (28–43 % on a moving neonate) dwarf it. So the choice is
structural, and there the window wins: numerator and denominator span the same 6 s (no lead/lag
when the level moves — probe pressure, an RF step), PI has no EMA, no seed and no warm-up concept
at all beyond "window full", and it is literally Masimo's "λDC = average value of the signal"
and ISO's "baseline (non-modulated) portion". Cost: a second 300-float ring and a running sum.
Implementation detail that matters: the DC sample pushed at each decimated step is the **mean of
the raw OT over the decimation block** (accumulated at the AFE rate), not every 10th raw sample —
anti-aliasing for free, at one addition per sample.

**What the DC does not fix — a finding to carry forward.** Around the 10 HGAC RF changes in the
SUBJ09 parts, PI deviates 25–250 % from its pre-step median for 6–14 s with *either* DC (EMA vs
window differ by 3–7 points of that). The transient is the numerator's: the 1–4 % OT step (RF
tolerance, §10 side finding) rings through the band-pass and sits in the 6 s RMS window. Same
conclusion §10 reached for R. The remedy is upstream of the DC — blank PI for one window after an
RF move, or compensate the filter state — and is logged as a pending task, not done here.

### The AC: peak-to-peak units, 2√2·RMS (the 2026-10-05 study)

**What the standard says.** ISO 80601-2-61:2026 §201.3.245 defines *percent modulation, %mod* as
"indicator of pulsatile signal strength resulting only from tissue arterial blood content changes
at the probe site over the cardiac cycle"; Note 1: "may be approximated as AC/DC of the detected
light signal multiplied by 100, where AC refers to the modulated signal portion and DC refers to
the baseline (non-modulated) signal portion"; Note 3: the infrared signal as the primary factor.
Annex II.2.1: *%mod* is "sometimes referred to as percent modulation, pulse amplitude, Perfusion
Index, or PI", normal 1–8 %. Formula EE.1 writes R with per-pulse max/min. **The standard fixes
neither peak-to-peak vs RMS nor a window**; it requires %mod to be disclosed in low-perfusion
claims (201.12.1.103) and per data point in the study report (201.12.1.105).

**What the manufacturers do.** Nellcor (US 6,675,031): Mod % = 100·AC/DC with **AC = Max − Min per
pulse, DC = (Max + Min)/2**. Masimo (US 8,280,473, US 11,006,867): **PI = (λmax − λmin)/λDC, λDC the
average of the signal**, per pulse; the "smoother" keeps a baseline PI from good-quality epochs and,
under motion, publishes the lower of the two ("motion noise tends to increase PI"). Masimo's
whitepaper: IR 940 nm, pulsatile over non-pulsatile. Literature: PI is not standardised and not
comparable across brands (7.07 / 5.80 / 6.80 % on the same subject with three devices;
NCT06635018 is measuring exactly this). **Every published neonatal threshold is Masimo, i.e.
peak-to-peak:** Granelli & Östman-Smith 2007 p5 = 0.70, p1 = 0.50, p95 = 4.50 (10 000 newborns);
De Felice 2002 high illness severity ≤ 1.24 (0.86 vs 2.02); Hakan 2013 term day-1 preductal
1.35 (1.02–1.91), postductal 0.88, preterm 0.88 / 0.61; Masimo "low perfusion" < 0.25; Philips
"ideally > 1.0". PILAB's S2_PEAKPK, for the record, is (max − min)/2 — half of all of these.

**What our data say** (6 s windows, same 0.5–5 Hz band-pass as the first cut):

| | SUBJ08 (HR 166) | SUBJ09 1238 p01 (HR 160) | MS100 60 / 100 / 140 / 220 BPM |
|---|---|---|---|
| pp/RMS of the band-passed PPG (sinusoid 2.83) | 2.79 (IQR 2.21–2.89) | 2.92 | 3.17 / 3.20 / 3.18 / 3.01 |
| PI v0.100 (EMA var / EMA mean), median | 1.66 | 1.35 | 1.71 |
| PI first cut of v0.101 (RMS of band / EMA) | 1.06 (−36 %) | 1.13 (−16 %) | 1.69 → 1.27 at 220 |
| PI peak-to-peak / window DC (Masimo-like) | 2.49 | 3.20 | 5.4 → 3.8 at 220 |
| Window-to-window variability, RMS vs per-beat pp median | 42 % vs 28 % | 4.5 % vs 8.7 % | — |

Two consequences the first cut had not seen. (1) **The decoupling had silently changed PI's
scale**: the EMA subtraction let through the < 0.5 Hz and > 5 Hz content, worth 16–36 % of the
AC. With `spo2_sqi_pi_lo/hi` = 0.5/2.0 unchanged, SUBJ08 went from the board's mean SQI 0.61 to
10.8 % of windows at SQI 0 and only 9.6 % at SQI 1. (2) **Those thresholds were copied from
Nellcor/Masimo, whose PI is peak-to-peak, while the library's had always been RMS — ÷2.8–3.2.**
In peak-to-peak units the same SUBJ08 windows give 0 % below 0.5, 61 % at or above 2.0 and 0.2 %
below Granelli's p5; the capture's first rows show the problem in one line: PI 0.62 (≈ 1.8 in
Masimo units, a normal neonate) with SQI 0.08.

**Reference ranges by age (2026-11-05).** The published PI percentiles by population (adult, child, term and preterm newborn), the five levels proposed for the display and alarms, and what they imply for the 0.5/2.0 SQI ramp are collected in PulseNest `docs/pi_reference_ranges_by_age.md`.

**Decisions (Alex, "adelante" to all four).** (a) DC = the window mean above. (b) **PI in
peak-to-peak units, as `2√2 · RMS` of the band-passed signal** (`pi_rms_to_pp_ratio`): exact for a
sinusoid, which the band-passed PPG nearly is (2.8–3.2 measured), so it under-reads a per-beat
Max − Min by 0–12 %; it needs no beat detector and is the smoother of the two on a quiet subject
(SUBJ09: 4.5 % vs 8.7 %). The per-beat median (Masimo/Nellcor exactly; more robust on a moving
subject, SUBJ08 28 % vs 42 %) is kept as a PILAB experiment — it would ride on HR1's detector.
(c) Band-pass 0.5–15 Hz (previous section). (d) `spo2_sqi_pi_lo/hi` stay 0.5/2.0, now in the
units they were quoted in, and **`spo2_pi_max` = 20 %** (the top of every manufacturer's displayed
range: Masimo 0.02–20, Mindray 0.05–20, Nellcor 0.03–20) withholds SpO2 above it — closing the
2026-08-22 finding that the SQI ramp clamped its *output* only, so a 147 % "PI" during the DC
transient of a probe coming off scored SQI = 1 and published a plausible SpO2.

### Why a sliding window instead of an EMA (STEP2)

The actual complaint motivating this whole change was PI's settling time: an EMA never truly
finishes converging (63 % at 1τ, 95 % at 3τ, 99 % at 5τ), which is why SpO2's warmup is 3×6 s =
18 s and even that is an approximation, not an exact bound. Alex asked for proposals, specifically
mentioning "replace EMA-6s with a 6s window of memory" as an example. Two shapes were considered:

- **A full-rate circular buffer** (6 s × sample rate, e.g. 3000 samples at 500 Hz, per channel) —
  rejected on RAM: 12 KB per channel is a real cost on an ESP32-S3 for no behavioural gain over
  the decimated version below, since PI's AC envelope changes on a timescale of seconds, far
  slower than the sample rate.
- **A decimated circular buffer with a fixed-cost running sum of squares** (chosen): the library
  already has exactly this shape for HR2/HR3 — band-pass → `Decimator` (target 50 Hz, §5.3.1;
  since v0.103 a 100 ms `WindowBlockClock` summing AC² at the full rate — §12 below) →
  circular buffer — the only difference being that HR2/HR3 need a slow/async Task (Task B/C) to
  run their autocorrelation (N × lags multiplications) or FFT on that buffer, whereas PI's "AC RMS" is just
  `sqrt(sum_of_squares / N)`: updating the running sum when one decimated sample enters and
  another leaves costs the same whatever the window length, so PI needs no Task at all — the fast path commits `AFE4490Data::pi`
  directly. Buffer: 300 floats (6 s @ 50 Hz) ≈ 1.2 KB for one channel (PI only needs IR, per
  existing clinical convention — unlike R, it is not a ratio of two channels). This settles
  EXACTLY at 6 s (the window is either full or it is not — no asymptotic tail) instead of needing
  ~18 s for an EMA with the same nominal averaging time to be considered converged.

**Known trade-off, accepted, not mitigated.** A true (rectangular) sliding window has a
"boxcar edge" artefact an EMA does not: when a single large sample (a motion artefact) exits the
window, PI can show a small instantaneous step rather than a smooth decay. Considered but not
built: a Hann/triangular-weighted window (same technique HR3 already uses spectrally) would
taper this, at the cost of a more complex incremental update (no longer a plain `sum += in − out`)
for a benefit that has not been observed to matter in practice. If it does, the window shape is
the one parameter to revisit — the decimation/buffer architecture does not need to change.

### Scope boundaries, by design (not oversights)

- **No `$SET`/`$LCFG` exposure yet.** `setPIFilter()` (mirroring `setHR2Filter()`) and the third
  parameter of `setSpO2SqiPiThresholds()` exist and `getConfig()` reports `pi_bp_f_low_hz`,
  `pi_bp_f_high_hz` and `spo2_pi_max`, but wiring them into the firmware's `$SET`/`$LCFG`
  commands and `pulsenest_lab.py`'s LIB CONFIG window is a separate, cross-repo task (PulseNest
  fw + spec), out of scope here.
- **No dedicated `TimingStat` for PI.** `_pi_update_sample()` is cheap (fixed cost per sample, same
  order as the EMA it replaces) and is not instrumented under `INCUNEST_TIMING_STATS` the way
  SpO2/HR1/HR2/HR3 are; add one if PI's cost is ever in question.
- **The reset while the probe is not applied stays unconditional (PI, HR2, HR3).** Alex asked
  whether the per-sample `memset` of the PI buffers (now 2 × 1.2 KB) was negligible and, if not,
  proposed gating it on the previous `probe_state`. Estimate: ~300 32-bit stores per buffer,
  2–3 µs each on the ESP32-S3; HR2 (1.6 KB) and HR3 (2 KB) already do the same, so the three
  together cost ~10 µs of the 2000 µs sample budget at 500 Hz (0.5 %; 1.5 % at 1600 Hz), only in
  the no-probe state, and HR2/HR3's share is already visible in `$TIMING` through `_ts_hr2`/
  `_ts_hr3`, where it has never stood out. Decision (delegated to the session): **no guard**. An
  exact one exists without storing any previous state — `buf_count == 0 && decim.phase == 0`
  ⟺ nothing processed since the last reset — but it is an implicit invariant that a future member
  added to the state and not to the condition would break silently, leaving stale state across a
  disconnection. The unconditional reset is trivially correct; 0.5 % of idle CPU does not buy a
  new way to be wrong in a medical device. If `$TIMING` ever says otherwise, the guard is one line.
- **The window length (`pi_window_s` = 6 s, `pi_buf_len` = 300) has no runtime setter**, same
  treatment as `hr2_buf_len` (8 s) — a fixed buffer capacity, not a tunable in the current design.
- **SpO2's SQI formula gained an explicit `isnan(pi)` guard.** Previously `fmaxf`/`fminf` happened
  to never see a NaN `pi` (the old EMA-based PI was effectively always a finite number once a
  probe was applied, even before SpO2's own warmup cleared). Decoupling makes PI's own 6 s
  warm-up externally observable as NaN, and `fmaxf`/`fminf`'s NaN-passthrough behaviour would read
  that as SQI = 1 (the opposite of correct) rather than SQI = 0 — fixed with an explicit check.
  In production this window never actually triggers the guard (PI's 6 s is shorter than SpO2's
  18 s, so PI is already valid by the time SpO2's own gate opens), but the guard protects any
  caller that feeds the two functions out of lockstep, which decoupling now makes possible for
  the first time. Existing tests in `test_spo2.cpp` had to be updated to feed PI in lockstep with
  SpO2 (same IR samples, same `probe_state`) precisely because SpO2's SQI still reads
  `AFE4490Data::pi` — the two are decoupled at the *algorithm* level, not at the *SQI-consumer*
  level; `spo2_sqi`'s dependency on PI is unchanged.

---

## 12. Averaging of R/SpO2 — from EMA to a sliding window; window lengths (2026-10-06 → 2026-10-07)

**Question (Alex, 2026-10-06).** PI v0.101 (§11) already averages over a true 6 s window instead of
an EMA, because the EMA's settling was too long. R-METHOD-1 (AC²) and R-METHOD-2 (Σdx·dy, Σdx², Σdy²)
still used EMAs with τ = `spo2_ema_var_tau_s` = 6 s. Should R/SpO2 follow? Analysed in depth before
deciding; the measurement scripts live in PulseNest
`captures/sessions/20260923_HOSPNAV_SUBJ08_ANALYSIS/r_averager_test.py` and `r_averager_test2.py`
(summaries `r_averager_summary*.txt`), built on `r_causal_test.py` (same causal chain as the
library: 2nd-order band-pass 0.5–5 Hz forward only, DC = EMA 2 s, first differences; only the
averager of the products changes).

**Prior art first.** Manufacturers speak in windows: Masimo 2–16 s selectable, 8 s NICU default;
Nellcor "Normal" 5–7 s; Renesas OB1203 a 4 s window plus a 5-point moving average. Nonin is the
exception — exponential, but per beat (4 beats). The neonatal averaging-time studies (Ahmed 2010,
Vagedes 2013/2014, McClure 2016, §4.5) reprocess with windows. An EMA's τ is not comparable with
any of those numbers without conversion.

**Theory, then confirmed.** For white noise an EMA with time constant τ has the noise-equivalent
bandwidth of a rectangular window of length **2τ**, and its group delay at low frequency is τ,
against T/2 for a window of length T. So "EMA τ = 6 s" ≡ "window 12 s" in both noise and delay;
the only differences are the settling (EMA: 63 % at τ, 95 % at 3τ = 18 s, never exact; window:
exact at T) and the memory shape (exponential tail vs. finite). Consequence: the library's declared
6 s averaging was in effect 12 s — slower than the 8 s NICU default and *outside* the 6–10 s of ISO
80601-2-61:2026 Annex JJ.2 d) the number had been taken from (§4.5). That, more than the settling,
is the finding of this section.

**Measured (3471 stationary pairs SUBJ08/09 against the ward monitor, curve 123.98 − 39.13·R).**

| Averager | Scatter vs reference [pts] | Noise of R (SD in 8 s) [pts] | Signed lag vs monitor | Return to ±1 pt after a clean RF-step burst |
|---|---|---|---|---|
| EMA 6 s (v0.101) | 1.79 | 0.25 | −1.5 s | ~17 s |
| window 12 s | 1.67 | 0.24 | −1.0 s | 7–8 s |
| window 6 s | 1.71 | 0.34 | −3.5 s | 2–3 s |
| EMA 3 s | 1.68 | 0.32 | −4.0 s | — |
| window 4 s | 1.66 | 0.38 | −5.5 s | — |

- Accuracy against the reference is identical for every averager (leave-one-subject-out too: median
  |error| 1.0–1.4 points for all). The averager does not change what R measures.
- Noise confirms the theory: EMA 6 s ≈ window 12 s (0.25 vs 0.24), window 6 s ≈ EMA 3 s (0.34 vs
  0.32), the ×1.36 against the EMA of equal nominal length is the predicted √2.
- Lag (our SpO2 leads the hospital monitor, which averages too): the 6 s window leads the 6 s EMA by
  ~2 s (theory 3 s, 0.5 s resolution). Only sessions whose reference moved ≥ 6 points can resolve it.
- Transients: the one clean HGAC RF step of SUBJ09 (session 1238, step 3) put a −9 point burst into
  R; the 6 s window purged it in 2–3 s (the burst preceded the `cause=hgac` record by ~3 s), the 12 s
  window in 7–8 s, the EMA decayed for ~17 s (3τ). Price of the window: a larger peak deviation
  while the burst sits inside it (median of the maxima 6.6 vs 4.8 points) — the "boxcar edge" of
  §11, accepted again. The other steps were useless for this: HGAC moves RF when the signal is
  already moving and the post-step level is not a level.
- The r gate (r < 0.8) never fired in these two recordings with any averager — it does not
  discriminate here; measuring r without a pulse remains pending (memory, step-3 tasks).

**Decision (Alex, 2026-10-07).** Sliding window for R-METHOD-2's three products and R-METHOD-1's
AC² (so a method comparison compares estimators, not averagers), length `spo2_window_s` = **6 s**;
PI stays at 6 s. The two lengths are **independent parameters** (`setSpO2WindowS()`,
`setPIWindowS()`), runtime-settable, **clamped to 2–12 s**: 4–12 s is the recommended range for an
incubator (8 s NICU default, literature pushing shorter, 12 s the longest outside the 16 s
oxygen-target trials), and the 2 s floor keeps McClure 2016's "2 s + 15 s alarm delay"
configuration reachable once the integrator has an alarm layer. `spo2_warmup_s` is removed: the
window is the warm-up (first value when full, exact from then on; the 2 s DC EMA is seeded on the
first sample and adds no step). `spo2_ema_var_tau_s` is removed with it; the SpO2 `EmaChannel`s
keep only the mean.

Why 6 s and not 8 or 12 (the options put to Alex): 12 s would have been "today without the hidden
12 s" — identical noise and delay, exact settling, no behavioural change; 8 s the NICU default
(+22 % noise, 4 s delay). 6 s was chosen for the neonatal focus of this library, for coherence with
PI's existing 6 s, and because the range now makes the length an explicit, declared setting rather
than a constant — the age-group profile planned for later (adult / neonate parameter sets) is the
place to revisit it.

**Implementation choices (and the alternatives rejected).**
- *Block sums at the 50 Hz cadence, products at full rate* (chosen; the shape of PI's DC in §11;
  the cadence itself was revisited the next day — see "Block cadence" below):
  dx·dy, dx², dy² and (x − mean)² are formed at the AFE rate and summed over each decimation block of
  a new `_spo2_decim` chain; each block sum enters a circular buffer with a fixed-cost running total. A sum
  of block sums is the full-rate sum — nothing is lost; the window edge moves in 20 ms steps and the
  outputs refresh at that cadence (the rest of the system consumes algorithms every 20 ms anyway).
  Rejected: a full-rate buffer (3 × 6 s × 500 Hz × 4 B = 36 KB, > 100 KB at 1600 Hz / 8 s, and a
  buffer that must be re-sized with the PRF — the decimated one keeps meaning seconds), and
  *decimating x, y before differencing* (ten times less CPU, but a 1.6 % attenuation of the derivative
  at 5 Hz and the decimator's anti-alias filter inside the estimator: a change to what was validated).
- *Drift guard:* float32 running add/subtract sums accumulate rounding; the totals are rebuilt from
  the buffers every time the index wraps (5·N adds once per window length — negligible).
- *Buffers sized for the maximum* (`alg_window_buf_len_max` = 600 = 12 s at 50 Hz; seven buffers ≈
  17 KB static — 120 and ≈ 3.4 KB since v0.103): no allocation, no re-sizing; the live length is
  `round(window_s · rate_hz)`, capped.
- *A length change empties the window* (NaN for one length) rather than keeping the newest samples:
  simpler, and a half-filled window of another length is not a valid estimate.
- *Not done, deliberately:* a Hann/triangular window (same reason as §11: tapering the boxcar edge
  costs the fixed-cost update for a benefit not yet observed); an EMA with τ = 3 s (noise-matched to the
  6 s window but still 3τ to settle and an infinite tail); exposing the lengths as `$SET`/`$LCFG`
  keys (firmware/lab work, a task of its own — the library version in `$CFG` identifies the
  defaults in force).

**Known costs.** +36 % noise of R at equal nominal length (0.34 vs 0.25 points median SD; p90 1.5 vs
1.1); larger peak deviation during a burst (above); at 30 BPM a 4 s window holds two beats and a 2 s
window one — R and r still come out, noisier; documented, not blocked (the age profile may couple
window and HR range later). **Advantage only the window has, for later:** blocks flagged as artefact
(motion, RF settling) can be *excluded* from the sums instead of frozen or waited out.

### Block cadence: 50 Hz → 100 ms blocks (v0.103, 2026-10-08)

**The question (Alex).** Why do the SpO2 and PI windows use 50 Hz and not another cadence?

**The honest answer: 50 Hz was inherited, not derived.** PI (v0.101) copied the block structure of
HR2/HR3's `Decimator`s, and the SpO2 window (v0.102) copied PI's. In HR2/HR3 the 50 Hz has real
signal-processing reasons (spec §5.3.1: Nyquist of the 15 Hz low-pass, FFT resolution, the GCD of
the PRF grid) because the decimator sits in the signal path and discards samples. In the windows
nothing is discarded: the products are summed at the full rate and the clock only decides how often
a block sum enters the buffer. None of §5.3.1's arguments apply, yet the spec and the header cited
it — and using the same `Decimator` at the same rate suggested a functional relation between the
windows and the HR chains that never existed. The cadence had to be chosen on its own merits and
named for what it is.

**What the cadence really determines**, and what each criterion says:

- *Divisibility of the PRF grid* — the block is a whole number of samples, so for the window to mean
  exact seconds at every PRF of the validated grid {500, 800, 1000, 1250, 1600} the block rate must
  divide their GCD, 50 Hz: {50, 25, 10, 5, 2, 1} Hz. A ceiling, not a value.
- *RAM* — buffers = window max × rate: 17 KB at 50 Hz, 8.4 KB at 25 Hz, 3.4 KB at 10 Hz. 50 Hz is the
  most expensive member of the allowed set; none of them is a problem on the ESP32-S3.
- *Output refresh and window-length resolution* — 20 ms vs 100 ms on a 6 s window: indistinguishable
  clinically and in our measurements (the consumer drains every 20 ms, SpO2 moves on seconds).
- *Granularity of a future block exclusion* — the only criterion that speaks about what the window
  will do with its blocks, so it is the one that fixes the value (Alex, 2026-10-08). But applied as
  "the finer the better" it lands back on 50 Hz, the ceiling. The usable form is **the coarsest
  block whose edge loss on an excision is negligible**: excising an artefact discards at most two
  partial blocks of good signal, 2·block/window.

| Block rate | Block | Loss, 6 s window | Loss, 2 s window | RAM (7 buffers) |
|---|---|---|---|---|
| 50 Hz | 20 ms | 0.7 % | 2 % | 17 KB |
| 25 Hz | 40 ms | 1.3 % | 4 % | 8.4 KB |
| **10 Hz** | **100 ms** | **3.3 %** | **10 %** | **3.4 KB** |
| 5 Hz | 200 ms | 6.7 % | 20 % | 1.7 KB |

Two physical anchors: the shortest cardiac cycle of interest is 250 ms (240 BPM neonatal), and the
block should stay clearly below it so an exclusion is not quantised to beats — 100 ms does, 200 ms
does not; and real exclusions will last seconds, not tens of ms (after an HGAC RF step the 2 s DC
EMA contaminates R for several seconds; a motion artefact lasts 0.3–2 s), so 100 ms granularity is
invisible and 20 ms is precision nothing will use. **Chosen: 100 ms (10 Hz), the same constant for
SpO2 and PI** (`alg_window_block_s`). 25 Hz was the alternative if margin on the 2 s minimum window
mattered; below 10 Hz the sub-beat scale is lost.

**Consequence for PI.** PI's AC² buffer held ONE band-passed sample per 20 ms block — a true 50 Hz
decimation of the 0.5–15 Hz band, acceptable at 50 Hz (Nyquist 25 Hz) but aliasing at 10 Hz. It now
sums AC² at the full rate over each block, like SpO2's products: the exact windowed RMS, same
expectation, lower variance. The DC buffer went from block means to block sums for symmetry (both
sums divide by the samples in the window).

**Naming.** `Decimator _spo2_decim/_pi_decim` and `*_decim_target_rate_hz` are gone; a `WindowBlockClock`
struct (configured by DURATION, `len` samples per block) replaces them, so the word "decimation" no
longer appears near the windows and §5.3.1 carries a scope note.

**`step()` → `tick()` (v0.111, 2026-10-27).** Alex proposed renaming `WindowBlockClock::step()` to
`completed()`. Rejected: the call advances the clock (`++phase`, reset at the block end) and returns
true once per block, so it is a command with a result, not a query; a participle reads as a
side-effect-free, `const` predicate, and called twice, or from a log line or an assert, it would
shift the blocks silently. A verb keeps the mutation visible; `tick` is a clock's verb, and it also
separates the two structs by name, as they were separated by concept on 2026-10-23
(`Decimator::step()` discards samples; the block clock discards nothing). Splitting into `tick()`
plus `completed() const` with a `fired` flag was considered and not taken: one more state for
`reset()`/`configure()` to clear and two calls per sample, for a three-line struct. No consumer
outside the library names the method (the lab mirror copies `configure()` and the block length).

**Measured (SUBJ09 HOSPNAV replay, v0.102 vs v0.103, 29 946 common samples):** R mean difference 0,
SD 0.0006 (max 0.006, the window edge now moving in 100 ms steps), r SD 0.0002, SpO2 SD 0.02 points
(max 0.19), PI SD 0.02 % (max 0.31 %), median 8 s SD of R 0.00510 → 0.00506. The first estimate
comes one block after the window is full: 6.1 s instead of 6.02 s. Host suites 10/10; the lab
mirror (`SpO2TestCalc`, lab v1.87) agrees with the library to R 8e-6, PI 5e-3, identical validity.

### What an RF change does to the window — measured (2026-10-08)

**The question (Alex).** The block-cadence argument above said that "after an HGAC RF step the 2 s DC
EMA contaminates R for several seconds". OT_LED was designed to be gain-invariant, so why would an RF
step show at all? Measured rather than argued, on the MS100 simulator (board 82:5C, lib v0.103,
HGAC off, `$SET tiagain1/tiagain2` 50K → 25K → 50K → 10K → 50K per colour, 30 s per step; the
scripts `rf_step_bench.py` / `rf_step_analyze.py` live in the session scratchpad, the numbers here).

**OT is gain-invariant up to the real RF tolerance.** The level step of the stepped colour's OT is
−0.9 / +0.8 % (IR) and −1.0 / +0.9 % (RED) between 50K and 25K, −4.4 / +4.5 % (IR) and −4.5 / +4.7 %
(RED) between 50K and 10K; the other colour moves < 0.2 %. That is the ±7 % feedback-resistor
tolerance of the datasheet (§2, `reference_rf_tolerance_ot_dc_shift`), tap by tap: one HGAC move is
one tap, so the operational case is ≈ 1 % on this chip (other tap pairs may differ, up to the
datasheet bound).

**The mechanism is the band-pass, not the DC EMA — and the duration is the window.** The
switched-RC settling freeze (~20 ms) holds the input flat while the front-end settles, but cannot
remove the level step that follows it. The 0.5–5 Hz band-pass turns that step into a ~1 s transient
whose amplitude is comparable to the pulsatile AC (PI 4.7 % on the simulator); its first differences
weigh in Sxx/Sxy for exactly as long as their blocks stay in the 6 s window and then drop out at once:
at 8 s after the step nothing is left (R within 0.007 of the baseline). The 2 s DC EMA only
normalises x and y and plays no part in the duration. The sentence above was therefore wrong about
the mechanism; the "several seconds" is the window length.

**Magnitude (MS100, R ≈ 0.61, SpO2 ≈ 100, Medle curve slope 39 points per unit R):**

| Step | OT step | ΔR (6 s) | Δr | ΔSpO2 (6 s) | ΔPI |
|---|---|---|---|---|---|
| one tap (50K ↔ 25K) | ≈ 1 % | 0.005 – 0.013 | −0.003 … −0.009 | −0.2 … −0.5 pt (clamp at 100 hides the other sign) | within noise (±0.2 %) |
| two taps (50K ↔ 10K) | ≈ 4.5 % | 0.03 – 0.09 | −0.05 … −0.15 | −2.2 pt, or invalid for 6 s (curve > 103, `spo2_clamp_margin`) | +0.6 … +0.75 % |

The effect scales with the step-to-AC ratio: with a 1 % PI (a neonate) the same one-tap step is
~4.7× larger relative to the pulse, ΔR ≈ 0.05 ≈ 2 points for 6 s. A synthetic step through the lab
mirror (`rf_step_model.py`, PI 2 %) reproduces the shape: 1 % → ΔR 0.04, 7 % → 0.35, 6 s flat then
gone.

**Not mitigated yet.** Two candidates, Alex's decision pending: (a) exclude from the window the
~1 s of blocks that follow an RF change — the 100 ms block granularity chosen above exists for
exactly this; (b) rescale the band-pass and DC state by the step factor in `_hgac_change_rf()`
(exact for RF, §2 / the O8.3 compensation). PI's jump after an HGAC move (§11, pending) has the same
cause.

**Follow-up, constant-light bench (2026-10-08, same day).** Alex covered the LEDs of the three probes
so the photodiode sees ambient light only, equal in the four phases: `V_TIA = I_amb · RF_nominal ·
(1 + ε_tap) + V_off`, measurable tap by tap with no pulse in the way (PulseNest
`tools/rf_tap_bench.py`, `tools/rf_tap_analyze.py`, reference CSVs in `docs/rf_taps/`). Two things
the simulator run could not show: an **additive offset** of the chain, per chip (+1.0 / −5.0 /
−6.6 mV, identical in the four phases), which dominates the low taps and must be fitted before any
tap comparison; and an **ε pattern nearly identical on the three chips** (10K +0.2, 25K −0.4,
50K −0.3, 100K −0.7, 250K +1.1, 500K 0.0 %), i.e. a systematic deviation of the design, far inside
the ±7 % bound, rather than per-chip scatter — which makes a library constant a candidate against a
per-board calibration. To measure the settling the datasheet only bounds (3 ms per phase), v0.104
adds the bench switch `afe_settle_freeze_enable` (spec §5.8.4): with the freeze on, the `$M4` rows
of the countdown repeat the pre-change values and the transient cannot be seen; a dedicated
raw-dump frame was the alternative and was rejected as more firmware for the same information.
Default true; never false in clinical use. The report of the bench lives in PulseNest
`docs/rf_taps/`.

---

## 13. HGAC under LED-only saturation — from "gate closed" to "slow step-down only" (2026-10-17)

**The problem.** `PROBE_ONLY_LED_SATURATING` (LED phases clip, ambient phases clean) has two
physical causes that RSQM cannot separate: the probe in air (the LED's light reaches the
photodiode with nothing in between) and tissue so thin that it saturates the TIA at the current
RF. Since v0.61 the state was "absent", and the HGAC gate (G0) closed on it, so that a removed
finger did not drag RF to the floor and ruin the operating point for the reattach. The spec
recorded the other half as an open check from v0.90: thin tissue at a high RF → absent → HGAC
off → nobody lowers RF → still saturated → still absent. Alex reproduced it on the bench on
2026-10-17: a V18 left at RF 500K, finger on, both LEDs clipping, APPLIED never reached, HGAC ON
did nothing, only a reset cleared it. The same day's audit of the hospital captures showed two of
four subjects recorded with `hgac_enable=0` all day (SUBJ08 on 23-09, SUBJ10/11 on 13-10), which
is how a board arrives at a stale RF in the first place.

**Options weighed.**
1. *OT discriminant inside the RSQM branch* (the spec's own proposal): "an OT far below
   `rsqm_ot_thr` under saturation still says tissue". Rejected on arithmetic: the OT computed from
   a clipped TIA is bounded by FS/(RF·I_LED); at 500K and 50 mA that is ≈ 2.4 µA / 50 mA ≈ 48 ppm,
   below the 150 ppm threshold **whatever** is in the optical path — air and tissue read the same.
   The discriminant only separates the two cases at low RF, where the deadlock does not occur.
2. *Open the gate fully* (treat the state like AMB_SATURATING): the fast guard walks RF to the
   floor in ~1.6 s on every finger removal; v0.61's reason for closing the gate was exactly this.
3. *Half-open: step-down only, slow branch only* — chosen. The fast guard exists for real light on
   a patient and is off here; the raise branch cannot fire under saturation and is off for
   symmetry; the slow leveling branch (τ 2 s, valid after 6 s, reset after each step) lowers RF
   one LUT step per ~6 s. Consequences: a removal shorter than ~6 s moves nothing (the common bench
   case); a persistent clip, whatever its cause, walks down until it clears; a long removal ends at
   the floor and the ordinary leveling climbs back at the same cadence after reattach (≤ 5 steps,
   ~30 s). A system that can take 30 s to recover beats one that never recovers.
4. *A time-qualified fast guard* (act after N s in the state): equivalent to 3 but adds a parameter
   with no measured basis; the slow EMA's warm-up already is that timer.

**What did not change.** `isProbeAbsent()` keeps the state: RSQM, the diagnostics task and every
algorithm still treat it as no patient. Only `_hgac_update()` makes the exception, documented
beside the function and in the ProbeState comment. The library default `hgac_enable = false` is
untouched (the clinical firmware never runs this path unless it opts in).

**Operational companion (PulseNest, not the library).** The deadlock needs a stale high RF, and a
board gets one when a host turns HGAC off and does not turn it back on (LAB CAPTURE's "Disable HGAC
during capture", restored only if the capture closes normally). The recorder now reads
`hgac_enable` from the board's `$LCFG`, shows **HGAC OFF** in red in the window and in `status`,
and notes the moment in the raw stream; it still writes nothing to the board. Open: whether the
firmware should re-arm HGAC on a recorder session start — rejected for now, the host-writes-to-
board rule of 2026-09-10 stands.

## 14. Naming the PI definition, and the date in an R-curve label (2026-10-20)

**The problem.** Between v0.100 and v0.101 the PI changed definition (EMA-based RMS → band-passed
RMS × 2√2, peak-to-peak units) and the same subject's PI went from a median 1.66 to ≈ 3 (HOSPNAV
SUBJ08). Nothing in a capture recorded which definition produced its PI column; the SQI thresholds
(0.5 / 2.0 %), `spo2_pi_max` and any comparison with a published neonatal threshold (Masimo
peak-to-peak, e.g. Granelli's p5 of 0.70) silently depend on it. R had the same problem and solved
it in v0.98 with `R-METHOD-n`.

**Options considered.**
1. *An id, by the R-method rule* (chosen): `PI_METHOD_ID` in the library, `pi_method_id` in
   `AFE4490Config`, `$CFG` and the capture snapshot; a registry table in the spec. Changes only when
   the number changes for the same input; parameters already travel in the snapshot. Cost: one
   constant, one field, one key. No downside found.
2. *Infer from `lib=` in the capture header.* Works for the past (≤ 0.100 → PI-METHOD-0, 0.101+ →
   PI-METHOD-1) and is what readers do for old captures, but a version number is not a definition:
   a future change that leaves the version bump to something else would be invisible. Kept only
   as the fallback for captures that predate the key.
3. *A setter and alternative methods in parallel, as R has.* Rejected for now: R got a second
   method because a measured problem (5.07 → 1.6 points of scatter) justified it; PI's open
   problems — the 25–250 % excursion for 6–14 s after an HGAC RF change, the biquad's 3–9 % ripple
   at 60–90 BPM, the 2√2 sinusoid assumption against a measured crest factor of 2.8–3.2 (up to −13 %
   versus a per-beat Max−Min; the MS100 reads PI = 1.47 × its Pulse Amp setting) — are properties
   of this method to fix or characterise, not a choice between methods. A per-beat peak-to-peak
   median (Masimo/Nellcor style) was tried offline on 2026-10-05 with an ambiguous result
   (variability 28 → 42 % in the subject who moved, 8.7 → 4.5 % in the one who was still); it stays
   a lab option until it improves something measurable.

**The intermediate state** of 2026-10-05 (band-pass 0.5–5 Hz, window DC, no 2√2) gets no id: it
never reached a board or a capture, and ids exist for definitions that produced data.

**The date in an R-curve label.** `R-CURVE-STS0163-HOSPNAV-20260923` was used twice: for the
discarded R-METHOD-1 proposal of 2026-09-24 and for the R-METHOD-2 fit of 2026-10-04 that ships.
§9 argued that (id, method) identifies a curve; the PulseNest register (`docs/spo2_r_curves.md`)
showed that a label that needs a second field to be unique is a weak label. Rule from 2026-10-20:
the date is the date of the fit, and the capture's date goes in the register row. The shipped
curve is relabelled `…-20261004` with identical coefficients; readers of captures keep matching the
old string.

## 15. The upper SpO2 bound — clamp margin 3 → 10 % (2026-10-20)

**The problem.** The library publishes SpO2 = a − b·R only while it lies in [`spo2_min`,
`spo2_max` + `spo2_clamp_margin`]; between 100 and the margin it is clamped to 100, beyond it the
output is NaN with SQI 0. The margin was 3 % with no recorded provenance. With the shipped curve
(123.98 − 39.13·R) the reject sits at R = 0.536.

**Measured.** Three adult-finger sessions of 2026-10-18 (SUBJ01, three Medle ST-S0163-26 units,
r 0.999, PI 7–9 %, HR3 valid 96–99 %): SpO2 withheld 1.3 / 2.3 / 3.9 % of the probe-on time, in
99 % of those samples because a − b·R exceeded 103; the r and PI gates never fired. Episodes of
0.7–20 s. In the same sessions a − b·R lay between 100 and 103 (clamped) 17–27 % of the time:
the neonatal-foot curve reads +3 pt on an adult finger (SUBJ01 bias analysis), so a true 99
computes as 102 and a true 100 as 103, exactly on the edge. On 2026-10-20, at rest with warm
hands, Alex saw the value disappear repeatedly. The MS100 showed the same on 2026-10-19: at
setpoint 100 % R was 0.48–0.53 (103–105) and SpO2 was invalid 51–99 % of the plateau.

**What the reject was punishing.** A calibration offset, not a signal defect. Every quantity the
library uses to judge the signal (r, PI, HR3's harmonic comb) was at its best in those samples.
The upper bound cannot distinguish "the curve is 3 pt high at this site" from "R is wrong": both
look like a − b·R > 100, and the first is expected whenever the probe is on a site other than the
one the curve was fitted on (§9: one curve, ~1 pt per site; the adult finger shows 3 pt with the
reference confounded).

**What manufacturers do.** Nellcor and Masimo display 100 for any R at or beyond the top of
their table; neither withholds a reading for being "too saturated". The ISO 80601-2-61 accuracy
claim is stated over 70–100 %; above 100 there is nothing to be accurate about.

**Options considered.**
1. Keep 3 % and fit a curve per site — rejected: the register (`docs/spo2_r_curves.md`) argues
   for one curve, and the dropout would return with every new probe lot or LED current (λ shift).
2. Remove the upper reject entirely (clamp always) — rejected by a small margin: a − b·R far above
   100 (R < 0.4 with this curve) has no physiological reading and is worth flagging as a
   configuration or estimator fault rather than reported as 100.
3. **Widen the margin to 10 % (chosen).** a − b·R up to 110 (R ≥ 0.357) is published as 100;
   beyond that the output stays NaN. 10 covers the observed offsets (3 pt) with room for a lot or
   current shift, and sits well outside anything the fitted curves produce at 100 % (R 0.48–0.61
   across the register). It is a sanity bound, not a signal gate; the signal gates remain r
   (`spo2_r_method2_reg_corr_min`) and PI (`spo2_pi_max`).

**Consequence.** On the 2026-10-18 sessions the change removes every dropout (max a − b·R was
106.4). It does not change any value below 100. Reverting is one constant. `pulsenest_lab.py`
mirrors it (`SPO2_CLAMP_MARGIN`).

## 16. Naming the R-METHOD-2 correlation: `spo2_r_method2_reg_corr` (2026-10-20)

**The problem (Alex).** Reading the source, R (the ratio of ratios) and r (the correlation of
R-METHOD-2's regression) were indistinguishable: snake_case drops the case, and `spo2_r_corr` sat
under the same `spo2_r_` root as `spo2_r_method` and `spo2_r_curve_*`, reading as "the
correlation of R". In five consecutive lines of `getConfig()` four were R and one was r.

**Options.**
1. `spo2_reg_corr` (own root, no `r`). Unambiguous, but it moves the quantity out of R's
   namespace although it is a by-product of the R computation; §9's rule is one root per concept.
2. `spo2_r_m2_reg_corr` (Alex's first proposal): keeps the root and names the method. Rejected on
   one point: `M2` is a frame mode in this project (`$M1`–`$M4`); a column `R_M2_REG_CORR` inside a
   `$M4` frame reads as a frame mode.
3. **`spo2_r_method2_reg_corr` (chosen).** Root `spo2_r_` (it is R's), `method2` spelt like
   `spo2_r_method`, `reg_corr` says what it is. Rule: under `spo2_r_` only R and its own
   quantities; the correlation is never a bare `r_`. The method number in the name pins the
   quantity to the implementation that produces it, which is what the `R-METHOD-n` ids do anyway.

**Scope.** Library (`AFE4490Data`, `AFE4490Config`, setter, test accessor), firmware (`$CFG`
key, `$SET`, `$M4` field 36 `R_METHOD2_REG_CORR`), capture dictionary and CSV column, the lab's
mirror and buffers, the bias-analysis tool, tests, specs. Captures already written keep `R_CORR`
and `spo2_r_corr_min`: the readers take the old names as synonyms, as they did for `spo2a/b`.

## 17. The SpO2 artefact gate: HR3's pulse identification, withhold, no hold (2026-10-20 → 2026-10-22)

**The problem.** SUBJ10 (term neonate, moving, 2026-10-13) read −16 points against the Mindray N17
with R 1.1–1.2 at a reference of 97 %, and 81 % of its reference windows passed every gate the
library had: probe applied, r ≥ 0.8, PI ≤ 20 %, 70 ≤ a − b·R ≤ 110. r stayed at 0.96 — the
regression of red on IR fits motion as well as it fits a pulse. The spectrograms of 2026-10-16
separated the two subjects of that day: a clean harmonic comb in SUBJ11, none in SUBJ10; HR3 was
valid in 18 of 760 SUBJ10 windows against 1500 of 1664 in SUBJ11.

**Prior art.** Nellcor (US 6,083,172), Philips FAST (US 6,631,281) and Masimo SET (Signal IQ) all
tie SpO2 validity to the identification of the pulse and its harmonic structure, not to the red-IR
correlation alone, which is one of their several criteria. Nellcor: harmonic energy decreasing
with order, states Pulse present / Not sure / Pulse lost / Non-pulse → UPDATE / HOLD / CLEAR, a
displayed value never older than 40 s. Philips: FFT every 1 s, harmonic families scored, six
artefact criteria, the failing cycle discarded, INOP after ~10 s. Masimo: engines in parallel with
a confidence → Low SIQ shows the value dimmed, Pulse Search blanks it. HR3 (FFT + Harmonic Product
Spectrum, true-peak validation, §5.4) already computes the harmonic test.

**Measured (PulseNest `experiments/spo2_artifact_gate_20261020.py`, report in
`captures/sessions/GATE_ANALYSIS_20261020/findings.md`).** The 21 sessions with a reference
(SUBJ08, 09, 10, 11, the adult finger SUBJ01, the MS100) replayed with lib v0.108 in OT-input mode,
SpO2 recomputed with the shipped curve and the current validity rule, each candidate ANDed per
sample; 8 s windows against the VideoNest OCR (confidence ≥ 0.9), published when ≥ 25 % of the
samples pass.

| subject | gate | windows published | Arms | > 4 pt |
|---|---|---|---|---|
| SUBJ10 moving | today | 81 % | 16.3 | 92 % |
| SUBJ10 | **HR3 valid** | **2.6 %** | **2.1** | **0 %** |
| SUBJ11 preterm, still | today / HR3 valid | 98 / 93 % | 1.45 / 1.23 | 1.5 / 0.7 % |
| SUBJ08, SUBJ09 | today / HR3 valid | 97–100 / 78–80 % | 3.0–3.2 / 2.4 | 14–15 / 7 % |
| adult finger, MS100 | today / HR3 valid | ≥ 98 % both | unchanged | unchanged |

What the gate withholds in the quiet neonates was mostly bad: |bias| median 2.7 (SUBJ08), 3.8
(SUBJ09), 2.5 (SUBJ11), with 42 / 45 / 20 % of those windows beyond 4 points; what it keeps:
1.2 / 0.9 / 0.7 and 7 / 7 / 0.7 %.

**Candidates rejected.** r ≥ 0.99 still publishes 11 % of SUBJ10, all at −14 points, and already
costs 10–15 % of SUBJ08/09; PI ≤ 10 % keeps 76 % of SUBJ10 and cuts the adult finger (PI 7–9 %).
HR3_SQI thresholds (0.3 / 0.5 / 0.7) add nothing over validity: when HR3 passes the true-peak test
its SQI is already 1.00 at the median and below 0.5 in only 1.5–3.5 % of samples (quiet subjects).
HR2 agreeing with HR3 within 10 % gains a little more (SUBJ09 Arms 2.39 → 1.87) for 3–7 % less
availability — kept for later; HR1 agreement costs too much (SUBJ11 59 %). **Holding the last
value** (Nellcor HOLD, 5 or 10 s) hurts: HR3's 10 s buffer stays valid into the first seconds of
motion, so the last "good" value is already contaminated and the hold extends it (SUBJ10 publishes
7–10 % of windows again at −12 to −13 points).

**Costs.** (1) Desaturations coinciding with movement are withheld: in SUBJ08/09 availability with
reference < 90 % drops from 97–100 % to 69–73 % (−27/−28 points) against −16/−19 at ≥ 95 %.
(2) The first value after applying the probe: median 6.1 → 10.5 s, p90 6.8 → 36.6 s (10
applications ≥ 30 s), HR3's 10.24 s buffer. Neither is fixable inside this gate; a faster
criterion at onset is open.

**The two options and the decision (Alex, 2026-10-22).** A — withhold (NaN, SQI 0) with a reason
code, the Philips/Nellcor-CLEAR model; B — publish with a low-quality flag, the Masimo model. Both
satisfy ISO 80601-2-61:2026 201.12.4.102 (signal inadequacy indicator): note 6 says not posting
SpO2 under inadequate signals satisfies it, note 5 that sensed movement artefact may enter the
criterion, example 1 that a visual signal or message also does — but B puts the indicator on the
consumer's UI and adds the 30 s "not current" indication of 201.12.4.101. **A chosen**: the
library's consumer is the IncuNest firmware (display, record, alarms), not a person reading a
flag; a −16-point value on the wire would reach the alarm system unless every consumer filtered it.
Revisit with more moving subjects with a reference (n = 1 today).

**Design.** `spo2_pulse_gate_enable` (default true, runtime setter and `$SET`); the gate reads
`_current_data.hr3` (NaN while HR3 has no valid result; written by Task C under `_state_mutex`,
held by Task A in `_spo2_update()`); the SpO2 window keeps running, so SpO2 returns in the block
HR3 does; no hold. `AFE4490Data::spo2_hold_reason` names the first failing gate (probe, window,
eps, reg_corr, pi, range, pulse) — a field, not a DiagCode bit, because any DiagCode bit zeroes
RSQI and would invalidate HR and PI. Verified on the host: gate on / off, return in the next
100 ms block after HR3 returns, reason order.

**Also in v0.110 (Alex).** The locals of `_spo2_update()` renamed so R and r are told apart in
the source: `valid` → `eps_valid` (the window sums clear their epsilons: division safety, not
SpO2 validity), `r` → `reg_corr`, `r_ok` → `reg_corr_ok`. And the three PI display-range
citations behind `spo2_pi_max` = 20 % are now in the constant's comment and §10, with the open
question of where that ceiling really comes from (none of the three discloses its PI estimator).

**Why r cannot be the artefact gate, and two cheap separations measured (2026-10-27).** The
Masimo/Nellcor prior-art search (`docs/masimo_nellcor_spo2_prior_art.md`, §2) shows that for two
uncorrelated sources with ratios ra and rv the regression's r cannot fall below
`2·√(ra·rv)/(ra + rv)` = 0.954 for arterial 95 % plus venous 70 %, whatever the motion power, while
the error at equal power is −12.5 points: the 0.8 gate catches uncorrelated noise, never coherent
admixture (SUBJ10: r 0.96, −16 points). The two separations the report simulated on sums the window
already keeps were then run on the corpus (`experiments/r_estimators_ab_20261027.py`, 21 sessions,
lib 0.108 OT replays; the Python chain reproduces the library's R to 7·10⁻⁶): (a) Masimo's
decorrelation constraint (US 5,632,272, expired) solved over six 1 s sub-windows is ill-conditioned
91-98 % of the time and noisier where it acts (SUBJ08 Arms 2.42 → 3.46 under the gate; SUBJ10
without the gate 15.7 → 10.2 points median, 70 % still beyond 4); (b) R on ±1 bin around the first
three harmonics of HR3 (0.5-15 Hz band, Hann, 6 s or 10.24 s) is NaN without a valid HR3, so it
carries the pulse gate by construction, and under it equals or slightly beats the regression on the
still neonates (SUBJ09 2.39 → 2.07, SUBJ11 1.23 → 1.08, SUBJ08 2.42 → 2.42 Arms) while being worse
on the 15 SUBJ10 windows the gate lets through (2.05 → 4.62). Decision: nothing changes; the pulse
gate stays the artefact criterion; the harmonic estimator is a candidate second R only if a later
campaign confirms the 0.1-0.3 Arms gain; the decorrelation as formulated is not usable here.

### 17.1 HR3 — spectrum preprocessing and harmonic combination, measured and not adopted (2026-10-21)

**Context.** HR3 finds the pulse fundamental with the Harmonic Product Spectrum `P[k]·P[2k]·P[3k]`
on the raw power spectrum (§5.4) and publishes it only while `hr3_sqi > 0`. Alex (2026-10-21):
a product is a multiplicative AND, one harmonic in the noise kills the candidate, and the
near-sinusoidal preterm PPG at 160-180 bpm would make that common. Two independent improvements
were proposed and crossed on the recorded corpus before touching the library
(`experiments/hr3_spectral_variants_20261021.py` in PulseNest; report
`captures/sessions/HR3_VARIANTS_20261021/findings.md`):

- **A, preprocessing of the spectrum** (the floor under the peaks: respiration, motion below 1 Hz,
  broadband noise): A0 none; A1 subtraction of the local floor by convolution with a zero-sum
  kernel, `S = max(0, P ⊛ h)` — Ricker, difference of Gaussians (1:1.6), CFAR blocks — with a
  positive half width of 2 bins (the Hann main lobe) either constant or scaled by n for the n-th
  harmonic term; A2 whitening `S = P / median(P, ±8 bins)`.
- **B, combination of harmonics**: B0 `S[k]·S[2k]·S[3k]` (today); B1 `S[k]·S[2k]`; B2 Noll's
  Harmonic Sum Spectrum; B3 Hermes' subharmonic summation (weights 0.84^(n−1)); B4 floored product
  `Σ log(S[nk] + c)`.

Same spectra for all 40 variants; the library's interpolation, two-bin SQI and `sqi > 0` rule for
every one. The A0 × B0 mirror reproduces the runner's in-order replay exactly (same value within
0.05 bpm or same NaN in 100 % of 1939 windows, 3 sessions). References: the pulse rate typed with
each REF_SPO2 on an MS100 sweep 30-240 bpm (30-bpm plateaus of 90 s), and for the neonates the
consensus of HR1 and HR2 (both valid, within 3 %), because the ward monitor's pulse rate was never
captured.

**What was measured.**

1. *The premise does not hold on this corpus.* The neonates' 2nd harmonic sits 10-12 dB and the
   3rd 18-21 dB below the fundamental (adult −8/−12.5; MS100 −2…−4/−5.5…−9.5 dB): weak, not in
   the noise. A B4 whose floor was the noise median came out identical to B0 on every session —
   no harmonic the product needs is at the noise level. Today's HR3 is right in 89.5-92.5 % of the
   neonates' consensus windows and in 100 % of every MS100 plateau from 30 to 240 bpm (max error
   0.18 bpm). What it lacks is availability — 69-88 % of the neonates' windows against 99.8 % on the
   adult finger — and that is the `sqi > 0` rule withholding.
2. *Every floor-removing preprocessing fills that gap on clean signal* (Ricker/DoG at constant
   width or whitening: 97.6-99.7 % availability, 99-100 % correct on SUBJ08/09/11) *and on
   artefact*: SUBJ10 (moving, SpO2 −16 pt against the Mindray) has HR3 valid in 6.8 % of its
   windows today and in 57-92 % with any A1/A2 variant. Re-run as the 2026-10-20 SpO2 gate
   "HR3 valid": SUBJ10 published 2.6 % / Arms 2.0 with today's HR3, 57-81 % / Arms 15-16 with any
   preprocessed one — the no-gate numbers; SUBJ09 likewise returns to 99.4 % / Arms 3.2 / 15 %
   beyond 4 points. The windows the preprocessing adds back are the artefact windows. **The
   product on the raw spectrum with the SQI rule is the artefact detector the pending SpO2 gate
   relies on; removing the floor removes the detector.**
3. *Sums lose the subharmonic defence*, as reasoned beforehand: B2/B3 admit SUBJ10 windows with
   SpO2 Arms 10-11 even without preprocessing, and with whitening fall to 84-89 % correct (harmonic
   errors at 90-150 bpm on the MS100, octaves on the adult).
4. *Kernel width is bounded by the harmonic spacing.* Kernels scaled by n score 0 % at 30 and
   60 bpm with the product: the n-th term's negative lobes reach the neighbouring harmonic, 5 bins
   apart at 30 bpm. CFAR blocks fail at 30 bpm even at constant width. Whitening over ±8 bins
   drops to 87 % at 120 bpm.
5. *SQI.* With any A1 kernel and a product the two-bin fraction saturates at 1: a sparse spectrum
   has nothing outside the peak to divide by. The SQI would have to be re-derived for any A1
   variant; the artefact class available (10-18 windows per subject, since in motion HR1 and HR2
   go invalid rather than disagree) is too thin to design it.
6. *The one measured gain without a gate cost: the two-harmonic product `P[k]·P[2k]` (B1).*
   Availability on the clean neonates 69.4 → 76.0, 70.6 → 76.5, 88.3 → 94.7 %; harmonic errors
   2.5 → 0.9, 1.1 → 0.9, 4.7 → 2.4 %; MS100 100 % on every plateau; SUBJ10 gate 2.6 → 2.1 %
   published; gate Arms on SUBJ08/09/11 within 0.2 of today's. It would also retire the N/6 search
   cap's reason (3rd harmonic inside Nyquist → N/4). Hermes weights are the most accurate on clean
   signal (98-99 %) but let 7.5 % of SUBJ10 through at Arms 11.

**Decision.** Nothing from the A axis enters the library while "HR3 valid" is the artefact-gate
candidate (§17 above, adopted in v0.110). If a preprocessing is ever
wanted for availability, the artefact detection must be rebuilt first on another basis (Philips
FAST's harmonic-family criteria, Nellcor's harmonic energy decreasing with order). B1 is on the
table for Alex; not adopted in this version. Open: whether true preterm PPG has weaker harmonics
than these neonates (no gestational age in the sessions; the MS100 cannot emulate it), and the
monitor's pulse rate for the next campaign (VideoNest's `pr` field exists and stayed empty).

## 18. One R curve or two? Neonatal foot vs adult finger — the evidence and the alternatives (2026-10-25)

**The question (Alex, several times since 2026-10-19).** Should the library carry one R→SpO2 curve
for neonates and another for adults? Each time the answer was "no" (§15 option 1; register
`docs/spo2_r_curves.md`), on three grounds: ISO 80601-2-61:2026 201.12.1.101 d) validates paediatric
accuracy with adult desaturation studies and knows no neonatal curve; fetal haemoglobin does not
change the red/IR optics (Pritišanac 2021); Nellcor MAX-N runs one calibration on adult fingers and
neonatal feet (Arms 2.0 vs 2.5). The 3-point adult-finger offset of 2026-10-18 was attributed to the
reference (a home oximeter with a Nellcor-type adult curve, vs the Mindray in the NICU). The two
sessions of 2026-10-23 (three probes, four home oximeters, two phones) reproduced the offset, and
Alex's own R at rest, 0.54-0.56, gives 102-103 with the shipped curve: only `spo2_clamp_margin`
(§15) hides it. This section records what is known, what is not, and the alternatives, so the next
decision is taken on the numbers rather than on the 2026-10-19 hypothesis.

**What the data say — median R at the same displayed SpO2, every data set with a stationary range.**

| reference SpO2 | neonatal feet, SUBJ08+09, Mindray N17 (fit base, 3455 windows) | preterm 30 wk, SUBJ11, Mindray N17 (1121) | adult finger, SUBJ01 18-oct, home oximeter (1179) | adult finger, SUBJ01 23-oct, home oximeter (2022) | MS100 "Nellcor" table (SUBJ90) |
|---|---|---|---|---|---|
| 94 % | 0.757 | 0.766 | 0.712 | 0.705 | 0.692 |
| 96 % | 0.715 | 0.739 | 0.636 | 0.615 | 0.641 |
| 98 % | 0.650 | 0.665 | 0.580 | 0.585 | 0.590 |
| 99 % | 0.629 | 0.635 | 0.545 | 0.531 | 0.564 |

(Same probe model, same R-METHOD-2, same 8 s windows; PI in today's peak-to-peak units: neonatal
≈ 2.3-3.2 % — the 2026-09-23 captures recorded the pre-v0.101 RMS PI, 0.8-1.1, ÷2.8 of it, a units
slip corrected 2026-10-27 —, preterm SUBJ11 2.8 %, adult 3-8 %.) Three
findings. (1) The three Mindray-referenced neonates agree with each other within 0.02 and the two
adult sessions agree with the simulator's Nellcor table within 0.02: each convention is internally
consistent. (2) Between the two groups R differs by 0.05-0.10 at every level, mean 0.07 ≈ 2.7 points
at slope 39, with no trend over 94-99 %: an offset, not a slope. (3) The slope itself is the same
everywhere a range exists (39.1 neonatal fit, 39.1 simulator; the adult-finger bin fits are flatter
only because their range is 91-99). So the shipped curve is right for what it was fitted on and
reads 2.7 ± 0.4 high on an adult finger judged by Nellcor-convention references. Whether an adult
finger judged by a CO-oximeter would also say so is unknown.

**Where the 2.7 points can come from — candidates by sign.** The Medle on a neonatal foot gives a
HIGHER R than on an adult finger at the same displayed saturation. Candidates that produce that
sign: (a) the references: the Mindray in neonatal mode and the home oximeters/Nellcor table are two
transfer references, each specified at ±2-3 Arms against CO-oximetry (Mindray 518B/520N validated in
neonates at 2.38/2.88; home devices ±2-3 in adults, worse below 90); a 2.7-point difference between
two such references at the top of the range is within their combined uncertainty, so it cannot be
excluded, nor confirmed, with them; (b) venous pulsation in the neonatal foot (dependent, wrapped
tissue): venous blood pulsing adds a low-saturation component to the AC and raises R; it would also
affect the Mindray's own foot sensor, so the fit absorbs only the difference between the two feet;
(c) Philips states that the adult/neonatal patient category "optimizes the calculation of the SpO2";
if the Mindray applies a neonatal-mode correction its displayed value is not the adult convention
(not documented for the N17; module type unknown — Mindray uSpO2, Masimo or Nellcor are all sold).
Candidates that produce the OPPOSITE sign, hence cannot be the cause: differential path-length
ratio (Monte Carlo of a finger at 660/940 nm gives red/IR 1.25-1.45; thin tissue tends to 1, which
LOWERS R at the same SaO2 — register, Atuar/Nitzan 2026); regression dilution at low PI (attenuates
the neonatal R, so adults would read LOW); the Mindray's clipping at 100 (flattens the top of the
fit, lowering the intercept). Fetal haemoglobin and skin tone (all subjects light-skinned) are
negligible on the optics. One more constraint: every published neonatal validation of an
adult-calibrated sensor (MAX-N 2.5, Masimo 2.9, Mindray 2.4-2.9 Arms) is incompatible with a
sensor-level foot/finger shift of 2.7 points, since Arms contains the bias; so the tissue part of
our offset is most likely ≤ 1 point and the rest sits in (a) or (c). That is the 2026-10-19
hypothesis again, now with the sign argument behind it, and still not a measurement.

**Why "a curve per age" was and remains the wrong axis.** Age is not a variable the optics see.
What varies is the sensor (λ, geometry), the application site (tissue, venous component), the R
estimator and the reference convention. The register already states the curve as probe × site ×
method; the open question is whether SITE needs a second entry for this probe, and that question
is not decidable against transfer references that disagree with each other by the same amount.

**Alternatives.**

1. *Status quo: one curve, neonatal foot, Mindray transfer; adult finger out of its domain.*
   Pros: the product population is neonatal (IncuNest), the curve is confirmed on a held-out
   30-week preterm (−0.2 ± 1.2), it agrees with the ward monitor the clinicians act on, nothing
   changes. Cons: every adult bench session sits at the 100 % clamp (52-84 % of the time on
   2026-10-23), so the top of the range cannot be tested on an adult; the library tells nobody that
   the probe is on a site the curve was not fitted for.
2. *Two labelled curves, selected by site in configuration, never auto-detected* (neonatal foot =
   default; adult finger = `R-CURVE-STS0163-ADULTFINGER-…` or the MS100 Nellcor table
   `R-CURVE-STS0163-MS100-20261019`, same slope, intercept −2.9). Pros: matches the physics
   statement (curve per probe × site); the register supports it; adult bench work becomes
   meaningful. Cons: the adult curve is referenced to consumer devices (non-traceable, 91-99 % only:
   intercept known, slope assumed from the neonatal/simulator fits); a wrong selection at the
   bedside shifts SpO2 by 2.7 points in the NICU target band (SUPPORT/BOOST: 1-2 points changed
   outcomes); one more configuration surface to keep in `$CFG`, the capture and the spec.
3. *One curve plus a per-site intercept offset* (slope 39.1 shared, offset 0 for the foot, −2.9 for
   the finger). Same as 2 with one number instead of two; honest about what the adult data can
   estimate (an intercept). Same selection risk.
4. *Replace the default by the Nellcor-convention curve for everyone* (121.06 − 39.11·R). Pros:
   adult finger, simulator and home devices all agree with it. Cons: reads −2.9 against the ward
   monitor on the three neonates, i.e. the device would disagree with what the NICU acts on, in the
   band where the targets live, on the basis of a simulator table and consumer oximeters. Rejected.
5. *Runtime-selectable curve as tooling* (`$SET spo2_r_curve_a/b/id`, LIB CONFIG row, label in
   `$CFG` and in every capture; the library has `setSpO2RCurve()` already; boot default unchanged).
   Orthogonal to 1-4 and needed by 2, 3 and by any bench comparison; today the curve can only be
   changed by rebuilding (see 2026-10-25 log). Risk: the same mis-selection as 2, mitigated by the
   label travelling with the data and the boot default.
6. *Attribute first, decide after.* Cheap, no ethics, in this order: (i) read the N17's SpO2 module
   type off its menu or sensor connector (Nellcor vs Masimo vs Mindray) — one minute, changes the
   reading of (a)/(c); (ii) Alex's finger on the N17 adult sensor and a home oximeter at the same time
   (planned since 2026-10-19): if the N17 reads the adult finger like the home devices, the Mindray's
   adult convention is Nellcor's and the neonatal offset is site or neonatal-mode; (iii) the Medle on
   Alex's own foot (toe / lateral foot) with a home oximeter on his finger: site effect in one adult
   under one convention, free; (iv) the real answer, step 4 of the plan: arterial gases taken
   clinically in the NICU, paired with our R (GE neonatal protocol: 50 pairs are considered adequate).

**Recommendation recorded (not yet decided by Alex).** Keep the neonatal-foot curve as the boot
default (1), implement 5 now (bench need, no clinical downside while the default and the label are
in place), register the adult-finger/Nellcor-convention curve as a labelled bench curve, and run
6 (i)-(iii) before the next hospital visit. Whether 2/3 enters the library follows from 6: if the
N17 on an adult finger reads like the home devices, the offset is site or neonatal mode and a
site-selected second curve is justified; if it reads 2-3 points above them, the offset is the
Mindray's convention and one curve stays, with the knowledge that "agreeing with the ward monitor"
and "agreeing with the adult convention" are 2.7 points apart for this probe. The ISO path
(201.12.1.101 d), Annex DD) eventually produces an adult-calibrated curve for the Medle; the neonatal
transfer data then measure how far the NICU monitor sits from it.

**Correction to earlier sessions.** On 2026-10-19/20 the 3 points were presented as "most likely
the reference"; the honest state was and is "reproducible 2.7 ± 0.4 points, unattributed". The sign
argument above makes the reference the leading candidate, not the established cause.

**Measured 2026-10-27: the offset is not venous admixture that the processing can separate.** One
of the attribution steps of alternative 6 was to recompute R with estimators that discard the
non-harmonic content (`experiments/r_estimators_ab_20261027.py`, §17). The neonatal-foot vs
adult-finger intercept at the shared slope 39.13 is 3.25 points with the regression, 3.08 with the
sub-window decorrelation, 3.10 with the harmonic bins at 6 s and 3.19 at 10.24 s; the per-level
median R differences (0.04-0.05 at 94 %, 0.09-0.10 at 99 %) do not move. Candidate (b), venous
pulsation, survives only in the form no linear second-order method can separate — a venous
component locked to the heartbeat — and candidates (a), the reference convention, and the site
remain. The remaining attribution steps are the N17 module, the adult finger on the N17 and the
Medle on an adult foot (6 i-iii), and the SUBJ08 vs SUBJ09 slope question (task H, 2026-10-27).

**Learned 2026-10-27: the reference is Nellcor OxiMax.** The unit where SUBJ08-11 were captured
reports that every one of its N17s carries a Nellcor OxiMax SpO2 module (sticker on the module; not
verifiable from the screen frames). This does not change a single fitted number — the fit is against
the displayed value — but it changes what the curve means. Candidate (c) is reduced to the undocumented
possibility that the OxiMax neonatal sensor memory carries coefficients other than the adult clip's:
Nellcor's neonatal mode is documented as alarm management only (SatSeconds off, SPD unavailable), not
a change of the SpO2 computation. Candidate (a) shrinks from "two conventions" to "two implementations
of Nellcor's adult calibration" (the OxiMax module with its per-sensor RCAL bin, and the MS100's
"R-curve Nellcor" table; the home oximeter stays unidentified). The applicable neonatal validation is
Nellcor's (MAX-N: 42 neonates, 63 observations, Arms 2.5 at 85-99 %), not Mindray's 518B/520N. What
the 2.7-3.2 points can still be: the site under our probe, a bias of Nellcor's adult calibration on
neonatal feet (bounded by that Arms), or the home oximeter not being Nellcor-like. Attribution step
6 i is answered; step 6 ii (Alex's finger on the N17's own adult sensor and the Medle at once) now
compares against the very module the neonates were referenced to and is the decisive cheap test.
The reference's response mode (Normal 5-7 s / Fast 2-4 s) was not recorded; the 8 s pairing window
of the fit covers either.

## 19. Tests, version check and release notes in the library's own repository (2026-10-30)

**Problem.** Until 2026-10-30 the host unit tests lived in PulseNest (`test_lib/`), not here. The
published repository, the one IncuNest pins in `platformio.ini`, could not verify a tag by itself:
whether `v0.111` passed its tests was knowable only from a PulseNest checkout of the same day. On the
same day three strings that must agree did not (`library.json` at 0.104.0 with the library at 0.111;
the `.h`, `.cpp` and example header comments at v0.102, v0.104 and v0.99), and no tag had release
notes: a consumer moving from `v0.94` to `v0.111` had to read seventeen rows of §14 to learn which
`AFE4490Data` fields had been renamed.

**Decision (Alex, 2026-10-30).**

- Tests move to `test/` (spec §9.10). The name is load-bearing: PlatformIO's default source filter
  for a library excludes `test/`, `tests/`, `example/` and `examples/`, so IncuNest's `lib_deps`
  build never sees the suites' `main()` or Unity. A directory named otherwise (`test_lib/`) would be
  compiled into the firmware and fail to link.
- PulseNest keeps a thin wrapper with the same command as before (`test_lib/runner/run_host_tests.py`):
  it fetches the pinned library (`scripts/get_lib.py`) and runs `test/run_host_tests.py` with its
  build directory inside PulseNest. *Alternative rejected:* deleting `test_lib/` from PulseNest and
  running the library's script directly — one command fewer, but the habit, the README and the
  sessions' rules all name the PulseNest command, and the build directory would land inside the
  library working copy.
- `scripts/check_version.py` checks every version string against the others and against the tag;
  CI (GitHub Actions, Ubuntu, cmake + ninja + g++) runs it before the tests on every push and tag.
  *Alternative not taken:* a single source of the version (the others generated from it) — it would
  need a generation step in three build systems (PlatformIO, ESP-IDF, host CMake) for a value that
  changes once per tag; a check that fails is enough.
- Every tag gets a GitHub Release in a fixed short format (what changes / breaking / when
  upgrading). §14 stays the full history; the release is the consumer's summary.
- `v0.111` was already tagged without tests or CI. No new tag for this (not a functional change,
  versioning policy); `v0.112` is the first tag the CI verifies, and the CI was run by hand on
  master, whose library sources equal `v0.111`'s except for the three header comments.

**Date of the specification (2026-10-30).** The spec carried no date: it was only knowable from
`git log` of the tag, which a reader of a downloaded release or of a copy without `.git` does not
have. Two places were weighed. A date in the title ("Specification v0.111, 2026-10-29") is one more
hand-kept string, and the title had just drifted (v0.106 while the library was at v0.111) until
`check_version.py` caught it. The §14 table already has one row per version and is the place a reader
asks "when": it got a *Date* column, filled once from git (the tag's commit; before tags, the first
commit carrying the version, reaching into PulseNest's history for v0.1 to v0.17), and the first row
must carry a date or `check_version.py` fails. Version and date in the same table is the usual form of
document control. The title paragraph was rewritten at the same time: it still described the library
as Arduino-only and "coexisting with protocentral-afe4490-arduino", the state of April 2026.

## 20. `rsqm_ot_thr` — second calibration on four patients, 150 → 250 ppm (2026-10-31 → 2026-11-01)

**Question (Alex, 2026-10-31).** Is the 150 ppm threshold of v0.95, set on one patient the day after the
first hospital campaign, adequate now that four babies have been measured?

**Data.** All 18 HOSPNAV sessions of SUBJ08–11 (220 min, 6.6 M samples at 500 Hz), columns `OT_LED1`,
`OT_LED2` and `ProbeState` as the board recorded them (SUBJ08/09 on lib v0.94 with the threshold at
100 ppm; SUBJ10/11 on v0.104 at 150 ppm). Probe Medle ST-S0163-26, one unit. Stretches with the probe
taken off (ProbeState 4, OT clipped at 240/480 ppm) excluded: the saturation branch, not this
threshold, handles them.

| Patient | GA, weight, skin | OT IR p50 / max [ppm] | OT RED p50 / max [ppm] | RED/IR |
|---|---|---|---|---|
| SUBJ08 | 27+3 wk, 1.16 kg, 21 d, white | 131 / 156 | 95 / 119 | 0.73 |
| SUBJ11 | 30 wk, 1.5 kg, white | 91 / 111 | 72 / 89 | 0.79 |
| SUBJ09 | 26+1 wk, 1.53 kg, dark | 75 / 109 | 27 / 35 | 0.37 |
| SUBJ10 | 39+4 wk, 2.75 kg, white | 43 / 85 | 16 / 50 | 0.38 |

Fraction of each patient's time the AND (`min(OT1, OT2) > thr`, runs ≥ 0.2 s = the debounce) would
have flagged as PROBE_OT_HIGH: at 100 ppm, SUBJ08 35 % (the board did record 35 % in state 1 that day),
the others 0 %; at 150 ppm and above, 0 % on all four.

**What the data say.**

1. *The channel that decides is RED, not IR.* RED < IR on all four babies with this unit, so under the
   AND the applied-side maximum that matters is RED's 119 ppm. The v0.95 note quoted IR's 145 ppm and a
   ×1.03 margin; the actual margin of 150 ppm was ×1.26. Both numbers are thin.
2. *IR alone did cross 150 ppm* on SUBJ08 (max 156, 1.1 % of the time). The in-air ratio RED/IR was
   measured on two units of this probe model on 2026-09-07: 0.716 and 1.35 (§5.6.2 of the spec, and the
   2026-09-07 analysis in PulseNest's log). With the unit whose RED is the higher channel, IR decides and
   SUBJ08 would have been declared absent at 150 ppm. The threshold is unit-dependent; this was already
   the conclusion of 2026-09-07 and is why normalisation by the in-air value remains the real fix.
3. *OT grows as the baby gets smaller.* RED rises ×6 from the term baby (SUBJ10) to the 1.16 kg preterm
   (SUBJ08). Patients under 1 kg are not in the sample and will sit above SUBJ08. The failure mode on
   that side, a correctly applied probe declared absent, resets SpO2/HR on every sample and has no
   downstream safety net (§5.6.2).
4. *The air side has room.* In-air minimum measured 592 ppm (unit with 827/592; the other ~800/1080).
   And air at RF ≥ 50K with 50 mA drives the TIA past full scale (clipped OT = FS / (RF·I_LED) = 480 ppm
   at 50K, 240 at 100K), which goes to PROBE_ONLY_LED_SATURATING regardless of `rsqm_ot_thr`. The
   threshold decides only at RF 10K/25K (air gives 0.4–1.0 V there, below FS) or at lower LED current.
5. *High OT on an applied probe is still a usable signal.* SUBJ08 replayed with lib v0.108 (threshold
   150 ppm, so every sample APPLIED): OT IR > 150 ppm gives PI 1.6 and SpO2_SQI 0.78; OT < 110 ppm gives
   PI 4–5.5 and SQI 0.93–0.96. Widening the threshold admits no bad data; it only stops discarding thin
   feet.
6. *SUBJ08's RED/IR (0.73) equals the in-air ratio of the 827/592 unit (0.716).* Almost no differential
   attenuation: OT normalised by air is ≈ 0.16 on both channels. A very thin foot, or optical shunting
   (light reaching the photodiode around the foot), which the PI drop with rising OT also suggests.
   Not resolvable from these data; a deliberate well-seated vs tilted capture would be.

**Alternatives.**

- *Keep 150 ppm.* Works on the four measured babies and this unit. Rejected: ×1.26 over the deciding
  channel, below the other channel's maximum, and the next smaller patient or the other probe unit
  breaks it. Margin by luck, as the MS100 case was in v0.89.
- *300 ppm.* ×2.5 / ×1.9 above the two channels' maxima, ×2.0 below air. Rejected only because
  250 already gives balanced margins and 300 spends air-side margin the threshold's only operating
  region (RF 10K/25K) does need.
- *250 ppm (chosen).* Geometric mean of the applied-side maximum (119–156) and the in-air minimum (592):
  ×2.1 above RED's maximum, ×1.6 above IR's, ×2.4 below air. At RF 100K the clipped in-air OT (240 ppm)
  sits just under it, which costs nothing: that case clips and is caught by the LED-only branch.
- *Normalise OT by its in-air value* (decided in principle 2026-09-07, not implemented). Removes the
  per-unit and per-channel factor and turns the threshold into a physical statement ("tissue attenuates
  ≥ N×"). Still the destination; needs the in-air reference captured automatically (the LED-only
  saturation branch sees air) and is a separate change.

**Decision (Alex, 2026-11-01).** `rsqm_ot_thr` default 1.5e-4 → 2.5e-4 (v0.115). Limits that stay
written down: n = 4 patients, one probe unit, no patient under 1 kg, in-air values from two units
measured on the bench, not the hospital unit identified. Recalibrate when a smaller patient or a second
unit is measured; the fix for the unit dependence is the normalisation above.

**Reproduction.** Script of the 2026-10-31 session (PulseNest scratchpad, not versioned): for each part
of each session, `OT_LED1`, `OT_LED2`, `ProbeState`; percentiles in ppm; runs of `min(OT1,OT2) > thr`
of ≥ 100 samples at 100/150/200/250/300/400 ppm. Quality per OT bin from
`derived/replay_lib0.108/*_replay.csv` of SUBJ08 and SUBJ11.

## 21. `ppg_disp` — normalized display modes, and a selector instead of new fields (2026-11-05)

**Question (Alex, 2026-11-05).** `ppg_disp` is the band-passed optical transmittance of one channel
(A/A, ~1e-6..1e-5, §1.3 of the spec): fine for an auto-ranging plot, useless for a display with a
fixed frame. Alex wants, besides the current signal, (a) a version normalized to a fixed range and
(b) a version whose range shrinks with low perfusion (PI), and asks whether to add fields to
`AFE4490Data` or a function that selects among the three. The PI-weighted normalization had been
promised in the `ppg_disp` comment since v0.69 (2026-08-21) and explicitly deferred by Alex then.

**Prior art.** Nellcor, Masimo and Philips draw the plethysmographic waveform **auto-scaled**: it
always fills its trace, and the manuals state that its amplitude is not proportional to pulse
volume. Perfusion is shown separately: as a number (Masimo's PI, Nellcor's since OxiMax) or as an
amplitude bar (Nellcor's "blip bar"). Mindray lets the operator set a fixed waveform gain. Mode (b),
a waveform that shrinks with low perfusion, is in none of them as far as we know: it is this
project's own choice and is justified here, not by precedent.

### 21.1 Fields or a selector

| Option | Cost | Benefit |
|---|---|---|
| A. Two new `float` fields (`ppg_disp_norm`, `ppg_disp_norm_pi`) | Two more columns in `$M1`–`$M4`, the capture CSV, every parser in `pulsenest_lab.py`, the fleet viewer, the tooltips, spec §2.1, the example; 8 B more per sample in a struct whose size the UDP batching watches | The three traces visible at once, live |
| **B. One field, a mode selector (chosen)** | One enum, one config field, one setter, one `$SET`/`$LCFG` key; `$M1`–`$M4` unchanged | Same pattern as `setPPGChannel()`: a function chooses what goes into `ppg_disp`; a product shows one trace anyway |
| C. Fields only in `AFE4490DebugData` | Keeps `AFE4490Data` clean but `$M4` would still have to carry them to be seen | Half of A's cost for the same benefit |

B wins because nothing is lost: the raw mode is recomputable offline from `LED1_SUB`/`LED2_SUB` and
the `afe:` records (the offline runner does exactly that), the normalized modes are deterministic
functions of the same inputs, and the lab already band-passes the raw codes itself for its own PPG
views. Seeing the three traces together, which is only needed while tuning the mapping, is a job
for the lab on the raw columns, not for the library. The existing `$LCFG` mechanism makes the mode
switchable at run time from the lab's configuration window without touching frames.

### 21.2 The three modes

`AFE4490PPGDispMode`: `NO_NORM` (what v0.69–v0.115 do, the default), `NORM`, `NORM_PI`. Spec §5.9
is normative; the reasons:

1. **Range `[−1, +1]`, not `[0, 1]`.** The band-pass removes the DC, so the waveform is bipolar
   with zero mean. Forcing `[0, 1]` would mean an offset of 0.5 that every consumer would have to
   remove again. A receiver that needs an unsigned value (a fixed-point waveform widget) applies
   `0.5 + x/2` itself.
2. **What divides the AC.** Three candidates: (i) a min/max tracker over a window (exactly ±1, but
   one motion spike sets the scale for the whole window and a flat line follows); (ii) the RMS of
   the band-passed signal with the 2√2 peak-to-peak convention PI-METHOD-1 already uses (§11), so
   that the same number scales the waveform and the PI; (iii) a fixed PI reference,
   `AC / (DC · PI_ref/100)`, the August sketch, which is a *proportional* display with a fixed
   gain (what Mindray's gain setting gives). Chosen: (ii). (iii) is dropped as a mode: the PI
   number already carries the proportional information, and it leaves a low-perfusion baby with an
   invisible trace, which `NORM_PI` is designed to avoid. The 2√2 convention makes a sinusoid fill
   exactly ±1; a real PPG has a crest factor of 2.8–3.2 (§11), so its systolic peak overshoots +1
   by up to ~12 % and the trough undershoots less. The output is **clamped** to ±1: the consumer is
   a display with a fixed frame, and a guaranteed range is worth more than the last 12 % of the
   peak shape (the HR algorithms never see this signal).
3. **Its own tracker, not SpO2's EMA.** The sketch of August reused `_spo2_ch_ir_ema`. Rejected: it
   tracks IR only (`ppg_disp` can show RED), its time constant belongs to SpO2 and its reset rules
   to the SpO2 hold logic. `ppg_disp` gets one `EmaChannel` on its own band-passed output
   (mean ≈ 0, var = AC²), time constant `ppgdisp_norm_tau_s`. Provisional default 2 s, 4–6 beats
   in a neonate; to be tuned on the HOSPNAV replays (§21.4). Shorter adapts faster after the probe
   is placed and flattens respiratory amplitude modulation; longer keeps it. Warm-up 2τ (var at
   86 %, amplitude 7 % high, clamped) before the first normalized sample.
4. **`NORM_PI`: amplitude `A` from the published PI.** `A = 0.5 + 0.5 · ln(PI/PI_lo) / ln(PI_hi/PI_lo)`,
   clamped to `[0.5, 1]`, with `PI_lo = spo2_sqi_pi_lo` (0.5 %) and `PI_hi = spo2_sqi_pi_hi` (2 %):
   the trace is at half scale exactly where SpO2's SQI says the PI is too low to trust (0), and at
   full scale where SQI is 1. Alex had asked for two new parameters; reusing the SQI pair was
   preferred because it adds no parameter without provenance (the 0.5/2.0 pair's own provenance is
   still open, see memory) and keeps one meaning of "low perfusion" in the library. **Reversed
   2026-11-07 (v0.117, Alex):** the two uses answer two different questions — the SQI, whether the
   SpO2 *measurement* is reliable (algorithm and sensor noise; to be calibrated against |error| vs
   PI on HOSPNAV); the display amplitude, whether *this baby's perfusion* is low (clinical and
   age-dependent; what a nurse expects from a commercial monitor). Crossed cases exist: a preterm
   with PI 0.8 %, normal for its age, with a reliable SpO2; a PI 1.5 % artefact with a doubtful
   SpO2. So `NORM_PI` got its own pair, `ppgdisp_norm_pi_lo/_hi` (`setPPGDispNormPI()`), with the
   SQI pair's values as defaults; each pair is justified on its own evidence and they stay equal
   only if the evidence says so, documented as coincidence by evidence and not by economy of
   parameters (§21.4). The SQI pair was renamed `spo2_sqi_pi_lo/_hi` at the same time (the SQI's
   PI thresholds, parallel to the display's PI thresholds; Alex's suggestion). Logarithmic
   rather than linear because PI spans more than a decade in neonates (0.3–5 %); over 0.5–2 % the
   two differ by at most 0.08 in `A` (PI = 1 %: 0.75 vs 0.67). The PI used is `AFE4490Data::pi`
   (PI-METHOD-1, IR, 6 s window): the amplitude the operator sees tracks the number the operator
   reads, even when the displayed channel is RED. PI = NaN (window filling, first 6 s after
   placement) → `A = 0.5`: unknown perfusion is shown at the minimum, not hidden. The output is
   clamped at ±1 after the scaling, not at ±A (§21.8, 2026-11-07).
5. **No probe → flat line, not auto-gained noise.** In `NO_NORM` the filter runs regardless of the
   probe state (unchanged). In `NORM` and `NORM_PI`, `probe_state ≠ PROBE_APPLIED` gives
   `ppg_disp = 0` and resets the tracker: an auto-gain on noise makes a disconnected or
   in-air probe look like a live patient, the one failure a display normalization must not have
   (commercial monitors blank the pleth in that state). PI-METHOD-1 resets on the same condition.
6. **Where it sits.** After the band-pass and the negation, before the struct write, inside the
   existing `_state_mutex` scope (the mode setter takes the same mutex as `setPPGChannel()`). The
   settle freeze of §5.8.4 applies unchanged: a frozen input gives a constant to the tracker too.

### 21.3 Not changed, by design

- `$M1`–`$M4` and the capture CSV carry `PPG_DISP` as before; a capture records the mode in `$LCFG`
  (and the lab shows it in the PPG plot title), otherwise a `[−1, 1]` column would be ambiguous.
- Nothing in HR1/HR2/HR3/SpO2/PI/RSQM/HGAC reads `ppg_disp`; the modes cannot change a measurement.
- `NO_NORM` is bit-identical to v0.115 (a test asserts it).

### 21.4 Open points

- Whether `PI_lo`/`PI_hi` stay coupled to the SQI ramp or get their own parameters is decided
  after the HOSPNAV replay, not before.
- The IncuNest HMI already plots `ppg_disp` on a fixed scale (facts in §21.7); `NORM_PI` is the
  intended product mode and `[−1, 1]` its contract.
- Settled 2026-11-07: the estimator (SLEW, §21.5 addendum), the clamp at ±1 (§21.8) and the headroom
  1.15 (§21.8); the display filter and the transport rate (§21.7) remain open.
- **Open (Alex, 2026-11-07): are `PI_lo` = 0.5 % and `PI_hi` = 2 % the right thresholds for the
  PI-dependent amplitude of `NORM_PI`?** Since v0.117 they are their own parameters
  (`ppgdisp_norm_pi_lo/_hi`), decoupled from the SQI ramp's `spo2_sqi_pi_lo/_hi` (§21.2 point 4,
  reversed), whose own provenance is also open; the analysis treats the two pairs separately and
  unifies them only if the evidence lands on the same numbers. Parked until there is time. To
  analyse: what ISO 80601-2-61 and the de-facto
  standards say, what the manufacturers do (Masimo, Nellcor, Philips: at which PI they flag low
  perfusion or shrink the pleth), the dependence on age (neonatal PI ranges: p5 0.70 / p95 4.5 %
  in Granelli; PulseNest `docs/pi_reference_ranges_by_age.md`), and whether the two thresholds should
  be settable from the HMI or kept fixed so as not to add complexity or confusion to the handling of
  the incubator.

### 21.5 The normalizer of `NORM` is not settled: RMS measured against peak-to-peak (2026-11-05)

Point 2 of §21.2 chose `x / (√2·RMS)` on the strength of the 2.8–3.2 crest factor of §11. That
figure was measured in PI's 0.5–5 Hz band. In `ppg_disp`'s 0.5–20 Hz band the systolic upstroke and
the dicrotic notch survive and the crest factor is higher: peak-to-peak over RMS in 2 s windows is
3.3–3.6 (p25–p75) on SUBJ08, SUBJ09 and SUBJ10 (the board's own `PPG` column, probe applied). The
consequence, measured per beat on the same captures: with 2√2 the median beat fills ±1 exactly but
the 95th percentile reaches 1.2–1.4, 14–16 % of all samples lie beyond ±1 and are clamped, and more
than half of the beats show a flattened systolic crest. The "up to ~12 %" of the first draft was
wrong. A classic envelope follower (instant attack, exponential decay towards the signal, centre at
`(hi+lo)/2`) filled ±1 on the median beat with 0.2–0.9 % of samples clamped, centred the asymmetric
waveform (peak +1.00, trough −1.00, where RMS gives −1.06), warmed up in one beat, and shrank the
beats after an injected 5× spike no longer than RMS did (3 beats at τ = 1 s, 7 at τ = 2 s, RMS 8).

The comparison was done before the criteria were written down, so it is not the decision. Alex fixed
the criteria on 2026-11-05 (and two preferences that narrow the field):

1. Scaling with PI (less PI, smaller range) — a property of `NORM_PI`, not of the normalizer; what
   the normalizer owes it is a trace that really fills ±1 (median fill per beat ≈ 1).
2. Risk of clipping when the pulse amplitude grows (samples beyond ±1, beats with fill > 1).
3. Immunity to spikes (motion): **Alex's preference — a spike may saturate the plot but must not
   re-scale it**, so the following beats keep their size. This excludes a pure instant-attack
   envelope and asks for a rate-limited rise of the scale, or a robust (median) amplitude statistic.
4. Update rate of the range: **continuous, asymmetric** — shrinking the range (trace gets small) may
   be fast, 1–2 s; growing it must be slow or slew-limited, which is what makes 3 hold without an
   artefact detector.
5. Others: centring of the asymmetric waveform; behaviour without signal (flat line, already
   decided); start-up time after placement; no dependency on HR1's beat detector; cost and state
   per sample at 500 Hz; exact offline reproducibility; no distortion of the diastole between two
   beats at low heart rate (short τ in bradycardia).

Candidates still to be measured in that form: an envelope with slew-limited rise and fast decay,
and a scale from the median amplitude of the last beats. The table of candidates against criteria,
with the metrics above on SUBJ08–11, decides; spec §5.9 carries the RMS text until then, marked.

**Addendum 2026-11-06/07 — measured and watched in PPGDISPLAB; decision: SLEW (Alex).** The
window (PulseNest `pulsenest_lab_spec.md` §7.22) runs the library's own filter arithmetic on the raw
OT of the HOSPNAV P1 captures (0.001–0.035 % from the board's `ppg_disp`) and four trackers,
every parameter a control. Metrics per beat, detected on the normalized trace (the 2026-11-05
scratch numbers above counted beats on the raw trace and merged them; they overstated the fill):

| Tracker | Median fill SUBJ08 / 09 / 10 / 11 | Clipped samples | Watched by Alex (NORM_PI, clamp at ±1) |
|---|---|---|---|
| SLEW (rise 0.5/s, win 1 s, τ 1 s) | 0.53 / 0.85 / 0.45 / 0.79 | 1.6 / 8.7 / — / 5.3 % (H = 1); 0.9 / 5.3 / — / 1.7 % (H = 1.15) | looks right |
| INSTANT (τ 1 s) | 0.62 / 1.00 / 0.60 / 1.00 | 0 % at any H | distorts: plateaus while a beat grows — the frame jumps on the very sample that makes the extreme, so the sample that should be the peak is pinned at the frame |
| MAXMIN (win 1 s) | — | — | distorts: the frame jumps when an extreme leaves the window |
| RMS (√2·RMS, τ 1 s) | — | 14–16 % (2026-11-05) | shape right, but the frame rises and falls continuously |

Why RMS breathes: the scale is √(EMA of x²) with τ = 1 s, of the order of the neonatal cardiac period
(0.4–0.5 s), so the EMA does not average several beats and oscillates inside each cycle; dividing by
a scale that oscillates at the heart rate modulates the amplitude at that rate — the systolic peak is
compressed (scale rising) and the diastole stretched (scale falling), and the dicrotic notch is the
first thing to deform. τ = 3–4 s would reduce it at the price of 3–4 s of tracking; it is the reason
PI-METHOD-1 uses a 6 s window and not a short EMA. SLEW has none of this by construction: its frame
moves on 100 ms block extrema, grows at a bounded rate and falls with τ, so it is constant inside a
cycle. INSTANT clips nothing and fills more, but the plateaus are a distortion of the waveform
itself, which the fill metric does not see — the visual check decided against it. SUBJ10 is a noisy
capture whose beats the detector under-counts; its numbers are indicative only.

Decision (Alex, 2026-11-07): SLEW, with the window's parameters as library defaults — rise 0.5 of the
frame per second, window 1.0 s, fall τ 1.0 s — and headroom 1.15 (§21.8). The contract of §5.9 is
rewritten accordingly; the fill of the median beat is 0.45–0.85 rather than ≈ 1, because the largest
beat of the last second is what fills the frame; Alex judged the trace right at that fill. Still
open in §21.7: the display filter (BP2 vs BP2+LP4, corner 8–10 Hz) and the transport rate.

**Why the rise is a linear rate limit and the fall an exponential (Alex asked, 2026-11-07, accepted).**
Growing the frame is the response to a larger extreme, and a larger extreme is as likely an
artefact as a real beat; a linear limit (a fraction of the frame per second) makes the response to a
spike independent of the spike's size — a 5× or a 50× spike both widen the frame by the same few
percent in 100 ms — which is exactly criterion 3. Shrinking the frame is the response to a smaller
signal, and a smaller signal is always real (an artefact only adds amplitude), so it is safe to
follow it faster the farther away the target is, which is what an exponential does; the exponential
also smooths the steps of the window target when an old extreme leaves the window — the jumps Alex
saw in MAXMIN. The window (1.0 s of 100 ms block extrema) is at least one neonatal beat; the headroom
only reserves chart.

**Prior art, stated carefully.** No manufacturer documents the dynamics of its waveform auto-scaling.
What is documented: Masimo acknowledges that it auto-scales the pleth and that this masks amplitude
changes (US 8414499 B2, US 2008/0188760 A1); Shelley 2007 (Anesth Analg 105:S31) states that every
clinical oximeter displays the waveform with an auto-gain that maximizes its size and that some let
the user disable it — neither says how the gain moves. Masimo's slew-rate limiting (US 6157850)
applies to the displayed pulse rate (20 bpm per 2 s) and to PVI, not to the waveform. The one
antecedent of an asymmetric envelope on the pulsatile waveform is Puritan-Bennett US 4883353 (1989):
instant jump to a new extreme, hold for 1/8 of a cycle, then decay at a constant slope proportional
to amplitude and period — the opposite asymmetry in the direction of the extreme (instant attack,
rate-limited release), used for the saturation computation and to overlay the envelopes on the
waveform, not to scale the display. SLEW therefore rests on the reasoning above and on the
PPGDISPLAB measurements, not on documented manufacturer practice. (A sentence attributing a
limited-attack / exponential-release auto-scale to Masimo and Nellcor was written in a side session
on 2026-11-07 and withdrawn the same day when Alex asked for references: there are none.)

**Implementation (v0.116, 2026-11-07) and what it measured.** `PPGDispTracker` in the library is
the window's SLEW line by line (block extrema on the PI block clock, slew-limited rise, exponential
fall, headroom), `_ppgdisp_update()` applies the mode and the clamp at ±1, and `test/test_ppgdisp/`
pins eight behaviours (spec §5.9). Two things the implementation made precise:

1. *A spike does re-scale the trace, partly and briefly.* The spec draft said a 5× spike of 100 ms
   widens the frame by ≈ 5 %. It widens it by much more, because the spike's extreme stays in the
   1 s target window and the frame keeps rising towards it for that whole second (exponentially,
   since the step is a fraction of the current frame). Synthetic 2 Hz sine at 500 Hz, the
   library's own band-pass, Python chain = library:

   | rise /s | win s | τ s | frame growth | back within 5 % after | trace saturated |
   |---|---|---|---|---|---|
   | 0.5 | 1.0 | 1.0 | ×2.3 | 4.5 s | 0.25 s |
   | 0.3 | 1.0 | 1.0 | ×1.8 | 4.1 s | 0.26 s |
   | 0.2 | 1.0 | 1.0 | ×1.5 | 3.9 s | 0.27 s |
   | 0.1 | 1.0 | 1.0 | ×1.2 | 3.2 s | 0.28 s |
   | 0.2 | 0.5 | 1.0 | ×1.3 | 2.6 s | 0.27 s |

   Criterion 3 is therefore met in degree: with the defaults the beats after a spike are drawn at
   ~45 % of their size for a second or two, then recover. The same `rise_per_s` sets how long a
   real ×2 amplitude step clips (doubling in ln 2 / rise: 1.4 s at 0.5, 3.5 s at 0.2). Alex chose
   0.5 by eye on the HOSPNAV captures, where real motion artefacts last longer than 100 ms; the
   defaults stand, and 0.2/s or a 0.5 s window are the knobs if the post-spike shrink proves
   visible in the ward (the window's SPIKE button reproduces it).
2. *A flat window must not trap the frame.* With a relative slew limit, a frame that has collapsed
   to ~0 (probe applied, no pulse) could only grow back exponentially from ~0: ln(1e-6 / 1e-9) /
   0.5 ≈ 15 s before a returning pulse is full size. The tracker therefore restarts its warm-up
   whenever the frame is narrower than `pi_div_eps` (1e-9 A/A, three decades under the ADC noise
   floor of OT, so no artefact can trigger it): the returning pulse shows one window later. The
   fall after a ×0.5 step is 1 s (window) + 3τ ≈ 4 s to 95 %, not the "1–2 s" of the draft.

### 21.6 What ISO 80601-2-61:2026 says about the waveform display, and the floor of `A` (2026-11-05)

Alex brought a NotebookLM reading of the standard. Checked against the PDF (`PulseNest/docs/`):

- 201.3.241 *normalized*: "displayed at constant pulse amplitude independent of the actual pulse
  magnitude of the signal being displayed". `NORM` is exactly that.
- 201.12.4.102 *Signal inadequacy*, NOTE 3: a non-normalized pulse waveform display satisfies the
  requirement for an indicator with respect to %mod; "a normalized waveform by itself does not
  satisfy this requirement and can mask an unreliable signal". NOTE 4: a non-normalized waveform
  does not indicate inadequate DC signals. **NOTE 6: not posting SpO2 or pulse rate under inadequate
  signals satisfies the requirement** — which the library already does (SpO2 hold reasons, §17; the
  SQI ramp on PI). d) 1): if a non-normalized waveform is used as the indicator, the accompanying
  document must describe the visible characteristic that identifies inadequate %mod.
- AA.2.14: a non-scaled plethysmographic display can lack the resolution to reveal clinically
  significant changes of pulsatile signal strength in the low range; scaling it to increase the
  resolution there "can enhance the utility of the plethysmogram for assessing signal adequacy".
- Not in the standard: any recommended scaling law (the "logarithmic, optimal compliance" label and
  the "PI 0.2 % → 25 % of the screen" example were NotebookLM's); 201.3.212 is *data update
  period*, a wrong citation.

Consequences. The waveform is not what carries 201.12.4.102 for this library (NOTE 6 and the RSQM
already do); its scaling is a usability choice judged by §21.5's criteria, the standard being a
source, not a mandate. `NORM` is the display the standard warns about: it stays as a laboratory
mode, and **`NORM_PI` is the product mode** (spec §5.9 says so). `NORM_PI` is the "appropriately
scaled plethysmographic display" of AA.2.14; if it is ever declared as the %mod inadequacy
indicator, d) 1) is met by one sentence: "the trace at half height means PI below 0.5 %".

**Floor of `A` = 0.5, kept (Alex, 2026-11-05).** A lower floor (0.25 was floated, from the
NotebookLM example, with no evidence behind it) would make low perfusion more conspicuous (4:1
contrast instead of 2:1), but the IncuNest display is far smaller than a hospital multiparameter
monitor, and the trace must keep showing the waveform's shape: the dicrotic notch is 10–20 % of the
peak-to-peak amplitude and needs ~3 px, so `floor ≥ 20 / H` for a widget H pixels tall (0.33 at
60 px, 0.20 at 100 px). 0.5 keeps the shape on any widget above 40 px, and the PI number, the SQI and
the SpO2 hold already carry the perfusion message; the shrink is a secondary cue. Below PI 0.5 %
(Granelli 2007: p1 of 10 000 newborns; Masimo "low perfusion" < 0.25 %) the trace is at its minimum
whatever the floor, so widening the mapped range downwards would only redistribute the shrink where
SpO2 is already withheld. The formula is `A = floor + (1 − floor) · ln(PI/PI_lo)/ln(PI_hi/PI_lo)`;
the floor is one number to revisit if the HMI widget ever makes 0.5 illegible or 0.25 legible.

### 21.7 The display chain downstream of `ppg_disp`: what exists and what it asks of the library (2026-11-06)

Facts, read in the IncuNest repo on 2026-11-05/06 (not in this repo's control; recorded here because the
display modes and the display filter are designed for this consumer):

- motherBoard `CommTask` sends `CTRL,PPG,<byte>` to the HMI every 40 ms (25 Hz) with the *latest*
  `ppg_disp` sample: a 500 → 25 Hz decimation by sample-and-hold, with no anti-alias filter.
  `byte = 128 + round(ppg_disp / 1.6e-8)` clipped to 0–255: a fixed transport scale, kept fixed on
  purpose by Pablo (commit e3a71567, 2026-08-15, "no adaptive scaling outside the library"); sent only
  while the probe is applied. `PpgSnapshot` keeps 8 s at 50 Hz for ThingsBoard, same scale, same
  sample-and-hold.
- Display_HMI (CrowPanel 7", 800×480 px, ≈154 mm wide → ≈5.2 px/mm, measured 2026-11-06) draws
  `ui_LockPPGChart`, an LVGL line chart of 460×105 px, Y range 0–255, `LOCK_PPG_POINTS = 128`,
  circular update with a 6-point erase gap. **128 points was chosen by time, not by pixels:** Pablo's
  commit f098e634 (2026-08-13) halved 256 points (10.2 s, "beats crammed together") to 128 = 5.1 s at
  25 Hz because clinical monitors show 4–6 s; the chart was 665 px wide then and 460 px now, so one
  point is ≈3.6 px and LVGL joins points with straight segments. Nothing ties the point count to the
  width.
- Sweep speed (Alex, 2026-11-06): the clinical convention for the pleth trace is **25 mm/s**, inherited
  from ECG; 12.5 and 50 mm/s are optional alternatives. ISO 80601-2-61 does not fix one. On the
  CrowPanel 25 mm/s is ≈130 px/s: the 460 px chart would hold ≈3.5 s, and today's 128 points over
  460 px run at ≈17 mm/s.
- Today's chart in numbers (2026-11-06, from the code; the panel's active area is *not* in the IncuNest
  repo: Alex measured ≈154 mm screen width and ≈88 mm chart width with a ruler, ±1 mm, matching the usual
  7" 800×480 TFT, 154 × 86 mm): chart 460 × 105 px ≈ 88 × 20 mm;
  Y range 0–255 → 2.4 byte values per px; 128 points at 25 Hz = 5.12 s, 3.6 px per point;
  sweep = 25 points/s × 3.6 px / 5.2 px/mm ≈ 90 px/s ≈ **17.3 mm/s**; erase gap 6 points ≈ 22 px ≈
  0.24 s. At the standard 25 mm/s the same chart holds 3.5 s = 88 points at 25 Hz (or 176 at 50 Hz).
- Prior art: Masimo's front end decimates to 62.5 Hz through a 519-tap FIR with a 7.5 Hz corner
  before anything is displayed or processed (US 5,632,272; `PulseNest/docs/masimo_nellcor_spo2_prior_art.md`).

Consequences for the library (open, to be settled with the lab test window before v0.116 is coded):

1. **Aliasing.** 25 Hz gives a Nyquist of 12.5 Hz; the display band-pass (one biquad, 0.5–20 Hz,
   6 dB/octave each side, an inherited default with no written rationale, spec §2.2) passes 20 Hz and
   attenuates 40 Hz by only 6 dB, so 12.5–20 Hz content and noise fold into the trace. Small on a clean
   PPG, not on a noisy one. Two remedies, both reasonable: a lower and steeper display filter in the
   library (candidates: 4th order, corner 8–10 Hz, like PI's and Masimo's), so that any consumer at
   ≥ 25 Hz is safe without knowing anything; and proper decimation (block average or FIR) in the
   consumer. A lower corner also reduces the crest factor the normalizer has to absorb (§21.5).
2. **Rate is a contract constant; geometry is not.** The transport rate should be one constant of the
   library/motherboard/HMI contract, chosen for the largest display we expect, and each consumer derives
   its point count from it: `points = rate × width_mm / sweep_mm_s`. Pixels and seconds change with
   every screen; the rate does not need to. Candidates: 25 Hz (today), 50 Hz (`PpgSnapshot` already),
   62.5 Hz (Masimo). At 25 mm/s and 5.2 px/mm, 25 Hz is 5.2 px per sample and the systolic upstroke
   (100–150 ms) is 3 samples: visibly polygonal, and the crest can fall between two samples, a few %
   lower than the library's. At 50 Hz both effects halve.
3. For the lab test window (`PPGDISPLAB`, PulseNest): the filter (order, corner) and the transport rate
   are *candidates* to choose with the window and then freeze; width px, height px, sweep speed and
   px/mm are *parameters* of the window, never constants, because they belong to the HMI and will change.

**Settled 2026-11-10 (v0.119, §21.10):** the high side of the display filter is 4th order at `f_high`
(default kept at 20 Hz), and IncuNest moves its transport to 50 Hz. The 8–10 Hz corner of point 1 was
written for a 25 Hz consumer; at 50 Hz the folded signal is negligible with any corner and the order,
not the corner, is what matters (mains and drivers at 50/100 Hz).

### 21.8 Scale by `A`, clamp at ±1 — not at ±A (2026-11-07)

The first draft of §5.9 and the PPGDISPLAB window wrote `NORM_PI` as `A · clamp(u, −1, 1)`: the
trace was clamped at ±A. Alex, seeing the cyan trace flatten against the magenta ±A line with the
chart empty above it: *scaling adaptively is useful (the envelope shows the PI falling); clamping
adaptively has no use.* Agreed, and the two are indeed separate decisions:

- **Scaling by `A`** carries the PI: over seconds the envelope of the trace falls with the
  perfusion. It is what justifies `NORM_PI` over `NORM` (ISO 201.12.4.102 NOTE 3, §21.6).
- **Clamping at ±A** carries nothing. The consumer's byte has room up to ±1; with A = 0.63 a
  clamp at ±A discards 37 % of the chart exactly when the most informative samples arrive: a beat
  larger than the tracked range, i.e. an artefact, a perfusion step, or a tracker that has not
  grown yet. Showing it above A, up to ±1, is what NOTE 3 asks for (do not mask an unreliable
  signal). The only argument for ±A would be "the amplitude is an exact PI reading", and it is
  not: `A(PI)` is a logarithmic approximation, and the envelope of several beats still reads A
  when one beat overshoots.

Measured in PPGDISPLAB (SLEW tracker, `NORM_PI`, HOSPNAV P1 captures, samples with the probe
applied after the first 5 s), percentage of clamped samples:

| Capture | median A | clamp at ±A | clamp at ±1 |
|---|---|---|---|
| SUBJ08 | 0.63 | 6.1 % | 1.6 % |
| SUBJ09 | 1.00 | 8.9 % | 8.7 % |
| SUBJ10 | 0.88 | 18.5 % | 13.6 % |
| SUBJ11 | 1.00 | 5.3 % | 5.3 % |

Where A < 1 the clipping falls to a quarter at no cost; where A = 1 nothing changes, and there the
headroom factor (§21.7, candidate 1.15, measured 2026-11-06: halves the clipping of SLEW for ~12 %
less fill) is still what reduces it. The two combine in one expression, which is now §5.9:

```
ppg_disp = clamp( A(PI) · u / H , −1, +1 )      u = (x − centre) / semi-amplitude,  H = headroom
```

Side finding: the window's `clip` metric had counted |A·u| > 1 from the start (a clamp at ±1)
while its trace clamped at ±A, so the headroom table of 2026-11-06 already described the ±1
variant and stands. v0.116 is not tagged, so there is no compatibility cost.

### 21.9 The application transient: why the trace took 7–8 s to fill, and the band-pass reset (2026-11-09)

**Observed.** IncuNest motherBoard 18.57.0 (lib v0.117, `NORM_PI`, `CTRL,PPG` = 128 + 127·`ppg_disp`
at 25 Hz) on the V18 with the MS100 simulator, 2026-11-08: after applying the probe the HMI trace
takes more than 5 s to fill its chart. Transport and HMI are linear and stateless, so the behaviour
is the library's. Brief from the IncuNest session: `IncuNest/Firmware/docs/notes/2026-11-09-brief-libreria-ppg-disp-arranque-lento.md`.

**Causes, confirmed in the v0.117 code and reproduced on the lab's replica of the chain
(`PPGDispCalc`, PulseNest `sim_ppgdisp_start`, 2026-11-09).**

1. The display band-pass `_ppgdisp_bpf` ran continuously from `begin()`, in every probe state;
   `_pi_bpf` is reset on every sample while the probe is not applied and precharges to steady state
   on the first applied sample (`BiquadFilter::process()`). An asymmetry without a reason: an
   oversight of v0.116, not a decision. Applying the probe steps the OT from ~5.9e-4 (air) to
   ~1.2e-4 (tissue): 4.7e-4, some 400 times a beat of 1.2e-6 (PI 1 %). The 0.5 Hz high-pass lets
   the step decay with τ ≈ 0.32 s; `PROBE_APPLIED` is confirmed 0.2 s after it, when the tail is
   still ~100× the beat; the SLEW warm-up (1 s of plain running extrema) takes it as the frame; the
   extrema stay in the 1 s window another second; then the frame falls with τ = 1 s, and ×100 is
   ln(100) ≈ 4.6 s. Measured on the replica with a synthetic step of exactly these numbers and a
   2.5 Hz beat: the trace sustains 80 % of its steady peak-to-peak **8.0 s** after the application in
   `NORM` and `NORM_PI` (6.5 s with a PI 3 % beat). The HGAC's residual RF steps during the first
   seconds (~1 % of the DC, the RF tolerance, §12) are a second, smaller source of the same kind
   and are not addressed here.
2. PI is NaN until `pi_window_s` (6 s) fills, and `A(NaN)` is the floor 0.5 by design (§21.6): in
   `NORM_PI` the trace is at half height for the first 6 s whatever the perfusion, then full.

The 14 re-applications of the SUBJ08 capture (2026-09-23) do not show the effect: all are
probe-state flickers with the probe on the foot (OT 1.37e-4 → 1.35e-4, no step), and there the old
and the new chain behave the same. No capture in the corpus holds a real air-to-tissue application;
the 8 s rest on the OT in air of §5.8 and match what the IncuNest bench showed.

**Alternatives (the IncuNest brief's table, assessed here).**

| | Where | What | Verdict |
|---|---|---|---|
| A | library | reset `_ppgdisp_bpf` while not applied, as `_pi_bpf` | **adopted (v0.118)**: attacks the cause at zero cost; on the replica the trace is full size from the first sample after the warm-up window (1.2 s after the application) |
| B | library | fixed blanking of 2–3 s after `PROBE_APPLIED` | hides the symptom; worse than A in everything |
| C | library | restart the warm-up when the targets fall far below the frame for a window | covers other collapses, but a new heuristic with a threshold to justify; a real PI drop would blank the trace 1 s. Not now |
| D | IncuNest | short fall τ (0.2–0.3 s) | the frame would breathe inside the neonatal cardiac cycle, what §21.5 rejected in RMS. Diagnostic only |
| E | library | A = 1 or the last A while PI is NaN | inverts §21.6's decision; with a low PI the trace would shrink at 6 s, which reads as deterioration. **Kept at 0.5** (Alex 2026-11-09): growing at 6 s reads as warming up |
| F | IncuNest | shorten `pi_window_s` | changes a measurement for a display reason. Rejected |
| G | HMI | auto-scale on screen | Pablo's rule; the HMI lacks the information. Rejected |

**A in every mode, not only in the NORM modes (Alex, 2026-11-09).** One chain, one filter state
whatever the mode, and the lab's replica follows it. The price: `NO_NORM` is no longer bit-identical
to v0.69–v0.117 around an application — it is exactly 0 while the probe is not applied (the
band-pass of a constant precharges to 0) and loses the application transient that the lab's
auto-ranging plot used to show; in steady state it is the same filter. Test 1 of `test_ppgdisp`
now pins the gated identity, test 10 the application. The alternative (reset only in `NORM` /
`NORM_PI`) would have kept `NO_NORM` untouched at the cost of two filter histories depending on
the mode, and a mode switch at run time inheriting whichever state the other mode left.

### 21.10 The high side of the display filter: 30 dB/octave with `f_high` the −3 dB corner of the chain, low side untouched, 20 Hz kept (2026-11-10)

**Where it comes from.** A brief of the IncuNest session (2026-11-10, approved by Alex; measured on
PulseNest's SUBJ08–11 HOSPNAV captures, read-only) asked for the high side of `ppg_disp`'s filter to go
from 1st order (the biquad band-pass, 6 dB/octave per side) to 4th order Butterworth, in cascade with
the band-pass, with the same API and keys, and for a consumer rule in the spec. Context: IncuNest's
`CommTask` samples `ppg_disp` by sample-and-hold with no anti-alias filter (§21.7), and moves from 25 to
50 Hz in the same change.

**What was measured (brief's tables; windows of 10.24 s with the probe applied, artefacts discarded,
median; 1 px of IncuNest's chart ≈ 1.1 % of the beat's peak-to-peak).** Harmonics of the neonatal PPG
relative to the fundamental (HR 158–170 bpm): h2 (~5.5 Hz) 26–32 %, h3 (~8 Hz) 8–12 %, h4 (~11 Hz)
4–6.5 %, h5 (~13.5 Hz) 1.4–3.3 %, h6 (~16 Hz) 0.7–1.5 %; power above 12.5 Hz 0.03–0.15 %. No dicrotic
notch visible. Masimo's 7.5 Hz corner (1997) is no justification for a low corner here.

| 4th-order low-pass at | RMS difference | max difference (systolic peak) | delay |
|---|---|---|---|
| 20 Hz | 0.3–0.55 % | 1.7–2.4 % (~2 px) | 20 ms |
| 15 Hz | 0.3–0.6 % | 1.6–2.2 % (~2 px) | 28 ms |
| 12 Hz | 0.5–1.0 % | 2.8–4.1 % (~3 px) | 36 ms |
| 10 Hz | 0.6–1.6 % | 3.9–5.3 % (~5 px) | 42 ms |
| 8 Hz | 0.8–2.2 % | 4.6–6.6 % (~6 px) | 54 ms |

Folded signal and own noise when the consumer samples (RMS of what lands in band, % of peak-to-peak):
band-pass alone 0.22–0.96 % at 25 Hz, 0.04–0.09 % at 50 Hz; with the low-pass at 20 Hz 0.20–0.91 % /
0.01–0.02 %; at 15 Hz 0.15–0.72 % / ≤ 0.01 %. At 50 Hz it is negligible with any corner, SUBJ08 (PI
0.7 %) included. What justifies the order is deterministic interference (mains 50/100 Hz, phototherapy
and heater drivers; spec §7, a residual risk never measured), which a 50 Hz consumer folds exactly onto
its baseline:

| whole chain | 20 Hz | 25 Hz | 50 Hz (mains) | 100 Hz |
|---|---|---|---|---|
| band-pass alone (v0.118) | −3.0 | −4.2 | −9.0 | −15.5 dB |
| + Butterworth LP4 at 20 Hz (first draft, discarded) | −6.0 | −12.7 | −41.8 | −76 |
| + Butterworth LP4 at 15 Hz (first draft) | −13.5 | −22.2 | −51.9 | −86 |
| + Butterworth LP4 at 12 Hz (first draft) | −20.9 | −29.9 | −59.7 | −94 |
| **+ Butterworth-5 pairs at 20 Hz (adopted)** | **−3.0** | **−10.3** | **−41.2** | **−76** |
| + Butterworth-5 pairs at 15 Hz | −12.9 | −22.6 | −53.9 | −89 |

**Decisions.**

- *Two more biquads on the high side, in cascade, order fixed.* `LowPass4` (each section precharging
  on its own after `reset()`), the block HR3 has used since v0.97, here with another Q pair (next
  point but two). Not PI's: PI-METHOD-1's AC filter is the same 2nd-order band-pass as the display's
  (the brief said "HR3 and PI"; only HR3). One implementation, one set of tests; the order is not a
  parameter.
- *Low side untouched.* A high-order high-pass at 0.5 Hz has seconds of settling and overshoot on an OT
  drift, which is exactly the application transient v0.118 removed (§21.9).
- *`f_high` is the −3 dB corner of the whole chain, for any corner (Alex, 2026-11-10, before the
  tag).* The first draft cascaded a 4th-order Butterworth (Q 0.5412 / 1.3066, −3 dB at `f_high`) after
  the band-pass (−3 dB at `f_high`): −6 dB at `f_high`, −3 dB near 0.8·`f_high` (16 Hz for 20). Alex's
  objection: the user asks for 20 Hz and gets 16. Three ways out: (a) document "−6 dB at `f_high`"
  (keeps the confusion); (b) design the low-pass at a scaled corner (~1.2·`f_high`) so the sum lands
  near −3 dB (approximate, and the factor depends on `f_low` and on the warping); (c) give the two
  biquads the pole pairs of a **5th-order** Butterworth, Q = 1/(2·cos 72°) = 0.6180 and 1/(2·cos 36°)
  = 1.6180. Adopted (c): a 2nd-order low-pass's gain at its own corner is its Q, and 0.6180 × 1.6180
  = 1 exactly, so the pair adds nothing at `f_high`; the band-pass is exactly −3 dB at its prewarped
  edges by construction; hence the chain is −3.01 dB at `f_high` for any `f_low` < `f_high` < fs/2
  and any sample rate, not only 20 Hz (checked with the library's coefficient formulas at 250, 500
  and 1000 Hz for 0.5–5, 0.5–10, 0.5–15, 0.5–20, 0.5–30, 0.5–40, 0.1–20, 2–20 and 5–10 Hz: −3.01 dB
  in all 27; test 11 pins 20 and 10 Hz). It is a Butterworth-5 only approximately: the band-pass's
  high pole is not at `f_high` but slightly below (18.97 Hz for 0.5–20: the roots of s² + BW·s + Ω0²),
  which leaves a +0.16 dB hump at ~6 Hz with the defaults; accepted (1.9 %, about a pixel and a half
  of IncuNest's chart on the harmonics there, not on the beat). Price against the first draft: 2.4 dB
  less at 1.25·`f_high` (−10.3 instead of −12.7 dB), 0.6 dB less at 50 Hz; in exchange the pulse band
  is flat to 15 Hz (−0.2 dB there; the first draft −2.3 dB). The cost table above (peak difference,
  delay) was measured with the first draft and was not re-measured with the adopted pair; the group
  delay of the whole chain is 30 ms at 5–10 Hz (31 ms in the first draft), rising to 40 ms at 20 Hz.
- *Same API and keys.* `setPPGDispFilter()`, `ppgdisp_f_low_hz` / `ppgdisp_f_high_hz`,
  `LowPass4::init_butter5_pairs()` inside. Nothing for a consumer to call when upgrading.
- *Reset with the band-pass at the probe gate.* Both filters precharge on the first applied sample
  (§21.9); test 10 still passes unchanged.
- *Default kept at 20 Hz (Alex, 2026-11-10).* 15 Hz was the alternative: a similar visible cost and
  12 dB more margin at a 50 Hz consumer's Nyquist, and the number HR3 and PI
  already use as the upper edge of the pulse band. Alex kept 20 Hz because IncuNest moves its
  transport to 50 Hz, where the folded signal is negligible with either; IncuNest sets its corner
  explicitly in `initSPO2()` anyway, and so does PulseNest's firmware, so the library default reaches
  only the example and the tests.
- *Consumer rule, with the numbers corrected.* The brief stated "`f_high` ≤ 0.4 × consumer rate gives
  ≥ 20 dB at Nyquist". Its own table says otherwise: at 0.4·R the Nyquist is 1.25·`f_high`, where the
  first draft was −12.7 dB and the adopted chain is −10.3 dB; ≥ 20 dB needs `f_high` ≤ 0.3·R (Nyquist
  = 1.67·`f_high`, −22.7 dB). The spec gives both figures (≥ 10 dB and ≥ 20 dB). For IncuNest at
  50 Hz with 20 Hz: 10 dB at its Nyquist, 41 dB at the mains, which is what the change is for; with
  16 Hz it gets the first draft's response back (−20 dB at 25 Hz, −51 dB at 50 Hz).

**What it changes for PulseNest.** `NO_NORM` on the lab's plots and in the captures is the new chain
(flat to 15 Hz, −3 dB at 20 Hz, about +20 ms); PPGDISPLAB's replica (`PPGDispCalc`) makes `BP2+LP4` its default
filter; the firmware keeps `setPPGDispFilter(0.5, 20)` and is rebuilt with the new pin. No
measurement changes.

## 22. The number in an HR name is the domain; `HR4` freed, the derivative detector becomes `HR1_DRV` (2026-11-12)

**The problem (Alex).** HR1, HR2 and HR3 were numbered in the order they were written, and the
roadmap (spec §5.5) gave the next number, `HR4`, to a derivative-based peak detector. Two things
changed the reading. PulseNest's prior-art index (`PulseNest/docs/algorithm_prior_art_index.md` §1)
shows that the three are analyses of the pulsatile structure in three domains — beats in time,
periodicity in the time lag, harmonic structure in the spectrum — and that each feeds more than the
rate: `hr3_sqi > 0` is already the SpO2 artefact gate (§17), and the makers build PI, pulse
validation and probe-off rules on the same analyses. And a fourth domain, the cepstrum, appeared as
a candidate family (Datex-Ohmeda US 7,139,599, expired; not evaluated), for which Alex wanted the
number 4.

**Options.**
1. *Leave the roadmap as it is* (`HR4` = derivative detector; a cepstral method would be `HR5`).
   The number then means nothing, and a time-domain beat detector sits in a family of its own
   although §5.3 already treats derivative detection as one strategy for HR1's detector.
2. **The number names the domain, the suffix names the method (chosen).** 1 time (beat detection),
   2 time lag (periodicity), 3 frequency (spectrum), 4 cepstrum. It is the pattern `HR2_ACF` /
   `HR2_MPM` / `HR2_YIN` already follows. The derivative detector becomes `HR1_DRV`. Cost: two
   lines of the spec — `HR4` was never implemented and existed in no field, frame, CSV column or
   line of code (searched in both repositories on 2026-11-12).
3. *Rename the families away from `HR`* (a neutral prefix, or `PR`, as ISO 80601-2-61 and the
   makers call a PPG-derived rate). It would break the data contract (`AFE4490Data`, the `$M`
   frames, the CSV columns, the lab, IncuNest's firmware) for a label, and the HR/PR decision stays
   parked (2026-11-02). Not done.

**Details.**
- `HR1_DRV` is entered as *catalogued, not planned*. The roadmap said "Planned", but §5.4
  (2026-09-06) measured that no change of HR1's detector improves what the device publishes behind
  the `hr1_sqi` gate; the row had not been updated since.
- 4 is *reserved*, not planned: no cepstral method has been evaluated.
- A family's first implemented method keeps its bare field name (`hr1`, `hr2`, `hr3`); the suffix
  tells the methods of one family apart, so today's HR1 gets none while it is the only detector
  implemented.
- The sentence of §5.5 that named the input and the future fields was stale (`led1_aled1`,
  `hr4_valid`: the algorithms read `ot_led1` since v0.38, and validity is NaN plus an SQI of 0) and
  was rewritten.

**Not decided.** Whether the products of an analysis other than a rate and its quality (a comb
purity from the spectrum, a pulse gate read by SpO2) keep the `hrN_` prefix or are named after what
they measure. Proposed in the session, not agreed; it matters when the first such product enters
the library.

**Scope.** Text only: spec §5.5, this section, and PulseNest's `docs/nomenclature.md` and prior-art
index. No code, no field and no version: it ships with the next release.

## Appendix — source rationale (to distill at consolidation)

This preliminary doc is distilled from working notes (Claude local memory), primarily: the HGAC design
(`project_agc_design`, `project_agc_design_history`), closed decisions (ENSEPGAIN/O10, `namespace adc`,
value-always analog state, ProbeState), the clinical/HW basis (thermal envelope, probe specs, preterm
foot dimensions, AFE selection), and the nomenclature dictionary. Full consolidation (edit to
documentation tone, dedupe, drop task cruft) is planned for the HGAC v1 freeze.
