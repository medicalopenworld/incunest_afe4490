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
| Neither the log nor the code ever mentions linear phase or O(1) cost — the two arguments that *would* justify an MA today. They are real but accidental | search of log + code |

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
| **TERMA** | Two event-related moving averages — ~111 ms (peak) and ~667 ms (beat) — whose crossing generates blocks of interest; the peak is sought inside the block. The published design rule `2·W1 <= W2 <= 8·W1` is what separates the systolic peak from the dicrotic notch. Reported 99.89 % sensitivity / 99.84 % PPV over 5 071 beats, **on heat-stressed PPG** | two incremental MAs, O(1) | **Very good** | soft low-pass + squaring | Elgendi, PLOS One 2013 (10.1371/journal.pone.0076585); Biosensors 2016 (10.3390/bios6040055) |
| **SSF (slope sum function)** | Slope sum over a window enhances the upslope and suppresses the rest: low-pass + SSF + decision rule | O(W), W ~ 100 ms | Good | soft low-pass | Zong et al., open-source ABP onset algorithm |
| **First derivative / local extrema** | Zero crossing of dy with the sign of d²y; variants cube the derivative to highlight search zones | ~0 | Medium | low cutoff (the derivative amplifies HF noise) | Billauer; Li; FDPPG variants |
| **`y × dy` delayed** | Product amplifies edges and suppresses flat regions | ~0 | Medium | low cutoff | own idea — `project_hr1_improvements_task` option A |
| **Template matching** | Cross-correlation with an expected pulse shape | O(L), L ~ pulse length | Good at low SNR | **linear phase** (distortion misaligns the template) | own idea — option B |
| **Hilbert / envelope (WEPD)** | Analytic envelope locates the pulse; designed to cut both false positives and negatives | High (FFT or long filter) | Good | indifferent | Hilbert-transform PPG onset literature |
| **Waveform validation** *(complement)* | Accept a candidate only if width, area and rise/fall ratio are plausible | ~0 | Complementary | indifferent | own idea |
| **Adaptive refractory** *(complement)* | Refractory = f(estimated HR) instead of a fixed 0.185 s | ~0 | Complementary | indifferent | own idea |
| **Cross-check against HR2** *(complement)* | Accept an interval only if it agrees with the autocorrelation period already computed for HR2 | ~0 (already computed) | **Complementary, strong** | indifferent | own idea — specific to this library |

#### Recommendation

**TERMA**, with **SSF** as the simpler fallback. Reasons: O(1) cost (two incremental averages, like
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
  R-METHOD-2, r < `spo2_r_corr_min` (0.8) sets SpO2 = NaN, SQI = 0 and leaves R and r reported, so
  the reason is on record. Mapping r itself onto [0, 1] was rejected: r = 0.9 is a sound fit on this
  corpus (p1 of the window medians 0.944) and would have read as "half quality". The 0.8 stays
  provisional (§9 above): it costs nothing on the corpus and has not been measured on pulse-less
  input. r travels in `AFE4490Data::spo2_r_corr`; whether the `$M4` frame carries it is the
  firmware's call (its 288-byte slot has ~10 B of margin after the 7-figure OT).
- **The curve is fitted on the library's own R** (`fit_method2.py`, same pairs, stationarity rule
  and bin fit as the step-1 table): SpO2 = 123.98 − 39.13·R, pooled over 3455 windows of the two
  subjects — the same numbers §10 obtained through the runner's candidate (124.0 − 39.1·R), by a
  different path. ρ −0.80, scatter 1.65, median |residual| 0.91; per subject 129.2 − 46.7·R and
  118.8 − 32.1·R (leave-one-out median |error| 0.95 / 1.38). Below a reference of 85 % (31 windows,
  all dynamic) it reads −3.4 points: the low range is unmeasured, not fitted. It is a hospital-monitor-
  referenced curve for one probe model (Medle ST-S0163), not an ISO Annex DD desaturation study; the
  reference monitor's make and averaging were not recorded. Its id reuses
  `R-CURVE-STS0163-HOSPNAV-20260923`: the 2026-09-24 proposal under that id (115.17 − 21.54·R,
  R-METHOD-1) was never applied, and the pair (id, method) is what identifies a curve.
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

## Appendix — source rationale (to distill at consolidation)

This preliminary doc is distilled from working notes (Claude local memory), primarily: the HGAC design
(`project_agc_design`, `project_agc_design_history`), closed decisions (ENSEPGAIN/O10, `namespace adc`,
value-always analog state, ProbeState), the clinical/HW basis (thermal envelope, probe specs, preterm
foot dimensions, AFE selection), and the nomenclature dictionary. Full consolidation (edit to
documentation tone, dedupe, drop task cruft) is planned for the HGAC v1 freeze.
