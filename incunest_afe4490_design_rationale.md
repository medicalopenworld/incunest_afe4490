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

## Appendix — source rationale (to distill at consolidation)

This preliminary doc is distilled from working notes (Claude local memory), primarily: the HGAC design
(`project_agc_design`, `project_agc_design_history`), closed decisions (ENSEPGAIN/O10, `namespace adc`,
value-always analog state, ProbeState), the clinical/HW basis (thermal envelope, probe specs, preterm
foot dimensions, AFE selection), and the nomenclature dictionary. Full consolidation (edit to
documentation tone, dedupe, drop task cruft) is planned for the HGAC v1 freeze.
