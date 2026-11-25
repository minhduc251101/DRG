# DRG parameters: what each one does, and how it was chosen

All values live in [`config/drg.yaml`](../config/drg.yaml) (ROS) and
[`src/drg/params.py`](../src/drg/params.py) (Python). The defaults are the values used for
**every** result in the paper. The numbers on this page come from the development runs stored
with the paper's results (seed 42, 30 pairs per set).

## Tuning protocol

* **Tuning sets.** Five windowed sets of 30 consecutive scan pairs from two handcart sequences:
  carpark (cp) sets 1–3 (from frames 4470, 2000 and 3000; 12 Hz, Δt ≈ 0.083 s) and garden sets 1–2
  (from keyframes 0 and 41; Δt ≈ 1 s).
* **Criterion.** Mean translation error (TE) and rotation error (RE) per pair against the ground
  truth, compared with FPFH + TEASER++ on the same pairs.
* **Design choices** (descriptor, matching, robust loss, gate, refinement, fusion) were compared on
  carpark set 1 and garden set 1 ([table below](#design-choices-compared-on-the-tuning-sets)).
* **Constants.** The gate thresholds start from the measured RCS and Doppler-residual repeatability of
  the radar. The geometric constants start from standard FPFH / RANSAC settings (0.3 m = 3 × the
  0.1 m voxel). They were then varied, alone and in combination, on the two garden sets. At 1 s
  keyframes the fusion weight is only 0.08, so geometry carries the translation and these constants
  matter most there. No variant changed the mean TE by more than 0.3 mm, so the starting values
  were kept. The one variant that was marginally better adds an acceleration term to the fusion
  variance (−0.3 mm on garden 1, no change on garden 2). It was not adopted, so Eq. 8 of the paper
  has no acceleration term.
* **Freeze.** All values were frozen before any other set was run. The 32 other windowed sets
  (cp 4–9, garden 3–8, nyl 1–5, loop1/2/3 1–5) and all 51,940 every-scan pairs were run once with
  these values. They were **never re-tuned**, including for the car sequences (5–7 m/s).
* **Held-out check.** The ablation (paper, Table 2) is reported on the 12 untouched sets (cp 4–9,
  garden 3–8), where it reaches the same conclusions as on the tuning sets.

The paper's finding is that DRG is **insensitive** to everything that was tuned. The translation
gain comes from the Doppler fusion (stage 6), whose weight follows from Δt through Eq. 8 and was not
tuned.

## Parameters that were tuned

Mean TE [m] / RE [°] of each variant. The released value is in bold.

| Parameter | Meaning | Values tried | garden set 1 | garden set 2 | Kept |
|---|---|---|---|---|---|
| `tau_rcs`, `tau_residual` | Gate thresholds (Eq. 4) | **1 dB, 0.01 m/s** | **0.0399 / 0.426** | **0.0460 / 0.485** | 1 dB, 0.01 m/s |
| | | 0.5 dB, 0.005 m/s | 0.0401 / 0.426 | 0.0460 / 0.485 | |
| `inlier_distance` (r_max) | RANSAC inlier distance and IRLS truncation | **0.3 m** | 0.0399 / 0.426 | 0.0460 / 0.485 | 0.3 m |
| | | 0.2 m | 0.0399 / 0.426 | 0.0460 / 0.485 | |
| `cauchy_c` | Cauchy scale c (Eq. 6) | **0.1 m** | 0.0399 / 0.426 | 0.0460 / 0.485 | 0.1 m |
| | | 0.05 m | 0.0399 / 0.425 | 0.0459 / 0.485 | |
| fusion acceleration term | extra `(0.5 a dt²)²` in the Doppler variance | **0 (off)** | 0.0399 / 0.426 | 0.0460 / 0.485 | off |
| | | a = 0.3 m/s² | 0.0396 / 0.426 | 0.0460 / 0.485 | |

The combinations (0.2 m with 0.05 m, with the acceleration term, or both) gave the same values to
within 0.1 mm. FPFH + TEASER++ on the same pairs: garden 1 0.0389 / 0.443, garden 2 0.0509 / 0.498.

**Why these values.**

* **`tau_rcs` = 1 dB.** For returns that the ground-truth motion maps within 0.1 m of a return in the
  next frame, the Oculii Eagle reports a bit-identical RCS for 85 % (cp) and 88 % (garden) of them,
  against 0.1 % for random pairs. 1 dB therefore keeps essentially all true matches. The threshold is
  a property of the radar: **re-tune it for another sensor**, for example at the 95th percentile of
  |ΔRCS| over ground-truth matches.
* **`tau_residual` = 0.01 m/s.** This is 3–10 × the median residual difference of true matches.
  Halving both thresholds (0.5 dB, 0.005 m/s) changed nothing. Only the ego-compensated residual
  is gated, because raw Doppler changes with the sensor motion. On true against random matches, the
  residual difference separates them by a factor of 23.5 at 12 Hz, against 2.2 for the raw Doppler
  difference.
* **`inlier_distance` = 0.3 m** is 3 × the 0.1 m voxel of the FPFH radii (the usual FPFH/RANSAC
  setting). 0.2 m gave identical results.
* **`cauchy_c` = 0.1 m.** 0.05 m changed TE by at most 0.1 mm.
* **The gate itself** does not change the pose (next table, and ablation rows B vs C of the paper).
  It raises the correspondence precision on 1 s keyframes (58 % → 70 %) and halves their
  registration time.

## Design choices compared on the tuning sets

Mean TE [m] / RE [°]. Each "+ fusion" row adds the Doppler translation fusion of stage 6.

| Variant | carpark set 1 (12 Hz) | garden set 1 (1 s) |
|---|---|---|
| FPFH + TEASER++ (baseline) | 0.0173 / 0.227 | 0.0389 / 0.443 |
| Earlier pipeline: 37-D, feature-matching RANSAC + weighted Arun | 0.0210 / 0.244 | 0.0712 / 0.464 |
| 37-D, mutual NN, correspondence RANSAC + Cauchy IRLS, no gate | 0.0172 / 0.228 | 0.0395 / 0.426 |
| 37-D, mutual NN, correspondence RANSAC + Cauchy IRLS, gate | 0.0172 / 0.228 | 0.0395 / 0.426 |
| — Huber instead of Cauchy | 0.0172 / 0.228 | 0.0394 / 0.426 |
| — two-pass weighted Arun (earlier pipeline) instead of Cauchy IRLS | 0.0173 / 0.228 | 0.0394 / 0.425 |
| — 3 nearest neighbours instead of mutual NN | 0.0172 / 0.228 | 0.0391 / 0.425 |
| — 33-D FPFH instead of 37-D | 0.0172 / 0.228 | 0.0398 / 0.425 |
| 33-D FPFH, mutual NN, Cauchy IRLS, gate + fusion | 0.0138 / 0.228 | 0.0401 / 0.425 |
| 37-D, mutual NN, Cauchy IRLS, no gate + fusion | 0.0138 / 0.228 | 0.0399 / 0.426 |
| **37-D, mutual NN, Cauchy IRLS, gate + fusion (DRG)** | **0.0138 / 0.228** | **0.0399 / 0.426** |
| DRG + dense RCS-gated ICP refinement | 0.0138 / 0.229 | 0.0397 / 0.425 |

Two choices move the error beyond the noise of 30 pairs:

1. **Explicit mutual-NN correspondences with correspondence RANSAC**, instead of the earlier
   pipeline's feature-matching RANSAC: carpark 21.0 → 17.2 mm, garden 71.2 → 39.5 mm. This closes
   the gap to FPFH + TEASER++.
2. **The Doppler fusion**: −3.4 mm on carpark set 1 at 12 Hz, where its weight is 0.93. At 1 s
   keyframes (weight 0.08) it changes garden by +0.4 mm.

Every other choice changes the mean TE by less than 1 mm on both sets. The paper's ablation on the
12 held-out sets (Table 2) confirms this. Upstream REVE in place of Soft-REVE (ablation row K) also
changes little.

## Parameters that were not tuned

| Parameter | Value | Source |
|---|---|---|
| `sigma_g` | 0.03 m | Nominal accuracy of the geometric translation: the per-pair error on the tuning sets (TEASER++ gives 0.017–0.051 m there). Fixed before the evaluation; no other value was tried. |
| `sigma_v` | 0.10 m/s | Nominal ego-velocity accuracy. It matches the 0.04–0.14 m/s per-axis RMSE reported for RANSAC + least-squares refit on motion-capture sequences (RAVE, Štironja et al. 2024). |
| `reve_*` | 0.4 / 0.9999 / 0.15 m/s | Defaults of the REVE implementation (Doer & Trommer). They give 38 RANSAC iterations. |
| `sigma_floor`, `sigma_cap`, `zero_velocity` | 0.10, 0.15, 0.05 m/s | Carried over from the earlier pipeline (Soft-REVE). Not re-tuned for DRG. |
| `min/max_range`, `min_rcs`, `max_azimuth/elevation_deg` | 0.25–100 m, > 0 dB, 75°, 45° | Field of view of the radar. On the paper's input, practically only the 100 m cut removes points: 0.7 % (garden) to 13 % (loop3) of all returns. The 45° elevation limit removes < 0.05 %. |
| FPFH radii | 0.2 m normals, 0.5 m features | 2 × and 5 × a 0.1 m voxel. The FPFH + TEASER++ baseline uses the same radii. |
| `density_radius`, `zscore_clip`, `radar_block_weight` α | 0.3 m, 5, 0.5 | α = 0.5 weights the FPFH block and the radar block equally. |
| `ransac_*` | 40 trials, 50 000 iterations, 0.999, edge ratio 0.9 | Carried over from the earlier pipeline. **One trial** gives the same estimates on all 1109 windowed pairs (max difference 1.8e-7 m) in 0.38 s instead of 5.05 s per pair. Use `ransac_trials: 1` for speed. |
| `max_translation`, `max_roll_pitch_deg` | 5 m, 10° | Plausibility check on a hypothesis (ground vehicle, consecutive scans). This is the only failure detection. |

## Sensitivity of the fusion constants (after the fact)

Only the ratio σ_g / σ_v matters. Equal weight falls at Δt* = σ_g / σ_v = 0.3 s. At 12 Hz the
fusion weight is ω = 0.93; at 1 s keyframes it is 0.08. Re-fusing the stored estimates on a grid
σ_v ∈ [0.05, 0.8] m/s × σ_g ∈ [0.01, 0.3] m puts the nominal pair within 1.8 % of the best grid
point on every sequence. This grid was evaluated **after** the results, as a robustness check. It
was not used to choose the values.

## Adapting DRG to another radar

1. Check that Doppler is the **signed range rate, not ego-compensated**, and the sign convention
   (DRG uses d = −n·v: approaching static targets are negative). Flip the sign if needed.
2. Set the field-of-view limits (`max_azimuth_deg`, `max_elevation_deg`, `max_range`).
3. Re-measure the RCS repeatability and set `tau_rcs`. Set `tau_residual` to a few times the
   median residual difference of true matches. `scripts/gate_repeatability.py --dump <dump> --gt <gt.txt>`
   measures both and prints what the gate keeps for a grid of thresholds. On the Continental ARS548 of
   SNAIL-Radar (integer dBsm, median residual difference 0.012 m/s) the paper's values keep 34 % of the
   true matches; 2 dB and 0.05 m/s (`config/snail_radar/ars548_gate.yaml`) keep 83 %
   ([results/snail_radar/README.md](../results/snail_radar/README.md)).
4. Recompute the normalisation statistics (`scripts/compute_norm_stats.py`).
5. Keep `sigma_v` / `sigma_g` unless your frame rate or ego-velocity accuracy differs a lot. The
   weight then follows from Δt alone.
