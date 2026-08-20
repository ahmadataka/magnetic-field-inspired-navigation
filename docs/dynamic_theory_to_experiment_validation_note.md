# Dynamic Theory-to-Experiment Validation Note

Date: 2026-08-20

This note consolidates the targeted dynamic 2D sensitivity studies that most directly connect the dynamic MFI theory discussion to numerical evidence.

Use this file in the paper thread when writing or revising:

- the main dynamic interpretation,
- the theory-to-experiment bridge,
- the discussion of what is and is not validated numerically.

## 1. Scope

This note focuses only on the cleanest theory-linked benchmark family:

- `dynamic_double_integrator_2d`

The main purpose is not to claim a full proof for dynamic obstacles. Instead, it shows that the predicted qualitative sensitivities appear in controlled numerical studies under the present benchmark assumptions.

The four targeted studies are:

- `beta` sensitivity
- `alpha` sensitivity
- heading-margin sensitivity
- curvature sensitivity

The strongest scenarios for this purpose are:

- `moving_circle_crossing`
- `head_on_circle`

These two scenarios are used repeatedly because they isolate dynamic obstacle interaction more clearly than mixed clutter scenes.

## 2. Canonical artifact paths

### 2.1 Beta sweep

- raw csv:
  `artifacts/benchmarks/dynamic_beta_sweep_multiseed_2d/dynamic_beta_sweep_multiseed_raw.csv`
- summary csv:
  `artifacts/benchmarks/dynamic_beta_sweep_multiseed_2d/dynamic_beta_sweep_multiseed_summary.csv`
- summary md:
  `artifacts/benchmarks/dynamic_beta_sweep_multiseed_2d/dynamic_beta_sweep_multiseed_summary.md`
- figure:
  `artifacts/benchmarks/dynamic_beta_sweep_multiseed_2d/dynamic_beta_sweep_multiseed.png`

### 2.2 Alpha sweep

- raw csv:
  `artifacts/benchmarks/dynamic_alpha_sweep_2d/dynamic_alpha_sweep_raw.csv`
- summary csv:
  `artifacts/benchmarks/dynamic_alpha_sweep_2d/dynamic_alpha_sweep_summary.csv`
- summary md:
  `artifacts/benchmarks/dynamic_alpha_sweep_2d/dynamic_alpha_sweep_summary.md`
- figure:
  `artifacts/benchmarks/dynamic_alpha_sweep_2d/dynamic_alpha_sweep.png`

### 2.3 Heading-margin sweep

- raw csv:
  `artifacts/benchmarks/dynamic_heading_margin_2d/dynamic_heading_margin_raw.csv`
- summary csv:
  `artifacts/benchmarks/dynamic_heading_margin_2d/dynamic_heading_margin_summary.csv`
- summary md:
  `artifacts/benchmarks/dynamic_heading_margin_2d/dynamic_heading_margin_summary.md`
- figure:
  `artifacts/benchmarks/dynamic_heading_margin_2d/dynamic_heading_margin.png`

### 2.4 Curvature sweep

- raw csv:
  `artifacts/benchmarks/dynamic_curvature_sweep_2d/dynamic_curvature_sweep_raw.csv`
- summary csv:
  `artifacts/benchmarks/dynamic_curvature_sweep_2d/dynamic_curvature_sweep_summary.csv`
- summary md:
  `artifacts/benchmarks/dynamic_curvature_sweep_2d/dynamic_curvature_sweep_summary.md`
- figure:
  `artifacts/benchmarks/dynamic_curvature_sweep_2d/dynamic_curvature_sweep.png`

## 3. Shared experimental assumptions

Common features across these studies:

- model: 2D double integrator
- local dynamic obstacle benchmark family
- seeds: `[0, 1, 2, 3, 4]` for the multi-seed targeted studies
- MFI variants included:
  - `MFI-PD`
  - `MFI-Geometric`
- baseline comparators included:
  - `APF`
  - `Haddadin`
  - `Sabattini`

Important interpretation rule:

- the MFI-targeted sweeps change the MFI-side theory-relevant quantity,
- the baseline methods are still plotted because they provide context,
- but a given sweep parameter is not necessarily meaningful for the baseline methods in the same mechanistic sense.

## 4. Beta study

### 4.1 Theoretical quantity

- reported ratio:
  `beta_ref = nu / v_ref`
- interpretation:
  obstacle speed relative to robot reference speed

### 4.2 What the data supports

This is the clearest dynamic theory-aligned study already available.

Key observations from the retained summary:

- `MFI-PD` keeps full success on both selected scenarios up to `beta_ref = 1.0`.
- `MFI-Geometric` also remains strong, though it is slightly weaker in the hardest head-on condition.
- `head_on_circle` is consistently the more difficult case.

Publication-safe interpretation:

- dynamic MFI performance degrades with harder obstacle closing conditions in a structured way,
- but the paper-style MFI controllers remain robust over a meaningful range of speed ratios in the clean 2D benchmark.

### 4.3 Suggested paper phrasing

Good claim:

- “The numerical dynamic 2D results are consistent with the speed-ratio intuition captured by the theoretical discussion.”

Avoid overclaiming:

- do not write that the beta sweep proves a full dynamic-obstacle convergence theorem.

## 5. Alpha study

### 5.1 Theoretical quantity

- reported quantity:
  `alpha_eff = c_perp / (m v_ref^2)`

This is the most direct numerical proxy in the present codebase for how strongly the collision-avoidance term influences the response relative to the nominal actuation scale.

### 5.2 Main findings

For `moving_circle_crossing`:

- `MFI-PD` stays at `100%` success across the whole sweep.
- Increasing `alpha_eff` increases clearance substantially.
- In the retained sweep, `MFI-PD` mean clearance rises from about `0.55 m` at `alpha_eff = 4.444` to about `1.228 m` at `alpha_eff = 17.778`.

For `head_on_circle`:

- `MFI-PD` also stays at `100%` success.
- Clearance does not improve monotonically with larger `alpha_eff`.
- Lower-to-mid `alpha_eff` values can outperform more aggressive values in the strict head-on case.

For `MFI-Geometric`:

- `moving_circle_crossing` is almost insensitive to the sweep in the current setup,
- while `head_on_circle` shows a clear monotone clearance increase with larger `alpha_eff`.

### 5.3 Interpretation

This supports a useful paper message:

- stronger transverse avoidance action is beneficial in some dynamic geometries,
- but “larger is always better” is not true for all encounter types,
- especially for head-on interactions where the balance between goal progress and transverse action still matters.

## 6. Heading-margin study

### 6.1 Theoretical quantity

This study varies the robot's initial heading margin relative to the nominal goal direction.

This is the best current numerical probe for the directional geometry of the encounter.

### 6.2 Main findings

For `MFI-PD`:

- success remains `100%` across all tested margins in both selected scenarios,
- but the clearance profile changes strongly with encounter geometry.

For `moving_circle_crossing`:

- mean clearance drops from about `1.273 m` at `-60 deg` to about `0.327 m` at `60 deg`,
- and the time-to-goal tends to increase near the harder heading conditions.

For `head_on_circle`:

- clearance is smallest near `0 deg`,
- and increases again as the initial heading moves away from the head-on alignment.

For `MFI-Geometric`:

- performance is more uniform across heading margin in the current setup,
- with little change in success and only modest clearance variation.

For `APF`:

- the strict head-on case at `0 deg` fails in the retained study,
- while offset initial headings recover success.

### 6.3 Interpretation

This is one of the strongest pieces of evidence for the paper because it numerically confirms:

- head-on alignment is the hardest dynamic geometry,
- the dynamic interaction is not governed only by speed magnitude,
- directional approach geometry matters materially.

## 7. Curvature study

### 7.1 Theoretical quantity

- reported quantity:
  `kappa_eff = 1 / R`

This study changes the moving circular obstacle radius and uses inverse radius as the effective curvature measure.

### 7.2 Main findings

For `moving_circle_crossing`:

- `MFI-PD` stays at `100%` success across the sweep,
- and larger obstacles in this crossing geometry produce larger clearances.
- In the retained sweep, mean `MFI-PD` clearance rises from about `0.818 m` at `R = 0.63` to about `0.995 m` at `R = 1.785`.

For `head_on_circle`:

- `MFI-PD` again stays at `100%` success,
- but larger obstacles now reduce the available minimum clearance.
- Mean `MFI-PD` clearance falls from about `1.337 m` at `R = 0.6` to about `0.517 m` at `R = 1.7`.

For `MFI-Geometric`:

- success also remains `100%`,
- but the trade-off is sharper in the crossing case, where very large obstacles lead to much tighter late-stage bypass behavior.

For `APF`:

- `head_on_circle` remains weak for smaller and medium radii,
- and only recovers full success for the larger-radius end of the retained sweep.

### 7.3 Interpretation

This study is useful because it shows:

- obstacle size/curvature sensitivity depends on encounter geometry,
- crossing and head-on cases cannot be summarized by a single monotone rule,
- yet the MFI controllers remain structurally robust in both scenarios.

## 8. Overall takeaways for the manuscript

### 8.1 What is strongly supported

The combined evidence strongly supports the following restrained claim:

- the dynamic 2D experiments are consistent with the theory-inspired dependence on relative speed, directional encounter geometry, avoidance gain scaling, and obstacle curvature.

### 8.2 What is not yet proved

The current experiments do not establish:

- a formal dynamic-obstacle avoidance theorem,
- a formal dynamic goal-convergence theorem,
- a universal guarantee for nonholonomic, 3D, or Webots embodiments.

Those other benchmark families should remain supporting evidence, not the main theoretical claim.

### 8.3 Best main-claim framing

Recommended framing:

- the theory section remains primarily about the static-obstacle guarantee structure,
- the dynamic paper can then argue that targeted 2D dynamic experiments show numerically consistent sensitivity trends with the proposed dynamic interpretation.

This is stronger and cleaner than claiming a full dynamic proof that the present manuscript does not actually contain.

## 9. Recommended figures to use in the paper thread

If the paper only has room for a small number of theory-linked figures, prioritize:

1. `dynamic_beta_sweep_multiseed_2d.png`
2. `dynamic_heading_margin_2d.png`
3. `dynamic_alpha_sweep_2d.png`
4. `dynamic_curvature_sweep_2d.png`

Recommended interpretation order:

1. `beta` for relative speed
2. heading margin for encounter geometry
3. `alpha` for controller transverse-action scaling
4. curvature for obstacle-shape sensitivity

## 10. Best one-paragraph summary for the paper agent

Suggested internal summary:

“The strongest theory-to-experiment evidence currently comes from the 2D double-integrator dynamic studies. Across multi-seed sweeps in relative obstacle speed, heading margin, avoidance-gain scaling, and obstacle curvature, the paper-style MFI controllers exhibit structured sensitivity trends that are consistent with the intended dynamic interpretation. In particular, head-on encounters are numerically confirmed to be the hardest geometry, stronger transverse action improves robustness in several cases but does not produce a uniform monotone benefit in every scenario, and obstacle curvature effects depend on whether the interaction is crossing or head-on. These studies support a restrained empirical claim of theory-consistent dynamic behavior, rather than a formal dynamic proof.”
