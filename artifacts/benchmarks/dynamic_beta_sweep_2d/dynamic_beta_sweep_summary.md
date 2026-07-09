# Dynamic Beta Sweep Summary

This sweep varies obstacle speed in selected dynamic 2D scenarios and reports results against a fixed reference robot speed.

- Reference speed: `v_ref = 1.5 m/s`
- Sweep ratio: `beta_ref = nu / v_ref`
- Scenarios: `moving_circle_crossing`, `head_on_circle`

| Scenario | Method | Max Success Beta | Max Safe Beta | Best Final Err | Worst Min Clearance |
| --- | --- | ---: | ---: | ---: | ---: |
| moving_circle_crossing | MFI-PD | 1.000 | 1.000 | 0.150 | 0.835 |
| moving_circle_crossing | MFI-Geometric | 1.000 | 1.000 | 0.017 | 0.795 |
| moving_circle_crossing | APF | 1.000 | 1.000 | 0.150 | 1.031 |
| moving_circle_crossing | Haddadin | 1.000 | 1.000 | 0.021 | 0.045 |
| moving_circle_crossing | Sabattini | 1.000 | 1.000 | 0.150 | -0.014 |
| head_on_circle | MFI-PD | 1.000 | 1.000 | 0.150 | 0.302 |
| head_on_circle | MFI-Geometric | 0.900 | 0.900 | 0.001 | -0.007 |
| head_on_circle | APF | 1.000 | 1.000 | 0.152 | 0.654 |
| head_on_circle | Haddadin | - | - | 3.697 | -0.028 |
| head_on_circle | Sabattini | - | - | 3.697 | -0.028 |
