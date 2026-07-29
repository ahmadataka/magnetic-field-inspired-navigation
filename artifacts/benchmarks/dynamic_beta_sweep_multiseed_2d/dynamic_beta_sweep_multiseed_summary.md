# Dynamic Beta Sweep Multi-seed Summary

This sweep repeats selected dynamic 2D circle scenarios across seeded perturbations.

- seeds: `[0, 1, 2, 3, 4]`
- obstacle speeds: `[0.0, 0.3, 0.6, 0.9, 1.2, 1.5, 1.8]` m/s
- reference speed: `v_ref = 1.5 m/s`

| Scenario | Method | Beta | Success Rate | Collision Rate | Safety Viol Rate | Final Error | Min Clearance | Worst Clearance |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| moving_circle_crossing | MFI-PD | 0.000 | 1.000 | 0.000 | 0.000 | 0.151 ± 0.000 | 2.749 ± 0.033 | 2.715 |
| moving_circle_crossing | MFI-PD | 0.200 | 1.000 | 0.000 | 0.000 | 0.151 ± 0.000 | 1.729 ± 0.014 | 1.710 |
| moving_circle_crossing | MFI-PD | 0.400 | 1.000 | 0.000 | 0.000 | 0.150 ± 0.000 | 0.681 ± 0.224 | 0.473 |
| moving_circle_crossing | MFI-PD | 0.600 | 1.000 | 0.000 | 0.000 | 0.151 ± 0.001 | 1.690 ± 0.111 | 1.502 |
| moving_circle_crossing | MFI-PD | 0.800 | 1.000 | 0.000 | 0.000 | 0.151 ± 0.000 | 2.162 ± 0.320 | 1.835 |
| moving_circle_crossing | MFI-PD | 1.000 | 1.000 | 0.000 | 0.000 | 0.151 ± 0.000 | 2.642 ± 0.194 | 2.369 |
| moving_circle_crossing | MFI-PD | 1.200 | 1.000 | 0.000 | 0.000 | 0.151 ± 0.001 | 2.954 ± 0.213 | 2.693 |
| moving_circle_crossing | MFI-Geometric | 0.000 | 1.000 | 0.000 | 0.000 | 0.033 ± 0.000 | 2.363 ± 0.035 | 2.325 |
| moving_circle_crossing | MFI-Geometric | 0.200 | 1.000 | 0.000 | 0.000 | 0.150 ± 0.000 | 1.058 ± 0.005 | 1.053 |
| moving_circle_crossing | MFI-Geometric | 0.400 | 1.000 | 0.000 | 0.000 | 0.029 ± 0.012 | 1.743 ± 0.575 | 0.718 |
| moving_circle_crossing | MFI-Geometric | 0.600 | 1.000 | 0.000 | 0.000 | 0.034 ± 0.001 | 3.042 ± 0.103 | 2.884 |
| moving_circle_crossing | MFI-Geometric | 0.800 | 1.000 | 0.000 | 0.000 | 0.034 ± 0.000 | 3.814 ± 0.102 | 3.648 |
| moving_circle_crossing | MFI-Geometric | 1.000 | 1.000 | 0.000 | 0.000 | 0.033 ± 0.000 | 4.189 ± 0.091 | 4.041 |
| moving_circle_crossing | MFI-Geometric | 1.200 | 1.000 | 0.000 | 0.000 | 0.033 ± 0.000 | 4.390 ± 0.082 | 4.255 |
| moving_circle_crossing | APF | 0.000 | 1.000 | 0.000 | 0.000 | 0.151 ± 0.000 | 2.169 ± 0.044 | 2.126 |
| moving_circle_crossing | APF | 0.200 | 1.000 | 0.000 | 0.000 | 0.151 ± 0.000 | 1.269 ± 0.035 | 1.224 |
| moving_circle_crossing | APF | 0.400 | 1.000 | 0.000 | 0.000 | 0.151 ± 0.000 | 1.048 ± 0.014 | 1.031 |
| moving_circle_crossing | APF | 0.600 | 1.000 | 0.000 | 0.000 | 0.151 ± 0.000 | 1.121 ± 0.051 | 1.077 |
| moving_circle_crossing | APF | 0.800 | 1.000 | 0.000 | 0.000 | 0.151 ± 0.000 | 1.461 ± 0.165 | 1.277 |
| moving_circle_crossing | APF | 1.000 | 1.000 | 0.000 | 0.000 | 0.151 ± 0.000 | 2.078 ± 0.322 | 1.696 |
| moving_circle_crossing | APF | 1.200 | 1.000 | 0.000 | 0.000 | 0.151 ± 0.000 | 2.695 ± 0.354 | 2.257 |
| moving_circle_crossing | Haddadin | 0.000 | 1.000 | 0.000 | 0.000 | 0.151 ± 0.000 | 2.376 ± 0.036 | 2.340 |
| moving_circle_crossing | Haddadin | 0.200 | 1.000 | 0.000 | 0.000 | 0.150 ± 0.000 | 1.082 ± 0.073 | 0.984 |
| moving_circle_crossing | Haddadin | 0.400 | 0.000 | 0.600 | 1.000 | 3.779 ± 3.342 | -0.005 ± 0.048 | -0.076 |
| moving_circle_crossing | Haddadin | 0.600 | 1.000 | 0.000 | 0.000 | 0.151 ± 0.000 | 0.619 ± 0.296 | 0.364 |
| moving_circle_crossing | Haddadin | 0.800 | 1.000 | 0.000 | 0.000 | 0.151 ± 0.000 | 1.676 ± 0.362 | 1.218 |
| moving_circle_crossing | Haddadin | 1.000 | 1.000 | 0.000 | 0.000 | 0.151 ± 0.000 | 2.349 ± 0.286 | 1.977 |
| moving_circle_crossing | Haddadin | 1.200 | 1.000 | 0.000 | 0.000 | 0.151 ± 0.000 | 2.847 ± 0.313 | 2.463 |
| moving_circle_crossing | Sabattini | 0.000 | 1.000 | 0.000 | 0.000 | 0.151 ± 0.000 | 2.169 ± 0.044 | 2.126 |
| moving_circle_crossing | Sabattini | 0.200 | 1.000 | 0.000 | 0.000 | 0.151 ± 0.000 | 1.371 ± 0.288 | 0.857 |
| moving_circle_crossing | Sabattini | 0.400 | 0.200 | 0.800 | 0.800 | 5.426 ± 2.983 | 0.228 ± 0.517 | -0.007 |
| moving_circle_crossing | Sabattini | 0.600 | 1.000 | 0.000 | 0.000 | 0.151 ± 0.001 | 1.392 ± 0.121 | 1.250 |
| moving_circle_crossing | Sabattini | 0.800 | 1.000 | 0.000 | 0.000 | 0.151 ± 0.000 | 1.655 ± 0.067 | 1.603 |
| moving_circle_crossing | Sabattini | 1.000 | 1.000 | 0.000 | 0.000 | 0.151 ± 0.000 | 2.238 ± 0.312 | 1.877 |
| moving_circle_crossing | Sabattini | 1.200 | 1.000 | 0.000 | 0.000 | 0.151 ± 0.000 | 2.812 ± 0.347 | 2.382 |
| head_on_circle | MFI-PD | 0.000 | 0.000 | 0.000 | 0.000 | 1.450 ± 0.008 | 1.885 ± 0.001 | 1.884 |
| head_on_circle | MFI-PD | 0.200 | 1.000 | 0.000 | 0.000 | 0.150 ± 0.000 | 1.560 ± 0.077 | 1.482 |
| head_on_circle | MFI-PD | 0.400 | 1.000 | 0.000 | 0.000 | 0.151 ± 0.000 | 1.280 ± 0.101 | 1.210 |
| head_on_circle | MFI-PD | 0.600 | 1.000 | 0.000 | 0.000 | 0.151 ± 0.000 | 1.063 ± 0.079 | 1.012 |
| head_on_circle | MFI-PD | 0.800 | 1.000 | 0.000 | 0.000 | 0.151 ± 0.001 | 0.851 ± 0.077 | 0.790 |
| head_on_circle | MFI-PD | 1.000 | 1.000 | 0.000 | 0.000 | 0.151 ± 0.000 | 0.642 ± 0.082 | 0.559 |
| head_on_circle | MFI-PD | 1.200 | 1.000 | 0.000 | 0.000 | 0.151 ± 0.000 | 0.457 ± 0.099 | 0.326 |
| head_on_circle | MFI-Geometric | 0.000 | 0.000 | 0.000 | 0.000 | 1.351 ± 0.004 | 1.193 ± 0.002 | 1.191 |
| head_on_circle | MFI-Geometric | 0.200 | 1.000 | 0.000 | 0.000 | 0.034 ± 0.003 | 1.055 ± 0.006 | 1.051 |
| head_on_circle | MFI-Geometric | 0.400 | 1.000 | 0.000 | 0.000 | 0.028 ± 0.001 | 0.833 ± 0.013 | 0.823 |
| head_on_circle | MFI-Geometric | 0.600 | 1.000 | 0.000 | 0.000 | 0.024 ± 0.001 | 0.575 ± 0.026 | 0.554 |
| head_on_circle | MFI-Geometric | 0.800 | 1.000 | 0.000 | 0.000 | 0.014 ± 0.004 | 0.319 ± 0.037 | 0.290 |
| head_on_circle | MFI-Geometric | 1.000 | 1.000 | 0.000 | 0.000 | 0.056 ± 0.011 | 0.172 ± 0.006 | 0.167 |
| head_on_circle | MFI-Geometric | 1.200 | 0.400 | 0.000 | 0.400 | 9862327370410906935567771873837056.000 ± 21391141898737305573005654541467648.000 | 0.065 ± 0.057 | 0.001 |
| head_on_circle | APF | 0.000 | 0.000 | 0.000 | 0.000 | 4.210 ± 0.000 | 1.279 ± 0.001 | 1.278 |
| head_on_circle | APF | 0.200 | 0.200 | 0.000 | 0.000 | 31.669 ± 17.619 | 1.046 ± 0.004 | 1.043 |
| head_on_circle | APF | 0.400 | 0.200 | 0.000 | 0.000 | 60.254 ± 33.599 | 0.903 ± 0.002 | 0.899 |
| head_on_circle | APF | 0.600 | 0.200 | 0.000 | 0.000 | 88.911 ± 49.619 | 0.790 ± 0.008 | 0.786 |
| head_on_circle | APF | 0.800 | 0.400 | 0.000 | 0.000 | 88.153 ± 80.334 | 0.714 ± 0.008 | 0.708 |
| head_on_circle | APF | 1.000 | 1.000 | 0.000 | 0.000 | 0.151 ± 0.001 | 0.656 ± 0.003 | 0.653 |
| head_on_circle | APF | 1.200 | 1.000 | 0.000 | 0.000 | 0.151 ± 0.001 | 0.599 ± 0.005 | 0.596 |
| head_on_circle | Haddadin | 0.000 | 1.000 | 0.000 | 0.000 | 0.150 ± 0.000 | 0.650 ± 0.000 | 0.650 |
| head_on_circle | Haddadin | 0.200 | 1.000 | 0.000 | 0.000 | 0.150 ± 0.000 | 0.569 ± 0.017 | 0.553 |
| head_on_circle | Haddadin | 0.400 | 1.000 | 0.000 | 0.000 | 0.151 ± 0.000 | 0.307 ± 0.023 | 0.288 |
| head_on_circle | Haddadin | 0.600 | 1.000 | 0.000 | 0.000 | 0.144 ± 0.014 | 0.114 ± 0.031 | 0.092 |
| head_on_circle | Haddadin | 0.800 | 1.000 | 0.000 | 0.000 | 0.128 ± 0.019 | 0.074 ± 0.014 | 0.057 |
| head_on_circle | Haddadin | 1.000 | 0.000 | 0.200 | 1.000 | 1.744 ± 3.623 | 0.017 ± 0.040 | -0.048 |
| head_on_circle | Haddadin | 1.200 | 0.200 | 0.600 | 0.800 | 5.212 ± 4.629 | -0.022 ± 0.075 | -0.135 |
| head_on_circle | Sabattini | 0.000 | 0.000 | 0.000 | 0.000 | 2.427 ± 0.516 | 1.632 ± 0.004 | 1.624 |
| head_on_circle | Sabattini | 0.200 | 1.000 | 0.000 | 0.000 | 0.150 ± 0.000 | 0.904 ± 0.021 | 0.885 |
| head_on_circle | Sabattini | 0.400 | 1.000 | 0.000 | 0.000 | 0.151 ± 0.000 | 0.623 ± 0.038 | 0.589 |
| head_on_circle | Sabattini | 0.600 | 1.000 | 0.000 | 0.000 | 0.151 ± 0.000 | 0.421 ± 0.042 | 0.388 |
| head_on_circle | Sabattini | 0.800 | 1.000 | 0.000 | 0.000 | 0.151 ± 0.000 | 0.232 ± 0.058 | 0.186 |
| head_on_circle | Sabattini | 1.000 | 0.600 | 0.000 | 0.400 | 0.151 ± 0.000 | 0.072 ± 0.063 | 0.017 |
| head_on_circle | Sabattini | 1.200 | 0.000 | 0.800 | 1.000 | 7.146 ± 3.918 | 0.001 ± 0.024 | -0.014 |
