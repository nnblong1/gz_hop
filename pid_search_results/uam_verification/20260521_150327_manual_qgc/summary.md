# UAM Verification Summary

- Case: `manual_qgc`
- Verdict: `FAIL`
- Analysis phase: `external_enabled_settled`
- Duration: 1017.750 s
- Samples: 20356 total, 12353 analyzed
- Analysis window: 281.613-899.213 s (617.600 s)
- External enabled fraction: 0.6089
- External enabled duration: 619.649 s
- First external enable: 279.564 s

## Hover Metrics

- Altitude mean/std: 2.316 / 1.158 m
- Altitude min/max: -0.085 / 3.748 m
- Altitude RMSE vs target: 1.201 m
- XY drift mean/max/final: 0.567 / 2.405 / 0.440 m
- Roll RMS/max abs: 1.446 / 14.113 deg
- Pitch RMS/max abs: 2.367 / 20.049 deg

## Controller Metrics

- Rate error norm RMS/max: 0.385 / 2.825 rad/s
- Torque norm RMS/max: 0.082 / 0.106
- RBFNN n_hat norm RMS/max: 0.000 / 0.000

## Arm Motion

- Motion detected: `False`
- Command seen: `True`
- Joint command norm RMS/max: 0.300 / 0.300 rad
- Joint command max span: 0.000 rad
- Joint actual norm RMS/max: 0.006 / 0.019 rad
- Joint actual max span: 0.019 rad

## Failure Flags

- roll_or_pitch_gt_35_deg: `False`
- altitude_outside_1_to_3m: `True`
- xy_drift_gt_1m: `True`
- arm_command_without_motion: `True`

Timeseries CSV: `/home/wicom/PX4-Autopilot/Tools/simulation/gz/pid_search_results/uam_verification/20260521_150327_manual_qgc/flight_timeseries.csv`
