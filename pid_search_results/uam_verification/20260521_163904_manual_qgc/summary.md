# UAM Verification Summary

- Case: `manual_qgc`
- Verdict: `FAIL`
- Analysis phase: `armed_or_airborne`
- Duration: 238.750 s
- Samples: 4776 total, 2105 analyzed
- Analysis window: 133.649-238.848 s (105.199 s)
- External enabled fraction: 0.0000
- External enabled duration: 0.000 s
- First external enable: nan s

## Hover Metrics

- Altitude mean/std: 2.414 / 1.173 m
- Altitude min/max: 0.038 / 3.061 m
- Altitude RMSE vs target: 1.244 m
- XY drift mean/max/final: 0.054 / 0.484 / 0.426 m
- Roll RMS/max abs: 0.483 / 7.723 deg
- Pitch RMS/max abs: 0.583 / 7.655 deg

## Controller Metrics

- Rate error norm RMS/max: 0.000 / 0.000 rad/s
- Torque norm RMS/max: nan / nan
- RBFNN n_hat norm RMS/max: 0.000 / 0.000

## Arm Motion

- Motion detected: `True`
- Command seen: `True`
- Joint command norm RMS/max: 0.053 / 0.065 rad
- Joint command max span: 0.066 rad
- Joint actual norm RMS/max: 0.021 / 0.065 rad
- Joint actual max span: 0.066 rad

## Failure Flags

- roll_or_pitch_gt_35_deg: `False`
- altitude_outside_1_to_3m: `True`
- xy_drift_gt_1m: `False`
- arm_command_without_motion: `False`

Timeseries CSV: `/home/wicom/PX4-Autopilot/Tools/simulation/gz/pid_search_results/uam_verification/20260521_163904_manual_qgc/flight_timeseries.csv`
