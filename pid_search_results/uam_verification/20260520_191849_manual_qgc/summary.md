# UAM Verification Summary

- Case: `manual_qgc`
- Verdict: `FAIL`
- Analysis phase: `armed_or_airborne`
- Duration: 570.250 s
- Samples: 11406 total, 9221 analyzed
- Analysis window: 20.467-570.317 s (549.850 s)
- External enabled fraction: 0.0000
- External enabled duration: 0.000 s
- First external enable: nan s

## Hover Metrics

- Altitude mean/std: 0.992 / 1.037 m
- Altitude min/max: -1.576 / 4.152 m
- Altitude RMSE vs target: 1.446 m
- XY drift mean/max/final: 9.929 / 15.380 / 14.369 m
- Roll RMS/max abs: 1.793 / 24.330 deg
- Pitch RMS/max abs: 3.805 / 35.469 deg

## Controller Metrics

- Rate error norm RMS/max: 0.000 / 0.000 rad/s
- Torque norm RMS/max: nan / nan
- RBFNN n_hat norm RMS/max: 0.000 / 0.000

## Arm Motion

- Motion detected: `True`
- Command seen: `False`
- Joint command norm RMS/max: nan / nan rad
- Joint command max span: nan rad
- Joint actual norm RMS/max: 0.034 / 0.053 rad
- Joint actual max span: 0.079 rad

## Failure Flags

- roll_or_pitch_gt_35_deg: `True`
- altitude_outside_1_to_3m: `True`
- xy_drift_gt_1m: `True`
- arm_command_without_motion: `False`

Timeseries CSV: `/home/wicom/PX4-Autopilot/Tools/simulation/gz/pid_search_results/uam_verification/20260520_191849_manual_qgc/flight_timeseries.csv`
