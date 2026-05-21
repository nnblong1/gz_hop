# UAM Verification Summary

- Case: `manual_qgc`
- Verdict: `FAIL`
- Analysis phase: `all_samples`
- Duration: 589.000 s
- Samples: 11781 total, 11781 analyzed
- Analysis window: 0.065-589.066 s (589.000 s)
- External enabled fraction: 0.0000
- External enabled duration: 0.000 s
- First external enable: nan s

## Hover Metrics

- Altitude mean/std: -0.053 / 0.018 m
- Altitude min/max: -0.111 / 0.006 m
- Altitude RMSE vs target: 2.053 m
- XY drift mean/max/final: 0.030 / 0.089 / 0.030 m
- Roll RMS/max abs: 0.013 / 0.051 deg
- Pitch RMS/max abs: 0.013 / 0.079 deg

## Controller Metrics

- Rate error norm RMS/max: 0.000 / 0.000 rad/s
- Torque norm RMS/max: nan / nan
- RBFNN n_hat norm RMS/max: 0.000 / 0.000

## Arm Motion

- Motion detected: `False`
- Command seen: `False`
- Joint command norm RMS/max: nan / nan rad
- Joint command max span: nan rad
- Joint actual norm RMS/max: 0.000 / 0.000 rad
- Joint actual max span: 0.000 rad

## Failure Flags

- roll_or_pitch_gt_35_deg: `False`
- altitude_outside_1_to_3m: `True`
- xy_drift_gt_1m: `False`
- arm_command_without_motion: `False`

Timeseries CSV: `/home/wicom/PX4-Autopilot/Tools/simulation/gz/pid_search_results/uam_verification/20260521_145225_manual_qgc/flight_timeseries.csv`
