# UAM Verification Summary

- Case: `manual_qgc`
- Verdict: `FAIL`
- Analysis phase: `external_enabled_settled`
- Duration: 592.550 s
- Samples: 11852 total, 5187 analyzed
- Analysis window: 325.524-584.824 s (259.300 s)
- External enabled fraction: 0.4411
- External enabled duration: 261.349 s
- First external enable: 323.475 s

## Hover Metrics

- Altitude mean/std: 2.874 / 0.709 m
- Altitude min/max: -1.453 / 3.064 m
- Altitude RMSE vs target: 1.126 m
- XY drift mean/max/final: 0.243 / 1.090 / 0.595 m
- Roll RMS/max abs: 0.098 / 0.536 deg
- Pitch RMS/max abs: 1.213 / 10.695 deg

## Controller Metrics

- Rate error norm RMS/max: 0.175 / 1.777 rad/s
- Torque norm RMS/max: 0.081 / 0.083
- RBFNN n_hat norm RMS/max: 0.004 / 0.019

## Arm Motion

- Motion detected: `True`
- Command seen: `True`
- Joint command norm RMS/max: 0.274 / 0.600 rad
- Joint command max span: 0.600 rad
- Joint actual norm RMS/max: 0.179 / 0.597 rad
- Joint actual max span: 1.194 rad

## Failure Flags

- roll_or_pitch_gt_35_deg: `False`
- altitude_outside_1_to_3m: `True`
- xy_drift_gt_1m: `True`
- arm_command_without_motion: `False`

Timeseries CSV: `/home/wicom/PX4-Autopilot/Tools/simulation/gz/pid_search_results/uam_verification/20260521_144224_manual_qgc/flight_timeseries.csv`
