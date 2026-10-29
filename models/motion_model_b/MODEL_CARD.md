# Motion Model B

Version: `aai16-motion-model-b-0.1.0`.

Scope: flat stationary desk REST (0) versus continuous hand movement/shaking MOVING (1).

Trained on 168 real windows: 58 REST and 110 MOVING, from one REST and two MOVING sessions. Held-out data is never read by training or calibration. All following scores are training-set resubstitution; overlapping windows are correlated. These scores do not establish generalization or project accuracy compliance.

Keras accuracy: 1.000000; INT8 accuracy: 1.000000; RMS baseline accuracy: 1.000000.

Architecture: 60 -> Dense(8, ReLU) -> Dense(2, logits), 506 parameters. Adam, learning rate 0.001, 150 epochs, batch 16, seed 20261028, inverse-frequency class weights. Final epoch; no validation/tuning split.

See input_contract.json for the chronological float32 per-axis mean, clamp +/-2g, /2, sample-major contract. See quantization.json for actual tensor scales/zero points and operators. Calibration uses training windows only; new movement can exceed its observed range.

Freeze this bundle before evaluating rest_orientation_02.csv (the correctly recorded retry) and moving_directions_03.csv. Run tools/ml_training/evaluate.py in its default independent mode, without retraining, threshold tuning, or recalibration. Keep the first evaluation result.

This bundle has only been exercised on the laptop. Hardware tensor arena size, latency, and firmware inference remain unverified. Host runtime compatibility results and independent scores belong in a separate report after this bundle is frozen. See tools/ml_training/README.md for reproduction, results, and deployment requirements.
