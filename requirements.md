# Requirements

## Required Software

- MATLAB
- Simulink
- Control System Toolbox

## Required Data Files

The experimental data are not distributed with the repository. To run system identification, create a local `data/` directory and supply files you are authorized to use:

- `data/input_level_data.mat` containing an `input_level` timeseries (seconds, valve opening in percent).
- `data/output_level_data.mat` containing an `output_level` timeseries (seconds, tank level in centimetres).

The `data/` directory is ignored by Git. Controller comparison uses the recorded model parameters and does not require these raw data files.

## MATLAB Version

Add the exact MATLAB version used after running:

```matlab
ver
```
