# Experimental System Identification and PI/PID Control of a Tank Level Process

MATLAB/Simulink project for experimental system identification and PI/PID control design of an industrial tank level process.

> This repository is being prepared step by step. Full documentation and results will be added after code paths and outputs are verified.

## Project Structure

```text
tank-level-control-system-identification/
├── README.md
├── .gitignore
├── requirements.md
├── data/
│   ├── input_level_data.mat
│   └── output_level_data.mat
├── src/
│   ├── system_identification.m
│   ├── pid_controller_comparison.m
│   └── advanced_pid_parameters.m
├── simulink/
│   ├── pi_pid_control_simulink.slx
│   ├── setpoint_weighting_simulink.slx
│   └── anti_windup_simulink.slx
├── figures/
├── results/
└── docs/
```

## Current Status

- Experimental data files are organized in `data/`.
- MATLAB scripts are organized in `src/`.
- Simulink models are organized in `simulink/`.
- Simulink cache files (`*.slxc`) are intentionally ignored.
