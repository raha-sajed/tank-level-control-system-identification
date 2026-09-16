%% Lab 3: Setpoint Weighting and Anti-Windup PID Control
% Author: Process Control Lab
% Clear environment
clear; clc; close all;

%% 1. Plant Identification Parameters (From Lab 2)
% First-Order Plus Time-Delay (FOPTD) Model Parameters
K_sys = 3.5682;       % Static Gain (cm/%)
tau_sys = 33.38;      % Time Constant (sec)
theta_sys = 13.50;    % Time Delay (sec)

% Transfer function of the un-delayed plant: Gp(s) = K / (tau*s + 1)
num_plant = K_sys;
den_plant = [tau_sys, 1];
Gp = tf(num_plant, den_plant);

%% 2. PID Tuning Parameters (CHR Disturbance Rejection / ZN)
% Example values using CHR / ZN Disturbance Rejection PID gains
Kp = 0.8306;          % Proportional Gain
Ti = 21.0371;         % Integral Time Constant (sec)
Td = 4.2235;          % Derivative Time Constant (sec)

Ki = Kp / Ti;         % Integral Gain
Kd = Kp * Td;         % Derivative Gain

% Derivative Filter Coefficient (N = 10 -> Tf = 0.1 * Td)
N = 10;
Tf = 0.1 * Td;

%% 3. Advanced Controller Parameters
% Setpoint weighting factor (a in [0, 1])
% a = 1: Standard PID
% a = 0: Setpoint applied only to Integral term (No P-kick on reference step)
a = 0.4;              % Chosen weighting factor for setpoint weighting

% Anti-Windup Tracking Time Constant (Tt)
% Standard choice: Tt = sqrt(Ti * Td)
Tt = sqrt(Ti * Td);   % Tracking gain Kb = 1 / Tt
Kb = 1 / Tt;

%% 4. Actuator Saturation Limits (Control Valve Limits)
U_max = 100;          % Maximum valve opening percentage (%)
U_min = 0;            % Minimum valve opening percentage (%)

%% 5. Simulation Setup Parameters
T_sim = 3500;         % Total simulation time (sec)

% Display parameters in Command Window
fprintf('=== Controller & System Parameters Initialized ===\n');
fprintf('Kp: %.4f, Ti: %.4f s, Td: %.4f s\n', Kp, Ti, Td);
fprintf('Setpoint Weight (a): %.2f\n', a);
fprintf('Anti-Windup Time Constant (Tt): %.4f s (Kb = %.4f)\n', Tt, Kb);