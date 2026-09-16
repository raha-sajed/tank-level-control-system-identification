%% System Identification for Tank Level Process
% Experimental System Identification and PI/PID Control of a Tank Level Process
%
% This script:
%   1. Loads experimental input/output tank level data.
%   2. Estimates local FOPTD parameters from multiple step responses.
%   3. Selects a linear operating region.
%   4. Builds an average FOPTD model.
%   5. Validates the identified model against measured data.
%   6. Saves figures and numerical results for GitHub documentation.

clear; clc; close all;

%% Step 1: Project Paths

% Resolve project root automatically.
% This makes the script runnable from any MATLAB current folder.
projectRoot = fileparts(fileparts(mfilename('fullpath')));

dataDir    = fullfile(projectRoot, 'data');
figuresDir = fullfile(projectRoot, 'figures');
resultsDir = fullfile(projectRoot, 'results');

if ~exist(figuresDir, 'dir')
    mkdir(figuresDir);
end

if ~exist(resultsDir, 'dir')
    mkdir(resultsDir);
end

inputDataPath  = fullfile(dataDir, 'input_level_data.mat');
outputDataPath = fullfile(dataDir, 'output_level_data.mat');

%% Step 2: Load Experimental Data

input_struct  = load(inputDataPath);
output_struct = load(outputDataPath);

% Expected variables:
%   input_level  - input valve-opening signal
%   output_level - measured tank-level response
u_ts = input_struct.input_level;
y_ts = output_struct.output_level;

t_in  = u_ts.Time(:);
u     = u_ts.Data(:);
t_out = y_ts.Time(:);
y     = y_ts.Data(:);

%% Step 3: Plot Raw Input and Output Signals

figure('Name', 'System Step Response', ...
       'Color', 'w', ...
       'Position', [100 100 900 600]);

subplot(2,1,1);
plot(t_in, u, 'Color', [0.05 0.25 0.55], 'LineWidth', 1.5);
grid on; box on;
title('Input Signal');
xlabel('Time (s)');
ylabel('Valve Opening (%)');

subplot(2,1,2);
plot(t_out, y, 'Color', [0.00 0.45 0.30], 'LineWidth', 1.5);
grid on; box on;
title('Measured Tank Level Response');
xlabel('Time (s)');
ylabel('Tank Level (cm)');

exportgraphics(gcf, fullfile(figuresDir, 'system_step_response.png'), ...
    'Resolution', 300);

%% Step 4: Estimate Local FOPTD Parameters from Step Responses

% Step times used in the experiment
step_times = [400, 800, 1200, 1600, 2000, 2400];
num_steps = length(step_times);

K_vec     = zeros(num_steps, 1);
delay_vec = zeros(num_steps, 1);
tau_vec   = zeros(num_steps, 1);

for i = 1:num_steps

    t_step = step_times(i);

    if i < num_steps
        t_end = step_times(i+1) - 2;
    else
        t_end = max(t_out);
    end

    % Estimate input change around the step
    u_before_idx = find(t_in <= t_step - 5, 1, 'last');
    u_after_idx  = find(t_in >= t_step + 10, 1);

    delta_u = u(u_after_idx) - u(u_before_idx);

    % Estimate initial and final output levels using local averages
    y_init = mean(y(t_out >= (t_step - 20) & t_out <= (t_step - 2)));
    y_final = mean(y(t_out >= (t_end - 30) & t_out <= t_end));

    delta_y = y_final - y_init;

    % Static gain
    K_vec(i) = delta_y / delta_u;

    % Extract local response region
    idx_region = find(t_out >= t_step & t_out <= t_end);
    t_region = t_out(idx_region);
    y_region = y(idx_region);

    % Delay threshold: 2% of output change
    % Time constant threshold: 63.2% of output change
    if delta_y < 0
        idx_delay = find(y_region <= y_init + 0.02 * delta_y, 1);
        idx_tau   = find(y_region <= y_init + 0.632 * delta_y, 1);
    else
        idx_delay = find(y_region >= y_init + 0.02 * delta_y, 1);
        idx_tau   = find(y_region >= y_init + 0.632 * delta_y, 1);
    end

    if isempty(idx_delay)
        idx_delay = 1;
    end

    if isempty(idx_tau)
        idx_tau = 1;
    end

    t_start = t_region(idx_delay);

    delay_vec(i) = t_start - t_step;
    tau_vec(i) = t_region(idx_tau) - t_start;
end

%% Step 5: Display Identification Results

fprintf('\n=========================================================\n');
fprintf('Local FOPTD Parameter Estimation\n');
fprintf('=========================================================\n');
fprintf('%-6s %-14s %-18s %-14s\n', ...
    'Step', 'Gain K', 'Delay theta (s)', 'Tau (s)');
fprintf('---------------------------------------------------------\n');

for i = 1:num_steps
    fprintf('%-6d %-14.5f %-18.2f %-14.2f\n', ...
        i, K_vec(i), delay_vec(i), tau_vec(i));
end

fprintf('=========================================================\n');

%% Step 6: Select Operating Region and Build Average FOPTD Model

% Based on the experiment report, steps 3 to 6 are used as the
% approximately linear operating region.
op_idx = 3:6;

K_avg     = mean(K_vec(op_idx));
theta_avg = mean(delay_vec(op_idx));
tau_avg   = mean(tau_vec(op_idx));

fprintf('\n=========================================================\n');
fprintf('Average FOPTD Model in Selected Operating Region\n');
fprintf('=========================================================\n');
fprintf('Operating region steps: %d to %d\n', op_idx(1), op_idx(end));
fprintf('Average static gain K      = %.4f cm/%%\n', K_avg);
fprintf('Average delay theta        = %.2f s\n', theta_avg);
fprintf('Average time constant tau  = %.2f s\n', tau_avg);
fprintf('\nIdentified FOPTD model:\n');
fprintf('           %.4f * exp(-%.2f s)\n', K_avg, theta_avg);
fprintf('G(s) = -----------------------------\n');
fprintf('             %.2f s + 1\n', tau_avg);
fprintf('=========================================================\n');

%% Step 7: Save Identification Results

identificationTable = table( ...
    (1:num_steps)', ...
    K_vec, ...
    delay_vec, ...
    tau_vec, ...
    'VariableNames', { ...
        'Step', ...
        'StaticGain_K_cm_per_percent', ...
        'Delay_theta_s', ...
        'TimeConstant_tau_s'} ...
);

writetable(identificationTable, ...
    fullfile(resultsDir, 'identification_step_parameters.csv'));

modelSummaryTable = table( ...
    K_avg, ...
    theta_avg, ...
    tau_avg, ...
    'VariableNames', { ...
        'Average_StaticGain_K_cm_per_percent', ...
        'Average_Delay_theta_s', ...
        'Average_TimeConstant_tau_s'} ...
);

writetable(modelSummaryTable, ...
    fullfile(resultsDir, 'identified_foptd_model.csv'));

%% Step 8: Model Validation

% Interpolate input and output data to a common time vector
dt = mean(diff(t_in));
t_sim = (t_in(1):dt:t_in(end))';

u_sim = interp1(t_in, u, t_sim, 'linear', 'extrap');
y_true_sim = interp1(t_out, y, t_sim, 'linear', 'extrap');

% Build FOPTD transfer function
s = tf('s');
G_sys = (K_avg * exp(-theta_avg * s)) / (tau_avg * s + 1);

% Simulate model response to the measured input variation
y_model = y_true_sim(1) + lsim(G_sys, u_sim - u_sim(1), t_sim);

% Validation over selected operating region
t_op_start = step_times(op_idx(1));
t_op_end = max(t_out);

idx_val = find(t_sim >= t_op_start & ...
               t_sim <= t_op_end & ...
               ~isnan(y_true_sim) & ...
               ~isnan(y_model));

rmse = sqrt(mean((y_true_sim(idx_val) - y_model(idx_val)).^2));

fit_percent = max(0, ...
    (1 - rmse / std(y_true_sim(idx_val))) * 100);

fprintf('\n=========================================================\n');
fprintf('Model Validation Metrics\n');
fprintf('=========================================================\n');
fprintf('RMSE in operating region  = %.4f cm\n', rmse);
fprintf('Model fit percentage      = %.2f %%\n', fit_percent);
fprintf('=========================================================\n');

validationTable = table( ...
    rmse, ...
    fit_percent, ...
    'VariableNames', {'RMSE_cm', 'Fit_Percentage'} ...
);

writetable(validationTable, ...
    fullfile(resultsDir, 'model_validation_metrics.csv'));

%% Step 9: Plot Model Validation

figure('Name', 'Model Validation Analysis', ...
       'Color', 'w', ...
       'Position', [100 100 1000 600]);

y_min_val = min([y_true_sim; y_model]);
y_max_val = max([y_true_sim; y_model]);

% Highlight selected operating region
fill([t_op_start t_op_end t_op_end t_op_start], ...
     [y_min_val-2 y_min_val-2 y_max_val+2 y_max_val+2], ...
     [0.92 0.96 0.94], ...
     'EdgeColor', 'none', ...
     'FaceAlpha', 0.65, ...
     'DisplayName', 'Selected operating region');

hold on; grid on; box on;

hActual = plot(t_out, y, ...
    'Color', [0.05 0.25 0.55], ...
    'LineWidth', 1.8, ...
    'DisplayName', 'Measured output');

hModel = plot(t_sim, y_model, '--', ...
    'Color', [0.85 0.15 0.15], ...
    'LineWidth', 2.0, ...
    'DisplayName', 'FOPTD model');

xline(t_op_start, '-.', ...
    'Operating region start', ...
    'Color', [0.00 0.45 0.30], ...
    'LineWidth', 1.3);

text(t_op_start + 50, y_max_val - 3, ...
    sprintf('Validation Metrics\\nRMSE = %.4f cm\\nFit = %.2f%%', ...
    rmse, fit_percent), ...
    'FontSize', 10, ...
    'FontWeight', 'bold', ...
    'BackgroundColor', 'w', ...
    'EdgeColor', [0.2 0.2 0.2], ...
    'Margin', 6);

title('Model Validation: Measured Tank Level vs. FOPTD Model');
xlabel('Time (s)');
ylabel('Tank Level (cm)');
legend([hActual, hModel], {'Measured output', 'FOPTD model'}, ...
    'Location', 'northeast');

ylim([y_min_val - 2, y_max_val + 2]);

exportgraphics(gcf, fullfile(figuresDir, 'model_validation.png'), ...
    'Resolution', 300);

%% Step 10: Plot Estimated Parameters per Step

figure('Name', 'FOPTD Parameters per Step', ...
       'Color', 'w', ...
       'Position', [100 100 1000 700]);

subplot(3,1,1);
plot(1:num_steps, K_vec, '-o', ...
    'Color', [0.05 0.25 0.55], ...
    'MarkerFaceColor', [0.05 0.25 0.55], ...
    'LineWidth', 1.5);
grid on; box on;
ylabel('K (cm/%)');
title('Estimated FOPTD Parameters per Step');

subplot(3,1,2);
plot(1:num_steps, delay_vec, '-o', ...
    'Color', [0.85 0.32 0.10], ...
    'MarkerFaceColor', [0.85 0.32 0.10], ...
    'LineWidth', 1.5);
grid on; box on;
ylabel('\theta (s)');

subplot(3,1,3);
plot(1:num_steps, tau_vec, '-o', ...
    'Color', [0.00 0.45 0.30], ...
    'MarkerFaceColor', [0.00 0.45 0.30], ...
    'LineWidth', 1.5);
grid on; box on;
xlabel('Step Number');
ylabel('\tau (s)');

exportgraphics(gcf, fullfile(figuresDir, 'foptd_parameters_per_step.png'), ...
    'Resolution', 300);

%% Step 11: Plot Operating Region Analysis

figure('Name', 'Operating Region Analysis', ...
       'Color', 'w', ...
       'Position', [100 100 900 500]);

plot(1:num_steps, K_vec, '-o', ...
    'Color', [0.05 0.25 0.55], ...
    'MarkerFaceColor', [0.05 0.25 0.55], ...
    'LineWidth', 1.8);

hold on; grid on; box on;

xline(op_idx(1), '--', 'Operating region start', ...
    'Color', [0.00 0.45 0.30], ...
    'LineWidth', 1.3);

yline(K_avg, '--', ...
    sprintf('Average K = %.4f', K_avg), ...
    'Color', [0.85 0.15 0.15], ...
    'LineWidth', 1.3);

title('Operating Region Selection Based on Local Static Gain');
xlabel('Step Number');
ylabel('Static Gain K (cm/%)');
xlim([1 num_steps]);

exportgraphics(gcf, fullfile(figuresDir, 'operating_region_analysis.png'), ...
    'Resolution', 300);

%% Step 12: Final Status Message

fprintf('\nSystem identification completed successfully.\n');
fprintf('Figures saved to: %s\n', figuresDir);
fprintf('Results saved to: %s\n', resultsDir);