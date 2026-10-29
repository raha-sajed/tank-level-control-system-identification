%% System Identification for Tank Level Process
% Experimental System Identification and PI/PID Control of a Tank Level Process
%
% This script:
%   1. Loads experimental input/output tank level data.
%   2. Estimates local FOPTD parameters from multiple step responses.
%   3. Selects a linear operating region.
%   4. Builds an average FOPTD model.
%   5. Validates the identified model against measured data.
%   6. Exports figures and numerical results for reproducible analysis.

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

figRaw = figure('Name', 'System Step Response', ...
                'Color', 'w', ...
                'Position', [100 100 1000 650]);

tiledlayout(2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

nexttile;
plot(t_in, u, ...
    'Color', [0.05 0.25 0.55], ...
    'LineWidth', 1.6);
hold on; grid on; box on;

raw_step_times = [400 800 1200 1600 2000 2400];

for k = 1:numel(raw_step_times)
    xline(raw_step_times(k), ':', ...
        'Color', [0.45 0.45 0.45], ...
        'LineWidth', 0.9);
end

title('Experimental Input Signal');
xlabel('Time (s)');
ylabel('Valve Opening (%)');

nexttile;
plot(t_out, y, ...
    'Color', [0.00 0.45 0.30], ...
    'LineWidth', 1.6);
hold on; grid on; box on;

raw_step_times = [400 800 1200 1600 2000 2400];

for k = 1:numel(raw_step_times)
    xline(raw_step_times(k), ':', ...
        'Color', [0.45 0.45 0.45], ...
        'LineWidth', 0.9);
end

title('Measured Tank Level Response');
xlabel('Time (s)');
ylabel('Tank Level (cm)');

exportIdentificationFigure(figRaw, fullfile(figuresDir, 'system_step_response.png'));

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

figValidation = figure('Name', 'Model Validation Analysis', ...
                       'Color', 'w', ...
                       'Position', [100 100 1100 650]);

y_min_val = min([y_true_sim; y_model]);
y_max_val = max([y_true_sim; y_model]);

% Highlight selected operating region
fill([t_op_start t_op_end t_op_end t_op_start], ...
     [y_min_val-2 y_min_val-2 y_max_val+2 y_max_val+2], ...
     [0.92 0.96 0.94], ...
     'EdgeColor', 'none', ...
     'FaceAlpha', 0.70, ...
     'DisplayName', 'Selected operating region');

hold on; grid on; box on;

hActual = plot(t_out, y, ...
    'Color', [0.05 0.25 0.55], ...
    'LineWidth', 1.9, ...
    'DisplayName', 'Measured output');

hModel = plot(t_sim, y_model, '--', ...
    'Color', [0.85 0.15 0.15], ...
    'LineWidth', 2.0, ...
    'DisplayName', 'FOPTD model');

xline(t_op_start, '-.', ...
    'Operating region start', ...
    'LabelOrientation', 'horizontal', 'LabelVerticalAlignment', 'top', ...
    'Color', [0.00 0.45 0.30], ...
    'LineWidth', 1.4);

% Add step markers
for k = 1:num_steps
    xline(step_times(k), ':', ...
        sprintf('Step %d', k), ...
        'Color', [0.60 0.60 0.60], ...
        'LineWidth', 0.8, ...
        'LabelOrientation', 'horizontal', ...
        'LabelVerticalAlignment', 'bottom');
end

text(.98, .20, ...
    sprintf('Validation Metrics\nRMSE = %.4f cm\nFit = %.2f%%', ...
    rmse, fit_percent), ...
    'Units', 'normalized', 'HorizontalAlignment', 'right', ...
    'VerticalAlignment', 'bottom', 'FontSize', 11, ...
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

exportIdentificationFigure(figValidation, fullfile(figuresDir, 'model_validation.png'));

%% Step 10: Plot Estimated Parameters per Step

figParams = figure('Name', 'FOPTD Parameters per Step', ...
                   'Color', 'w', ...
                   'Position', [100 100 1100 800]);

tiledlayout(3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

% Gain K
nexttile;
plot(1:num_steps, K_vec, '-o', ...
    'Color', [0.05 0.25 0.55], ...
    'MarkerFaceColor', [0.05 0.25 0.55], ...
    'LineWidth', 1.6);
hold on; grid on; box on;
xline(op_idx(1)-.5, '--', 'Selected steps: 3-6', ...
    'LabelOrientation', 'horizontal', 'LabelVerticalAlignment', 'top', ...
    'Color', [0.00 0.45 0.30], ...
    'LineWidth', 1.2);
yline(K_avg, '--', sprintf('Mean K = %.4f', K_avg), ...
    'Color', [0.85 0.15 0.15], ...
    'LineWidth', 1.1);
for i = 1:num_steps
    text(i, K_vec(i), sprintf(' %.2f', K_vec(i)), ...
        'FontSize', 11, ...
        'VerticalAlignment', 'bottom');
end
xlim([.7 num_steps+.3]); ylim([0 max(K_vec)*1.2]); xticks(1:num_steps);
ylabel('K (cm/%)');
title('Estimated FOPTD Parameters per Step');

% Delay theta
nexttile;
plot(1:num_steps, delay_vec, '-o', ...
    'Color', [0.85 0.32 0.10], ...
    'MarkerFaceColor', [0.85 0.32 0.10], ...
    'LineWidth', 1.6);
hold on; grid on; box on;
xline(op_idx(1)-.5, '--', ...
    'Color', [0.00 0.45 0.30], ...
    'LineWidth', 1.2);
yline(theta_avg, '--', sprintf('Mean \\theta = %.2f s', theta_avg), ...
    'Color', [0.85 0.15 0.15], ...
    'LineWidth', 1.1);
for i = 1:num_steps
    text(i, delay_vec(i), sprintf(' %.1f', delay_vec(i)), ...
        'FontSize', 11, ...
        'VerticalAlignment', 'bottom');
end
xlim([.7 num_steps+.3]); ylim([min(delay_vec)-1 max(delay_vec)+1.5]); xticks(1:num_steps);
ylabel('\theta (s)');

% Time constant tau
nexttile;
plot(1:num_steps, tau_vec, '-o', ...
    'Color', [0.00 0.45 0.30], ...
    'MarkerFaceColor', [0.00 0.45 0.30], ...
    'LineWidth', 1.6);
hold on; grid on; box on;
xline(op_idx(1)-.5, '--', ...
    'Color', [0.00 0.45 0.30], ...
    'LineWidth', 1.2);
yline(tau_avg, '--', sprintf('Mean \\tau = %.2f s', tau_avg), ...
    'Color', [0.85 0.15 0.15], ...
    'LineWidth', 1.1);
for i = 1:num_steps
    text(i, tau_vec(i), sprintf(' %.1f', tau_vec(i)), ...
        'FontSize', 11, ...
        'VerticalAlignment', 'bottom');
end
xlabel('Step Number');
xlim([.7 num_steps+.3]); ylim([min(tau_vec)-3 max(tau_vec)+5]); xticks(1:num_steps);
ylabel('\tau (s)');

exportIdentificationFigure(figParams, fullfile(figuresDir, 'foptd_parameters_per_step.png'));

%% Step 11: Restore Original Identification Visualizations
% Plot annotations use the same windows and thresholds as Step 4.
u_before = zeros(num_steps, 1);
u_after = zeros(num_steps, 1);
figDetailed = figure('Name', 'Detailed Step Dynamics', 'Color', 'w', ...
    'Position', [100 100 1200 820]);
layout = tiledlayout(figDetailed, 2, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
title(layout, {'Step Response Identification: Delay and Time Constant', ...
    'Orange: delay (theta) | Green: time constant (tau) | Marker: 63.2% response'}, ...
    'FontSize', 14, 'FontWeight', 'bold');
for i = 1:num_steps
    t0 = step_times(i);
    if i < num_steps
        t_end = step_times(i+1) - 2;
    else
        t_end = max(t_out);
    end
    u_before(i) = u(find(t_in <= t0 - 5, 1, 'last'));
    u_after(i) = u(find(t_in >= t0 + 10, 1));
    y0 = mean(y(t_out >= t0 - 20 & t_out <= t0 - 2));
    yf = mean(y(t_out >= t_end - 30 & t_out <= t_end));
    y63 = y0 + 0.632 * (yf - y0);
    mask = t_out >= t0 & t_out <= t_end;
    tl = t_out(mask) - t0;
    yl = y(mask);
    delay = delay_vec(i);
    t63 = delay + tau_vec(i);
    ydelay = interp1(tl, yl, delay);
    span = max(yl) - min(yl);
    lo = min(yl) - 0.08 * span;
    hi = max(yl) + 0.12 * span;
    nexttile(layout); hold on; grid on; box on;
    fill([0 delay delay 0], [lo lo hi hi], [1 .92 .80], ...
        'EdgeColor', 'none', 'FaceAlpha', .6);
    fill([delay t63 t63 delay], [lo lo hi hi], [.85 .95 .85], ...
        'EdgeColor', 'none', 'FaceAlpha', .6);
    plot(tl, yl, 'Color', [.05 .30 .65], 'LineWidth', 1.8);
    xline(delay, '-.', 'Color', [.8 .35 0], 'LineWidth', 1.2);
    xline(t63, '--', 'Color', [0 .5 .25], 'LineWidth', 1.2);
    yline(y63, ':', 'Color', [.5 .5 .5]);
    plot(delay, ydelay, 'd', 'MarkerFaceColor', [.9 .4 0], ...
        'MarkerEdgeColor', [.7 .3 0], 'MarkerSize', 6);
    plot(t63, y63, 'o', 'MarkerFaceColor', [0 .6 .3], ...
        'MarkerEdgeColor', [0 .4 .2], 'MarkerSize', 6);
    text(t63 + 5, y63 + .035 * span, '63.2%', ...
        'Color', [0 .4 .2], 'FontSize', 11);
    text(.97, .94, sprintf('K = %.2f cm/%%\n\\theta = %.1f s\n\\tau = %.1f s', ...
        K_vec(i), delay, tau_vec(i)), 'Units', 'normalized', ...
        'HorizontalAlignment', 'right', 'VerticalAlignment', 'top', ...
        'FontSize', 11, 'BackgroundColor', 'w', 'Margin', 4);
    title(sprintf('Step %d: %.1f%% \\rightarrow %.1f%%', ...
        i, u_before(i), u_after(i)), 'FontSize', 13);
    xlabel('Time since step (s)'); ylabel('Tank level (cm)');
    xlim([0 200]); ylim([lo hi]); xticks(0:50:200);
end
exportIdentificationFigure(figDetailed, fullfile(figuresDir, 'step_response_identification.png'));

%% Step 11.1: Multi-Step Response and Steady-State Mapping
figStaircase = figure('Name', 'Steady-State Mapping', 'Color', 'w', ...
    'Position', [100 100 1100 620]);
hold on; grid on; box on;
bounds = [t_out(1), step_times, max(t_out)];
for i = 1:numel(bounds)-1
    ts = bounds(i); te = bounds(i+1);
    if mod(i, 2) == 1
        fill([ts te te ts], [0 0 75 75], [.94 .96 .98], ...
            'EdgeColor', 'none', 'HandleVisibility', 'off');
    end
    if i > 1
        xline(ts, '--', 'Color', [.65 .65 .70], 'HandleVisibility', 'off');
    end
    yss = mean(y(t_out >= te - 40 & t_out <= te - 2));
    uss = mean(u(t_in >= te - 40 & t_in <= te - 2));
    line([ts te], [yss yss], 'Color', [.85 .2 .2], ...
        'LineWidth', 2, 'HandleVisibility', 'off');
    tc = (ts + te) / 2;
    text(tc, yss + 3, sprintf('y_{ss} = %.1f', yss), ...
        'HorizontalAlignment', 'center', 'Color', [.75 .1 .1], 'FontSize', 11);
    text(tc, yss - 4, sprintf('u = %.1f%%', uss), ...
        'HorizontalAlignment', 'center', 'Color', [0 .45 .3], 'FontSize', 11);
end
hMeasured = plot(t_out, y, 'Color', [.05 .25 .55], 'LineWidth', 1.8);
hSteady = plot(nan, nan, '-', 'Color', [.85 .2 .2], 'LineWidth', 2);
legend([hMeasured hSteady], {'Measured level', 'Steady-state level'}, ...
    'Location', 'northeast', 'FontSize', 11);
title('Multi-Step Response and Steady-State Levels');
xlabel('Time (s)'); ylabel('Tank level (cm)');
xlim([t_out(1) max(t_out)]); ylim([0 75]);
exportIdentificationFigure(figStaircase, fullfile(figuresDir, 'steady_state_mapping.png'));

%% Step 11.2: Local Gain and Nonlinearity (Original Two-Panel View)
figOperating = figure('Name', 'Operating Region Analysis', 'Color', 'w', ...
    'Position', [100 100 1150 560]);
layout = tiledlayout(figOperating, 1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
nexttile(layout); hold on; grid on; box on;
fill([2.5 6.5 6.5 2.5], [0 0 6.8 6.8], [.92 .96 .94], 'EdgeColor', 'none');
bar(1:num_steps, K_vec, .48, 'FaceColor', [.05 .3 .65], 'EdgeColor', 'none');
line([2.5 6.5], [K_avg K_avg], 'Color', [.85 .25 .1], ...
    'LineWidth', 1.8, 'LineStyle', '--');
for i = 1:num_steps
    text(i, K_vec(i) + .15, sprintf('%.2f', K_vec(i)), ...
        'HorizontalAlignment', 'center', 'FontSize', 11);
end
text(4.5, 6.2, sprintf('Steps 3-6: mean K = %.2f', K_avg), ...
    'HorizontalAlignment', 'center', 'Color', [.7 .2 0], 'FontSize', 12);
xlim([.5 6.5]); ylim([0 6.8]); xticks(1:num_steps);
title('A) Local Gain and Selected Region');
xlabel('Step number'); ylabel('Static gain K (cm/%)');
nexttile(layout); hold on; grid on; box on;
u_mid = (u_before + u_after) / 2;
u_fit = linspace(min(u_mid), max(u_mid), 100);
hTrend = plot(u_fit, polyval(polyfit(u_mid, K_vec, 2), u_fit), '--', ...
    'Color', [.1 .45 .75], 'LineWidth', 1.6);
hData = plot(u_mid, K_vec, 'o-', 'Color', [.8 .15 .15], ...
    'LineWidth', 1.8, 'MarkerFaceColor', [.8 .15 .15], 'MarkerSize', 6);
for i = 1:num_steps
    text(u_mid(i) + .25, K_vec(i) + .2, sprintf('S%d', i), 'FontSize', 11);
end
set(gca, 'XDir', 'reverse'); ylim([0 6.8]);
xlim([min(u_mid)-1 max(u_mid)+1]);
title('B) Gain vs. Mean Input');
xlabel('Mean valve opening (%)'); ylabel('Static gain K (cm/%)');
legend([hData hTrend], {'Calculated gains', 'Quadratic trend'}, ...
    'Location', 'northeast', 'FontSize', 10);
exportIdentificationFigure(figOperating, fullfile(figuresDir, 'operating_region_analysis.png'));

%% Step 12: Final Status Message

fprintf('\nSystem identification completed successfully.\n');
fprintf('Figures saved to: %s\n', figuresDir);
fprintf('Results saved to: %s\n', resultsDir);

function exportIdentificationFigure(fig, path)
% Consistent typography and complete figure export for GitHub previews.
    axesList = findall(fig, 'Type', 'axes');
    for k = 1:numel(axesList)
        set(axesList(k), 'FontName', 'Helvetica', 'FontSize', 12, ...
            'LineWidth', .8, 'GridAlpha', .15);
    end
    drawnow;
    exportgraphics(fig, path, 'Resolution', 180);
end
