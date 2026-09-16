%% Fatemeh Sajed 40119113
%% Step 1 & 2: Load Data & Signal Extraction
clear; clc; close all;
input_struct  = load('matlab.mat');  output_struct = load('output.mat');
u_ts = input_struct.input_level;    y_ts = output_struct.output_level;
t_in = u_ts.Time(:); u = u_ts.Data(:); t_out = y_ts.Time(:); y = y_ts.Data(:);

figure('Name', 'System Step Response');
subplot(2,1,1); plot(t_in, u, 'Color', [0.05 0.25 0.55], 'LineWidth', 1.5); grid on; title('Input Signal (u)'); xlabel('Time (s)'); ylabel('Input Level');
subplot(2,1,2); plot(t_out, y, 'Color', [0.0 0.45 0.3], 'LineWidth', 1.5); grid on; title('Output Signal (y)'); xlabel('Time (s)'); ylabel('Output Level');

%% Step 3: Parameter Estimation
step_times = [400, 800, 1200, 1600, 2000, 2400]; num_steps = length(step_times);
[K_vec, delay_vec, tau_vec] = deal(zeros(num_steps, 1));

for i = 1:num_steps
    t_step = step_times(i);
    if i < num_steps, t_end = step_times(i+1) - 2; else, t_end = max(t_out); end
    
    delta_u = u(find(t_in >= t_step + 10, 1)) - u(find(t_in <= t_step - 5, 1, 'last'));
    
    y_init  = mean(y(t_out >= (t_step - 20) & t_out <= (t_step - 2)));
    y_final = mean(y(t_out >= (t_end - 30) & t_out <= t_end));
    delta_y = y_final - y_init;
    
    K_vec(i) = delta_y / delta_u;
    
    idx_region = find(t_out >= t_step & t_out <= t_end);
    t_region = t_out(idx_region); y_region = y(idx_region);
    
    % Delay (2%) & Tau (63.2%) Thresholds
    if delta_y < 0
        idx_d = find(y_region <= y_init + 0.02*delta_y, 1);
        idx_t = find(y_region <= y_init + 0.632*delta_y, 1);
    else
        idx_d = find(y_region >= y_init + 0.02*delta_y, 1);
        idx_t = find(y_region >= y_init + 0.632*delta_y, 1);
    end
    
    if isempty(idx_d), idx_d = 1; end
    if isempty(idx_t), idx_t = 1; end
    
    t_start = t_region(idx_d); 
    delay_vec(i) = t_start - t_step;
    tau_vec(i) = t_region(idx_t) - t_start;
end

%% Step 4: Display Command Window Results
print_header = @(txt) fprintf('\n=========================================================\n                %s                 \n=========================================================\n', txt);

print_header('Part 1: Gain (K) Results');
fprintf('%-6s %-12s\n------ --------\n', 'Step', 'Gain_K');
arrayfun(@(i) fprintf('%-6d %-12.5f\n', i, K_vec(i)), 1:num_steps);

print_header('Part 3: Delay (Theta) Results');
arrayfun(@(i) fprintf('Step %d: Delay = %.1f seconds\n', i, delay_vec(i)), 1:num_steps);
fprintf('---------------------------------------------------------\nAverage Delay (All Steps): %.2f seconds\n', mean(delay_vec));

print_header('Part 4: Time Constant (Tau) Results');
fprintf('%-6s %-12s\n------ ------\n', 'Step', 'Tau_s');
arrayfun(@(i) fprintf('%-6d %-12.2f\n', i, tau_vec(i)), 1:num_steps);

print_header('Summary Table of All Estimated Parameters');
fprintf('%-6s %-12s %-16s %-12s\n------ -------- ---------------- -------\n', 'Step', 'Gain (K)', 'Delay_Theta (s)', 'Tau (s)');
arrayfun(@(i) fprintf('%-6d %-12.5f %-16.1f %-12.2f\n', i, K_vec(i), delay_vec(i), tau_vec(i)), 1:num_steps);
fprintf('=========================================================\n');

%% Step 5: Operating Region System Model 
op_idx = 3:6;
K_avg = mean(K_vec(op_idx)); theta_avg = mean(delay_vec(op_idx)); tau_avg = mean(tau_vec(op_idx));

print_header('Part 5: Overall System Model (Operating Region 3-6)');
fprintf('Mean Static Gain (K_avg)      : %.4f\nMean Delay Time (Theta_avg)   : %.2f seconds\nMean Time Constant (Tau_avg)  : %.2f seconds\n', K_avg, theta_avg, tau_avg);
fprintf('---------------------------------------------------------\nFOPTD Transfer Function Model:\n           %.4f * exp(-%.2f * s)\n  G(s) = -------------------------\n               %.2f * s + 1       \n=========================================================\n', K_avg, theta_avg, tau_avg);

%% Step 5.1: Model Validation Analysis
dt = mean(diff(t_in)); t_sim = (t_in(1):dt:t_in(end))';
u_sim = interp1(t_in, u, t_sim, 'linear', 'extrap');
y_true_sim = interp1(t_out, y, t_sim, 'linear', 'extrap');

s = tf('s'); G_sys = (K_avg * exp(-theta_avg * s)) / (tau_avg * s + 1);
y_sim = y_true_sim(1) + lsim(G_sys, u_sim - u_sim(1), t_sim);

t_op_start = step_times(3); t_op_end = max(t_out);
idx_val = find(t_sim >= t_op_start & ~isnan(y_true_sim) & ~isnan(y_sim));
rmse = sqrt(mean((y_true_sim(idx_val) - y_sim(idx_val)).^2));
fit_percent = max(0, (1 - rmse / std(y_true_sim(idx_val))) * 100);

print_header('MODEL VALIDATION METRICS');
fprintf('RMSE in Operating Region (Steps 3-6) : %.4f cm\nModel Fit Percentage                 : %.2f%%\n=========================================================\n', rmse, fit_percent);

% --- Plot Model Validation ---
figure('Name', 'Model Validation Analysis', 'Color', [0.98 0.98 0.98], 'Units', 'normalized', 'Position', [0.08 0.12 0.84 0.65]);
y_min_val = min([y_true_sim; y_sim]); y_max_val = max([y_true_sim; y_sim]);

fill([t_op_start t_op_end t_op_end t_op_start], [y_min_val-2 y_min_val-2 y_max_val+2 y_max_val+2], [0.92 0.96 0.94], 'EdgeColor', 'none', 'FaceAlpha', 0.65); hold on; grid on;
hActual = plot(t_out, y, 'Color', [0.05 0.25 0.55], 'LineWidth', 1.8);
hModel  = plot(t_sim, y_sim, '--', 'Color', [0.85 0.15 0.15], 'LineWidth', 2.0);
line([t_op_start t_op_start], [y_min_val-2 y_max_val+2], 'Color', [0.0 0.45 0.3], 'LineStyle', '-.', 'LineWidth', 1.4);

text(t_op_start + 15, y_max_val - 2, 'Linear Operating Region \rightarrow', 'Color', [0.0 0.45 0.3], 'FontSize', 11, 'FontWeight', 'bold', 'FontName', 'Helvetica');
text(2500, y_min_val + 4, sprintf('Validation Metrics (Steps 3-6):\n  • RMSE  = %.4f cm\n  • Fit %%  = %.2f%%', rmse, fit_percent), ...
     'HorizontalAlignment', 'left', 'VerticalAlignment', 'bottom', 'FontSize', 11, 'FontWeight', 'bold', 'FontName', 'Helvetica', 'BackgroundColor', [1 1 1 0.9], 'EdgeColor', [0.15 0.25 0.4], 'LineWidth', 1.0, 'Margin', 7);

xlim([0, max(t_sim)]); ylim([y_min_val - 2, y_max_val + 2]);
set(gca, 'FontSize', 11, 'FontWeight', 'bold', 'FontName', 'Helvetica', 'LineWidth', 1.2, 'GridColor', [0.8 0.8 0.8], 'GridAlpha', 0.6);
title('Model Validation: Measured System Response vs. FOPTD Linear Model', 'FontSize', 14, 'FontWeight', 'bold', 'Color', [0.1 0.1 0.2]);
xlabel('Time [s]', 'FontSize', 12, 'FontWeight', 'bold'); ylabel('Fluid Level y [cm]', 'FontSize', 12, 'FontWeight', 'bold');
legend([hActual, hModel], {'Measured System Output (y)', 'FOPTD Model Response (y_{sim})'}, 'Location', 'northeast', 'FontSize', 11, 'FontName', 'Helvetica');
uistack([hActual, hModel], 'top');

%% Step 5.2: Per-Step & Staircase Visualization
u_percent = [96, 93.5, 91, 88.5, 86, 83.5, 81];

% --- FIGURE 1: Detailed Step Dynamics ---
figure('Name', 'Detailed Step Dynamics', 'Color', [0.98 0.98 0.98], 'Units', 'normalized', 'Position', [0.03 0.03 0.92 0.92]);
sgtitle('Step Response Identification: Dead Time (\theta) & Time Constant (\tau) Mapping', 'FontSize', 17, 'FontWeight', 'bold', 'FontName', 'Helvetica', 'Color', [0.1 0.1 0.2]);

for idx = 1:num_steps
    t0_step = step_times(idx);
    if idx < num_steps, t_final = step_times(idx+1); else, t_final = max(t_out); end
    
    m_idx = find(t_out >= t0_step & t_out <= t_final);
    t_local = t_out(m_idx) - t0_step; y_local = y(m_idx);
    
    dy = mean(y(t_final - t_out <= 30 & t_out <= t_final)) - mean(y(t_out >= (t0_step - 20) & t_out <= (t0_step - 2)));
    
    if dy < 0
        k_d = find(y_local <= y_local(1) + 0.02*dy, 1);
        k_63 = find(y_local <= y_local(1) + 0.632*dy, 1);
    else
        k_d = find(y_local >= y_local(1) + 0.02*dy, 1);
        k_63 = find(y_local >= y_local(1) + 0.632*dy, 1);
    end
    
    if isempty(k_d), k_d = 1; end
    if isempty(k_63), k_63 = 1; end
    
    t_lag = t_local(k_d); y_lag = y_local(k_d);
    t_63_loc = t_local(k_63); y_th63 = y_local(1) + 0.632 * dy;
    
    hAx = subplot(2, 3, idx);
    y_min_loc = min(y_local); y_max_loc = max(y_local);
    
    fill([0 t_lag t_lag 0], [y_min_loc y_min_loc y_max_loc y_max_loc], [1.0 0.92 0.8], 'EdgeColor', 'none', 'FaceAlpha', 0.45); hold on;
    fill([t_lag t_63_loc t_63_loc t_lag], [y_min_loc y_min_loc y_max_loc y_max_loc], [0.85 0.95 0.85], 'EdgeColor', 'none', 'FaceAlpha', 0.45);
    
    hCurve = plot(t_local, y_local, 'Color', [0.05 0.3 0.65], 'LineWidth', 2.0); grid on; pbaspect([1 1 1]);
    set(hAx, 'FontSize', 11, 'LineWidth', 1.2, 'FontWeight', 'bold', 'FontName', 'Helvetica', 'GridColor', [0.75 0.75 0.75], 'GridAlpha', 0.6);
    
    plot(t_lag, y_lag, 'd', 'MarkerEdgeColor', [0.7 0.3 0.0], 'MarkerFaceColor', [0.9 0.4 0.0], 'MarkerSize', 8);
    line([t_lag t_lag], [y_min_loc y_max_loc], 'Color', [0.8 0.35 0.0], 'LineStyle', '-.', 'LineWidth', 1.3);
    
    plot(t_63_loc, y_th63, 'o', 'MarkerEdgeColor', [0.0 0.4 0.2], 'MarkerFaceColor', [0.0 0.65 0.35], 'MarkerSize', 8);
    line([t_63_loc t_63_loc], [y_min_loc y_max_loc], 'Color', [0.0 0.5 0.25], 'LineStyle', '--', 'LineWidth', 1.3);
    line([0 200], [y_th63 y_th63], 'Color', [0.6 0.6 0.6], 'LineStyle', ':');
    
    text(195, y_min_loc + (y_max_loc - y_min_loc)*0.12, sprintf('\\theta = %.1fs  |  \\tau = %.1fs\nK_%d = %.2f', delay_vec(idx), tau_vec(idx), idx, K_vec(idx)), ...
         'HorizontalAlignment', 'right', 'VerticalAlignment', 'bottom', 'FontSize', 10.5, 'FontWeight', 'bold', 'FontName', 'Helvetica', 'BackgroundColor', [1 1 1 0.85], 'EdgeColor', [0.6 0.6 0.6], 'Margin', 5);
    text(t_lag + 4, y_lag, '\theta', 'Color', [0.7 0.3 0.0], 'FontSize', 12, 'FontWeight', 'bold');
    text(t_63_loc + 4, y_th63, '63%', 'Color', [0.0 0.4 0.2], 'FontSize', 11, 'FontWeight', 'bold');
    
    xlim([0, 200]); ylim([y_min_loc, y_max_loc]);
    title(sprintf('Step %d: %.1f%% \\rightarrow %.1f%%', idx, u_percent(idx), u_percent(idx+1)), 'FontSize', 12.5, 'FontWeight', 'bold', 'Color', [0.1 0.1 0.2]);
    xlabel('Time t - t_0 [s]', 'FontSize', 11.5, 'FontWeight', 'bold'); ylabel('Level y [cm]', 'FontSize', 11.5, 'FontWeight', 'bold');
    uistack(hCurve, 'top');
end

% --- FIGURE 2: Staircase Dynamics Analysis ---
figure('Name', 'Staircase Dynamics Analysis', 'Color', [0.98 0.98 0.98], 'Units', 'normalized', 'Position', [0.1 0.1 0.8 0.7]);
hLine = plot(t_out, y, 'Color', [0.1 0.25 0.5], 'LineWidth', 1.8); hold on; grid on;

all_b = unique([0, step_times, max(t_out)]); y_ss_vec = zeros(1, length(all_b)-1);
for i = 1:length(all_b)-1
    t_s = all_b(i); t_e = all_b(i+1);
    if mod(i, 2) == 1, fill([t_s t_e t_e t_s], [0 0 75 75], [0.94 0.96 0.98], 'EdgeColor', 'none', 'FaceAlpha', 0.5); end
    if t_s > 0, line([t_s t_s], [0 75], 'Color', [0.6 0.6 0.65], 'LineStyle', '--', 'LineWidth', 1.1); end
    
    y_ss = mean(y(t_out >= (t_e - 40) & t_out <= (t_e - 2))); y_ss_vec(i) = y_ss;
    line([t_s t_e], [y_ss y_ss], 'Color', [0.85 0.2 0.2], 'LineWidth', 2.2);
    
    t_c = t_s + (t_e - t_s) * 0.25;
    text(t_c, y_ss + 2.8, sprintf('y_{ss} = %.1f cm', y_ss), 'Color', [0.75 0.1 0.1], 'FontSize', 9.5, 'FontWeight', 'bold', 'BackgroundColor', [1 1 1 0.7]);
    if i <= length(u_percent), text(t_c, y_ss - 4.8, sprintf('u = %.1f%%', u_percent(i)), 'Color', [0.0 0.45 0.35], 'FontSize', 9.5, 'FontWeight', 'bold'); end
    if i > 1, text(t_s + 10, (y_ss_vec(i-1) + y_ss)/2, sprintf('\\Delta y = %.1f', y_ss - y_ss_vec(i-1)), 'Color', [0.3 0.3 0.3], 'FontSize', 8.5, 'FontAngle', 'italic'); end
end

uistack(hLine, 'top'); xlim([0, max(t_out)]); ylim([0, 75]);
title('Tank System Multi-Step Response & Steady-State Mapping', 'FontSize', 13, 'FontWeight', 'bold', 'FontName', 'Helvetica', 'Color', [0.1 0.1 0.2]);
xlabel('Time [s]', 'FontSize', 10.5, 'FontWeight', 'bold'); ylabel('Fluid Level y [cm]', 'FontSize', 10.5, 'FontWeight', 'bold');
set(gca, 'FontSize', 10, 'LineWidth', 1.1, 'FontName', 'Helvetica', 'GridColor', [0.85 0.85 0.85], 'GridAlpha', 0.7);
legend(hLine, {'Measured Tank Level'}, 'Location', 'northeast', 'AutoUpdate', 'off');

% --- FIGURE 3: Static Gain & Nonlinearity ---
figure('Name', 'Gain Analysis & Nonlinearity', 'Color', [0.98 0.98 0.98], 'Units', 'normalized', 'Position', [0.08 0.1 0.84 0.58]);
u_mid = (u_percent(1:end-1) + u_percent(2:end)) / 2;

hAx1 = subplot(1, 2, 1);
fill([2.5 6.5 6.5 2.5], [0 0 6.8 6.8], [0.92 0.96 0.94], 'EdgeColor', 'none', 'FaceAlpha', 0.6); hold on; grid on;
bar(1:num_steps, K_vec, 0.48, 'FaceColor', [0.05 0.3 0.65], 'EdgeColor', 'none');
line([2.5 6.5], [K_avg K_avg], 'Color', [0.85 0.25 0.1], 'LineWidth', 2.0, 'LineStyle', '--');

text(4.5, 5.2, sprintf('K_{avg} (Op. Region) = %.2f', K_avg), 'HorizontalAlignment', 'center', 'Color', [0.8 0.2 0.0], 'FontSize', 10.5, 'FontWeight', 'bold', 'FontName', 'Helvetica', 'BackgroundColor', [1 1 1 0.9], 'EdgeColor', [0.8 0.2 0.0], 'Margin', 4);
arrayfun(@(i) text(i, K_vec(i) + 0.18, sprintf('%.2f', K_vec(i)), 'HorizontalAlignment', 'center', 'FontSize', 10, 'FontWeight', 'bold', 'FontName', 'Helvetica'), 1:num_steps);

set(hAx1, 'XTick', 1:num_steps, 'XTickLabel', arrayfun(@(x) sprintf('Step %d', x), 1:num_steps, 'UniformOutput', false), 'FontSize', 11, 'FontWeight', 'bold', 'LineWidth', 1.2, 'GridColor', [0.8 0.8 0.8], 'GridAlpha', 0.6);
xlabel('Operating Step', 'FontSize', 12, 'FontWeight', 'bold'); ylabel('Static Gain K [cm / %]', 'FontSize', 12, 'FontWeight', 'bold');
title('A) Local Static Gain per Operating Step', 'FontSize', 13, 'FontWeight', 'bold', 'Color', [0.1 0.1 0.2]); ylim([0, 6.8]);

hAx2 = subplot(1, 2, 2);
u_fit = linspace(min(u_mid), max(u_mid), 100);
hTrend = plot(u_fit, polyval(polyfit(u_mid, K_vec, 2), u_fit), '--', 'Color', [0.1 0.45 0.75], 'LineWidth', 1.6); hold on; grid on;
hData  = plot(u_mid, K_vec, 'o-', 'Color', [0.8 0.15 0.15], 'LineWidth', 2.0, 'MarkerSize', 8, 'MarkerFaceColor', [0.85 0.2 0.2]);

arrayfun(@(i) text(u_mid(i) + 0.35, K_vec(i) + 0.2, sprintf('S%d', i), 'FontSize', 10, 'FontWeight', 'bold', 'FontName', 'Helvetica'), 1:num_steps);
set(hAx2, 'FontSize', 11, 'FontWeight', 'bold', 'FontName', 'Helvetica', 'LineWidth', 1.2, 'XDir', 'reverse', 'GridColor', [0.8 0.8 0.8], 'GridAlpha', 0.6);
xlabel('Mean Input Level u [%]', 'FontSize', 12, 'FontWeight', 'bold'); ylabel('Static Gain K [cm / %]', 'FontSize', 12, 'FontWeight', 'bold');
title('B) System Characteristic: Gain vs. Input u%', 'FontSize', 13, 'FontWeight', 'bold', 'Color', [0.1 0.1 0.2]); ylim([0, 6.8]);
legend([hData, hTrend], {'Calculated Gains', 'Nonlinear Trend (Quadratic)'}, 'Location', 'northeast', 'FontSize', 10, 'FontName', 'Helvetica');