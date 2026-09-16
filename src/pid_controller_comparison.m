 %% Fatemeh Sajed 40119113
clc;

%% Process Parameters
K_sys     = 3.5682;     % Plant Static Gain [cm/%]
tau_sys   = 33.38;      % Time Constant [s]
theta_sys = 13.50;      % Dead Time [s]

alpha = theta_sys / tau_sys;
Ka = K_sys * alpha;
s = tf('s');


%% Controller Parameters Calculation

% --- 1. Disturbance Rejection (Ziegler-Nichols) ---
Kp_PI_ZN   = 0.9 / Ka;    Ti_PI_ZN   = 3.3 * theta_sys; Td_PI_ZN=0;
Kp_PID_ZN  = 1.2 / Ka;    Ti_PID_ZN  = 2.0 * theta_sys; Td_PID_ZN = 0.5 * theta_sys;

% --- 2. Disturbance Rejection (CHR 0% Overshoot) ---
Kp_PI_CHRd  = 0.6 / Ka;   Ti_PI_CHRd  = 4.0 * theta_sys; Td_PI_CHRd = 0;
Kp_PID_CHRd = 0.95 / Ka;  Ti_PID_CHRd = 2.38 * theta_sys; Td_PID_CHRd = 0.42 * theta_sys;

% --- 3. Setpoint Tracking (CHR 0% Overshoot) ---
Kp_PI_CHRt  = 0.35 / Ka;  Ti_PI_CHRt  = 1.2 * tau_sys; Td_PI_CHRt = 0;
Kp_PID_CHRt = 0.6 / Ka;   Ti_PID_CHRt = tau_sys;       Td_PID_CHRt = 0.5 * theta_sys;

% Construct Transfer Functions with Filtered Derivative: Td*s / (0.1*Td*s + 1)
make_pid = @(Kp, Ti, Td) Kp * (1 + (1/(Ti*s)) + (Td*s)/(0.1*Td*s + 1));
make_pi  = @(Kp, Ti)     Kp * (1 + (1/(Ti*s)));

C_pi_zn   = make_pi(Kp_PI_ZN, Ti_PI_ZN);
C_pid_zn  = make_pid(Kp_PID_ZN, Ti_PID_ZN, Td_PID_ZN);
C_pi_chrd = make_pi(Kp_PI_CHRd, Ti_PI_CHRd);
C_pid_chrd= make_pid(Kp_PID_CHRd, Ti_PID_CHRd, Td_PID_CHRd);
C_pi_chrt = make_pi(Kp_PI_CHRt, Ti_PI_CHRt);
C_pid_chrt= make_pid(Kp_PID_CHRt, Ti_PID_CHRt, Td_PID_CHRt);


disp('--> Parameters successfully loaded into Workspace for Simulink!');


%% Plotting for Report
t_sim = out.y_sim.Time;

if isa(out.y_sim, 'timeseries')
    y_data = out.y_sim.Data;
else
    y_data = out.y_sim.signals.values;
end

% Assigning columns
y_pi_zn   = y_data(:,1);  y_pid_zn   = y_data(:,2);
y_pi_chrd = y_data(:,3);  y_pid_chrd = y_data(:,4);
y_pi_chrt = y_data(:,5);  y_pid_chrt = y_data(:,6);

%% 0. Generate Reference Signal for Comparison
r_ref = zeros(size(t_sim));
r_ref(t_sim < 400) = 35;
r_ref(t_sim >= 400 & t_sim < 800) = 55;
r_ref(t_sim >= 800) = 45;

%% Define Professional Color Palette
c_ref  = [0.2 0.2 0.2];     % Dark Gray for Reference
c_zn1  = [0.85 0.32 0.10];  % Coral Red (PI ZN)
c_zn2  = [0.63 0.08 0.18];  % Deep Red (PID ZN)
c_chrd1= [0.30 0.74 0.93];  % Cyan/Light Blue (PI CHR-Dist)
c_chrd2= [0.00 0.45 0.74];  % Dark Blue (PID CHR-Dist)
c_chrt1= [0.47 0.67 0.19];  % Olive Green (PI CHR-Track)
c_chrt2= [0.13 0.55 0.13];  % Emerald Green (PID CHR-Track)

%% --- Figure 1: Disturbance Rejection Performance ---
fig1 = figure('Name', 'Disturbance Rejection Methods', 'Color', [1 1 1], 'Position', [100 100 850 500]);
plot(t_sim, r_ref, ':', 'Color', c_ref, 'LineWidth', 1.8, 'DisplayName', 'Reference (r)'); hold on; grid on;
plot(t_sim, y_pi_zn, '--', 'Color', c_zn1, 'LineWidth', 1.2, 'DisplayName', 'PI ZN');
plot(t_sim, y_pid_zn, '-', 'Color', c_zn2, 'LineWidth', 1.6, 'DisplayName', 'PID ZN');
plot(t_sim, y_pi_chrd, '--', 'Color', c_chrd1, 'LineWidth', 1.2, 'DisplayName', 'PI CHR (Dist)');
plot(t_sim, y_pid_chrd, '-', 'Color', c_chrd2, 'LineWidth', 1.6, 'DisplayName', 'PID CHR (Dist)');

% Add Disturbance Markers
xline(600, '-.', 'Input Disturbance', 'Color', [0.5 0.5 0.5], 'LabelVerticalAlignment', 'bottom', 'FontSize', 9);
xline(1000, '-.', 'Output Disturbance', 'Color', [0.5 0.5 0.5], 'LabelVerticalAlignment', 'bottom', 'FontSize', 9);

title('Disturbance Rejection Performance (Ziegler-Nichols vs CHR)', 'FontSize', 11, 'FontWeight', 'bold');
xlabel('Time [s]', 'FontSize', 10); ylabel('Fluid Level [cm]', 'FontSize', 10);
legend('Location', 'northeast', 'FontSize', 9);
ylim([15 65]); set(gca, 'GridAlpha', 0.15, 'MinorGridAlpha', 0.1);

%% --- Figure 2: Setpoint Tracking Performance ---
fig2 = figure('Name', 'Setpoint Tracking Methods', 'Color', [1 1 1], 'Position', [150 150 850 500]);
plot(t_sim, r_ref, ':', 'Color', c_ref, 'LineWidth', 1.8, 'DisplayName', 'Reference (r)'); hold on; grid on;
plot(t_sim, y_pi_chrt, '--', 'Color', c_chrt1, 'LineWidth', 1.4, 'DisplayName', 'PI CHR (Tracking)');
plot(t_sim, y_pid_chrt, '-', 'Color', c_chrt2, 'LineWidth', 1.8, 'DisplayName', 'PID CHR (Tracking)');

% Add Disturbance Markers
xline(600, '-.', 'Input Dist', 'Color', [0.5 0.5 0.5], 'LabelVerticalAlignment', 'bottom', 'FontSize', 9);
xline(1000, '-.', 'Output Dist', 'Color', [0.5 0.5 0.5], 'LabelVerticalAlignment', 'bottom', 'FontSize', 9);

title('Setpoint Tracking Method (CHR 0% Overshoot Strategy)', 'FontSize', 11, 'FontWeight', 'bold');
xlabel('Time [s]', 'FontSize', 10); ylabel('Fluid Level [cm]', 'FontSize', 10);
legend('Location', 'northeast', 'FontSize', 9);
ylim([15 65]); set(gca, 'GridAlpha', 0.15, 'MinorGridAlpha', 0.1);

%% --- Figure 3: Core PID Controllers Comparison ---
fig3 = figure('Name', 'Core PID Comparison', 'Color', [1 1 1], 'Position', [200 200 850 500]);
plot(t_sim, r_ref, ':', 'Color', c_ref, 'LineWidth', 1.8, 'DisplayName', 'Reference (r)'); hold on; grid on;
plot(t_sim, y_pid_zn, '-', 'Color', c_zn2, 'LineWidth', 1.5, 'DisplayName', 'PID ZN (Dist)');
plot(t_sim, y_pid_chrd, '-', 'Color', c_chrd2, 'LineWidth', 1.5, 'DisplayName', 'PID CHR (Dist)');
plot(t_sim, y_pid_chrt, '-', 'Color', c_chrt2, 'LineWidth', 1.8, 'DisplayName', 'PID CHR (Track)');

% Add Disturbance Markers
xline(600, '-.', 'Input Disturbance', 'Color', [0.5 0.5 0.5], 'LabelVerticalAlignment', 'bottom', 'FontSize', 9);
xline(1000, '-.', 'Output Disturbance', 'Color', [0.5 0.5 0.5], 'LabelVerticalAlignment', 'bottom', 'FontSize', 9);

title('Comparative Study of Main PID Design Strategies', 'FontSize', 11, 'FontWeight', 'bold');
xlabel('Time [s]', 'FontSize', 10); ylabel('Fluid Level [cm]', 'FontSize', 10);
legend('Location', 'northeast', 'FontSize', 9);
ylim([15 65]); set(gca, 'GridAlpha', 0.15, 'MinorGridAlpha', 0.1);
%% Corrected Metric Calculation Script (Base & Peak Aware)
t = out.y_sim.Time;

if isa(out.y_sim, 'timeseries')
    y = out.y_sim.Data;
else
    y = out.y_sim.signals.values;
end

labels = {'PI ZN', 'PID ZN', 'PI CHR-Dist', 'PID CHR-Dist', 'PI CHR-Track', 'PID CHR-Track'};

% Window Indices
idx1   = (t >= 0 & t < 400);        % Step 1: 0 to 35 cm
idx2   = (t >= 400 & t < 600);      % Step 2: 35 to 55 cm
idx_d1 = (t >= 600 & t < 800);      % Input Disturbance
idx3   = (t >= 800 & t < 1000);     % Step 3: 55 to 45 cm
idx_d2 = (t >= 1000 & t <= 1200);   % Output Disturbance

fprintf('\n=========================================================================================================================\n');
fprintf('%-14s | %-8s %-8s | %-8s %-8s | %-8s %-8s | %-9s %-9s\n', ...
    'Controller', 'OS1(%)', 'ess1', 'OS2(%)', 'ess2', 'US3(%)', 'ess3', 'Dev Dist1', 'Dev Dist2');
fprintf('-------------------------------------------------------------------------------------------------------------------------\n');

for i = 1:6
    % --- Step 1 (Ref = 35, Base = 0) ---
    peak1 = max(y(idx1, i));
    y_final1 = y(find(idx1, 1, 'last'), i);
    os1 = max(0, (peak1 - y_final1) / y_final1 * 100);
    ess1 = abs(35 - y_final1);
    
    % --- Step 2 (Ref = 55, Base = y_final1) ---
    y_base2 = y_final1;
    peak2 = max(y(idx2, i));
    y_final2 = y(find(idx2, 1, 'last'), i);
    step_size2 = y_final2 - y_base2;
    
    if step_size2 > 0
        os2 = max(0, (peak2 - y_final2) / step_size2 * 100);
    else
        os2 = 0;
    end
    ess2 = abs(55 - y_final2);
    
    % --- Disturbance 1 Impact ---
    dev_d1 = max(abs(y(idx_d1, i) - y_final2));
    
    % --- Step 3 (Ref = 45, Base = y_final_d1) ---
    y_base3 = y(find(idx_d1, 1, 'last'), i);
    min3 = min(y(idx3, i));
    y_final3 = y(find(idx3, 1, 'last'), i);
    step_size3 = abs(y_final3 - y_base3);
    
    if step_size3 > 0
        us3 = max(0, (y_final3 - min3) / step_size3 * 100);
    else
        us3 = 0;
    end
    ess3 = abs(45 - y_final3);
    
    % --- Disturbance 2 Impact ---
    dev_d2 = max(abs(y(idx_d2, i) - y_final3));
    
    fprintf('%-14s | %-8.2f %-8.2f | %-8.2f %-8.2f | %-8.2f %-8.2f | %-9.2f %-9.2f |\n', ...
        labels{i}, os1, ess1, os2, ess2, us3, ess3, dev_d1, dev_d2);
end
fprintf('=========================================================================================================================\n\n');

%% Simulation Environment Setup
t = 0:0.2:1200;

% Setpoint Profile: 55 cm -> 45 cm -> 35 cm
r = zeros(size(t));
r(t >= 0 & t < 400)   = 35;
r(t >= 400 & t < 800)  = 55;
r(t >= 800)           = 45;

% Disturbance Steps (Amplitude = 10)
d_in = zeros(size(t));  d_in(t >= 600)  = 10; % Input Disturbance at t=600s
d_out = zeros(size(t)); d_out(t >= 1000) = 10; % Output Disturbance at t=1000s

% Pade Approximation for System Delay
[num_g, den_g] = pade(theta_sys, 2);
G_process = (K_sys / (tau_sys*s + 1)) * tf(num_g, den_g);

%% Closed-Loop Simulations
[y_pi_zn, u_pi_zn]     = sim_lsim(G_process, C_pi_zn, r, d_in, d_out, t);
[y_pid_zn, u_pid_zn]   = sim_lsim(G_process, C_pid_zn, r, d_in, d_out, t);
[y_pi_chrd, u_pi_chrd] = sim_lsim(G_process, C_pi_chrd, r, d_in, d_out, t);
[y_pid_chrd, u_pid_chrd]= sim_lsim(G_process, C_pid_chrd, r, d_in, d_out, t);
[y_pi_chrt, u_pi_chrt] = sim_lsim(G_process, C_pi_chrt, r, d_in, d_out, t);
[y_pid_chrt, u_pid_chrt]= sim_lsim(G_process, C_pid_chrt, r, d_in, d_out, t);

%%  Command Window Results Summary
print_header = @(txt) fprintf('\n=========================================================\n                %s                 \n=========================================================\n', txt);
print_header('Part 1: Calculated Controller Parameters');
fprintf('%-18s %-10s %-12s %-12s\n------------------ ---------- ------------ ------------\n', 'Controller Type', 'Kp', 'Ti (s)', 'Td (s)');
fprintf('%-18s %-10.4f %-12.2f %-12s\n', 'PI - ZN (Dist)', Kp_PI_ZN, Ti_PI_ZN, '-');
fprintf('%-18s %-10.4f %-12.2f %-12.2f\n', 'PID - ZN (Dist)', Kp_PID_ZN, Ti_PID_ZN, Td_PID_ZN);
fprintf('%-18s %-10.4f %-12.2f %-12s\n', 'PI - CHR (Dist)', Kp_PI_CHRd, Ti_PI_CHRd, '-');
fprintf('%-18s %-10.4f %-12.2f %-12.2f\n', 'PID - CHR (Dist)', Kp_PID_CHRd, Ti_PID_CHRd, Td_PID_CHRd);
fprintf('%-18s %-10.4f %-12.2f %-12s\n', 'PI - CHR (Track)', Kp_PI_CHRt, Ti_PI_CHRt, '-');
fprintf('%-18s %-10.4f %-12.2f %-12.2f\n', 'PID - CHR (Track)', Kp_PID_CHRt, Ti_PID_CHRt, Td_PID_CHRt);
fprintf('=========================================================\n');

%%  Figure  - Primary Comparison Plots (PID Architectures)


figure('Name', 'PID Closed-Loop Performance Analysis', 'Color', [0.98 0.98 0.98], 'Units', 'normalized', 'Position', [0.05 0.08 0.88 0.82]);

% --- Subplot 1: Water Level Response ---
hAx1 = subplot(2, 1, 1);
% Highlight Disturbance Regions
fill([600 800 800 600], [0 0 90 90], [1.0 0.92 0.85], 'EdgeColor', 'none', 'FaceAlpha', 0.5); hold on; grid on;
fill([1000 1200 1200 1000], [0 0 90 90], [0.92 0.90 0.98], 'EdgeColor', 'none', 'FaceAlpha', 0.5);

hR    = plot(t, r, '--', 'Color', c_ref, 'LineWidth', 1.8);
hZN   = plot(t, y_pid_zn, 'Color', c_zn1, 'LineWidth', 1.6);
hCHRd = plot(t, y_pid_chrd, 'Color', c_chrd1, 'LineWidth', 1.6);
hCHRt = plot(t, y_pid_chrt, 'Color', c_chrt2, 'LineWidth', 1.8);

% Annotations for Disturbances
text(610, 68, '\leftarrow Input Disturbance (d_{in} = +10)', 'FontSize', 10, 'FontWeight', 'bold', 'Color', [0.7 0.3 0.0]);
text(1000, 68, '\leftarrow Output Disturbance (d_{out} = +10)', 'FontSize', 10, 'FontWeight', 'bold', 'Color', [0.3 0.1 0.6]);

title('Tank Level Control: Tracking and Disturbance Rejection Performance', 'FontSize', 13, 'FontWeight', 'bold', 'Color', [0.1 0.1 0.2]);
ylabel('Fluid Level y(t) [cm]', 'FontSize', 11, 'FontWeight', 'bold');
xlim([0 1200]); ylim([20 75]);
set(hAx1, 'FontSize', 10.5, 'FontWeight', 'bold', 'LineWidth', 1.1, 'GridColor', [0.8 0.8 0.8], 'GridAlpha', 0.6);

% --- Subplot 2: Control Action (Valve Input) ---
hAx2 = subplot(2, 1, 2);
fill([600 800 800 600], [-100 -100 200 200], [1.0 0.92 0.85], 'EdgeColor', 'none', 'FaceAlpha', 0.5); hold on; grid on;
fill([1000 1200 1200 1000], [-100 -100 200 200], [0.92 0.90 0.98], 'EdgeColor', 'none', 'FaceAlpha', 0.5);

plot(t, u_pid_zn, 'Color', c_zn, 'LineWidth', 1.2);
plot(t, u_pid_chrd, 'Color', c_chrd, 'LineWidth', 1.2);
plot(t, u_pid_chrt, 'Color', c_chrt, 'LineWidth', 1.4);

title('Control Effort / Actuator Signal u(t)', 'FontSize', 13, 'FontWeight', 'bold', 'Color', [0.1 0.1 0.2]);
xlabel('Time t [s]', 'FontSize', 11, 'FontWeight', 'bold'); ylabel('Control Output u [%]', 'FontSize', 11, 'FontWeight', 'bold');
xlim([0 1200]); ylim([-40 120]);
set(hAx2, 'FontSize', 10.5, 'FontWeight', 'bold', 'LineWidth', 1.1, 'GridColor', [0.8 0.8 0.8], 'GridAlpha', 0.6);

% --- Shared Central Legend Between Subplots ---
lgd = legend(hAx1, [hR, hZN, hCHRd, hCHRt], ...
    {'Setpoint (r)', 'PID ZN (Disturbance)', 'PID CHR (Disturbance)', 'PID CHR (Tracking)'}, ...
    'Orientation', 'horizontal', 'FontSize', 10, 'FontWeight', 'bold');

% Position Legend in the Vertical Center
set(lgd, 'Units', 'normalized');
lgd_pos = get(lgd, 'Position');
lgd_pos(1) = 0.5 - (lgd_pos(3) / 2);  % Center horizontally
lgd_pos(2) = 0.51 - (lgd_pos(4) / 2); % Position between subplots
set(lgd, 'Position', lgd_pos, 'Box', 'on', 'EdgeColor', [0.7 0.7 0.7], 'Color', [1 1 1 0.95]);
%%  Figure 2 - PI vs PID Detailed Effect Analysis
figure('Name', 'PI vs PID Structural Comparison', 'Color', [0.98 0.98 0.98], 'Units', 'normalized', 'Position', [0.1 0.1 0.82 0.75]);

subplot(2, 1, 1);
plot(t, r, 'k--', 'LineWidth', 1.5); hold on; grid on;
plot(t, y_pi_chrd, ':', 'Color', c_chrd, 'LineWidth', 1.8, 'DisplayName', 'PI CHR Disturbance');
plot(t, y_pid_chrd, 'Color', c_chrd, 'LineWidth', 1.8, 'DisplayName', 'PID CHR Disturbance');
title('Effect of Derivative Term (Td): CHR Disturbance Rejection Strategy', 'FontSize', 12, 'FontWeight', 'bold');
ylabel('Level [cm]', 'FontSize', 11, 'FontWeight', 'bold'); xlim([0 1200]); ylim([25 70]);
legend('Location', 'northeast'); set(gca, 'FontSize', 10, 'FontWeight', 'bold');

subplot(2, 1, 2);
plot(t, r, 'k--', 'LineWidth', 1.5); hold on; grid on;
plot(t, y_pi_chrt, ':', 'Color', c_chrt, 'LineWidth', 1.8, 'DisplayName', 'PI CHR Tracking');
plot(t, y_pid_chrt, 'Color', c_chrt, 'LineWidth', 1.8, 'DisplayName', 'PID CHR Tracking');
title('Effect of Derivative Term (Td): CHR Setpoint Tracking Strategy', 'FontSize', 12, 'FontWeight', 'bold');
xlabel('Time t [s]', 'FontSize', 11, 'FontWeight', 'bold'); ylabel('Level [cm]', 'FontSize', 11, 'FontWeight', 'bold');
xlim([0 1200]); ylim([25 70]);
legend('Location', 'northeast'); set(gca, 'FontSize', 10, 'FontWeight', 'bold');

%%  Helper Function for Closed-Loop System Simulation
function [y, u] = sim_lsim(G, C, r, d_in, d_out, t)
    L = C * G;
    
    % Transfer Functions
    T_r    = feedback(L, 1);     % Reference -> Output
    T_din  = feedback(G, C);     % Input Disturbance -> Output
    T_dout = feedback(1, L);     % Output Disturbance -> Output
    S_u    = feedback(C, G);     % Reference -> Control Signal
    
    % Outputs Simulation
    y = lsim(T_r, r, t) + lsim(T_din, d_in, t) + lsim(T_dout, d_out, t);
    
    % Control Signal Simulation
    u = lsim(S_u, r, t) - lsim(feedback(C*G, 1), d_in, t) - lsim(S_u, d_out, t);
end