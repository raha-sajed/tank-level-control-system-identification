%% PI/PID Controller Comparison for Identified Tank Level Process
% Experimental System Identification and PI/PID Control of a Tank Level Process
%
% This script:
%   1. Defines the identified FOPTD tank-level model.
%   2. Computes PI/PID controller parameters using Ziegler-Nichols and CHR rules.
%   3. Saves controller parameters for reproducible analysis.
%   4. Loads and runs the Simulink controller-comparison model if available.
%   5. Exports simulation figures when expected Simulink outputs are available.

clear; clc; close all;

%% Step 1: Project Paths

projectRoot = fileparts(fileparts(mfilename('fullpath')));

figuresDir = fullfile(projectRoot, 'figures');
resultsDir = fullfile(projectRoot, 'results');
simulinkDir = fullfile(projectRoot, 'simulink');

if ~exist(figuresDir, 'dir')
    mkdir(figuresDir);
end

if ~exist(resultsDir, 'dir')
    mkdir(resultsDir);
end

addpath(simulinkDir);

%% Step 2: Identified FOPTD Model Parameters

% Identified from experimental step-response data.
K_sys     = 3.5682;   % Static gain [cm/%]
tau_sys   = 33.38;    % Time constant [s]
theta_sys = 13.50;    % Time delay [s]

% Dimensionless dead-time ratio
alpha = theta_sys / tau_sys;

% Common term used in Ziegler-Nichols and CHR open-loop tuning rules
Ka = K_sys * alpha;

fprintf('\n=========================================================\n');
fprintf('Identified FOPTD Model\n');
fprintf('=========================================================\n');
fprintf('Static gain K       = %.4f cm/%%\n', K_sys);
fprintf('Time constant tau   = %.2f s\n', tau_sys);
fprintf('Time delay theta    = %.2f s\n', theta_sys);
fprintf('Dead-time ratio     = %.4f\n', alpha);
fprintf('=========================================================\n');

%% Step 3: Ziegler-Nichols Open-Loop Tuning

% PI controller
Kp_PI_ZN = 0.9 / Ka;
Ti_PI_ZN = 3.3 * theta_sys;
Td_PI_ZN = 0;

Ki_PI_ZN = Kp_PI_ZN / Ti_PI_ZN;
Kd_PI_ZN = Kp_PI_ZN * Td_PI_ZN;

% PID controller
Kp_PID_ZN = 1.2 / Ka;
Ti_PID_ZN = 2.0 * theta_sys;
Td_PID_ZN = 0.5 * theta_sys;

Ki_PID_ZN = Kp_PID_ZN / Ti_PID_ZN;
Kd_PID_ZN = Kp_PID_ZN * Td_PID_ZN;

%% Step 4: CHR Tuning for Disturbance Rejection

% CHR PI controller for disturbance rejection
Kp_PI_CHRd = 0.6 / Ka;
Ti_PI_CHRd = 4.0 * theta_sys;
Td_PI_CHRd = 0;

Ki_PI_CHRd = Kp_PI_CHRd / Ti_PI_CHRd;
Kd_PI_CHRd = Kp_PI_CHRd * Td_PI_CHRd;

% CHR PID controller for disturbance rejection
Kp_PID_CHRd = 0.95 / Ka;
Ti_PID_CHRd = 2.38 * theta_sys;
Td_PID_CHRd = 0.42 * theta_sys;

Ki_PID_CHRd = Kp_PID_CHRd / Ti_PID_CHRd;
Kd_PID_CHRd = Kp_PID_CHRd * Td_PID_CHRd;

%% Step 5: CHR Tuning for Setpoint Tracking

% CHR PI controller for setpoint tracking
Kp_PI_CHRt = 0.35 / Ka;
Ti_PI_CHRt = 1.2 * tau_sys;
Td_PI_CHRt = 0;

Ki_PI_CHRt = Kp_PI_CHRt / Ti_PI_CHRt;
Kd_PI_CHRt = Kp_PI_CHRt * Td_PI_CHRt;

% CHR PID controller for setpoint tracking
Kp_PID_CHRt = 0.6 / Ka;
Ti_PID_CHRt = tau_sys;
Td_PID_CHRt = 0.5 * theta_sys;

Ki_PID_CHRt = Kp_PID_CHRt / Ti_PID_CHRt;
Kd_PID_CHRt = Kp_PID_CHRt * Td_PID_CHRt;

%% Step 6: Save Controller Parameters

controllerTable = table( ...
    ["PI Ziegler-Nichols"; ...
     "PID Ziegler-Nichols"; ...
     "PI CHR Disturbance"; ...
     "PID CHR Disturbance"; ...
     "PI CHR Tracking"; ...
     "PID CHR Tracking"], ...
    [Kp_PI_ZN; ...
     Kp_PID_ZN; ...
     Kp_PI_CHRd; ...
     Kp_PID_CHRd; ...
     Kp_PI_CHRt; ...
     Kp_PID_CHRt], ...
    [Ti_PI_ZN; ...
     Ti_PID_ZN; ...
     Ti_PI_CHRd; ...
     Ti_PID_CHRd; ...
     Ti_PI_CHRt; ...
     Ti_PID_CHRt], ...
    [Td_PI_ZN; ...
     Td_PID_ZN; ...
     Td_PI_CHRd; ...
     Td_PID_CHRd; ...
     Td_PI_CHRt; ...
     Td_PID_CHRt], ...
    [Ki_PI_ZN; ...
     Ki_PID_ZN; ...
     Ki_PI_CHRd; ...
     Ki_PID_CHRd; ...
     Ki_PI_CHRt; ...
     Ki_PID_CHRt], ...
    [Kd_PI_ZN; ...
     Kd_PID_ZN; ...
     Kd_PI_CHRd; ...
     Kd_PID_CHRd; ...
     Kd_PI_CHRt; ...
     Kd_PID_CHRt], ...
    'VariableNames', { ...
        'Controller', ...
        'Kp', ...
        'Ti_s', ...
        'Td_s', ...
        'Ki', ...
        'Kd'} ...
);

disp(' ');
disp('Controller parameters:');
disp(controllerTable);

writetable(controllerTable, ...
    fullfile(resultsDir, 'controller_parameters.csv'));

%% Step 7: Export Parameters to Base Workspace for Simulink

% The Simulink model may use these variables directly from the MATLAB base
% workspace. assignin keeps compatibility with the original model design.

controllerVariables = who;

for i = 1:numel(controllerVariables)
    variableName = controllerVariables{i};

    if ~strcmp(variableName, 'controllerVariables') && ...
       ~strcmp(variableName, 'i') && ...
       ~strcmp(variableName, 'variableName')
        assignin('base', variableName, eval(variableName));
    end
end

fprintf('\nController parameters were exported to the MATLAB base workspace.\n');

%% Step 8: Run Simulink Model if Available

modelName = 'pi_pid_control_simulink';
modelPath = fullfile(simulinkDir, [modelName '.slx']);

simOut = [];

if exist(modelPath, 'file') ~=0
    fprintf('\nLoading Simulink model: %s\n', modelPath);

    try
        if ~bdIsLoaded(modelName)
            load_system(modelPath);
        end

        fprintf('Running Simulink model...\n');
        simOut = sim(modelName);

        fprintf('Simulink simulation completed successfully.\n');

    catch ME
        warning('Simulink model could not be executed automatically.');
        warning('%s', ME.message);
        fprintf('\nYou can still open and run the model manually:\n');
        fprintf('open_system(''%s'')\n', modelPath);
    end
else
    warning('Simulink model was not found at: %s', modelPath);
end

%% Step 9: Try to Extract Simulation Output

% Different Simulink models may save output signals with different names.
% This section checks common output locations without forcing one format.

hasSimulationOutput = false;

try
    if ~isempty(simOut)

        availableVars = simOut.who;

        fprintf('\nAvailable Simulink output variables:\n');
        disp(availableVars);

        % Common expected format from the original report/code:
        % y_sim is usually a timeseries or structure containing simulation data.
        if any(strcmp(availableVars, 'y_sim'))
            y_sim = simOut.get('y_sim');
            hasSimulationOutput = true;
        elseif any(strcmp(availableVars, 'out'))
            out = simOut.get('out');
            hasSimulationOutput = true;
        else
            warning('Expected output variable y_sim was not found in Simulink output.');
        end
    end
catch ME
    warning('Could not inspect Simulink output variables.');
    warning('%s', ME.message);
end

%% Step 10: Export Readable Controller Comparisons
% Column order follows the six-input Mux in the original Simulink model.
if exist('y_sim', 'var')
    t_sim = y_sim.Time(:);
    y_data = squeeze(y_sim.Data);
    assert(size(y_data, 1) == numel(t_sim) && size(y_data, 2) == 6, ...
        'Expected six controller signals with one row per time sample.');

    % Reference and disturbance times from the original experiment.
    r_ref = 35 * ones(size(t_sim));
    r_ref(t_sim >= 400 & t_sim < 800) = 55;
    r_ref(t_sim >= 800) = 45;

    colors = [0.85 0.32 0.10; 0.63 0.08 0.18; ...
              0.30 0.65 0.85; 0.00 0.35 0.65; ...
              0.47 0.67 0.19; 0.13 0.50 0.13];
    labels = {'PI ZN', 'PID ZN', 'PI CHR (Dist)', ...
              'PID CHR (Dist)', 'PI CHR (Tracking)', 'PID CHR (Tracking)'};
    styles = {'--', '-', '--', '-', '--', '-'};
    groups = {1:6, 1:4, 5:6, [2 4 6]};
    titles = {'PI/PID Controller Comparison', ...
              'Disturbance Rejection: Ziegler-Nichols vs. CHR', ...
              'Setpoint Tracking: CHR Controllers', ...
              'PID Design Strategies: ZN vs. CHR'};
    files = {'pid_controller_comparison', 'pid_disturbance_rejection', ...
             'pid_setpoint_tracking', 'pid_strategy_comparison'};

    for panel = 1:numel(groups)
        figResponse = figure('Name', titles{panel}, 'Color', 'w', ...
            'Units', 'pixels', 'Position', [100 100 1000 620]);
        ax = axes(figResponse);
        hold(ax, 'on'); grid(ax, 'on'); box(ax, 'on');
        set(ax, 'FontName', 'Helvetica', 'FontSize', 12, ...
            'LineWidth', 0.8, 'GridAlpha', 0.12);
        plot(ax, t_sim, r_ref, ':', 'Color', [0.2 0.2 0.2], ...
            'LineWidth', 2, 'DisplayName', 'Reference');
        for column = groups{panel}
            plot(ax, t_sim, y_data(:, column), styles{column}, ...
                'Color', colors(column, :), 'LineWidth', 1.7, ...
                'DisplayName', labels{column});
        end
        xline(ax, 600, '-.', 'Input disturbance', ...
            'Color', [0.4 0.4 0.4], 'FontSize', 10, ...
            'LabelVerticalAlignment', 'bottom', ...
            'LabelOrientation', 'horizontal', 'HandleVisibility', 'off');
        xline(ax, 1000, '-.', 'Output disturbance', ...
            'Color', [0.4 0.4 0.4], 'FontSize', 10, ...
            'LabelVerticalAlignment', 'bottom', ...
            'LabelOrientation', 'horizontal', 'HandleVisibility', 'off');
        title(ax, titles{panel}, 'FontSize', 15, 'FontWeight', 'bold');
        xlabel(ax, 'Time (s)', 'FontSize', 13);
        ylabel(ax, 'Tank level (cm)', 'FontSize', 13);
        xlim(ax, [0 1200]); ylim(ax, [0 80]);
        xticks(ax, 0:200:1200);
        legend(ax, 'Location', 'northeast', 'NumColumns', 2, ...
            'FontSize', 11, 'Box', 'on');
        drawnow;
        exportgraphics(figResponse, fullfile(figuresDir, [files{panel} '.png']), ...
            'Resolution', 180);
    end
    fprintf('Four controller comparison figures saved successfully.\n');
else
    warning('No y_sim output was found; controller figures were not updated.');
end

%% Step 11: Save Reference Performance Metrics from Report

% These values are taken from the project report and are saved here so the
% numerical comparison is available in machine-readable form.
%
% If you later recompute the metrics directly from Simulink outputs, this
% table can be replaced with automatically computed values.

performanceTable = table( ...
    ["PI Ziegler-Nichols"; ...
     "PID Ziegler-Nichols"; ...
     "PI CHR Disturbance"; ...
     "PID CHR Disturbance"; ...
     "PI CHR Tracking"; ...
     "PID CHR Tracking"], ...
    [27.77; 64.50; 0.00; 22.35; 0.00; 2.44], ...
    [0.00; 0.00; 0.03; 0.00; 0.01; 0.00], ...
    [28.27; 64.11; 0.00; 22.58; 0.00; 2.37], ...
    [0.08; 0.07; 0.34; 0.00; 0.24; 0.01], ...
    [26.13; 62.48; 0.00; 21.63; 0.00; 2.42], ...
    [0.05; 0.03; 0.25; 0.00; 0.15; 0.01], ...
    [14.97; 12.17; 16.31; 12.63; 18.33; 13.63], ...
    [10.00; 10.04; 10.00; 10.00; 10.00; 10.00], ...
    'VariableNames', { ...
        'Controller', ...
        'Overshoot_Test1_percent', ...
        'SteadyStateError_Test1_cm', ...
        'Overshoot_Test2_percent', ...
        'SteadyStateError_Test2_cm', ...
        'Undershoot_Test3_percent', ...
        'SteadyStateError_Test3_cm', ...
        'DisturbanceDeviation1_cm', ...
        'DisturbanceDeviation2_cm'} ...
);

disp(' ');
disp('Reference performance metrics from report:');
disp(performanceTable);

writetable(performanceTable, ...
    fullfile(resultsDir, 'controller_performance_metrics_from_report.csv'));

%% Step 12: Final Status Message

fprintf('\nPI/PID controller comparison completed.\n');
fprintf('Results saved to: %s\n', resultsDir);
fprintf('Figures saved to: %s\n', figuresDir);