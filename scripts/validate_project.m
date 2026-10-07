function validate_project()
%VALIDATE_PROJECT Check that the repository is complete and the model works.
%   Run after build_bldc_model, design_speed_controller and run_simulations.
%   Any failed check raises an error (non-zero exit code in matlab -batch).

projectRoot = fileparts(fileparts(mfilename('fullpath')));
modelName   = 'BLDC_ClosedLoop';
nFail = 0;

%% 1. Source files and model
src = {'parameters.m', 'build_bldc_model.m', 'design_speed_controller.m', ...
       'run_simulations.m', 'analyze_results.m', 'plot_results.m', 'smoke_test.m'};
for k = 1:numel(src)
    nFail = nFail + report(isfile(fullfile(projectRoot, 'scripts', src{k})), ...
        ['source file scripts/' src{k}]);
end
modelFile = fullfile(projectRoot, 'model', [modelName '.slx']);
nFail = nFail + report(isfile(modelFile), 'model/BLDC_ClosedLoop.slx exists');

%% 2. Load and update (compile) the model
evalin('base', sprintf('run(''%s'');', fullfile(projectRoot, 'scripts', 'parameters.m')));
try
    load_system(modelFile);
    set_param(modelName, 'SimulationCommand', 'update');
    ok = true;  msg = '';
catch err
    ok = false;  msg = [' (' err.message ')'];
end
nFail = nFail + report(ok, ['model loads and updates without errors' msg]);

%% 3. Short simulation with signal checks
try
    smoke_test;
    ok = true;  msg = '';
catch err
    ok = false;  msg = [' (' err.message ')'];
end
nFail = nFail + report(ok, ['smoke test: finite signals, valid Hall states, no shoot-through' msg]);

%% 4. Results of the full scenario run
metricDir = fullfile(projectRoot, 'results', 'metrics');
files = {'metrics_summary.csv', 'metrics_summary.md', 'metrics.mat'};
for k = 1:numel(files)
    nFail = nFail + report(isfile(fullfile(metricDir, files{k})), ...
        ['results/metrics/' files{k}]);
end
if isfile(fullfile(metricDir, 'metrics.mat'))
    S = load(fullfile(metricDir, 'metrics.mat'), 'checks', 'metricTable');
    nFail = nFail + report(all([S.checks.pass]), sprintf( ...
        'all %d engineering sanity checks of run_simulations passed', numel(S.checks)));
    nFail = nFail + report(~any(isnan(S.metricTable.Value)), 'every reported metric is available (no NaN)');
end

figs = {'fig0_pi_design_open_loop', 'fig1_speed_tracking', 'fig2_torque', ...
        'fig3_phase_currents', 'fig4_hall_signals', 'fig5_gate_signals', ...
        'fig6_load_disturbance', 'fig7_antiwindup'};
for k = 1:numel(figs)
    nFail = nFail + report(isfile(fullfile(projectRoot, 'results', 'figures', [figs{k} '.png'])), ...
        ['results/figures/' figs{k} '.png']);
end

%% Summary
if nFail > 0
    error('validate_project:failed', '%d validation check(s) failed.', nFail);
end
fprintf('validate_project: all checks passed.\n');
end

function failed = report(pass, description)
if pass, tag = 'PASS'; else, tag = 'FAIL'; end
fprintf('  [%s] %s\n', tag, description);
failed = double(~pass);
end
