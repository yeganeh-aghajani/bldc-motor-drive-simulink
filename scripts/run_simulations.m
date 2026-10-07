%% run_simulations.m
% Runs all test scenarios on model/BLDC_ClosedLoop.slx, computes the
% performance metrics, performs sanity checks, saves the figures and writes
% the results table into results/metrics/ and README.md.
%
% Usage (from the repository root or the scripts folder):
%   >> run_simulations

clear; close all; clc
projectRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(projectRoot, 'scripts'));
figDir    = fullfile(projectRoot, 'results', 'figures');
metricDir = fullfile(projectRoot, 'results', 'metrics');
if ~exist(figDir, 'dir'),    mkdir(figDir);    end
if ~exist(metricDir, 'dir'), mkdir(metricDir); end

parameters;                                   % nominal values -> workspace
modelName = 'BLDC_ClosedLoop';
modelFile = fullfile(projectRoot, 'model', [modelName '.slx']);
if ~exist(modelFile, 'file')
    fprintf('Model not found, building it first...\n');
    build_bldc_model;
end
load_system(modelFile);

%% Scenario definitions (only the values that differ from parameters.m)
sc = struct([]);
sc(1).name = 'startup';    sc(1).title = 'Test 1: Startup to 1500 rpm';
sc(1).vars = {'Tsim', 0.3};

sc(2).name = 'load_step';  sc(2).title = 'Test 2: 1 N*m load step at 1500 rpm';
sc(2).vars = {'Tsim', 0.5, 'TloadStepTime', 0.25};

sc(3).name = 'ref_change'; sc(3).title = 'Test 3: Reference 0 -> 1000 -> 1500 rpm';
sc(3).vars = {'Tsim', 0.5, 'refSpeed1', 1000, 'refTime2', 0.25, 'refSpeed2', 1500};

sc(4).name = 'aw_off';     sc(4).title = 'Test 4a: 1500 -> 1000 rpm, anti-windup off';
sc(4).vars = {'Tsim', 0.8, 'refTime2', 0.3, 'refSpeed2', 1000, 'antiWindupEnable', 0};

sc(5).name = 'aw_on';      sc(5).title = 'Test 4b: 1500 -> 1000 rpm, anti-windup on';
sc(5).vars = {'Tsim', 0.8, 'refTime2', 0.3, 'refSpeed2', 1000, 'antiWindupEnable', 1};

%% Run
results = struct();
for k = 1:numel(sc)
    fprintf('Running %s ...\n', sc(k).title);
    in = Simulink.SimulationInput(modelName);
    for v = 1:2:numel(sc(k).vars)
        in = in.setVariable(sc(k).vars{v}, sc(k).vars{v+1});
    end
    stopTime = sc(k).vars{find(strcmp(sc(k).vars(1:2:end), 'Tsim'))*2};
    in = in.setModelParameter('StopTime', num2str(stopTime));
    tic; out = sim(in); fprintf('   done in %.1f s\n', toc);

    % scenario parameters = nominal values + overrides
    p = nominalParameters();
    for v = 1:2:numel(sc(k).vars), p.(sc(k).vars{v}) = sc(k).vars{v+1}; end

    results.(sc(k).name).data   = extractSignals(out.logsout);
    results.(sc(k).name).params = p;
    results.(sc(k).name).title  = sc(k).title;
end

%% Metrics, checks, figures
[metrics, metricTable, checks] = analyze_results(results);
plot_results(results, figDir);

save(fullfile(metricDir, 'metrics.mat'), 'metrics', 'metricTable', 'checks');
writetable(metricTable, fullfile(metricDir, 'metrics_summary.csv'));
md = [tableToMarkdown(metricTable), newline, checksToMarkdown(checks)];
writeText(fullfile(metricDir, 'metrics_summary.md'), md);
updateReadme(fullfile(projectRoot, 'README.md'), md);

disp(metricTable);
fprintf('\nSanity checks:\n');
for c = 1:numel(checks)
    fprintf('  [%s] %s  (%s)\n', ternary(checks(c).pass, 'PASS', 'FAIL'), ...
        checks(c).description, checks(c).detail);
end

%% ------------------------------------------------------------------------
function p = nominalParameters()
% Evaluate parameters.m in an isolated workspace and return a struct.
parameters;
w = whos;
p = struct();
for i = 1:numel(w), p.(w(i).name) = eval(w(i).name); end
end

function data = extractSignals(logsout)
% Convert the logged Dataset into a struct: data.<name>.t / data.<name>.y
data = struct();
for i = 1:logsout.numElements
    el = logsout.getElement(i);
    ts = el.Values;
    y  = ts.Data;
    if ndims(y) == 3, y = permute(y, [3 1 2]); end   % 1 x n x N -> N x n
    data.(el.Name).t = ts.Time;
    data.(el.Name).y = double(squeeze(y));
end
end

function md = tableToMarkdown(T)
md = sprintf('| Test | Metric | Result | Unit |\n|------|--------|--------|------|\n');
for i = 1:height(T)
    md = [md, sprintf('| %s | %s | %s | %s |\n', T.Test{i}, T.Metric{i}, ...
          formatValue(T.Value(i)), T.Unit{i})]; %#ok<AGROW>
end
end

function s = formatValue(x)
if abs(x) >= 100 || x == round(x), s = sprintf('%.0f', x);
elseif abs(x) >= 1,                s = sprintf('%.2f', x);
else,                              s = sprintf('%.3f', x);
end
end

function md = checksToMarkdown(checks)
md = sprintf('**Sanity checks**\n\n');
for c = 1:numel(checks)
    md = [md, sprintf('- **%s** %s (%s)\n', ternary(checks(c).pass, 'PASS', 'FAIL'), ...
          checks(c).description, checks(c).detail)]; %#ok<AGROW>
end
end

function updateReadme(readmeFile, md)
% Replace the text between the two markers in README.md with the new table.
startTag = '<!-- RESULTS_TABLE_START -->';
endTag   = '<!-- RESULTS_TABLE_END -->';
if ~exist(readmeFile, 'file'), return; end
txt = fileread(readmeFile);
i1 = strfind(txt, startTag);  i2 = strfind(txt, endTag);
if isempty(i1) || isempty(i2), return; end
txt = [txt(1:i1+numel(startTag)-1), newline, md, txt(i2:end)];
writeText(readmeFile, txt);
end

function writeText(file, txt)
fid = fopen(file, 'w', 'n', 'UTF-8');
fprintf(fid, '%s', txt);
fclose(fid);
end

function out = ternary(cond, a, b)
if cond, out = a; else, out = b; end
end
