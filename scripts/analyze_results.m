function [metrics, T, checks] = analyze_results(results)
%ANALYZE_RESULTS Performance metrics and sanity checks from logged data.
%   All metrics are computed directly from the logged signals. stepinfo is
%   not used because the responses include a ramped reference, load steps
%   and saturation, i.e. they are not linear step responses.

rows = {};
checks = struct('description', {}, 'pass', {}, 'detail', {});

%% Test 1: startup
r = results.startup;  d = r.data;  p = r.params;
[t, w] = sig(d, 'omega_meas_rpm');
s = stepMetrics(t, w, 0, p.Tsim, 0, p.refSpeed1);
iabc = [d.i_a.y, d.i_b.y, d.i_c.y];
ss   = d.i_a.t >= p.Tsim - 0.05;
m.startup = s;
m.startup.peakCurrent_A   = max(abs(iabc(:)));
m.startup.peakTorque_Nm   = max(d.T_e.y);
m.startup.ssTorque_Nm     = mean(d.T_e.y(d.T_e.t >= p.Tsim - 0.05));
m.startup.rmsCurrentSS_A  = rmsValue(d.i_a.y(ss));
rows = [rows; {
    'Startup', 'Rise time (10-90 %)',          s.riseTime_ms,        'ms';
    'Startup', 'Overshoot',                    s.overshoot_pct,      '%';
    'Startup', 'Settling time (2 % band)',     s.settling_ms,        'ms';
    'Startup', 'Steady-state speed error',     s.ssError_rpm,        'rpm';
    'Startup', 'Peak phase current',           m.startup.peakCurrent_A,  'A';
    'Startup', 'Peak electromagnetic torque',  m.startup.peakTorque_Nm,  'N*m';
    'Startup', 'Steady-state torque',          m.startup.ssTorque_Nm,    'N*m';
    'Startup', 'RMS phase current (steady state)', m.startup.rmsCurrentSS_A, 'A'}];
checks(end+1) = check('Speed reaches the 1500 rpm reference (|error| < 0.5 %)', ...
    abs(s.ssError_rpm) < 0.005*p.refSpeed1, sprintf('error = %.2f rpm', s.ssError_rpm));
% The base load is an ideal torque source, so at standstill it pulls the
% rotor slightly backwards until the PI has built up motor torque.
checks(end+1) = check('Rotor turns in the commanded direction (reverse creep at start < 5 rpm)', ...
    min(w) > -5 && w(end) > 0, sprintf('min speed = %.2f rpm', min(w)));
checks(end+1) = check('DC-link voltage equals Vdc (polarity check)', ...
    abs(mean(d.v_dc.y) - p.Vdc) < 0.01*p.Vdc, sprintf('mean v_dc = %.1f V', mean(d.v_dc.y)));

%% Test 2: load disturbance
r = results.load_step;  d = r.data;  p = r.params;
[t, w] = sig(d, 'omega_meas_rpm');
tL  = p.TloadStepTime;  ref = p.refSpeed1;
aft = t >= tL;
[wMin, iMin] = min(w(aft));  tAft = t(aft);
band = 0.005*ref;                               % +/-0.5 % recovery band
outIdx = find(aft & abs(w - ref) > band, 1, 'last');
if isempty(outIdx),        recovery = 0;
elseif outIdx == numel(t), recovery = NaN;          % not recovered before Tsim
else,                      recovery = (t(outIdx) - tL)*1e3; end
before = @(x) x.y(x.t >= tL - 0.05 & x.t < tL);
last   = @(x) x.y(x.t >= p.Tsim - 0.05);
m.load.maxDrop_rpm   = ref - wMin;
m.load.tMaxDrop_ms   = (tAft(iMin) - tL)*1e3;
m.load.recovery_ms   = recovery;
m.load.ssError_rpm   = mean(w(t >= p.Tsim - 0.05)) - ref;
m.load.TeBefore_Nm   = mean(before(d.T_e));
m.load.TeAfter_Nm    = mean(last(d.T_e));
m.load.IrmsBefore_A  = rmsValue(before(d.i_a));
m.load.IrmsAfter_A   = rmsValue(last(d.i_a));
% Torque per ampere: with two phases conducting, (|ia|+|ib|+|ic|)/2 is the
% current in the conducting pair. A ratio close to Kt shows that the Hall
% decoder energises the phases whose back-EMF is on its flat top.
Ipair = (abs(last(d.i_a)) + abs(last(d.i_b)) + abs(last(d.i_c)))/2;
m.load.KtEff         = m.load.TeAfter_Nm/mean(Ipair);
rows = [rows; {
    'Load step', 'Maximum speed drop',               m.load.maxDrop_rpm,  'rpm';
    'Load step', 'Time to maximum drop',             m.load.tMaxDrop_ms,  'ms';
    'Load step', 'Recovery time (+/-0.5 % band)',    m.load.recovery_ms,  'ms';
    'Load step', 'Steady-state speed error after step', m.load.ssError_rpm, 'rpm';
    'Load step', 'Mean torque before step',          m.load.TeBefore_Nm,  'N*m';
    'Load step', 'Mean torque after step',           m.load.TeAfter_Nm,   'N*m';
    'Load step', 'RMS phase current before step',    m.load.IrmsBefore_A, 'A';
    'Load step', 'RMS phase current after step',     m.load.IrmsAfter_A,  'A';
    'Load step', 'Effective torque constant after step', m.load.KtEff,   'N*m/A'}];
dTe = m.load.TeAfter_Nm - m.load.TeBefore_Nm;
checks(end+1) = check('Electromagnetic torque follows the load step (within 10 %)', ...
    abs(dTe - p.TloadStep) < 0.1*p.TloadStep, sprintf('torque increase = %.3f N*m', dTe));
checks(end+1) = check('Phase current increases with load', ...
    m.load.IrmsAfter_A > m.load.IrmsBefore_A, ...
    sprintf('%.3f A -> %.3f A rms', m.load.IrmsBefore_A, m.load.IrmsAfter_A));
checks(end+1) = check('Torque per ampere matches Kt = 2*p*lambda within 5 % (correct commutation alignment)', ...
    abs(m.load.KtEff - p.Kt) < 0.05*p.Kt, sprintf('%.3f N*m/A vs %.3f N*m/A', m.load.KtEff, p.Kt));
checks(end+1) = check('Load step causes a temporary dip that the PI removes', ...
    m.load.maxDrop_rpm > 0 && abs(m.load.ssError_rpm) < band, ...
    sprintf('dip %.1f rpm, final error %.2f rpm', m.load.maxDrop_rpm, m.load.ssError_rpm));

%% Test 3: reference change
r = results.ref_change;  d = r.data;  p = r.params;
[t, w] = sig(d, 'omega_meas_rpm');
s1 = stepMetrics(t, w, p.refTime1, p.refTime2, 0, p.refSpeed1);
s2 = stepMetrics(t, w, p.refTime2, p.Tsim, p.refSpeed1, p.refSpeed2);
m.refChange.segment1 = s1;  m.refChange.segment2 = s2;
lbl1 = sprintf('0 -> %d rpm', p.refSpeed1);
lbl2 = sprintf('%d -> %d rpm', p.refSpeed1, p.refSpeed2);
rows = [rows; {
    'Reference change', [lbl1 ': rise time'],         s1.riseTime_ms,   'ms';
    'Reference change', [lbl1 ': overshoot'],         s1.overshoot_pct, '%';
    'Reference change', [lbl1 ': settling time'],     s1.settling_ms,   'ms';
    'Reference change', [lbl1 ': steady-state error'], s1.ssError_rpm,  'rpm';
    'Reference change', [lbl2 ': rise time'],         s2.riseTime_ms,   'ms';
    'Reference change', [lbl2 ': overshoot'],         s2.overshoot_pct, '%';
    'Reference change', [lbl2 ': settling time'],     s2.settling_ms,   'ms';
    'Reference change', [lbl2 ': steady-state error'], s2.ssError_rpm,  'rpm'}];

%% Test 4: anti-windup comparison (speed reduction, duty limited at 0)
names = {'aw_off', 'aw_on'};  labels = {'without anti-windup', 'with anti-windup'};
for k = 1:2
    r = results.(names{k});  d = r.data;  p = r.params;
    [t, w] = sig(d, 'omega_meas_rpm');
    tc = p.refTime2;  target = p.refSpeed2;
    aft = t >= tc;
    [du_t, du] = sig(d, 'duty_cmd');
    [xi_t, xi] = sig(d, 'pi_integrator');
    satTime = sum(du(du_t >= tc) <= p.uMin + 1e-6) * mean(diff(du_t))*1e3;
    s = stepMetrics(t, w, tc, p.Tsim, p.refSpeed1, target);
    aw.undershoot_rpm  = target - min(w(aft));
    aw.satTime_ms      = satTime;
    aw.minIntegrator   = min(xi(xi_t >= tc));
    aw.settling_ms     = s.settling_ms;
    m.(names{k}) = aw;
    rows = [rows; {
        'Anti-windup', ['Undershoot below 1000 rpm, ' labels{k}],   aw.undershoot_rpm, 'rpm';
        'Anti-windup', ['Time at duty limit, ' labels{k}],           aw.satTime_ms,     'ms';
        'Anti-windup', ['Minimum integrator state, ' labels{k}],     aw.minIntegrator,  '-';
        'Anti-windup', ['Settling time (2 % of step), ' labels{k}],  aw.settling_ms,    'ms'}]; %#ok<AGROW>
end
checks(end+1) = check('Anti-windup reduces the undershoot after saturation', ...
    m.aw_on.undershoot_rpm < m.aw_off.undershoot_rpm, ...
    sprintf('%.1f rpm -> %.1f rpm', m.aw_off.undershoot_rpm, m.aw_on.undershoot_rpm));

%% Commutation checks (all runs)
[nValid, nFwd, nOther, nIllegal] = deal(0);
[nShoot, nMulti, nSamples, nNonFinite] = deal(0);
[duMin, duMax] = deal(Inf, -Inf);
fn = fieldnames(results);
for k = 1:numel(fn)
    d = results.(fn{k}).data;
    sn = fieldnames(d);
    for i = 1:numel(sn), nNonFinite = nNonFinite + sum(~isfinite(d.(sn{i}).y(:))); end
    duMin = min(duMin, min(d.duty_cmd.y));  duMax = max(duMax, max(d.duty_cmd.y));
    [hv, nf, no, ni] = hallSequence(d);
    nValid = nValid + hv;  nFwd = nFwd + nf;  nOther = nOther + no;  nIllegal = nIllegal + ni;
    g = d.gate_signals.y > 0.5;                     % N x 6: [AH AL BH BL CH CL]
    nShoot = nShoot + sum((g(:,1) & g(:,2)) | (g(:,3) & g(:,4)) | (g(:,5) & g(:,6)));
    nMulti = nMulti + sum(sum(g(:,[1 3 5]), 2) > 1 | sum(g(:,[2 4 6]), 2) > 1);
    nSamples = nSamples + size(g, 1);
end
checks(end+1) = check('All logged signals are finite (no NaN/Inf) in every run', ...
    nNonFinite == 0, sprintf('%d non-finite samples', nNonFinite));
p = results.startup.params;
checks(end+1) = check('Duty command stays within its saturation limits', ...
    duMin >= p.uMin - 1e-9 && duMax <= p.uMax + 1e-9, sprintf('range %.3f ... %.3f', duMin, duMax));
checks(end+1) = check('Only legal Hall states (001...110) occur while rotating', ...
    nIllegal == 0, sprintf('%d samples, %d illegal', nValid, nIllegal));
checks(end+1) = check('Hall states advance in the order 5-4-6-2-3-1 (no skipped or reversed steps)', ...
    nFwd > 0 && nOther == 0, sprintf('%d forward transitions, %d other', nFwd, nOther));
checks(end+1) = check('No leg has its upper and lower switch on together', ...
    nShoot == 0, sprintf('%d of %d gate samples', nShoot, nSamples));
checks(end+1) = check('At most one upper and one lower switch conduct at a time', ...
    nMulti == 0, sprintf('%d of %d gate samples', nMulti, nSamples));

metrics = m;
T = cell2table(rows, 'VariableNames', {'Test', 'Metric', 'Value', 'Unit'});
end

%% ------------------------------------------------------------------------
function s = stepMetrics(t, y, t0, t1, y0, yf)
% Rise time, overshoot, 2 % settling time and steady-state error of the
% response between t0 and t1 for a set-point change y0 -> yf.
sel  = t >= t0 & t < t1;
tt   = t(sel) - t0;  yy = y(sel);
span = yf - y0;  n = (yy - y0)/span;
i10 = find(n >= 0.1, 1);  i90 = find(n >= 0.9, 1);
if isempty(i10) || isempty(i90), s.riseTime_ms = NaN;
else, s.riseTime_ms = (tt(i90) - tt(i10))*1e3; end
s.overshoot_pct = max(0, (max(n) - 1)*100);
iOut = find(abs(yy - yf) > 0.02*abs(span), 1, 'last');
if isempty(iOut),           s.settling_ms = 0;
elseif iOut == numel(tt),   s.settling_ms = NaN;          % never settled
else,                       s.settling_ms = tt(iOut)*1e3; end
s.ssError_rpm = mean(yy(tt >= tt(end) - 0.03)) - yf;
end

function [nValid, nFwd, nOther, nIllegal] = hallSequence(d)
% Hall state = 4*A + 2*B + C, checked only while the rotor is turning.
state = round(4*d.Hall_A.y + 2*d.Hall_B.y + d.Hall_C.y);
w = interp1(d.omega_meas_rpm.t, d.omega_meas_rpm.y, d.Hall_A.t, 'previous', 'extrap');
state = state(w > 100);
nValid   = numel(state);
nIllegal = sum(state == 0 | state == 7);
order = [5 4 6 2 3 1];               % forward rotation order
pos = zeros(1, 7);  pos(order + 1) = 1:6;
st = state(state >= 1 & state <= 6);
st = st([true; diff(st) ~= 0]);      % keep state changes only
step = mod(pos(st(2:end) + 1) - pos(st(1:end-1) + 1), 6);
nFwd   = sum(step == 1);
nOther = sum(step ~= 1);
end

function r = rmsValue(x)
r = sqrt(mean(x.^2));
end

function [t, y] = sig(d, name)
t = d.(name).t;  y = d.(name).y;
end

function c = check(description, pass, detail)
c = struct('description', description, 'pass', logical(pass), 'detail', detail);
end
