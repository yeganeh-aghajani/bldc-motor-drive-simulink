function plot_results(results, figDir)
%PLOT_RESULTS Generate and save the result figures (PNG) into figDir.

lw = 1.3;
set(groot, 'defaultAxesFontSize', 10, 'defaultLineLineWidth', lw);

%% Figure 1: reference vs measured speed (Test 1 and Test 3)
f = figure('Name', 'Speed tracking', 'Position', [100 100 800 600]);
tests = {'startup', 'ref_change'};
for k = 1:2
    d = results.(tests{k}).data;
    subplot(2, 1, k);
    plot(d.omega_ref_rpm.t, d.omega_ref_rpm.y, '--k'); hold on
    plot(d.omega_meas_rpm.t, d.omega_meas_rpm.y, 'b');
    grid on; ylabel('Speed (rpm)'); xlabel('Time (s)');
    title(results.(tests{k}).title);
    legend('Reference \omega_{ref}', 'Measured \omega', 'Location', 'southeast');
end
save_png(f, figDir, 'fig1_speed_tracking');

%% Figure 2: electromagnetic torque and load torque (Test 2)
r = results.load_step;  d = r.data;  tL = r.params.TloadStepTime;
f = figure('Name', 'Torque', 'Position', [100 100 800 400]);
plot(d.T_e.t, d.T_e.y, 'Color', [0.2 0.4 0.8]); hold on
plot(d.T_load.t, d.T_load.y, 'r', 'LineWidth', 2);
grid on; xlabel('Time (s)'); ylabel('Torque (N\cdotm)');
title('Test 2: Electromagnetic torque and load torque');
legend('T_e (electromagnetic)', 'T_{load} (mechanical load)', 'Location', 'northeast');
save_png(f, figDir, 'fig2_torque');

%% Figure 3: three-phase currents around the load step (Test 2)
f = figure('Name', 'Phase currents', 'Position', [100 100 800 400]);
win = @(x) x.t >= tL - 0.02 & x.t <= tL + 0.03;
w = win(d.i_a);
plot(d.i_a.t(w), d.i_a.y(w), d.i_b.t(w), d.i_b.y(w), d.i_c.t(w), d.i_c.y(w));
hold on; xline(tL, ':k', 'load step');
grid on; xlabel('Time (s)'); ylabel('Current (A)');
title('Test 2: Phase currents before and after the 1 N\cdotm load step');
legend('i_a', 'i_b', 'i_c', 'Location', 'northwest');
save_png(f, figDir, 'fig3_phase_currents');

%% Figure 4: Hall signals over one electrical period (Test 2, steady state)
[t0, t1] = steadyWindow(r);
f = figure('Name', 'Hall signals', 'Position', [100 100 800 550]);
names = {'Hall_A', 'Hall_B', 'Hall_C'};
for k = 1:3
    subplot(4, 1, k);
    w = d.(names{k}).t >= t0 & d.(names{k}).t <= t1;
    stairs(d.(names{k}).t(w), d.(names{k}).y(w), 'b');
    ylim([-0.2 1.2]); grid on; ylabel(strrep(names{k}, '_', ' '));
    if k == 1, title('Hall sensor signals over one electrical period (1500 rpm)'); end
end
subplot(4, 1, 4);
state = 4*d.Hall_A.y + 2*d.Hall_B.y + d.Hall_C.y;
w = d.Hall_A.t >= t0 & d.Hall_A.t <= t1;
stairs(d.Hall_A.t(w), state(w), 'k');
ylim([0 7]); yticks(1:6); grid on; ylabel('State'); xlabel('Time (s)');
save_png(f, figDir, 'fig4_hall_signals');

%% Figure 5: inverter gate signals over the same period
f = figure('Name', 'Gate signals', 'Position', [100 100 800 700]);
g = d.gate_signals;
w = g.t >= t0 & g.t <= t1;
gateNames = {'Gate AH', 'Gate AL', 'Gate BH', 'Gate BL', 'Gate CH', 'Gate CL'};
for k = 1:6
    subplot(6, 1, k);
    stairs(g.t(w), g.y(w, k), 'Color', [0.1 0.1 0.1], 'LineWidth', 0.8);
    ylim([-0.2 1.2]); grid on; ylabel(gateNames{k});
    if k == 1, title('Gate signals: PWM on the high-side switch, low side on for the full sector'); end
end
xlabel('Time (s)');
save_png(f, figDir, 'fig5_gate_signals');

%% Figure 6: load-disturbance speed response (zoom) and duty command
ref = r.params.refSpeed1;  band = 0.005*ref;
f = figure('Name', 'Load disturbance', 'Position', [100 100 800 550]);
subplot(2, 1, 1);
w = d.omega_meas_rpm.t >= tL - 0.02 & d.omega_meas_rpm.t <= tL + 0.1;
plot(d.omega_meas_rpm.t(w), d.omega_meas_rpm.y(w), 'b'); hold on
yline(ref, '--k'); yline(ref + band, ':k'); yline(ref - band, ':k');
xline(tL, ':r', 'load step');
grid on; ylabel('Speed (rpm)');
title('Test 2: Speed response to a 1 N\cdotm load step');
legend('Measured \omega', 'Reference', '\pm0.5 % band', 'Location', 'southeast');
subplot(2, 1, 2);
w = d.duty_cmd.t >= tL - 0.02 & d.duty_cmd.t <= tL + 0.1;
plot(d.duty_cmd.t(w), d.duty_cmd.y(w), 'Color', [0 0.5 0]);
grid on; ylabel('Duty command (-)'); xlabel('Time (s)');
save_png(f, figDir, 'fig6_load_disturbance');

%% Figure 7: anti-windup comparison (Test 4)
a = results.aw_off.data;  b = results.aw_on.data;
f = figure('Name', 'Anti-windup', 'Position', [100 100 800 700]);
subplot(3, 1, 1);
plot(a.omega_ref_rpm.t, a.omega_ref_rpm.y, '--k'); hold on
plot(a.omega_meas_rpm.t, a.omega_meas_rpm.y, 'r');
plot(b.omega_meas_rpm.t, b.omega_meas_rpm.y, 'b');
grid on; ylabel('Speed (rpm)');
title('Test 4: Speed reduction with the duty command limited at 0');
legend('Reference', 'PI without anti-windup', 'PI with anti-windup', 'Location', 'southwest');
subplot(3, 1, 2);
plot(a.duty_cmd.t, a.duty_cmd.y, 'r', b.duty_cmd.t, b.duty_cmd.y, 'b');
grid on; ylabel('Duty command (-)');
subplot(3, 1, 3);
plot(a.pi_integrator.t, a.pi_integrator.y, 'r', b.pi_integrator.t, b.pi_integrator.y, 'b');
grid on; ylabel('Integrator state (-)'); xlabel('Time (s)');
save_png(f, figDir, 'fig7_antiwindup');
end

%% ------------------------------------------------------------------------
function [t0, t1] = steadyWindow(r)
% One electrical period at the reference speed, just before the load step.
fe = r.params.refSpeed1/60*r.params.polePairs;          % electrical frequency [Hz]
t1 = r.params.TloadStepTime - 0.02;
t0 = t1 - 1/fe;
end

function save_png(f, figDir, name)
file = fullfile(figDir, [name '.png']);
theme(f, 'light');                                 % independent of the desktop theme
set(findall(f, 'Type', 'axes'), 'Toolbar', []);   % keep the toolbar out of the PNG
try
    exportgraphics(f, file, 'Resolution', 150);
catch
    print(f, file, '-dpng', '-r150');
end
end
