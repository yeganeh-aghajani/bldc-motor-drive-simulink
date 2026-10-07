%% design_speed_controller.m
% PI tuning on an averaged (linear) model of the six-step drive.
%
% With two phases conducting and PWM averaged over a switching period, the
% motor behaves like a DC motor fed with the average voltage D*Vdc:
%   D*Vdc = 2*Rs*i + 2*Ls*di/dt + Ke*w
%   J*dw/dt = Kt*i - B*w - T_load
% This gives the plant  G(s) = w(s)/D(s) = Vdc*Kt / ((2Ls s + 2Rs)(J s + B) + Kt*Ke).
% The model ignores commutation, PWM ripple and the discontinuous conduction
% that appears at very light load, so the final gains are checked in Simulink.
%
% Only base MATLAB is used (no Control System Toolbox).

clear; clc
projectRoot = fileparts(fileparts(mfilename('fullpath')));
run(fullfile(projectRoot, 'scripts', 'parameters.m'));

%% Plant
den = conv([2*Ls, 2*Rs], [J, B]) + [0 0 Kt*Ke];
num = Vdc*Kt;
poles = roots(den);
wn = abs(poles(1));  zeta = -real(poles(1))/wn;
fprintf('Plant static gain       : %.1f (rad/s) per unit duty\n', num/den(end));
fprintf('Plant poles             : %.1f +/- j%.1f rad/s (wn = %.0f rad/s, zeta = %.2f)\n', ...
    real(poles(1)), abs(imag(poles(1))), wn, zeta);

%% Loop analysis for several gain pairs
% A small delay represents speed sampling (Ts_speed) and PWM averaging.
Td = 1.5*Ts_speed;
w  = logspace(0, 4, 20000);
s  = 1i*w;
G  = num ./ polyval(den, s) .* exp(-s*Td);

candidates = [0      0.5;     % integral only
              0.002  0.3;
              0.002  0.5;     % selected
              0.002  0.8;
              0.004  0.5;
              0.004  1.0];
fprintf('\n   Kp      Ki    crossover  phase margin  gain margin  overshoot  settling(2%%)\n');
for k = 1:size(candidates, 1)
    kp = candidates(k, 1);  ki = candidates(k, 2);
    [wc, pm, gm] = loopMargins(kp, ki, G, w);
    [os, ts] = linearStep(kp, ki, Rs, Ls, Kt, Ke, J, B, Vdc);
    fprintf(' %6.3f  %5.2f   %6.0f rad/s    %5.1f deg     %5.1f dB    %5.1f %%    %5.1f ms\n', ...
        kp, ki, wc, pm, gm, os, ts*1e3);
end

%% Plot the loop frequency response for the selected gains
Lsel = (Kp + Ki./s).*G;
figure('Name', 'PI design', 'Position', [100 100 700 550]);
subplot(2, 1, 1);
semilogx(w, 20*log10(abs(Lsel)), 'b', 'LineWidth', 1.3); hold on
yline(0, ':k'); grid on; ylabel('|L(j\omega)| (dB)');
title(sprintf('Open-loop response, averaged model (K_p = %.3f, K_i = %.2f)', Kp, Ki));
subplot(2, 1, 2);
semilogx(w, unwrap(angle(Lsel))*180/pi, 'b', 'LineWidth', 1.3); hold on
yline(-180, ':k'); grid on; ylabel('Phase (deg)'); xlabel('\omega (rad/s)');
figDir = fullfile(projectRoot, 'results', 'figures');
if ~exist(figDir, 'dir'), mkdir(figDir); end
theme(gcf, 'light');
set(findall(gcf, 'Type', 'axes'), 'Toolbar', []);
exportgraphics(gcf, fullfile(figDir, 'fig0_pi_design_open_loop.png'), 'Resolution', 150);

%% ------------------------------------------------------------------------
function [wc, pm, gm] = loopMargins(kp, ki, G, w)
L  = (kp + ki./(1i*w)).*G;
mag = abs(L);  ph = unwrap(angle(L))*180/pi;
ic = find(diff(sign(mag - 1)) ~= 0, 1, 'last');     % last 0 dB crossing
wc = w(ic);  pm = 180 + ph(ic);
ip = find(diff(sign(ph + 180)) ~= 0, 1, 'first');   % first -180 deg crossing
if isempty(ip), gm = Inf; else, gm = -20*log10(mag(ip)); end
end

function [os, ts] = linearStep(kp, ki, Rs, Ls, Kt, Ke, J, B, Vdc)
% Unsaturated 1 rad/s reference step on the averaged model (Euler, 10 us).
dt = 1e-5;  N = round(0.3/dt);
i = 0;  wm = 0;  xi = 0;  y = zeros(N, 1);
for k = 1:N
    e = 1 - wm;  u = kp*e + xi;  xi = xi + dt*ki*e;
    di = (u*Vdc - Ke*wm - 2*Rs*i)/(2*Ls);
    dw = (Kt*i - B*wm)/J;
    i = i + dt*di;  wm = wm + dt*dw;  y(k) = wm;
end
t  = (1:N)'*dt;
os = max(0, (max(y) - 1)*100);
iOut = find(abs(y - 1) > 0.02, 1, 'last');
if isempty(iOut), ts = 0; else, ts = t(iOut); end
end
