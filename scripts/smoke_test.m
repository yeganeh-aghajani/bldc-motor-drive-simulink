function smoke_test()
%SMOKE_TEST Short (60 ms) start-up run of BLDC_ClosedLoop.slx with basic checks.
%   Confirms that the model simulates, that all logged signals are finite,
%   that the Hall states and gate commands are valid and that the rotor
%   accelerates in the commanded (positive) direction. Errors on failure.

projectRoot = fileparts(fileparts(mfilename('fullpath')));
modelName   = 'BLDC_ClosedLoop';
evalin('base', sprintf('run(''%s'');', fullfile(projectRoot, 'scripts', 'parameters.m')));
load_system(fullfile(projectRoot, 'model', [modelName '.slx']));

in  = Simulink.SimulationInput(modelName);
in  = in.setModelParameter('StopTime', '0.06');
out = sim(in);
L   = out.logsout;
val = @(name) squeeze(double(L.get(name).Values.Data));

names = {'omega_ref_rpm', 'omega_meas_rpm', 'T_e', 'T_load', 'i_a', 'i_b', 'i_c', ...
         'Hall_A', 'Hall_B', 'Hall_C', 'gate_signals', 'duty_cmd', 'v_dc'};
for k = 1:numel(names)
    x = val(names{k});
    assert(~isempty(x), 'Signal %s was not logged.', names{k});
    assert(all(isfinite(x(:))), 'Signal %s contains NaN or Inf.', names{k});
end

w  = val('omega_meas_rpm');
hs = round(4*val('Hall_A') + 2*val('Hall_B') + val('Hall_C'));
g  = val('gate_signals') > 0.5;                  % N x 6, [AH AL BH BL CH CL]
iabc = [val('i_a'), val('i_b'), val('i_c')];
du = val('duty_cmd');
P  = evalin('base', 'struct(''uMin'', uMin, ''uMax'', uMax)');

assert(size(g, 2) == 6, 'Gate vector has %d elements instead of 6.', size(g, 2));
assert(~any(hs == 0 | hs == 7), 'Invalid Hall state 000 or 111 occurred.');
assert(~any((g(:,1) & g(:,2)) | (g(:,3) & g(:,4)) | (g(:,5) & g(:,6))), ...
    'Upper and lower switch of one leg were commanded on together.');
assert(any(g(:)), 'No gate was ever switched on.');
assert(max(abs(iabc(:))) > 0.1, 'No phase current flows.');
assert(min(w) > -1 && w(end) > 100, 'Rotor did not accelerate in the positive direction.');
assert(all(du >= P.uMin - 1e-9 & du <= P.uMax + 1e-9), 'Duty command left its limits.');

fprintf(['smoke_test PASSED: speed %.0f rpm at t = 60 ms, peak |i| = %.1f A, ' ...
         'peak T_e = %.2f N*m, Hall states seen: %s\n'], ...
    w(end), max(abs(iabc(:))), max(val('T_e')), mat2str(unique(hs)'));
end
