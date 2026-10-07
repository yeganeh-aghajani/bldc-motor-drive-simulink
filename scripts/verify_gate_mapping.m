function pass = verify_gate_mapping()
%VERIFY_GATE_MAPPING Check the gate-vector order of the Universal Bridge.
%   The Commutation Logic of BLDC_ClosedLoop.slx sends the gate vector
%   [AH AL BH BL CH CL] to a three-arm MOSFET/diode Universal Bridge. This
%   script builds a small temporary test bench with the same bridge settings
%   (rectifier port layout, Ron, snubber) and finds out from measurements
%   which element of the six-element gate vector drives which switch.
%
%   Test bench: ideal 100 V DC link, star-connected 10 Ohm phase resistors.
%   Every pair of gate elements (15 pairs) is switched on in turn. When the
%   upper switch of phase x and the lower switch of phase y conduct, the
%   terminal voltages (measured against the DC- rail) are v_x = Vdc and
%   v_y = 0, and the current Vdc/(2R) = 5 A flows out of x and back into y.
%   An element that always puts its phase at Vdc is an upper switch; one
%   that always pulls its phase to 0 V is a lower switch.
%
%   The temporary model is closed without saving; no files are written.
%   Returns true and prints PASS if the measured order is [AH AL BH BL CH CL].

projectRoot = fileparts(fileparts(mfilename('fullpath')));
run(fullfile(projectRoot, 'scripts', 'parameters.m'));   % Ron, Rsnubber, Csnubber, Ts

Vtest = 100;          % [V]   test DC-link voltage
Rload = 10;           % [Ohm] phase resistor
expected = {'AH', 'AL', 'BH', 'BL', 'CH', 'CL'};          % order used by the BLDC model

mdl = 'tmp_gate_mapping_bench';
if bdIsLoaded(mdl), close_system(mdl, 0); end
new_system(mdl);
cleanup = onCleanup(@() close_system(mdl, 0));
buildBench(mdl, Vtest, Rload, Ron, Rsnubber, Csnubber, Ts);

%% Switch on every pair of gate elements and measure
pairs = nchoosek(1:6, 2);
role  = cell(6, 1);           % e.g. {'AH'} if every valid pair agrees
fprintf('Gate pair   v_a     v_b     v_c   [V]    i_a    i_b    i_c   [A]\n');
for k = 1:size(pairs, 1)
    g = zeros(1, 6);  g(pairs(k, :)) = 1;
    [v, i] = runBench(mdl, g);
    fprintf('  %d+%d   %7.1f %7.1f %7.1f     %6.2f %6.2f %6.2f\n', ...
        pairs(k, 1), pairs(k, 2), v, i);

    hi = find(abs(v - Vtest) < 1);            % phase tied to DC+
    lo = find(abs(v) < 1);                    % phase tied to DC-
    I  = Vtest / (2*Rload);
    if isscalar(hi) && isscalar(lo) && ...
            abs(i(hi) - I) < 0.05 && abs(i(lo) + I) < 0.05
        % A valid upper/lower pair. Which of the two elements is the upper
        % switch is not known yet, so both candidate roles are recorded for
        % each element and resolved by intersection below.
        cand = {[char('A' + hi - 1) 'H'], [char('A' + lo - 1) 'L']};
        for e = pairs(k, :)
            role{e} = [role{e}, {cand}];
        end
    end
end

%% Resolve each element's role
% An upper switch xH forms a valid pair with yL and zL, so phase x is at Vdc
% in both of its valid pairs; a lower switch xL always has phase x at 0 V.
measured = repmat({'?'}, 1, 6);
for e = 1:6
    if numel(role{e}) ~= 2, continue; end
    common = intersect(role{e}{1}, role{e}{2});
    if isscalar(common), measured{e} = common{1}; end
end

fprintf('\nGate element:  %s\n', strjoin(compose('%6d', 1:6), ''));
fprintf('Measured:      %s\n', strjoin(compose('%6s', string(measured)), ''));
fprintf('Expected:      %s\n', strjoin(compose('%6s', string(expected)), ''));

pass = isequal(measured, expected);
if pass
    fprintf('\nPASS: Universal Bridge gate order is [AH AL BH BL CH CL].\n');
else
    fprintf('\nFAIL: measured gate order differs from [AH AL BH BL CH CL].\n');
end
end

%% ======================================================================
function buildBench(mdl, Vtest, Rload, Ron, Rsnubber, Csnubber, Ts)
% Same bridge block, settings and DC-link wiring as buildInverter() in
% build_bldc_model.m, with resistors instead of the motor.
load_system('sps_lib');
add_block('sps_lib/powergui', [mdl '/powergui']);
set_param([mdl '/powergui'], 'SimulationMode', 'Discrete', 'SampleTime', num2str(Ts));

add_block('simulink/Sources/Constant', [mdl '/gates'], 'Value', 'zeros(1,6)');
add_block('simulink/Sources/Constant', [mdl '/Vdc'], 'Value', num2str(Vtest));
src = [mdl '/DC Link'];
add_block('sps_lib/Sources/Controlled Voltage Source', src);
set_param(src, 'Initialize', 'on', 'Source_Type', 'DC', 'Amplitude', num2str(Vtest), ...
    'Measurements', 'None');

br = [mdl '/Universal Bridge'];
add_block('sps_lib/Power Electronics/Universal Bridge', br);
set_param(br, 'converterType', 'Rectifier', 'Arms', '3', 'Device', 'MOSFET / Diodes', ...
    'Ron', num2str(Ron), 'SnubberResistance', num2str(Rsnubber), ...
    'SnubberCapacitance', num2str(Csnubber), 'Measurements', 'None');

add_line(mdl, 'Vdc/1', 'DC Link/1');
add_line(mdl, 'gates/1', 'Universal Bridge/1');
pb = get_param(br, 'PortHandles');
ps = get_param(src, 'PortHandles');
add_line(mdl, ps.RConn(1), pb.RConn(1));          % source + -> bridge +
add_line(mdl, ps.LConn(1), pb.RConn(2));          % source - -> bridge -

% Per phase: current sensor (bridge -> resistor), resistor to the star
% point, and voltage sensor from the phase terminal to the DC- rail.
phase = 'abc';
star = [];
for k = 1:3
    cm = sprintf('%s/I_%s', mdl, phase(k));
    vm = sprintf('%s/V_%s', mdl, phase(k));
    r  = sprintf('%s/R_%s', mdl, phase(k));
    add_block('sps_lib/Sensors and Measurements/Current Measurement', cm);
    add_block('sps_lib/Sensors and Measurements/Voltage Measurement', vm);
    add_block('sps_lib/Passives/Series RLC Branch', r);
    set_param(r, 'BranchType', 'R', 'Resistance', num2str(Rload));
    pc = get_param(cm, 'PortHandles');
    pv = get_param(vm, 'PortHandles');
    pr = get_param(r,  'PortHandles');
    add_line(mdl, pb.LConn(k), pc.LConn(1));      % bridge terminal -> sensor +
    add_line(mdl, pc.RConn(1), pr.LConn(1));      % sensor -> resistor
    add_line(mdl, pv.LConn(1), pb.LConn(k));      % v_x measured against DC-
    add_line(mdl, pv.LConn(2), pb.RConn(2));
    if isempty(star)
        star = pr.RConn(1);
    else
        add_line(mdl, pr.RConn(1), star);         % star point
    end
    add_block('built-in/Outport', sprintf('%s/v_%s', mdl, phase(k)), 'Port', num2str(k));
    add_block('built-in/Outport', sprintf('%s/i_%s', mdl, phase(k)), 'Port', num2str(k+3));
    add_line(mdl, sprintf('V_%s/1', phase(k)), sprintf('v_%s/1', phase(k)));
    add_line(mdl, sprintf('I_%s/1', phase(k)), sprintf('i_%s/1', phase(k)));
end
end

function [v, i] = runBench(mdl, g)
% Resistive load: the circuit settles within one step, 20 us is plenty.
set_param([mdl '/gates'], 'Value', mat2str(g));
out = sim(mdl, 'StopTime', '2e-5', 'SaveOutput', 'on', 'OutputSaveName', 'yout', ...
    'SaveFormat', 'Array', 'ReturnWorkspaceOutputs', 'on');
y = out.yout(end, :);
v = y(1:3);
i = y(4:6);
end
