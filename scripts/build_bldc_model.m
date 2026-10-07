function build_bldc_model()
%BUILD_BLDC_MODEL Create model/BLDC_ClosedLoop.slx from scratch.
%
%   Requires Simulink and Simscape Electrical (Specialized Power Systems).
%   The script builds the complete drive: speed reference, discrete PI speed
%   controller with anti-windup, Hall decoder + PWM + gate interlock,
%   three-phase MOSFET inverter, trapezoidal PMSM (BLDC) and signal logging.
%   All numerical values are taken from parameters.m by name.

projectRoot = fileparts(fileparts(mfilename('fullpath')));
modelName   = 'BLDC_ClosedLoop';
modelFile   = fullfile(projectRoot, 'model', [modelName '.slx']);

% Parameters must exist in the base workspace so block dialogs can resolve them
evalin('base', sprintf('run(''%s'');', fullfile(projectRoot, 'scripts', 'parameters.m')));

if bdIsLoaded(modelName), close_system(modelName, 0); end
if exist(modelFile, 'file'), delete(modelFile); end
new_system(modelName);
lib = findPowerLibrary();

%% Solver and logging configuration
set_param(modelName, ...
    'SolverType', 'Fixed-step', 'Solver', 'FixedStepDiscrete', ...
    'FixedStep', 'Ts', 'StopTime', 'Tsim', ...
    'SignalLogging', 'on', 'SignalLoggingName', 'logsout', ...
    'ReturnWorkspaceOutputs', 'on', 'SaveTime', 'off', 'SaveOutput', 'off', ...
    'AutoInsertRateTranBlk', 'on');
setChecked(modelName, 'EnableMultiTasking', 'off');   % single-tasking

%% Top-level blocks
add_block(lib.powergui, [modelName '/powergui'], 'Position', [40 30 110 70]);
setChecked([modelName '/powergui'], 'SimulationMode', 'Discrete');
setChecked([modelName '/powergui'], 'SampleTime', 'Ts');

buildReferenceSpeed([modelName '/Reference Speed'],       [40 150 150 200]);
buildSpeedController([modelName '/Speed Controller'],     [220 145 340 215]);
buildCommutationLogic([modelName '/Commutation Logic'],   [420 130 550 260]);
buildInverter([modelName '/Three-Phase Inverter'], lib,   [620 150 740 250]);
buildMechanicalLoad([modelName '/Mechanical Load'],       [640 330 750 370]);

motor = [modelName '/BLDC Motor'];
add_block(lib.machine, motor, 'Position', [830 160 930 260]);
configureMotor(motor);

buildMeasurements([modelName '/Measurements'],            [1010 150 1130 330]);
buildMonitoring([modelName '/Performance Monitoring'],    [1010 430 1150 620]);

%% Top-level signal connections
m = modelName;
addLineNamed(m, 'Reference Speed/1', 'Speed Controller/1', 'omega_ref_rpm');
logPort([m '/Reference Speed'], 1, 20);
add_line(m, 'Reference Speed/1', 'Performance Monitoring/1', 'autorouting', 'on');

addLineNamed(m, 'Speed Controller/1', 'Commutation Logic/1', 'duty_cmd');
logPort([m '/Speed Controller'], 1, 1);
add_line(m, 'Speed Controller/1', 'Performance Monitoring/8', 'autorouting', 'on');

addLineNamed(m, 'Commutation Logic/1', 'Three-Phase Inverter/1', 'gate_cmd');
addLineNamed(m, 'Three-Phase Inverter/1', 'Performance Monitoring/9', 'v_dc');
logPort([m '/Three-Phase Inverter'], 1, 200);

addLineNamed(m, 'Mechanical Load/1', 'BLDC Motor/1', 'T_load');
logPort([m '/Mechanical Load'], 1, 20);
add_line(m, 'Mechanical Load/1', 'Performance Monitoring/4', 'autorouting', 'on');

addLineNamed(m, 'BLDC Motor/1', 'Measurements/1', 'motor_bus');

% Measurements outputs: 1 omega_meas_rpm, 2-4 Hall_A..C, 5-7 i_a..i_c, 8 T_e
addLineNamed(m, 'Measurements/1', 'Speed Controller/2', 'omega_meas_rpm');
add_line(m, 'Measurements/1', 'Performance Monitoring/2', 'autorouting', 'on');
hallNames = {'Hall_A', 'Hall_B', 'Hall_C'};
for k = 1:3
    addLineNamed(m, sprintf('Measurements/%d', k+1), ...
        sprintf('Commutation Logic/%d', k+1), hallNames{k});
end
curNames = {'i_a', 'i_b', 'i_c'};
for k = 1:3
    addLineNamed(m, sprintf('Measurements/%d', k+4), ...
        sprintf('Performance Monitoring/%d', k+4), curNames{k});
end
addLineNamed(m, 'Measurements/8', 'Performance Monitoring/3', 'T_e');
for k = 1:8, logPort([m '/Measurements'], k, 20); end

% Power connections: inverter phase terminals -> motor terminals
inv = get_param([m '/Three-Phase Inverter'], 'PortHandles');
mot = get_param(motor, 'PortHandles');
for k = 1:3
    add_line(m, inv.RConn(k), mot.LConn(k), 'autorouting', 'on');
end

addAnnotation(m, [40 690], sprintf([ ...
    'Closed-loop BLDC drive: Hall-based six-step commutation with PI speed control.\n' ...
    'Parameters: scripts/parameters.m   |   Scenarios: scripts/run_simulations.m']));

set_param(m, 'ZoomFactor', 'FitSystem');
save_system(m, modelFile);
fprintf('Model saved to %s\n', modelFile);
end

%% ======================================================================
%% Subsystem builders
%% ======================================================================
function buildReferenceSpeed(sys, pos)
add_block('built-in/Subsystem', sys, 'Position', pos);
% The reference is sampled at the controller rate. With a discrete input
% the Rate Limiter starts from its initial condition (0 rpm); with a
% continuous input it would pass a step at t = 0 straight through.
add_block('simulink/Sources/Step', [sys '/Reference Step 1'], 'Position', [40 40 70 70], ...
    'Time', 'refTime1', 'Before', '0', 'After', 'refSpeed1', 'SampleTime', 'Ts_speed');
add_block('simulink/Sources/Step', [sys '/Reference Step 2'], 'Position', [40 110 70 140], ...
    'Time', 'refTime2', 'Before', '0', 'After', 'refSpeed2 - refSpeed1', 'SampleTime', 'Ts_speed');
add_block('simulink/Math Operations/Sum', [sys '/Sum'], 'Position', [120 75 140 105], ...
    'Inputs', '++', 'IconShape', 'round');
add_block('simulink/Discontinuities/Rate Limiter', [sys '/Soft-Start Ramp'], ...
    'Position', [190 70 240 110], 'RisingSlewLimit', 'speedRampRate', ...
    'FallingSlewLimit', '-speedRampRate', 'InitialCondition', '0', 'SampleTimeMode', 'inherited');
add_block('built-in/Outport', [sys '/omega_ref_rpm'], 'Position', [300 83 330 97]);
add_line(sys, 'Reference Step 1/1', 'Sum/1', 'autorouting', 'on');
add_line(sys, 'Reference Step 2/1', 'Sum/2', 'autorouting', 'on');
add_line(sys, 'Sum/1', 'Soft-Start Ramp/1');
addLineNamed(sys, 'Soft-Start Ramp/1', 'omega_ref_rpm/1', 'omega_ref_rpm');
end

function buildSpeedController(sys, pos)
% Discrete PI with output saturation and back-calculation anti-windup:
%   u_unsat = Kp*e + x_I
%   duty    = sat(u_unsat, uMin, uMax)
%   x_I[k+1] = x_I[k] + Ts_speed*( Ki*e + Kaw*(duty - u_unsat) ),  Kaw = Ki/Kp
add_block('built-in/Subsystem', sys, 'Position', pos);
add_block('built-in/Inport', [sys '/omega_ref_rpm'],  'Position', [20 43 50 57]);
add_block('built-in/Inport', [sys '/omega_meas_rpm'], 'Position', [20 103 50 117], 'Port', '2');
add_block('simulink/Math Operations/Gain', [sys '/rpm2rad_ref'],  'Position', [90 35 130 65], 'Gain', 'pi/30');
add_block('simulink/Math Operations/Gain', [sys '/rpm2rad_meas'], 'Position', [90 95 130 125], 'Gain', 'pi/30');
add_block('simulink/Math Operations/Sum', [sys '/Error'], 'Position', [170 65 190 95], ...
    'Inputs', '+-', 'IconShape', 'round');
add_block('simulink/Discrete/Zero-Order Hold', [sys '/Speed Sampling'], ...
    'Position', [220 65 255 95], 'SampleTime', 'Ts_speed');
add_block('simulink/Math Operations/Gain', [sys '/Kp'], 'Position', [310 20 350 50], 'Gain', 'Kp');
add_block('simulink/Math Operations/Gain', [sys '/Ki'], 'Position', [310 110 350 140], 'Gain', 'Ki');
add_block('simulink/Math Operations/Sum', [sys '/Integrator Input'], 'Position', [390 115 410 145], ...
    'Inputs', '++', 'IconShape', 'round');
add_block('simulink/Discrete/Discrete-Time Integrator', [sys '/Integrator'], ...
    'Position', [440 110 490 150], 'IntegratorMethod', 'Integration: Forward Euler', ...
    'gainval', '1.0', 'InitialCondition', '0', 'SampleTime', '-1');
add_block('simulink/Math Operations/Sum', [sys '/PI Sum'], 'Position', [540 55 560 85], ...
    'Inputs', '++', 'IconShape', 'round');
add_block('simulink/Discontinuities/Saturation', [sys '/Duty Limit'], ...
    'Position', [600 55 640 85], 'UpperLimit', 'uMax', 'LowerLimit', 'uMin');
add_block('built-in/Outport', [sys '/duty_cmd'], 'Position', [700 63 730 77]);
add_block('simulink/Math Operations/Sum', [sys '/Saturation Excess'], 'Position', [600 180 620 210], ...
    'Inputs', '+-', 'IconShape', 'round');
add_block('simulink/Math Operations/Gain', [sys '/Kaw'], 'Position', [470 175 520 215], ...
    'Gain', 'antiWindupEnable*Ki/Kp');
set_param([sys '/Kaw'], 'Orientation', 'left');

add_line(sys, 'omega_ref_rpm/1', 'rpm2rad_ref/1');
add_line(sys, 'omega_meas_rpm/1', 'rpm2rad_meas/1');
add_line(sys, 'rpm2rad_ref/1', 'Error/1', 'autorouting', 'on');
add_line(sys, 'rpm2rad_meas/1', 'Error/2', 'autorouting', 'on');
add_line(sys, 'Error/1', 'Speed Sampling/1');
addLineNamed(sys, 'Speed Sampling/1', 'Kp/1', 'speed_error');
logPort(sys_block(sys, 'Speed Sampling'), 1, 1);
add_line(sys, 'Speed Sampling/1', 'Ki/1', 'autorouting', 'on');
add_line(sys, 'Ki/1', 'Integrator Input/1', 'autorouting', 'on');
add_line(sys, 'Integrator Input/1', 'Integrator/1');
addLineNamed(sys, 'Integrator/1', 'PI Sum/2', 'pi_integrator');
logPort(sys_block(sys, 'Integrator'), 1, 1);
addLineNamed(sys, 'Kp/1', 'PI Sum/1', 'p_term');
addLineNamed(sys, 'PI Sum/1', 'Duty Limit/1', 'u_unsat');
addLineNamed(sys, 'Duty Limit/1', 'duty_cmd/1', 'duty_cmd');
add_line(sys, 'Duty Limit/1', 'Saturation Excess/1', 'autorouting', 'on');
add_line(sys, 'PI Sum/1', 'Saturation Excess/2', 'autorouting', 'on');
add_line(sys, 'Saturation Excess/1', 'Kaw/1', 'autorouting', 'on');
addLineNamed(sys, 'Kaw/1', 'Integrator Input/2', 'aw_feedback');
end

function buildCommutationLogic(sys, pos)
add_block('built-in/Subsystem', sys, 'Position', pos);
add_block('built-in/Inport', [sys '/duty_cmd'], 'Position', [20 223 50 237]);
names = {'Hall_A', 'Hall_B', 'Hall_C'};
for k = 1:3
    add_block('built-in/Inport', [sys '/' names{k}], 'Position', [20 23+50*(k-1) 50 37+50*(k-1)], ...
        'Port', num2str(k+1));
end

% --- Hall decoder: phase command c_x in {-1, 0, +1} ---
hd = [sys '/Hall Decoder'];
add_block('built-in/Subsystem', hd, 'Position', [120 20 230 170]);
pairs = {'Hall_A','Hall_B'; 'Hall_B','Hall_C'; 'Hall_C','Hall_A'};
phase = 'ABC';
for k = 1:3
    add_block('built-in/Inport', [hd '/' names{k}], 'Position', [20 40+80*(k-1) 50 54+80*(k-1)]);
end
outNames = {};
for k = 1:3
    y = 30 + 80*(k-1);
    sumBlk = sprintf('c_%s', lower(phase(k)));
    add_block('simulink/Math Operations/Sum', [hd '/' sumBlk], ...
        'Position', [110 y 130 y+30], 'Inputs', '+-', 'IconShape', 'round');
    add_line(hd, [pairs{k,1} '/1'], [sumBlk '/1'], 'autorouting', 'on');
    add_line(hd, [pairs{k,2} '/1'], [sumBlk '/2'], 'autorouting', 'on');
    hi = sprintf('High %s', phase(k));  lo = sprintf('Low %s', phase(k));
    add_block('simulink/Logic and Bit Operations/Compare To Zero', [hd '/' hi], ...
        'Position', [180 y-15 220 y+5], 'relop', '>');
    add_block('simulink/Logic and Bit Operations/Compare To Zero', [hd '/' lo], ...
        'Position', [180 y+25 220 y+45], 'relop', '<');
    lh = add_line(hd, [sumBlk '/1'], [hi '/1'], 'autorouting', 'on');
    set_param(lh, 'Name', sprintf('phase_cmd_%s', lower(phase(k))));
    add_line(hd, [sumBlk '/1'], [lo '/1'], 'autorouting', 'on');
    outNames{end+1} = sprintf('high_%s', phase(k)); %#ok<AGROW>
    outNames{end+1} = sprintf('low_%s', phase(k));  %#ok<AGROW>
    add_block('built-in/Outport', [hd '/' outNames{end-1}], 'Position', [280 y-12 310 y+2]);
    add_block('built-in/Outport', [hd '/' outNames{end}],   'Position', [280 y+28 310 y+42]);
    add_line(hd, [hi '/1'], [outNames{end-1} '/1']);
    add_line(hd, [lo '/1'], [outNames{end} '/1']);
end

% --- PWM generator: triangular carrier, duty > carrier -> pwm = 1 ---
pw = [sys '/PWM Generator'];
add_block('built-in/Subsystem', pw, 'Position', [120 210 230 250]);
add_block('built-in/Inport', [pw '/duty_cmd'], 'Position', [20 33 50 47]);
add_block('simulink/Sources/Repeating Sequence', [pw '/Carrier'], 'Position', [20 90 60 120], ...
    'rep_seq_t', '[0 0.5 1]/fPWM', 'rep_seq_y', '[0 1 0]');
add_block('simulink/Logic and Bit Operations/Relational Operator', [pw '/Compare'], ...
    'Position', [110 45 140 85], 'Operator', '>');
add_block('built-in/Outport', [pw '/pwm'], 'Position', [190 58 220 72]);
add_line(pw, 'duty_cmd/1', 'Compare/1', 'autorouting', 'on');
lh = add_line(pw, 'Carrier/1', 'Compare/2', 'autorouting', 'on');
set_param(lh, 'Name', 'pwm_carrier');
addLineNamed(pw, 'Compare/1', 'pwm/1', 'pwm');

% --- Gate logic with leg interlock ---
%   Gate_xH = high_x AND pwm AND NOT low_x      (PWM on the high-side switch)
%   Gate_xL = low_x AND NOT high_x              (low side on for the whole sector)
gl = [sys '/Gate Logic'];
add_block('built-in/Subsystem', gl, 'Position', [310 40 420 250]);
for k = 1:6
    add_block('built-in/Inport', [gl '/' outNames{k}], 'Position', [20 30+50*(k-1) 50 44+50*(k-1)]);
end
add_block('built-in/Inport', [gl '/pwm'], 'Position', [20 330 50 344], 'Port', '7');
gateNames = {};
for k = 1:3
    hiIn = outNames{2*k-1};  loIn = outNames{2*k};
    y = 20 + 100*(k-1);
    add_block('simulink/Logic and Bit Operations/Logical Operator', [gl sprintf('/NOT low %s', phase(k))], ...
        'Position', [110 y 140 y+20], 'Operator', 'NOT');
    add_block('simulink/Logic and Bit Operations/Logical Operator', [gl sprintf('/NOT high %s', phase(k))], ...
        'Position', [110 y+50 140 y+70], 'Operator', 'NOT');
    add_block('simulink/Logic and Bit Operations/Logical Operator', [gl sprintf('/AND %sH', phase(k))], ...
        'Position', [200 y 230 y+40], 'Operator', 'AND', 'Inputs', '3');
    add_block('simulink/Logic and Bit Operations/Logical Operator', [gl sprintf('/AND %sL', phase(k))], ...
        'Position', [200 y+50 230 y+80], 'Operator', 'AND', 'Inputs', '2');
    add_line(gl, [loIn '/1'], sprintf('NOT low %s/1', phase(k)), 'autorouting', 'on');
    add_line(gl, [hiIn '/1'], sprintf('NOT high %s/1', phase(k)), 'autorouting', 'on');
    add_line(gl, [hiIn '/1'], sprintf('AND %sH/1', phase(k)), 'autorouting', 'on');
    add_line(gl, 'pwm/1', sprintf('AND %sH/2', phase(k)), 'autorouting', 'on');
    add_line(gl, sprintf('NOT low %s/1', phase(k)), sprintf('AND %sH/3', phase(k)), 'autorouting', 'on');
    add_line(gl, [loIn '/1'], sprintf('AND %sL/1', phase(k)), 'autorouting', 'on');
    add_line(gl, sprintf('NOT high %s/1', phase(k)), sprintf('AND %sL/2', phase(k)), 'autorouting', 'on');
    gateNames = [gateNames, {sprintf('Gate_%sH', phase(k)), sprintf('Gate_%sL', phase(k))}]; %#ok<AGROW>
end
add_block('simulink/Signal Routing/Mux', [gl '/Gate Mux'], 'Position', [290 30 295 290], 'Inputs', '6');
andBlocks = {'AND AH', 'AND AL', 'AND BH', 'AND BL', 'AND CH', 'AND CL'};
for k = 1:6
    addLineNamed(gl, [andBlocks{k} '/1'], sprintf('Gate Mux/%d', k), gateNames{k});
end
add_block('built-in/Outport', [gl '/gate_signals'], 'Position', [340 153 370 167]);
addLineNamed(gl, 'Gate Mux/1', 'gate_signals/1', 'gate_signals');
logPort([gl '/Gate Mux'], 1, 2);   % logged at 1 us to resolve 20 kHz PWM

% --- wiring inside Commutation Logic ---
for k = 1:3
    add_line(sys, [names{k} '/1'], sprintf('Hall Decoder/%d', k), 'autorouting', 'on');
end
add_line(sys, 'duty_cmd/1', 'PWM Generator/1', 'autorouting', 'on');
for k = 1:6
    add_line(sys, sprintf('Hall Decoder/%d', k), sprintf('Gate Logic/%d', k), 'autorouting', 'on');
end
add_line(sys, 'PWM Generator/1', 'Gate Logic/7', 'autorouting', 'on');
add_block('simulink/Signal Attributes/Data Type Conversion', [sys '/To Double'], ...
    'Position', [470 130 510 160], 'OutDataTypeStr', 'double');
add_block('built-in/Outport', [sys '/gate_cmd'], 'Position', [560 138 590 152]);
add_line(sys, 'Gate Logic/1', 'To Double/1', 'autorouting', 'on');
addLineNamed(sys, 'To Double/1', 'gate_cmd/1', 'gate_cmd');
end

function buildInverter(sys, lib, pos)
% Ideal DC link + 3-leg MOSFET bridge. Gate order: [AH AL BH BL CH CL].
add_block('built-in/Subsystem', sys, 'Position', pos);
add_block('built-in/Inport', [sys '/gate_cmd'], 'Position', [220 20 250 34]);
add_block('simulink/Sources/Constant', [sys '/Vdc'], 'Position', [20 110 60 130], 'Value', 'Vdc');
src = [sys '/DC Link'];
add_block(lib.cvs, src, 'Position', [100 100 140 160]);
% The source voltage is the Simulink input (Vdc); the dialog values only
% set the initial state of the power circuit.
setChecked(src, 'Initialize', 'on');  setChecked(src, 'Source_Type', 'DC');
setChecked(src, 'Amplitude', 'Vdc');  setChecked(src, 'Measurements', 'None');
br = [sys '/Universal Bridge'];
add_block(lib.bridge, br, 'Position', [260 80 340 180]);
setChecked(br, 'converterType', 'Rectifier');   % port layout: A,B,C left; +,- right
setChecked(br, 'Arms', '3');
setChecked(br, 'Device', 'MOSFET / Diodes');
setChecked(br, 'Ron', 'Ron');
setChecked(br, 'SnubberResistance', 'Rsnubber');
setChecked(br, 'SnubberCapacitance', 'Csnubber');
setChecked(br, 'Measurements', 'None');
vm = [sys '/Vdc Measurement'];
add_block(lib.vm, vm, 'Position', [180 220 220 260]);
add_block('built-in/Outport', [sys '/v_dc'], 'Position', [290 233 320 247]);

add_line(sys, 'Vdc/1', 'DC Link/1');
add_line(sys, 'gate_cmd/1', 'Universal Bridge/1', 'autorouting', 'on');
add_line(sys, 'Vdc Measurement/1', 'v_dc/1');
% Universal Bridge (rectifier orientation): LConn1..3 = A,B,C ; RConn1 = +, RConn2 = -
% Controlled Voltage Source: RConn1 = positive terminal, LConn1 = negative terminal
% Voltage Measurement: LConn1 = +, LConn2 = -
pb = get_param(br, 'PortHandles');
ps = get_param(src, 'PortHandles');
pv = get_param(vm, 'PortHandles');
add_line(sys, ps.RConn(1), pb.RConn(1), 'autorouting', 'on');
add_line(sys, ps.LConn(1), pb.RConn(2), 'autorouting', 'on');
add_line(sys, pv.LConn(1), pb.RConn(1), 'autorouting', 'on');
add_line(sys, pv.LConn(2), pb.RConn(2), 'autorouting', 'on');
phase = 'ABC';
for k = 1:3
    cp = [sys '/' phase(k)];
    add_block('built-in/PMIOPort', cp, 'Position', [420 60+40*k 450 74+40*k], 'Side', 'Right');
    pc = get_param(cp, 'PortHandles');
    add_line(sys, pb.LConn(k), [pc.LConn pc.RConn], 'autorouting', 'on');
end
end

function buildMechanicalLoad(sys, pos)
add_block('built-in/Subsystem', sys, 'Position', pos);
add_block('simulink/Sources/Constant', [sys '/Base Load'], 'Position', [20 20 70 40], 'Value', 'TloadInitial');
add_block('simulink/Sources/Step', [sys '/Load Step'], 'Position', [30 70 60 100], ...
    'Time', 'TloadStepTime', 'Before', '0', 'After', 'TloadStep');
add_block('simulink/Math Operations/Sum', [sys '/Sum'], 'Position', [110 45 130 75], ...
    'Inputs', '++', 'IconShape', 'round');
add_block('built-in/Outport', [sys '/T_load'], 'Position', [170 53 200 67]);
add_line(sys, 'Base Load/1', 'Sum/1', 'autorouting', 'on');
add_line(sys, 'Load Step/1', 'Sum/2', 'autorouting', 'on');
addLineNamed(sys, 'Sum/1', 'T_load/1', 'T_load');
end

function configureMotor(blk)
% Trapezoidal-flux PMSM = BLDC machine. Order matters: mask options first.
setChecked(blk, 'NbPhases', '3');
setChecked(blk, 'FluxDistribution', 'Trapezoidal');
setChecked(blk, 'MechanicalLoad', 'Torque Tm');
setChecked(blk, 'PresetModel', 'No');
setChecked(blk, 'MeasurementBus', 'off');   % keep the descriptive bus labels
setChecked(blk, 'Resistance', 'Rs');
setChecked(blk, 'Inductance', 'Ls');
setChecked(blk, 'MachineConstant', 'Flux linkage established by magnets (V.s)');
setChecked(blk, 'Flux', 'fluxPM');
setChecked(blk, 'Flat', 'flatTopDeg');
% [inertia, viscous friction, pole pairs, static friction]
setChecked(blk, 'Mechanical', '[J, B, polePairs, 0]');
setChecked(blk, 'PolePairs', 'polePairs');
% [initial speed (rad/s), electrical angle (deg), ia, ib]
setChecked(blk, 'InitialConditions', '[0, 0, 0, 0]');
end

function buildMeasurements(sys, pos)
add_block('built-in/Subsystem', sys, 'Position', pos);
add_block('built-in/Inport', [sys '/motor_bus'], 'Position', [20 163 50 177]);
sigs = {'Rotor speed wm (rad/s)', 'Hall effect signal h_a', 'Hall effect signal h_b', ...
        'Hall effect signal h_c', 'Stator current is_a (A)', 'Stator current is_b (A)', ...
        'Stator current is_c (A)', 'Electromagnetic torque Te (N*m)'};
outs = {'omega_meas_rpm', 'Hall_A', 'Hall_B', 'Hall_C', 'i_a', 'i_b', 'i_c', 'T_e'};
bs = [sys '/Bus Selector'];
add_block('simulink/Signal Routing/Bus Selector', bs, 'Position', [100 20 105 320]);
set_param(bs, 'OutputSignals', strjoin(sigs, ','));
add_line(sys, 'motor_bus/1', 'Bus Selector/1');
add_block('simulink/Math Operations/Gain', [sys '/rad2rpm'], 'Position', [160 25 200 55], 'Gain', '30/pi');
% (lines leaving a Bus Selector keep the bus element names and cannot be
% relabelled; the logged names are set on the top-level lines instead)
add_line(sys, 'Bus Selector/1', 'rad2rpm/1', 'autorouting', 'on');
for k = 1:numel(outs)
    add_block('built-in/Outport', [sys '/' outs{k}], 'Position', [260 33+37*(k-1) 290 47+37*(k-1)]);
end
addLineNamed(sys, 'rad2rpm/1', 'omega_meas_rpm/1', 'omega_meas_rpm');
for k = 2:numel(outs)
    add_line(sys, sprintf('Bus Selector/%d', k), [outs{k} '/1'], 'autorouting', 'on');
end
end

function buildMonitoring(sys, pos)
add_block('built-in/Subsystem', sys, 'Position', pos);
ins = {'omega_ref_rpm', 'omega_meas_rpm', 'T_e', 'T_load', 'i_a', 'i_b', 'i_c', 'duty_cmd', 'v_dc'};
for k = 1:numel(ins)
    add_block('built-in/Inport', [sys '/' ins{k}], 'Position', [20 23+40*(k-1) 50 37+40*(k-1)]);
end
scopes = {'Speed Scope', [1 2]; 'Torque Scope', [3 4]; 'Current Scope', [5 6 7]; ...
          'Duty Scope', 8; 'DC Link Scope', 9};
for s = 1:size(scopes, 1)
    idx = scopes{s, 2};
    y = 20 + 75*(s-1);
    add_block('simulink/Sinks/Scope', [sys '/' scopes{s,1}], ...
        'Position', [200 y 230 y+40], 'NumInputPorts', '1');
    if numel(idx) > 1
        mux = sprintf('%s/Mux%d', sys, s);
        add_block('simulink/Signal Routing/Mux', mux, 'Position', [140 y 145 y+40], ...
            'Inputs', num2str(numel(idx)));
        for k = 1:numel(idx)
            add_line(sys, sprintf('%s/1', ins{idx(k)}), sprintf('Mux%d/%d', s, k), 'autorouting', 'on');
        end
        add_line(sys, sprintf('Mux%d/1', s), [scopes{s,1} '/1']);
    else
        add_line(sys, sprintf('%s/1', ins{idx}), [scopes{s,1} '/1'], 'autorouting', 'on');
    end
end
end

%% ======================================================================
%% Helpers
%% ======================================================================
function lib = findPowerLibrary()
% Specialized Power Systems blocks (library paths verified in R2025a).
if ~license('test', 'power_system_blocks')
    error('Simscape Electrical (Specialized Power Systems) is required.');
end
load_system('sps_lib');
lib.machine  = sprintf('sps_lib/Electrical Machines/Permanent Magnet\nSynchronous Machine');
lib.bridge   = 'sps_lib/Power Electronics/Universal Bridge';
lib.cvs      = 'sps_lib/Sources/Controlled Voltage Source';
lib.vm       = 'sps_lib/Sensors and Measurements/Voltage Measurement';
lib.powergui = 'sps_lib/powergui';
fields = fieldnames(lib);
for f = 1:numel(fields)
    if getSimulinkBlockHandle(lib.(fields{f})) == -1
        error('Library block not found: %s', strrep(lib.(fields{f}), newline, ' '));
    end
end
end

function lh = addLineNamed(sys, src, dst, name)
lh = add_line(sys, src, dst, 'autorouting', 'on');
set_param(lh, 'Name', name);
end

function logPort(blockPath, portIdx, decimation)
ph = get_param(blockPath, 'PortHandles');
p = ph.Outport(portIdx);
set_param(p, 'DataLogging', 'on', 'DataLoggingNameMode', 'SignalName');
if decimation > 1
    set_param(p, 'DataLoggingDecimateData', 'on', ...
        'DataLoggingDecimation', num2str(decimation));
end
end

function p = sys_block(sys, name)
p = [sys '/' name];
end

function setChecked(obj, param, value)
% Set a parameter and read it back; any failure stops the build, so the
% model never silently keeps a library default for an engineering value.
set_param(obj, param, value);
actual = get_param(obj, param);
if ~strcmp(strtrim(actual), strtrim(value))
    error('build_bldc_model:paramMismatch', ...
        'Parameter %s of %s is "%s" after setting it to "%s".', ...
        param, strrep(obj, newline, ' '), actual, value);
end
end

function addAnnotation(sys, xy, text)
try
    a = Simulink.Annotation([sys '/Model Notes']);
    a.Text = text;
    a.Position = [xy(1) xy(2) xy(1)+520 xy(2)+40];
catch
    % annotations are cosmetic; ignore on releases without this API
end
end
