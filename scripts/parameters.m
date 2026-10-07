%% parameters.m
% Nominal parameters for the closed-loop BLDC drive (BLDC_ClosedLoop.slx).
% All quantities are in SI units unless the variable name says otherwise
% (e.g. *_rpm). Scenario-specific values are overridden in run_simulations.m.

%% Simulation
Ts       = 5e-7;      % [s]  fixed step of the discrete power circuit (powergui)
Ts_speed = 1e-4;      % [s]  sample time of the digital speed controller (10 kHz)
Tsim     = 0.5;       % [s]  default stop time

%% DC link and inverter
Vdc  = 160;           % [V]  DC-link voltage (ideal source)
fPWM = 20e3;          % [Hz] PWM carrier frequency
Ron  = 1e-3;          % [Ohm] MOSFET on-state resistance
Rsnubber = 1e5;       % [Ohm] resistive snubber across each switch
Csnubber = inf;       % [F]  inf -> purely resistive snubber

%% BLDC motor (trapezoidal-flux PMSM block, 120 deg flat back-EMF)
Rs         = 0.7;         % [Ohm]   stator phase resistance
Ls         = 2.72e-3;     % [H]     stator phase inductance
fluxPM     = 0.105;       % [V*s]   flux linkage established by the magnets
polePairs  = 4;           % [-]
flatTopDeg = 120;         % [deg]   width of the flat part of the back-EMF
J          = 0.8e-3;      % [kg*m^2] rotor inertia
B          = 1e-3;        % [N*m*s] viscous friction coefficient

% Derived constants (two phases conducting, used for documentation and design)
Kt = 2*polePairs*fluxPM;  % [N*m/A]     torque constant          = 0.84
Ke = 2*polePairs*fluxPM;  % [V/(rad/s)] line-line back-EMF const. = 0.84

%% Speed reference
speedRefRPM   = 1500;     % [rpm]   nominal reference
speedRampRate = 30000;    % [rpm/s] soft-start slew limit (0 -> 1500 rpm in 50 ms)
% Two-step reference profile: refSpeed1 at refTime1, refSpeed2 at refTime2
refTime1  = 0;     refSpeed1 = speedRefRPM;   % [s], [rpm]
refTime2  = 1e3;   refSpeed2 = speedRefRPM;   % 1e3 s = "never" in the default run

%% Speed controller (discrete PI, output = normalized duty cycle)
Kp = 0.002;               % [1/(rad/s)]     proportional gain
Ki = 0.5;                 % [1/(rad/s*s)]   integral gain
uMin = 0;                 % [-] duty lower limit (the inverter cannot brake)
uMax = 1;                 % [-] duty upper limit
antiWindupEnable = 1;     % 1 = back-calculation active, 0 = disabled
% The back-calculation gain used in the model is antiWindupEnable*Ki/Kp,
% i.e. tracking time constant Tt = Kp/Ki = 4 ms.

%% Mechanical load
TloadInitial  = 0.2;      % [N*m] light base load, present from t = 0
TloadStep     = 1.0;      % [N*m] additional load applied as a step
TloadStepTime = 1e3;      % [s]   "never" in the default run
