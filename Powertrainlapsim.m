function output = Powertrainlapsim(initialV)
% For the love of the FSM, keep everything in metric
% initialV is in m/s **********************************
global powertrainpackage tire_radius
% Input RPM and torque [N-m] curve from dyno. Got rid of xlsread for run time reasons
engineSpeed = powertrainpackage{1}; %[6200:100:14100]; 
engineTq = powertrainpackage{2}; %[41.57 42.98 44.43 45.65 46.44 47.09 47.52 48.58 49.57 50.41 51.43 51.48 51 49.311 48.94 48.66 49.62 49.60 47.89 47.91 48.09 48.57 49.07 49.31 49.58 49.56 49.84 50.10 50.00 50.00 50.75 51.25 52.01 52.44 52.59 52.73 53.34 53.72 52.11 52.25 51.66 50.5 50.34 50.50 50.50 50.55 50.63 50.17 50.80 49.73 49.35 49.11 48.65 48.28 48.28 47.99 47.68 47.43 47.07 46.67 45.49 45.37 44.67 43.8 43.0 42.3 42.00 41.96 41.70 40.43 39.83 38.60 38.46 37.56 36.34 35.35 33.75 33.54 32.63 31.63];

tyreRadius = tire_radius/3.28;
%tyreRadius = 0.45974/2;     % metres

% Gear Ratios
primaryReduction = powertrainpackage{3}; %76/36;
gear = powertrainpackage{4}; %[33/12, 32/16, 30/18, 26/18, 30/23, 29/24];
finalDrive = powertrainpackage{5}; %40/12;

% Variable declaration
shiftpoint = powertrainpackage{6}; % 14000; % optimal shiftpoint for most gears [RPM] (11k for lift/coast, 14 for max)
drivetrainLosses = powertrainpackage{7}; %.85;     % 15% drivetrain losses
gearSelect = 0;
rpm = 15000;

while rpm > shiftpoint
    gearSelect = gearSelect + 1;
    gearTot = gear(gearSelect)*finalDrive*primaryReduction;     % Total gear reduction for first gear
    rpm = initialV*gearTot/tyreRadius*60/(2*pi);        % calculate engine speed based on new wheel speed [RPM]
end

i = 2;

while rpm > engineSpeed(i)
    i = i + 1;
end

torque = engineTq(i-1)+(engineTq(i)-engineTq(i-1))/(engineSpeed(i)-engineSpeed(i-1))*(rpm-engineSpeed(i-1));        % calculate torque (N-m) at crank based on linear interpolation of dyno data

torque = torque*gearTot*drivetrainLosses;   % torque output at the wheels [N-m]
Fx = torque/tyreRadius;      % force on contact patch from drivetrain [N]

output = [Fx, gearSelect]; % where Fx has the units of Newtons and Shift_indicator is unitless