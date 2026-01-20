%-------------------------------------------------------
%-------------------------------------------------------
%------------- User Configurables ----------------------
%-------------------------------------------------------
%-------------------------------------------------------

% --- Controller Parameters ---
K_P = 16;           % Proportional Constant
K_I = 17;           % Integral Constant
K_D = 8;            % Derivative Constant
N = 100;            % Filter Coefficient (Derivative)
Feedforward = 0;    % Reg Valve Feed Forward step in degrees
Servo_Speed = 180;  % Forward and Reverse Speed of the Servo [deg/s]

% --- Initial Conditions (User Inputs) ---
T_0_N2 = 300;       % Initial temperature of nitrogen in the HP tank and ullage [K]
P_1_0 = 300;        % Initial pressure of nitrogen in the HP tank [bar] (NOS)
P_2_0 = 50;         % Initial pressure of nitrogen ullage in prop tank [bar] (Fuel/Ullage)
P_3_0 = 50;         % Initial injector upstream pressure [bar]
P_4_0 = 50;         % Initial pressure of nitrogen ullage in prop tank [bar]
m_dot_N2_0 = 0;     % Initial N2 mass flow rate [kg/s]
m_dot_L_0 = 0;      % Initial Propellant mass flow rate [kg/s]
m_3_0 = 20;          % Initial mass of liquid propellant [kg]
Kv_2 = 1;           % Set water valve opening Kv for step change

% --- Hardware Constants (User Config) ---
V_1 = 6.8;           % HP Tank volume [L] (N2 Tank)
V_2 = 25;           % Prop tank volume [L] (Fuel Tank)
A_3 = 60*1e-6;      % Injector Orifice Area [m^2]
Cd_3 = 0.7;         % Injector Orifice Discharge Coefficient
K_v_4 = 0.5;        % Flow coefficient of the check valve
Kv_1_max = 1;       % Reg Valve Maximum Kv

% --- Simulation Timing & Setpoints ---
Sim_Duration = 10;   % Simulation Duration [s]
Time_Step = 0.001;  % Time Step [s]
Num_Steps = round(Sim_Duration / Time_Step);
end_time = Num_Steps; % Backwards compatibility (Index Count)
time = linspace(0, Sim_Duration, Num_Steps);

Target_Pressure = 50; % Main Target Regulated Pressure [bar]

% Initialize Setpoint Arrays
setpoint = zeros(1, Num_Steps);
servo_off_setpoint = zeros(1, Num_Steps);
run_valve_setpoint = zeros(1, Num_Steps);

% Define Setpoint Profile
% Phase 1: Initialize (0-1s) (Indices 1 to 1000)
for idx=1:1000
    setpoint(idx)=P_2_0*1e-5;
    servo_off_setpoint(idx)=0;
    run_valve_setpoint(idx)=0;
end
% Phase 2: Open Run Valve (1s+)
for idx = 1001:Num_Steps
    run_valve_setpoint(idx)=1;
end
% Phase 3: Servo Active / Setpoint Hold (1s-2s)
for idx=1000:2000
    setpoint(idx)=Target_Pressure;
    servo_off_setpoint(idx)=1;
end
% Phase 4: Extended Servo Active (2s-2.5s)
for idx=2000:2500
    setpoint(idx)=Target_Pressure;
    servo_off_setpoint(idx)=1;
end
% Phase 5: Run Valve Active Check (2s-3s)
for idx=2000:3000
    setpoint(idx)=Target_Pressure;
    servo_off_setpoint(idx)=1;
    run_valve_setpoint(idx)=1;
end
% Point Fix
setpoint(3000) = Target_Pressure;

% Phase 6: Ramp Down (3s-4s)
for idx=3001:4000
    setpoint(idx)=Target_Pressure;
    servo_off_setpoint(idx)=1;
end

% Point Fix - Ensure smooth transition or set specific value
%if Num_Steps >= 4001
%setpoint(4001) = 50;
%end

% Phase 7: Flat (4s to End)
if Num_Steps > 4001
    for idx=4001:Num_Steps
        setpoint(idx)=Target_Pressure; % Hold flat
        servo_off_setpoint(idx)=1;
    end
end

% Throttle Setpoint Logic
throttle_setpoint = servo_off_setpoint; % Default assignment
%for idx=1720:2000
%    servo_off_setpoint(idx)=1;
%end
%for idx = 2800:Num_Steps
%    servo_off_setpoint(idx)=1;
%end

% Custom Throttle Profile (2.3s - 3.7s)
% Ensure indices are within bounds
%throttle_max_idx = min(3700, Num_Steps);

%if Num_Steps >= 2699
%    diff_val = throttle_setpoint(2699);
%end

%for idx = 2300:min(3200, Num_Steps)
%    throttle_setpoint(idx) = throttle_setpoint(idx-1)-(diff_val/500);
%end
%for idx = 3201:throttle_max_idx
%    throttle_setpoint(idx) = throttle_setpoint(idx-1)+(1/500);
%end

% Final Cleanup for extended times
%if Num_Steps > 6000
% Any specific logic for >6s if needed, though Phase 7 covers setpoint
% Just ensuring consistency if logic required it
%end


%-------------------------------------------------------
%-------------------------------------------------------
%------------- System Constants (Fixed) ----------------
%-------------------------------------------------------
%-------------------------------------------------------

% --- Physical Constants ---
rho_L = 1000;       % Liquid propellant density [kg/m^3]
R_N2 = 296;         % Gas constant for nitrogen [J/kgK]
gamma_N2 = 1.4;     % Specific Heat Ratio for nitrogen
P_atm = 0;          % Atmospheric pressure (gauge) [Pa]
Tst = 273.15;       % Standard Temperature [K]
Pst = 101325;       % Standard Pressure [Pa]
rhost = Pst/(R_N2*Tst); % Standard Nitrogen Density [kg/m^3]



%-------------------------------------------------------
%-------------------------------------------------------
%---------- Calculations & Conversions -----------------
%-------------------------------------------------------
%-------------------------------------------------------

% --- Unit Conversions ---
V_1 = V_1*1e-3;     % convert volumes to m^3
V_2 = V_2*1e-3;     % convert volumes to m^3
P_1_0 = P_1_0*1e5;  % convert pressure to Pa
P_2_0 = P_2_0*1e5;  % convert pressure to Pa
P_4_0 = P_4_0*1e5;  % convert pressure to Pa

% --- Derived Initial Conditions ---
rho_1_0 = (P_1_0)/(R_N2*T_0_N2); % Initial density of nitrogen in the HP tank [kg/m^3]
rho_2_0 = (P_2_0)/(R_N2*T_0_N2); % Initial density of nitrogen in the prop tank [kg/m^3]
rho_4_0 = (P_4_0)/(R_N2*T_0_N2); % Initial density of nitrogen in regulator downstream [kg/m^3]

m_2_0 = rho_2_0*(V_2-(m_3_0/rho_L)); % Get ullage gas initial mass
m_1_0 = V_1*rho_1_0;                 % Get high pressure N2 initial mass


%-------------------------------------------------------
%------------- Run Simulink Models ---------------------
%-------------------------------------------------------
results = sim("EReg_Tank_Drain.slx", "StopTime", num2str(Sim_Duration));
