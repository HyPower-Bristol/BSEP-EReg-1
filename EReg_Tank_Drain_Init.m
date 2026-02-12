%-------------------------------------------------------
%-------------------------------------------------------
%------------- User Configurables ----------------------
%-------------------------------------------------------
%-------------------------------------------------------

% --- Controller Parameters ---
K_P = 4;           % Proportional Constant
K_I = 2;           % Integral Constant
K_D = 12;            % Derivative Constant
N = 50;            % Filter Coefficient (Derivative)
Feedforward = 8;    % Reg Valve Feed Forward step in degrees
Servo_Speed = 180;  % Forward and Reverse Speed of the Servo [deg/s]

% --- Initial Conditions (User Inputs) ---
T_0_N2 = 300;       % Initial temperature of nitrogen [K]
% Calculated: P = (m*R*T)/V = (2.5 * 296 * 300) / 0.0068 / 1e5 = 326.47 bar
P_1_0 = 326.47;     % Initial pressure of nitrogen in the HP tank [bar] (Based on 2.5kg N2)
P_2_0 = 50;         % Initial pressure of nitrogen ullage in prop tank [bar]
P_3_0 = 50;         % Initial injector upstream pressure [bar]
P_4_0 = 50;         % Initial pressure of nitrogen ullage in prop tank [bar]
m_dot_N2_0 = 0;     % Initial N2 mass flow rate [kg/s]
m_dot_L_0 = 0;      % Initial Propellant mass flow rate [kg/s]

% --- Mass Adjustment for 0.00496 m^3 Tank ---
% With rho = 789, 3.3kg of Ethanol is ~4.18L, leaving ~0.78L (15%) ullage.
m_3_0 = 3.3;        % Initial mass of Ethanol propellant [kg] 

Kv_2 = 1;           % Set water valve opening Kv for step change

% --- Hardware Constants (User Config) ---
V_1 = 6.8;          % HP Tank volume [L] (Nitrogen Tank)
V_2 = 4.96;         % Prop tank volume [L] (Converted from 0.00496 m^3)
A_3 = 13.27e-6;     % Injector Orifice Area [m^2] (Converted from 13.27 mm^2)
Cd_3 = 0.543;      % Injector Orifice Discharge Coefficient
K_v_4 = 0.5;        % Flow coefficient of the check valve
Kv_1_max = 1;       % Reg Valve Maximum Kv

% --- Simulation Timing & Setpoints ---
Sim_Duration = 10;   % Simulation Duration [s]
Time_Step = 0.001;  % Time Step [s]
Num_Steps = round(Sim_Duration / Time_Step);
end_time = Num_Steps; 
time = linspace(0, Sim_Duration, Num_Steps);
Target_Pressure = 50; % Main Target Regulated Pressure [bar]

% Initialize Setpoint Arrays
setpoint = zeros(1, Num_Steps);
servo_off_setpoint = zeros(1, Num_Steps);
run_valve_setpoint = zeros(1, Num_Steps);

% Phase 1: Initialize (0-1s)
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
setpoint(3000) = Target_Pressure;
% Phase 6: Ramp Down (3s-4s)
for idx=3001:4000
    setpoint(idx)=Target_Pressure;
    servo_off_setpoint(idx)=1;
end
% Phase 7: Flat (4s to End)
if Num_Steps > 4001
    for idx=4001:Num_Steps
        setpoint(idx)=Target_Pressure; 
        servo_off_setpoint(idx)=1;
    end
end

throttle_setpoint = servo_off_setpoint; 

%-------------------------------------------------------
%------------- System Constants (Fixed) ----------------
%-------------------------------------------------------

% --- Physical Constants ---
rho_L = 789;        % Ethanol density [kg/m^3]
R_N2 = 296;         % Gas constant for nitrogen [J/kgK]
gamma_N2 = 1.4;     % Specific Heat Ratio for nitrogen
P_atm = 0;          % Atmospheric pressure (gauge) [Pa]
Tst = 273.15;       % Standard Temperature [K]
Pst = 101325;       % Standard Pressure [Pa]
rhost = Pst/(R_N2*Tst); 

%-------------------------------------------------------
%---------- Calculations & Conversions -----------------
%-------------------------------------------------------

% --- Unit Conversions ---
V_1_m3 = V_1*1e-3;     
V_2_m3 = V_2*1e-3;     
P_1_0_Pa = P_1_0*1e5;  
P_2_0_Pa = P_2_0*1e5;  
P_4_0_Pa = P_4_0*1e5;  

% --- Derived Initial Conditions ---
rho_1_0 = (P_1_0_Pa)/(R_N2*T_0_N2); 
rho_2_0 = (P_2_0_Pa)/(R_N2*T_0_N2); 

% Map back to original variable names for Simulink
V_1 = V_1_m3;
V_2 = V_2_m3;
P_1_0 = P_1_0_Pa;
P_2_0 = P_2_0_Pa;
P_4_0 = P_4_0_Pa;

m_2_0 = rho_2_0*(V_2-(m_3_0/rho_L)); % Get ullage gas initial mass
m_1_0 = V_1*rho_1_0;                 % Should equal ~2.5 kg

%-------------------------------------------------------
%------------- Run Simulink Models ---------------------
%-------------------------------------------------------
results = sim("EReg_Tank_Drain.slx", "StopTime", num2str(Sim_Duration));