# E-Reg Controller Model (v2.0)

This repository implements a feed system tank drain model with an Electronic Regulator (E-reg) controller, using an actuated ball valve. It includes both MATLAB/Simulink models and Python analysis scripts.

> **Compatibility Note:** This project is built using **MATLAB 2025b**. Users on **MATLAB 2025a** must use the `EReg_Tank_Drain_2025a.slx` file instead of the main model.

## Key Features
- **Parametrized Setup**: Easily configure Tank Sizes, Simulation Duration, and Target Pressure.
- **Dynamic Simulation**: Simulation loop automatically adjusts to the defined duration.
- **Advanced Plotting**: Automated generation of Pressure and Volume/Mass traces.

## Contents

### MATLAB / Simulink
- **`EReg_Tank_Drain.slx`**: The main Simulink model for the tank drain process and E-reg controller.
- **`EReg_Tank_Drain_Init.m`**: **Master Initialization Script**.
    - Sets all Physical Constants and Controller Gains.
    - **User Configurables**: Set `Sim_Duration`, `Target_Pressure`, and `V_1`/`V_2` (Tank Sizes) here.
    - Runs the Simulink model automatically.
- **`EregPlot.m`**: **Visualization Script**.
    - Generates and saves plots from the simulation results.
    - Output 1: `pressure_plot.png` (N2 Pressure, Fuel Pressure, Desired Pressure).
    - Output 2: `tank_levels.png` (Fuel Volume in Liters, N2 Mass in kg).
    - Output 3: `servo_demand.png` (Servo Demand vs Valve Angle).

### Python
- **`N2O_characterisation_executable.py`**: Script for N2O tank blowdown characterisation.
- **`combustion-model-test.py`**: Testing script for the combustion model.


## Quick Start

### Running the Simulation
1. Open MATLAB in the `E-reg` folder.
2. Open `EReg_Tank_Drain_Init.m`.
    - Adjust `Sim_Duration` (default 6s) or `Target_Pressure` (default 50 bar) at the top of the file if needed.
3. Run the Init script:
   ```matlab
   EReg_Tank_Drain_Init
   ```
   *This automatically simulates the model.*
4. Run the Plotting script:
   ```matlab
   EregPlot
   ```
   *This generates `pressure_plot.png` and `tank_levels.png`.*

## Configuration Guide

All primary user settings are located at the top of **`EReg_Tank_Drain_Init.m`**:

```matlab
% --- User Configurables ---
Kv_2 = 1;           % Water valve opening
Sim_Duration = 6;   % Simulation Duration [s] (e.g., set to 10 for longer run)
Target_Pressure = 50; % Target Regulated Pressure [bar]

% --- Hardware Constants ---
V_1 = 10;           % HP Tank volume [L] (N2 Tank)
V_2 = 13;           % Prop tank volume [L] (Fuel Tank)
```
## Commit Types

| Type | Description | Example |
| :--- | :--- | :--- |
| **feat** | A new feature for the user | `feat: add email notification system` |
| **fix** | A bug fix | `fix: correct typo in landing page` |
| **chore** | Regular maintenance/tooling (no code change) | `chore: update npm packages` |
| **docs** | Documentation changes only | `docs: update README with API keys` |
| **style** | Formatting, missing semi-colons, etc. (no logic change) | `style: run prettier on auth folder` |
| **refactor** | Code change that neither fixes a bug nor adds a feature | `refactor: clean up redundant if-statements` |
| **perf** | A code change that improves performance | `perf: optimize database query for users` |
| **test** | Adding missing tests or correcting existing tests | `test: add unit tests for login controller` |
| **build** | Changes that affect the build system (e.g., Gulp, Webpack) | `build: change output directory to /dist` |
| **ci** | Changes to CI configuration files/scripts | `ci: update github actions to v4` |
| **revert** | Reverts a previous commit | `revert: feat: add email notification system` |
