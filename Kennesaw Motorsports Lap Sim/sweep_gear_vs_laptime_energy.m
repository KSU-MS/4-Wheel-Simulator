%% sweep_motor_ratio_vs_laptime_energy.m
% Sweep MOTOR.n_gear (overall ratio) at a fixed software power.
% Plots: lap time, energy per lap, and lap-average powertrain efficiency.

clear; clc; close all;

%% ---- Inputs ----
track_file     = '../Data/accel.csv';          % or 'track_enduro_24.csv'
vehicle_file   = '../Data/KS9E.csv';
motor_file     = '../Data/emrax_208_mv.csv';     % loader will produce a 'motor' struct with .n_gear

fixed_power_kW = 50;                   % fixed software limit
n_laps_event   = 22;                   % just for optional event kWh print

% Sweep range (edit to your car)
ratio_min = 3.8; ratio_max = 4.6; n_ratios = 20;
ratio_values = linspace(ratio_min, ratio_max, n_ratios);

%% ---- Load once ----
veh0   = load_vehicle(vehicle_file);
motor0 = load_motor(motor_file);
track  = load_track(track_file);

% hold software power fixed at the vehicle level
veh0.software_power_limit_W = fixed_power_kW*1e3;

% sanity: ensure the field exists
if ~isfield(motor0, 'n_gear')
    error('motor.n_gear not found. Check your motor loader/CSV field names.');
end
fprintf('Baseline motor.n_gear = %.3f\n', motor0.n_gear);

% Preallocate
LapTime_s = nan(size(ratio_values));
Energy_Wh = nan(size(ratio_values));
Eff_ratio = nan(size(ratio_values));    % mech work / battery energy (0..1)

fprintf('Sweeping motor.n_gear from %.2f to %.2f at %.0f kW fixed...\n', ...
        ratio_min, ratio_max, fixed_power_kW);

%% ---- Sweep ----
for k = 1:numel(ratio_values)
    veh   = veh0;
    motor = motor0;

    % overwrite the motor ratio HERE
    motor.n_gear = ratio_values(k);

    % run one lap
    R = simulate_lap(track, veh, motor, false);

    LapTime_s(k) = R.lap_time_s;
    Energy_Wh(k) = R.energy_used_Wh;

    % mechanical work at wheels (preferred integration of P_mech)
    E_mech_Wh = get_mech_energy_Wh_from_R(R);
    % battery energy (prefer integration of P_batt; fallback to aggregate)
    E_batt_Wh = get_batt_energy_Wh_from_R(R, Energy_Wh(k));

    Eff_ratio(k) = E_mech_Wh / max(E_batt_Wh, 1e-9);

    fprintf('  n_gear=%5.2f -> lap=%7.3f s | E_lap=%7.2f Wh | eff=%5.3f\n', ...
            motor.n_gear, LapTime_s(k), E_batt_Wh, Eff_ratio(k));
end

E_event_kWh = (Energy_Wh * n_laps_event)/1000;

%% ---- Plots ----
figure('Color','w','Position',[80 80 1120 440]);
tiledlayout(1,2,'Padding','compact','TileSpacing','compact');

% Left: Lap time + Energy (dual y-axis)
nexttile;
yyaxis left;  plot(ratio_values, LapTime_s, '-o','LineWidth',1.6,'MarkerSize',5); grid on;
ylabel('Lap Time [s]');
yyaxis right; plot(ratio_values, Energy_Wh/1000, 's-','LineWidth',1.6,'MarkerSize',5);
ylabel('Energy per Lap [kWh]');
xlabel('motor.n\_gear (overall ratio)');
title(sprintf('Fixed Power Limit = %.0f kW', fixed_power_kW));
legend('Lap Time','Energy per Lap','Location','best');

% Right: Lap-average powertrain efficiency
nexttile;
plot(ratio_values, Eff_ratio, 'd-','LineWidth',1.6,'MarkerSize',5); grid on;
xlabel('motor.n\_gear (overall ratio)'); ylabel('Powertrain Efficiency over Lap [-]');
ylim([0,1]); title('Mechanical Work / Battery Energy');

[~,idx_t] = min(LapTime_s); [~,idx_e] = min(Energy_Wh);
sgtitle(sprintf('Best Lap: n\\_gear=%.2f (%.3f s)   |   Best Energy: n\\_gear=%.2f (%.2f Wh/lap)', ...
        ratio_values(idx_t), LapTime_s(idx_t), ratio_values(idx_e), Energy_Wh(idx_e)));

%% ---- Summary ----
fprintf('\nSummary @ %.0f kW:\n', fixed_power_kW);
fprintf('  Best lap:    n_gear=%.2f -> %.3f s (E=%.2f Wh, eff=%.3f)\n', ...
        ratio_values(idx_t), LapTime_s(idx_t), Energy_Wh(idx_t), Eff_ratio(idx_t));
fprintf('  Best energy: n_gear=%.2f -> %.2f Wh/lap (t=%.3f s, eff=%.3f)\n', ...
        ratio_values(idx_e), Energy_Wh(idx_e), LapTime_s(idx_e), Eff_ratio(idx_e));
fprintf('  Event energy (x%d laps): min %.2f kWh @ n_gear=%.2f\n', ...
        n_laps_event, min(E_event_kWh), ratio_values(idx_e));

%% ===== Local helpers =====
function E_mech_Wh = get_mech_energy_Wh_from_R(R)
E_mech_Wh = NaN;
if isfield(R,'t') && isfield(R,'P_mech')
    t = R.t(:); P = R.P_mech(:);
    if numel(t)>1 && numel(t)==numel(P)
        dt = [diff(t); t(end)-t(end-1)];
        E_mech_Wh = sum(P .* dt) / 3600; return;
    end
end
if isfield(R,'t_vec') && isfield(R,'P_mech_vec')
    t = R.t_vec(:); P = R.P_mech_vec(:);
    if numel(t)>1 && numel(t)==numel(P)
        dt = [diff(t); t(end)-t(end-1)];
        E_mech_Wh = sum(P .* dt) / 3600; return;
    end
end
% leave NaN if unavailable; efficiency will still plot from battery energy
end

function E_batt_Wh = get_batt_energy_Wh_from_R(R, fallback_Wh)
if isfield(R,'t') && isfield(R,'P_batt')
    t = R.t(:); P = R.P_batt(:);
    if numel(t)>1 && numel(t)==numel(P)
        dt = [diff(t); t(end)-t(end-1)];
        E_batt_Wh = sum(P .* dt) / 3600; return;
    end
end
if isfield(R,'t_vec') && isfield(R,'P_batt_vec')
    t = R.t_vec(:); P = R.P_batt_vec(:);
    if numel(t)>1 && numel(t)==numel(P)
        dt = [diff(t); t(end)-t(end-1)];
        E_batt_Wh = sum(P .* dt) / 3600; return;
    end
end
E_batt_Wh = fallback_Wh;  % aggregate from simulate_lap
end
