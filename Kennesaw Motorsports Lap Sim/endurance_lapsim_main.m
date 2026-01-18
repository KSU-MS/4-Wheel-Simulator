% endurance_lapsim_main.m
% CSV-driven endurance lap simulator (single rear motor, chain drive)
clear; clc; close all;

%% Load inputs
veh   = load_vehicle('../Data/KS9E.csv');
motor = load_motor('../Data/emrax_208_mv.csv');
track = load_track('../Data/skidpad.csv');
%% Run one lap at configured software power limit
R = simulate_lap(track, veh, motor, true);

fprintf('\n--- Endurance Lap (limit = %.0f kW) ---\n', veh.power_limit_W/1e3);
fprintf('Lap time:          %.2f s\n', R.lap_time_s);
fprintf('Energy (battery):  %.2f kWh\n', R.energy_used_Wh/1000);
fprintf('Peak speed:        %.1f km/h\n', R.v_max*3.6);

%% Efficiency breakdown (pack→wheel, and mechanical if available)
eff = compute_efficiency_breakdown(R, veh, motor);
fprintf('\n--- Efficiency breakdown ---\n');
fprintf('Wheel energy:      %.2f Wh\n', eff.E_wheel_Wh);
fprintf('Pack energy:       %.2f Wh\n', eff.E_pack_Wh);
fprintf('η_pt pack→wheel:   %.3f\n', eff.eta_pt);
if isfield(eff,'eta_mech')
    fprintf('η_mech shaft→wheel: %.3f\n', eff.eta_mech);
else
    fprintf('η_mech (assumed):   %.3f  (edit veh.drivetrain_eff to change)\n', eff.eta_mech_est);
end

%% Optional: compare to a real-lap energy to infer mechanical target
% real_energy_kWh = 4.60;                            % <-- put your measured per-lap kWh here
% E_pack_real_Wh  = real_energy_kWh*1000;
% eta_mech_target = infer_eta_mech_target(eff, veh, E_pack_real_Wh);
% fprintf('Implied η_mech to match real energy: %.3f\n', eta_mech_target);

%% Velocity heat map + corner labels (requires plot_track_velocity_corners.m on path)
if exist('plot_track_velocity_corners','file') == 2
    plot_track_velocity_corners(track, R, struct('ds_plot_m',0.20,'colormap_name','turbo','show_arrows',true,'units_mph',false));
else
    warning('plot_track_velocity_corners.m not found on path – skipping track heat map.');
end

%% Sweep software power limit for time/energy trade
limits_kW = 10:1:30;
S = sweep_power_limit(track, veh, motor, limits_kW*1e3);

figure('Color','w'); 
plot(S.power_limit_kW, S.lap_time_s, 'o-','LineWidth',1.5); grid on;
xlabel('Power Limit (kW)'); ylabel('Lap Time (s)');
title('Lap Time vs Software Power Limit');

figure('Color','w');
plot(S.power_limit_kW, S.energy_kWh, 'o-','LineWidth',1.5); grid on;
xlabel('Power Limit (kW)'); ylabel('Energy per Lap (kWh)');
title('Energy vs Software Power Limit');

%% ===== Helpers (kept local for one-file runability) =====

function eff = compute_efficiency_breakdown(R, veh, motor, opts)
% Returns lap-integrated efficiencies using your sim result struct R.
if nargin<4, opts = struct; end

% Try to get traces first; else fall back to aggregates
[t_w, Pmech] = get_trace(R,'P_mech');              % wheel mechanical
[t_b, Ppack] = get_trace(R,'P_batt');              % pack terminal

if ~isempty(Pmech) && ~isempty(t_w)
    E_wheel_Wh = trapz(t_w(:), Pmech(:))/3600;
else
    E_wheel_Wh = get_scalar(R, {'energy_mech_Wh','E_mech_Wh'}, NaN);
end
if ~isempty(Ppack) && ~isempty(t_b)
    E_pack_Wh  = trapz(t_b(:), Ppack(:))/3600;
else
    E_pack_Wh  = get_scalar(R, {'energy_used_Wh','E_batt_Wh'}, NaN);
end

eff.E_wheel_Wh = E_wheel_Wh;
eff.E_pack_Wh  = E_pack_Wh;
eff.eta_pt     = E_wheel_Wh / max(E_pack_Wh, 1e-9);

% Mechanical (shaft→wheel) if you log it
[t_s, Pshaft] = get_trace(R,'P_shaft');
if ~isempty(Pshaft) && ~isempty(t_s)
    E_shaft_Wh  = trapz(t_s(:), Pshaft(:))/3600;
    eff.eta_mech = E_wheel_Wh / max(E_shaft_Wh, 1e-9);
else
    % use assumed constant from veh if present
    eta_guess = get_scalar(veh, {'drivetrain_eff','eta_mech'}, 0.96);
    eff.eta_mech_est = eta_guess;
end
end

function [t, P] = get_trace(R, base)
% Returns (t, P_base) using either [t, P] or [t_vec, P_base_vec]
P = []; t = [];
if isfield(R, base) && isfield(R, 't'),      P = R.(base);      t = R.t;      end
if isempty(P) && isfield(R,[base '_vec']) && isfield(R,'t_vec')
    P = R.([base '_vec']); t = R.t_vec;
end
if ~isempty(P), P = P(:); end
if ~isempty(t), t = t(:); end
end

function val = get_scalar(S, names, def)
val = def;
for k=1:numel(names)
    if isfield(S, names{k})
        v = S.(names{k});
        if isnumeric(v) && isscalar(v), val = v; return; end
    end
end
end

function eta_mech_target = infer_eta_mech_target(eff, veh, E_pack_real_Wh)
% If you don’t log motor shaft power, this gives a simple “what η_mech would
% make the sim match the real pack energy?” estimate.
% Interpretation: higher η_mech_target than your current assumption means
% your mechanical system is likely better than modeled.
eta_assumed = get_scalar(veh, {'drivetrain_eff','eta_mech'}, 0.96);
% If we assume pack→wheel stays same proportion, scale η_mech by the ratio of
% simulated pack energy to real pack energy.
scale = max(eff.E_pack_Wh,1e-9) / max(E_pack_real_Wh,1e-9);
eta_mech_target = min(0.995, max(0.85, eta_assumed / scale));
end
