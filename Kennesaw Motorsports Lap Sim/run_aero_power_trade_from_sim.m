%% run_aero_power_trade_from_sim.m
% One-click sweep of aero (Cdf) and power limit (kW) using your lap sim,
% scored with FSAE EV 2025 Endurance + Efficiency rules, scaled to YOUR TRACK.
% Adds diagnostics so you can see why the heatmap might be flat.

clear; clc; close all;

%% ---------------- User inputs ----------------
track_file   = '../Data/track.csv';          % or 'track_enduro_24.csv'
vehicle_file = '../Data/vehicle.csv';
motor_file   = '../Data/motor_rear.csv';
n_laps       = 22;                   % EV Endurance laps
Ecap_kWh     = 6.73;                 % dashed line at this total energy

% Sweep ranges
cdf_values       = linspace(0, 5.0, 15);     % 0..5 downforce coeff
power_limits_W   = (25:5:75)*1e3;            % 25..75 kW

% Cdf -> Cd mapping (tune to your L/D)
drag_model.cd_base      = 1.5;               % base Cd at current car
drag_model.k_cd_per_cdf = 0.08;              % ΔCd per +1 Cdf

% Solver speed knobs (OPTIONAL)
use_parallel = false;                        % true if Parallel Toolbox available

% >>> NEW: how to align scoring with your track <<<
% 'scale_targets'  : set per-lap best/worst on your course (recommended)
% 'scale_times'    : keep event totals, rescale your LapTime to 1200m equiv
scaling_mode   = 'scale_times';
t_best_lap_s   = 65;      % only used for 'scale_targets'
time_factor    = 1.45;    % worst/best ratio from rules

%% --------------- Track-length aware targets ---------------
Ttrack   = readtable(track_file);
L_track  = sum(Ttrack.length_m);     % meters per lap
L_ref    = 1200;                     % representative FS endurance length
scale_L  = max(L_track / L_ref, 1e-6);

% "Best energy" reference (scaled to your track)
Emin_kWh_baseline = 3.275;           % from 2025; different course length
Emin_kWh          = Emin_kWh_baseline * scale_L;

% Build time targets OR set up time rescaling
switch lower(scaling_mode)
    case 'scale_targets'
        t_worst_lap_s = time_factor * t_best_lap_s;
        Tmin = t_best_lap_s  * n_laps;       % total best time
        Tmax = t_worst_lap_s * n_laps;       % total time cap
        rescale_times = false;
    case 'scale_times'
        Tmin = 1361.936;                     % event best total time
        Tmax = 1.45*Tmin;                    % event cap
        rescale_times = true;
    otherwise
        error('scaling_mode must be ''scale_targets'' or ''scale_times''.');
end

% Efficiency-factor bounds from 2025 (kept)
EffFactor_max = 0.848;
EffFactor_min = 0.333;

% Full-lap completion points
EnduranceLapScore = 25;

fprintf('Track length: %.1f m | scaling=%s\n', L_track, scaling_mode);
if ~rescale_times
    fprintf('Per-lap targets: best=%.1f s, cap=%.1f s  (Tmin=%.1f, Tmax=%.1f)\n', Tmin/n_laps, Tmax/n_laps, Tmin, Tmax);
else
    fprintf('Using event totals (Tmin=%.1f, Tmax=%.1f); lap times will be rescaled to %.0f m equivalent for scoring.\n', Tmin, Tmax, L_ref);
end
fprintf('Scaled energy best reference (Emin) = %.3f kWh\n', Emin_kWh);

%% --------------- Run sim grid ----------------
[LapTime, Energy] = sim_grid(track_file, vehicle_file, motor_file, ...
                             cdf_values, power_limits_W, n_laps, drag_model, use_parallel);

% Quick sanity prints
lap_min = min(LapTime(:)); lap_max = max(LapTime(:));
ener_min = min(Energy(:)); ener_max = max(Energy(:));
fprintf('LapTime range: %.1f–%.1f s\n', lap_min, lap_max);
fprintf('Total energy range: %.2f–%.2f kWh\n', ener_min, ener_max);

%% --------------- Score (FSAE EV 2025 rules; clamped) ----------
% If you used 'scale_times', make sure TotalTime was already computed;
% otherwise:
if ~exist('TotalTime','var') || isempty(TotalTime)
    TotalTime = LapTime * n_laps;    % total endurance time [s]
end

[EnduranceTimeScore, EfficiencyScore, TotalScore] = score_fsae_ev_2025( ...
    TotalTime, Energy, Tmin, Tmax, Emin_kWh, 0.333, 0.848, 25);

% Diagnostics
fprintf('Endurance time pts:  min/max = %.1f / %.1f (cap 250)\n', ...
        min(EnduranceTimeScore(:)), max(EnduranceTimeScore(:)));
fprintf('Efficiency pts:      min/max = %.1f / %.1f (cap 100)\n', ...
        min(EfficiencyScore(:)),    max(EfficiencyScore(:)));
fprintf('TOTAL pts:           min/max = %.1f / %.1f (cap 375)\n', ...
        min(TotalScore(:)),         max(TotalScore(:)));

%% --------------- Plot like your figure -------
figure('Color','w','Position',[80 80 1200 520]);
imagesc(power_limits_W/1e3, cdf_values, TotalScore); set(gca,'YDir','normal'); colorbar;
xlabel('Software Power Limit (kW)'); ylabel('Downforce Cdf');
title('FSAE EV 2025 Total Score (Endurance + Efficiency)'); grid on; hold on;

% Energy cap contour
if ~isnan(Ecap_kWh)
    [Ccap,hcap] = contour(power_limits_W/1e3, cdf_values, Energy, [Ecap_kWh Ecap_kWh], 'w--','LineWidth',1.6);
    clabel(Ccap,hcap,'Color','w','FontWeight','bold');
end

% Time-cap boundary (avg lap must satisfy the cap)
[XX,YY] = meshgrid(power_limits_W/1e3, cdf_values);
too_slow = TotalTime > Tmax;
contour(XX,YY,too_slow,[1 1],'k--','LineWidth',1.2);
text(min(XX(:))+1, max(YY(:))-0.15, sprintf('Time cap: avg lap \\le %.1f s', Tmax/n_laps), 'Color','k');

% Score contours
contour(power_limits_W/1e3, cdf_values, TotalScore, 12, 'k:','LineWidth',0.7);

% Mark best and best under energy cap
[maxScore, idx] = max(TotalScore(:));
[ib, jb] = ind2sub(size(TotalScore), idx);
plot(power_limits_W(jb)/1e3, cdf_values(ib), 'wo', 'MarkerFaceColor','w', 'MarkerSize',8);

mask = Energy <= Ecap_kWh;
if any(mask(:))
    [capScore, capIdx] = max(TotalScore(mask));
    [ii_all, jj_all] = ind2sub(size(TotalScore), find(mask));
    ii = ii_all(capIdx); jj = jj_all(capIdx);
    plot(power_limits_W(jj)/1e3, cdf_values(ii), 'ws', 'MarkerFaceColor','w', 'MarkerSize',8);
end

%% --------------- Print recommendations --------
best.lap_s  = LapTime(ib,jb);
best.energy = Energy(ib,jb);
best.cdf    = cdf_values(ib);
best.kW     = power_limits_W(jb)/1e3;

fprintf('Best total score: %.1f pts @ Cdf=%.2f, P=%.0f kW -> lap=%.1f s, total energy=%.2f kWh\n', ...
        maxScore, best.cdf, best.kW, best.lap_s, best.energy);

if any(mask(:))
    [capScore, capIdx] = max(TotalScore(mask));
    [ii_all, jj_all] = ind2sub(size(TotalScore), find(mask));
    ii = ii_all(capIdx); jj = jj_all(capIdx);
    fprintf('Best under %.2f kWh: %.1f pts @ Cdf=%.2f, P=%.0f kW -> lap=%.1f s, energy=%.2f kWh\n', ...
        Ecap_kWh, TotalScore(ii,jj), cdf_values(ii), power_limits_W(jj)/1e3, LapTime(ii,jj), Energy(ii,jj));
end

%% --------------- helper: run sim on a grid --
function [LapTime, Energy] = sim_grid(track_file, vehicle_file, motor_file, cdf_values, power_limits_W, n_laps, drag_model, use_parallel)
    veh   = load_vehicle(vehicle_file);
    motor = load_motor(motor_file);
    track = load_track(track_file);

    nc = numel(cdf_values); np = numel(power_limits_W);
    LapTime = nan(nc,np); Energy = nan(nc,np);

    veh0 = veh;
    if use_parallel && isempty(gcp('nocreate')), parpool; end

    for ic = 1:nc
        veh.cdf = cdf_values(ic);
        veh.cd  = drag_model.cd_base + drag_model.k_cd_per_cdf * veh.cdf;   % simple Cdf->Cd mapping

        if use_parallel
            parfor ip = 1:np
                veh_i = veh;                              % worker-local copy
                veh_i.software_power_limit_W = power_limits_W(ip);
                R = simulate_lap(track, veh_i, motor, false);
                LapTime(ic,ip) = R.lap_time_s;                 % average lap time [s]
                Energy(ic,ip)  = (R.energy_used_Wh/1000) * n_laps; % total energy [kWh]
            end
        else
            for ip = 1:np
                veh.software_power_limit_W = power_limits_W(ip);
                R = simulate_lap(track, veh, motor, false);
                LapTime(ic,ip) = R.lap_time_s;
                Energy(ic,ip)  = (R.energy_used_Wh/1000) * n_laps;
            end
        end
    end
    veh = veh0; %#ok<NASGU>
end

function [EndTimePts, EffPts, TotalPts] = score_fsae_ev_2025( ...
    TotalTime_s, TotalEnergy_kWh, Tmin, Tmax, Emin_kWh, EffMin, EffMax, LapPts)

% Endurance time score (0..250), only if within time cap
EndTimePts = zeros(size(TotalTime_s));
mask = isfinite(TotalTime_s) & TotalTime_s <= Tmax;
den = (Tmax / Tmin) - 1;
EndTimePts(mask) = 250 .* ((Tmax ./ TotalTime_s(mask)) - 1) ./ den;
EndTimePts = max(0, min(250, EndTimePts));  % clamp

% Efficiency score (0..100) using efficiency factor bounds
EffFactor = (Tmin ./ TotalTime_s) .* (Emin_kWh ./ TotalEnergy_kWh);
EffPts = 100 .* (EffFactor - EffMin) ./ (EffMax - EffMin);
EffPts = max(0, min(100, EffPts));          % clamp

% Total (max 375 = 250 + 25 + 100)
TotalPts = EndTimePts + LapPts + EffPts;

% Safety: warn if anything exceeds rule caps
if any(TotalPts(:) > 375 + 1e-6)
    warning('Total points exceed 375 (%.1f max found). Check inputs/scaling.', max(TotalPts(:)));
end
end
