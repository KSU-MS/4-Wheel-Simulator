function [t_vec,s_vec,v_vec,ax_vec,ay_vec,P_elec_vec,P_mech_vec,P_drag_vec,P_batt_vec,Fx_vec,Fy_vec,E_Wh] = simulate_segment(state, seg, veh, motor)
% SIMULATE_SEGMENT  Distance-accurate integrator for one segment.
% - Integrates until s >= seg.L
% - Corner speed cap from tire+aero lateral capacity
% - Single rear motor via motor_force_limits_single
% - Battery energy at PACK TERMINALS (includes I^2R + regen efficiency)

dt = veh.dt_s;

% Defaults for new energy params if missing
if ~isfield(veh,'eta_regen'),      veh.eta_regen = 0.90; end
if ~isfield(veh,'i_charge_max_A'), veh.i_charge_max_A = inf; end

% Preallocate (grow by chunks if needed)
chunk = 20000;
max_steps = 2e7;
t_vec = zeros(chunk,1); s_vec = zeros(chunk,1); v_vec = zeros(chunk,1);
ax_vec = zeros(chunk,1); ay_vec = zeros(chunk,1);
P_elec_vec = zeros(chunk,1); P_mech_vec = zeros(chunk,1);
P_drag_vec = zeros(chunk,1); P_batt_vec = zeros(chunk,1);
Fx_vec = zeros(chunk,1); Fy_vec = zeros(chunk,1);

v = state.v; s = 0; t = 0; E_Wh = 0; i = 0;
Fx_total = 0; P_elec_motor = 0;

while s < seg.L && i < max_steps
    i = i + 1;
    if i > numel(t_vec)   % grow arrays
        t_vec = [t_vec; zeros(chunk,1)];
        s_vec = [s_vec; zeros(chunk,1)];
        v_vec = [v_vec; zeros(chunk,1)];
        ax_vec = [ax_vec; zeros(chunk,1)];
        ay_vec = [ay_vec; zeros(chunk,1)];
        P_elec_vec = [P_elec_vec; zeros(chunk,1)];
        P_mech_vec = [P_mech_vec; zeros(chunk,1)];
        P_drag_vec = [P_drag_vec; zeros(chunk,1)];
        P_batt_vec = [P_batt_vec; zeros(chunk,1)];
        Fx_vec = [Fx_vec; zeros(chunk,1)];
        Fy_vec = [Fy_vec; zeros(chunk,1)];
    end

    % Lateral accel request from segment geometry
    if seg.R > 0
        ay = v^2 / seg.R;
    else
        ay = 0;
    end

    % Aero forces (downforce, drag)
    [Fdf, Fdrag] = aero_forces(veh, v);

    % Quasi-steady solve for ax respecting traction + motor limits
    Fy_req = veh.m_total_kg * ay;
    ax = 0; ax_prev = 1e9; it = 0;
    while it < veh.iter_max && abs(ax - ax_prev) > veh.iter_tol_ax
        it = it + 1; ax_prev = ax;

        % Normal loads with aero + transfers
        loads = normal_loads_two_track(veh, Fdf, ax, ay);

        % Per-tire capacities with load sensitivity
        muL = veh.tire.mu_long_0 .* (loads.Fz ./ veh.tire.Fz_ref) .^ veh.tire.load_sens;
        muT = veh.tire.mu_lat_0  .* (loads.Fz ./ veh.tire.Fz_ref) .^ veh.tire.load_sens;
        Fx_max = muL .* loads.Fz;
        Fy_max = muT .* loads.Fz;
        % Distribute lateral demand by axle (equal L/R on each axle)
        Fz_ax = [sum(loads.Fz(1:2)); sum(loads.Fz(3:4))];
        if Fy_req == 0
            Fy_tire = zeros(4,1);
        else
            Fy_ax_req = Fy_req * Fz_ax / max(sum(Fz_ax),1e-6);
            Fy_tire = [Fy_ax_req(1)/2; Fy_ax_req(1)/2; Fy_ax_req(2)/2; Fy_ax_req(2)/2];
        end

        % Remaining longitudinal via traction ellipse
        Fx_cap_tire = traction_ellipse(Fx_max, Fy_max, Fy_tire);

        % Single rear motor -> only rear tires deliver Fx
        Fx_cap_rear = Fx_cap_tire(3:4);
        [Fx_total, P_elec_motor, ~, ~, ~] = motor_force_limits_single( ...
            motor, v, veh.r_tire_m, Fx_cap_rear, veh.power_limit_W);

        % Longitudinal accel including aero drag
        ax = (Fx_total - Fdrag) / veh.m_total_kg;
    end

    % Corner speed cap from available lateral grip
    if seg.R > 0
        loads_cap = normal_loads_two_track(veh, Fdf, ax, ay);
        muT_cap = veh.tire.mu_lat_0 .* (loads_cap.Fz ./ veh.tire.Fz_ref) .^ veh.tire.load_sens;
        Fy_max_total = sum(muT_cap .* loads_cap.Fz);
        a_y_max = max(Fy_max_total / veh.m_total_kg, 1e-6);
        v_corner_cap = sqrt(a_y_max * seg.R);
        v = min(v, v_corner_cap);   % cap before advancing
        ay = min(ay, a_y_max);
    end

    % Advance state (semi-implicit Euler)
    v = max(0, v + ax*dt);
    s = s + v*dt;
    t = t + dt;
    % Mechanical & drag power (for plotting)
    P_mech = Fx_total * v;
    P_drag = Fdrag * v;

    % -------- Battery terminal power & energy --------
    % +P = discharge, -P = charge
    Vpack   = max(veh.pack_voltage_v, 1.0);
    I_pack  = P_elec_motor / Vpack;                    % can be +/- 
    P_loss  = (I_pack.^2) * veh.r_internal_ohm;        % I^2R loss >= 0
    if P_elec_motor >= 0
        P_pack = P_elec_motor + P_loss;                % motoring
    else
        P_pack = P_elec_motor * veh.eta_regen + P_loss;% regen (reduced by eff)
        % charge current limit (regen clamp)
        P_pack = max(P_pack, -veh.i_charge_max_A * Vpack);
    end
    % Energy accumulation (regen subtracts)
    E_Wh = E_Wh + (P_pack * dt)/3600;

    % -------- Log --------
    t_vec(i) = t; s_vec(i) = s; v_vec(i) = v;
    ax_vec(i) = ax; ay_vec(i) = ay;
    Fx_vec(i) = Fx_total; Fy_vec(i) = Fy_req;
    P_elec_vec(i) = P_elec_motor;
    P_mech_vec(i) = P_mech;
    P_drag_vec(i) = P_drag;
    P_batt_vec(i) = P_pack;
end

% Trim to actual length
t_vec = t_vec(1:i); s_vec = s_vec(1:i); v_vec = v_vec(1:i);
ax_vec = ax_vec(1:i); ay_vec = ay_vec(1:i);
P_elec_vec = P_elec_vec(1:i); P_mech_vec = P_mech_vec(1:i);
P_drag_vec = P_drag_vec(1:i); P_batt_vec = P_batt_vec(1:i);
Fx_vec = Fx_vec(1:i); Fy_vec = Fy_vec(1:i);
end
