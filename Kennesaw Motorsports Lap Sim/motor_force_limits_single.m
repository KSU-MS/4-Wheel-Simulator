function [Fx_rear_total, P_elec, P_mech, I_phase, omega] = motor_force_limits_single(motor, v, r_tire, Fx_cap_rear, P_pack_limit)
% Single rear motor -> both rear tires through chain/final drive.
% Fx_cap_rear is 2x1 [RL; RR] capacity after traction ellipse.

% Motor speed & wheel speed
omega_wheel = v / r_tire;         % rad/s
omega = omega_wheel * motor.n_gear;

% Torque & power limits at motor, then to wheels
T_motor_max = motor.tmax_nm;
T_wheel_max = T_motor_max * motor.n_gear * motor.tran_eff;
Fx_torque_cap = T_wheel_max / r_tire;

P_motor_max = motor.pmax_w;
Fx_power_cap = P_motor_max / max(v,1e-6);

% Motor's total axle capability at the contact patch
Fx_motor_cap_total = min(Fx_torque_cap, Fx_power_cap);

% Tire-limited total demand from both rears
Fx_tire_total_cap = sum(Fx_cap_rear);

% Enforce pack/software power limit (convert to mech force)
Fx_pack_cap = (P_pack_limit) / max(v,1e-6);

% Final total available longitudinal force at rear axle
Fx_rear_total = min([Fx_motor_cap_total, Fx_tire_total_cap, Fx_pack_cap]);

% Electrical & mechanical power bookkeeping (approximate)
% Back-calc motor torque used
T_used = min(T_motor_max, Fx_rear_total * r_tire / (motor.n_gear * motor.tran_eff));
I_phase = T_used / max(motor.kt_nm_per_a,1e-6);
P_mech  = Fx_rear_total * v;
V_back  = motor.ke_v_per_rad * omega;
P_elec  = P_mech/motor.tran_eff + I_phase^2 * motor.r_phase_ohm + V_back * motor.i0_a;
end
