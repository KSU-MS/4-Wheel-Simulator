# def Eff(R, veh, motor, opts):
#     [t_w, Pmech] = get_trace(R,'P_mech')
#     [t_b, Ppack] = get_trace(R,'P_batt')             

#     if ~isempty(Pmech) && ~isempty(t_w)
#         E_wheel_Wh = trapz(t_w(:), Pmech(:))/3600
#     else
#         E_wheel_Wh = get_scalar(R, {'energy_mech_Wh','E_mech_Wh'}, NaN)

#     if ~isempty(Ppack) && ~isempty(t_b)
#         E_pack_Wh  = trapz(t_b(:), Ppack(:))/3600
#     else
#         E_pack_Wh  = get_scalar(R, {'energy_used_Wh','E_batt_Wh'}, NaN)

#     eff.E_wheel_Wh = E_wheel_Wh
#     eff.E_pack_Wh  = E_pack_Wh
#     eff.eta_pt     = E_wheel_Wh / max(E_pack_Wh, 1e-9)

#     [t_s, Pshaft] = get_trace(R,'P_shaft')
#     if ~isempty(Pshaft) && ~isempty(t_s)
#         E_shaft_Wh  = trapz(t_s(:), Pshaft(:))/3600
#         eff.eta_mech = E_wheel_Wh / max(E_shaft_Wh, 1e-9)
#     else
#         % use assumed constant from veh if present
#         eta_guess = get_scalar(veh, {'drivetrain_eff','eta_mech'}, 0.96)
#         eff.eta_mech_est = eta_guess
#     
#     
    # def get_trace(R, base):
        # % Returns (t, P_base) using either [t, P] or [t_vec, P_base_vec]
        # P = [] t = []
        # if isfield(R, base) && isfield(R, 't'),      P = R.(base)      t = R.t
        # if isempty(P) && isfield(R,[base '_vec']) && isfield(R,'t_vec')
        #     P = R.([base '_vec']) t = R.t_vec

        # if ~isempty(P), P = P(:)
        # if ~isempty(t), t = t(:)

        # def get_scalar(S, names,val):
        # for k=1:numel(names)
        #     if isfield(S, names{k})
        #         v = S.(names{k})
        #         if isnumeric(v) && isscalar(v), val = v return


        # def infer_eta_mech_target(eff, veh, E_pack_real_Wh):
        #     eta_assumed = get_scalar(veh, {'drivetrain_eff','eta_mech'}, 0.96)
        #     scale = max(eff.E_pack_Wh,1e-9) / max(E_pack_real_Wh,1e-9)
        #     eta_mech_target = min(0.995, max(0.85, eta_assumed / scale))


# eff = compute_efficiency_breakdown(R, veh, motor)
    # fprintf('\n--- Efficiency breakdown ---\n')
    # fprintf('Wheel energy:      %.2f Wh\n', eff.E_wheel_Wh)
    # fprintf('Pack energy:       %.2f Wh\n', eff.E_pack_Wh)
    # fprintf('η_pt pack→wheel:   %.3f\n', eff.eta_pt)
    # if isfield(eff,'eta_mech')
    #     fprintf('η_mech shaft→wheel: %.3f\n', eff.eta_mech)
    # else
    #     fprintf('η_mech (assumed):   %.3f  (edit veh.drivetrain_eff to change)\n', eff.eta_mech_est)

    # if exist('plot_track_velocity_corners','file') == 2
    #     plot_track_velocity_corners(track, R, struct('ds_plot_m',0.20,'colormap_name','turbo','show_arrows',true,'units_mph',false))
    # else
    #     warning('plot_track_velocity_corners.m not found on path – skipping track heat map.')

    # limits_kW = np.arange(10,31,1)
    # S = SweepPowerLimit(track, veh, motor, limits_kW*1e3)

    # figure('Color','w') 
    # plot(S.power_limit_kW, S.lap_time_s, 'o-','LineWidth',1.5) grid on
    # xlabel('Power Limit (kW)') ylabel('Lap Time (s)')
    # title('Lap Time vs Software Power Limit')

    # figure('Color','w')
    # plot(S.power_limit_kW, S.energy_kWh, 'o-','LineWidth',1.5) grid on
    # xlabel('Power Limit (kW)') ylabel('Energy per Lap (kWh)')
    # title('Energy vs Software Power Limit')