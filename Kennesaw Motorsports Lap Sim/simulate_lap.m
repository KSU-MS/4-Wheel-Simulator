function result = simulate_lap(track, veh, motor, make_plots)
if nargin < 4, make_plots = false; end
time=[]; s_path=[]; v_hist=[]; ax_hist=[]; ay_hist=[];
P_elec_hist=[]; P_mech_hist=[]; P_drag_hist=[]; P_batt_hist=[];
Fx_hist=[]; Fy_hist=[]; E_Wh=0;
state.v=0; state.s=0; t0=0;
for k=1:numel(track.type)
    seg.type=lower(track.type(k)); seg.R=abs(track.radius(k)); seg.L=track.length(k);
    [t,s,v,ax,ay,P_elec,P_mech,P_drag,P_batt,Fx,Fy,Eseg] = simulate_segment(state, seg, veh, motor);
    time=[time; t0+t]; s_path=[s_path; state.s+s]; v_hist=[v_hist; v];
    ax_hist=[ax_hist; ax]; ay_hist=[ay_hist; ay];
    P_elec_hist=[P_elec_hist; P_elec]; P_mech_hist=[P_mech_hist; P_mech];
    P_drag_hist=[P_drag_hist; P_drag]; P_batt_hist=[P_batt_hist; P_batt];
    Fx_hist=[Fx_hist; Fx]; Fy_hist=[Fy_hist; Fy]; E_Wh=E_Wh+Eseg;
    state.v=v(end); state.s=state.s+seg.L; t0=t0+t(end);
end
result.time_s=time; result.s_path=s_path; result.v=v_hist;
result.a_long=ax_hist; result.a_lat=ay_hist;
result.P_elec=P_elec_hist; result.P_mech=P_mech_hist; result.P_drag=P_drag_hist; result.P_batt_loss=P_batt_hist;
result.Fx_total=Fx_hist; result.Fy_total=Fy_hist;
result.lap_time_s=time(end); result.energy_used_Wh=E_Wh; result.v_max=max(v_hist);
if make_plots
    figure; subplot(3,1,1); plot(time, v_hist,'LineWidth',1.2); grid on; ylabel('Speed (m/s)');
    subplot(3,1,2); plot(time, ax_hist,'LineWidth',1.2); grid on; ylabel('a_x (m/s^2)');
    subplot(3,1,3); plot(time, ay_hist,'LineWidth',1.2); grid on; ylabel('a_y (m/s^2)'); xlabel('t (s)');
    sgtitle('Kinematics');
    figure; plot(time, P_elec_hist/1e3,'LineWidth',1.2); hold on;
    plot(time, P_mech_hist/1e3,'LineWidth',1.2); plot(time, P_drag_hist/1e3,'LineWidth',1.2);
    plot(time, P_batt_hist/1e3,'LineWidth',1.2); grid on;
    xlabel('t (s)'); ylabel('kW'); legend('Electrical','Mechanical','Drag','Battery loss'); title('Power Flows');
end
end
