function loads = normal_loads_two_track(veh, Fdf, ax, ay)
m = veh.m_total_kg; g = 9.81;
Fz_f = m*g*(1-veh.swd_frac) + Fdf*(1-veh.cp_frac);
Fz_r = m*g*veh.swd_frac + Fdf*veh.cp_frac;
WT_long = m * ax * veh.cg_z_m / veh.wb_m;
Fz_f = Fz_f - WT_long; Fz_r = Fz_r + WT_long;
WT_lat_f = m * ay * veh.cg_z_m / veh.tw_f_m * (Fz_f / (Fz_f + Fz_r + eps));
WT_lat_r = m * ay * veh.cg_z_m / veh.tw_r_m * (Fz_r / (Fz_f + Fz_r + eps));
Fz_FL = max(0, Fz_f/2 + WT_lat_f/2);
Fz_FR = max(0, Fz_f/2 - WT_lat_f/2);
Fz_RL = max(0, Fz_r/2 + WT_lat_r/2);
Fz_RR = max(0, Fz_r/2 - WT_lat_r/2);
loads.Fz = [Fz_FL; Fz_FR; Fz_RL; Fz_RR];
end
