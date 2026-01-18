function sweep = sweep_power_limit(track, veh, motor, power_limits_W)
n = numel(power_limits_W);
lap_time_s = zeros(n,1);
energy_kWh = zeros(n,1);
veh0 = veh;
for i=1:n
    veh.software_power_limit_W = power_limits_W(i);
    R = simulate_lap(track, veh, motor, false);
    lap_time_s(i) = R.lap_time_s;
    energy_kWh(i) = R.energy_used_Wh/1000;
end
veh = veh0; %#ok<NASGU>
sweep.power_limit_kW = power_limits_W(:)/1e3;
sweep.lap_time_s     = lap_time_s;
sweep.energy_kWh     = energy_kWh;
end
