function [Fdf, Fdrag] = aero_forces(veh, v)
q = 0.5 * veh.rho_air_kgpm3 * v.^2;
Fdf   = veh.ClA .* q;
Fdrag = veh.CdA  .* q;
end
