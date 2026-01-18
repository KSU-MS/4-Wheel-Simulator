
% 4-Wheel Cornering Model with Imperial Pacejka4 Integration + Tire Plotter
clear; clc;

% === Vehicle Parameters ===
m = 320; g = 9.81;
a = 0.9; b = 0.7; L = a + b;
h_CG = 0.25; track_f = 1.2; track_r = 1.2;
rho = 1.2; Af = 1.2;

% === Aero Setup ===
Cdf_front = 1.5; Cdf_rear = 1.5;
v = 50 / 3.6;   % [m/s]
R = 15;         % [m]
a_lat = v^2 / R;

% === Downforce ===
F_aero_f = 0.5 * rho * Af * Cdf_front * v^2;
F_aero_r = 0.5 * rho * Af * Cdf_rear * v^2;

% === Static Loads ===
W = m * g;
W_f = W * (b / L); W_r = W * (a / L);

% === Load Transfer ===
dFz_lat = m * a_lat * h_CG / ((track_f + track_r)/2);
dFz_f = dFz_lat * (b / L); dFz_r = dFz_lat * (a / L);

% === Dynamic Fz ===
Fz_f = W_f + F_aero_f;
Fz_r = W_r + F_aero_r;

% === Required Fy (N) ===
Fy_total = m * a_lat;
Fy_f_req = (b / L) * Fy_total;
Fy_r_req = (a / L) * Fy_total;

% === Convert to lb for Pacejka ===
Fz_f_lb = Fz_f * 2.20462;
Fz_r_lb = Fz_r * 2.20462;
Fy_f_req_lb = Fy_f_req * 2.20462;
Fy_r_req_lb = Fy_r_req * 2.20462;

% === Slip Angle & Tire Model ===
alpha_range = linspace(-0.3, 0.3, 200); gamma = 0;
Fy_f = pacejka4_eval(alpha_range, Fz_f_lb, gamma);
Fy_r = pacejka4_eval(alpha_range, Fz_r_lb, gamma);

% === Find closest slip angle by minimizing force error ===
[~, idx_f] = min(abs(Fy_f - Fy_f_req_lb));
alpha_f_required = alpha_range(idx_f);

[~, idx_r] = min(abs(Fy_r - Fy_r_req_lb));
alpha_r_required = alpha_range(idx_r);

% === Output to Text File ===
fid = fopen('cornering_results.txt', 'w');
fprintf(fid, '================= Cornering Analysis =================\n');
fprintf(fid, 'Speed                : %6.2f km/h\n', v * 3.6);
fprintf(fid, 'Turn Radius          : %6.2f m\n', R);
fprintf(fid, 'Lateral Acceleration : %6.2f m/s^2\n', a_lat);
fprintf(fid, '                     : %6.2f G\n', a_lat / g);

fprintf(fid, '\n--- FRONT AXLE ---\n');
fprintf(fid, '  Fz                 : %6.1f N\n', Fz_f);
fprintf(fid, '  Fy Required        : %6.1f N\n', Fy_f_req);
fprintf(fid, '  Slip Angle (rad)   : %6.3f\n', alpha_f_required);
fprintf(fid, '  Slip Angle (deg)   : %6.2f\n', rad2deg(alpha_f_required));

fprintf(fid, '\n--- REAR AXLE ---\n');
fprintf(fid, '  Fz                 : %6.1f N\n', Fz_r);
fprintf(fid, '  Fy Required        : %6.1f N\n', Fy_r_req);
fprintf(fid, '  Slip Angle (rad)   : %6.3f\n', alpha_r_required);
fprintf(fid, '  Slip Angle (deg)   : %6.2f\n', rad2deg(alpha_r_required));
fprintf(fid, '======================================================\n');
fclose(fid);
fprintf('✅ Results written to "cornering_results.txt"\n');

% === Plot Tire Model at Multiple Loads ===
figure; hold on; grid on;
alpha = linspace(-0.3, 0.3, 200);  % rad
gamma = 0;
Fz_values = [50, 100, 150, 200, 250, 300];

for i = 1:length(Fz_values)
    Fy = pacejka4_eval(alpha, Fz_values(i), gamma);
    plot(rad2deg(alpha), Fy, 'DisplayName', sprintf('Fz = %d lb', Fz_values(i)));
end

xlabel('Slip Angle [deg]');
ylabel('Lateral Force Fy [lb]');
title('Pacejka4 Tire Model - Fy vs Slip Angle');
legend('Location', 'best');
