function Fy = pacejka4_eval(alpha, Fz, gamma)
% PACEJKA4_EVAL Evaluates lateral force using a 4-term Pacejka tire model.
% Supports load and camber sensitivity.
%
% Inputs:
%   alpha - slip angle [rad] (scalar or array)
%   Fz    - normal load [lb] (same size as alpha or scalar)
%   gamma - camber angle [rad] (same size as alpha or scalar)
%
% Output:
%   Fy    - lateral force [lb]

    % --- Coefficients (tunable constants) ---
    B = 10;            % Stiffness factor
    C = 1.3;           % Shape factor
    D_base = 450;      % Peak Fy at nominal Fz and 0 camber [lb]
    E = 0.97;          % Curvature factor
    Fz_nom = 150;      % Nominal reference Fz [lb]

    % --- Load sensitivity ---
    D_load = D_base .* sqrt(Fz ./ Fz_nom);

    % --- Camber sensitivity ---
    camber_effect = 1 - 0.05 .* gamma.^2;
    D = D_load .* camber_effect;
    display(D)
    % --- Pacejka 4-term model ---
    Fy = D .* sin(C .* atan(B .* alpha - E .* (B .* alpha - atan(B .* alpha))));
end
