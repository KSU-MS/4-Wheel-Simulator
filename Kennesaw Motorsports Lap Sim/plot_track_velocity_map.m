function plot_track_velocity_map(track_in, R, opts)
% PLOT_TRACK_VELOCITY_MAP  XY centerline colored by velocity.
% Works with track structs/tables that use:
%   - length_m OR L OR length/len/segment_length (m/ft/in)
%   - radius_m OR R OR radius (m/ft/in) (+ sign = left / - sign = right)
%   - type ('straight'/'corner')  [optional; inferred if missing]
% Optional opts:
%   ds_plot_m (0.25), colormap_name ('turbo'), mph (false),
%   show_arrows (true), start_xy ([0 0]), start_psi_rad (0), save_fig (''),
%   units: 'm'|'ft'|'in'  (forces units if headers don't reveal)

if nargin < 3 || ~isstruct(opts), opts = struct; end
start_xy      = getopt(opts,'start_xy',[0 0]);
start_psi     = getopt(opts,'start_psi_rad',0);
ds_plot       = getopt(opts,'ds_plot_m',0.25);
cmap_name     = getopt(opts,'colormap_name','turbo');
show_arrows   = getopt(opts,'show_arrows',true);
use_mph       = getopt(opts,'mph',false);
save_fig_path = getopt(opts,'save_fig','');
units_hint    = lower(getopt(opts,'units',''));  % '', 'm', 'ft', 'in'

% --- Normalize the track (handles L/R/type, feet/inches) ---
track = normalize_track(track_in, units_hint);   % -> fields: length_m (Nx1), R (Nx1, signed), type (Nx1 string)

% --- Build XY centerline ---
[s_c, x_c, y_c, psi_c] = compute_track_xy(track, start_xy, start_psi, ds_plot);

% --- Pull speed vs distance from result R ---
[s_sim, v_sim] = extract_sv(R);

% --- Interpolate speed onto the centerline s-grid ---
v_line = interp1(s_sim, v_sim, s_c, 'linear', 'extrap');
if use_mph, v_disp = v_line*2.23693629; cbar_label = 'Speed (mph)';
else,       v_disp = v_line;            cbar_label = 'Speed (m/s)';
end

% --- Plot ---
figure('Color','w','Name','Track Velocity Map'); hold on; axis equal;
scatter(x_c, y_c, 14, v_disp, 'filled');
colormap(cmap_name); cb = colorbar; ylabel(cb, cbar_label);
xlabel('X [m]'); ylabel('Y [m]'); title('Velocity Heat Trace'); grid on; box on;

if show_arrows
    step = max(1, round(3.0/ds_plot));   % ~3 m spacing
    quiver(x_c(1:step:end), y_c(1:step:end), cos(psi_c(1:step:end)), ...
           sin(psi_c(1:step:end)), 0.5, 'Color',[0.15 0.15 0.15], 'LineWidth',0.8);
end

if ~isempty(save_fig_path), exportgraphics(gcf, save_fig_path, 'Resolution',200); end
end

% ================= helpers =================
function val = getopt(s, name, default)
if isstruct(s) && isfield(s, name) && ~isempty(s.(name)), val = s.(name); else, val = default; end
end

function track = normalize_track(track_in, units_hint)
% Output: struct with fields length_m (Nx1), R (Nx1 signed), type (Nx1 string)

u = lower(string(units_hint));
u_factor = @(name) 1.0; % default meters

% map units hints
if u == "ft", uF = 0.3048; elseif u == "in", uF = 0.0254; else, uF = NaN; end

if istable(track_in)
    T = track_in;
    % LENGTH
    L = col_any(T, {'length_m','L','length','len','segment_length','Length_m','Length','len_m','len_ft','length_ft','Length_ft','length_in','Length_in'});
    if isempty(L), error('Track needs a length column (length_m or L or length/len/segment_length).'); end
    % UNITS for length
    if      hascol(T,'length_ft') || hascol(T,'Length_ft') || endsWithi(name_of(T,L),'_ft'), L = L*0.3048;
    elseif  hascol(T,'length_in') || hascol(T,'Length_in') || endsWithi(name_of(T,L),'_in'), L = L*0.0254;
    elseif ~isnan(uF), L = L*uF; end

    % RADIUS (signed if turn_dir exists)
    R = zeros(size(L));
    Rcand = col_any(T, {'signed_radius_m','radius_m','R','radius','corner_radius_m','Radius_m','Radius','radius_ft','radius_in'});
    if ~isempty(Rcand)
        R = Rcand;
        if hascol(T,'turn_dir')
            dir = upper(string(T.turn_dir));
            sgn = ones(size(R)); sgn(dir=="L")=+1; sgn(dir=="R")=-1; R = abs(R).*sgn;
        end
        if hascol(T,'radius_ft') || endsWithi(name_of(T,R),'_ft'), R = R*0.3048; end
        if hascol(T,'radius_in') || endsWithi(name_of(T,R),'_in'), R = R*0.0254; end
        if ~isnan(uF) && (hascol(T,'radius') || hascol(T,'R')), R = R*uF; end
    end

    % TYPE (optional)
    if hascol(T,'type'), type = lower(string(T.type)); else, type = repmat("straight", numel(L),1); end
    type(strcmp(R,0)) = "straight"; type(R~=0 & type=="straight") = "corner";

    track.length_m = L(:);
    track.R        = R(:);
    track.type     = type(:);

else
    % struct/array of segments
    S = track_in;
    % LENGTH
    if isfield(S,'length_m'), L = S.length_m(:);
    elseif isfield(S,'L'),    L = S.L(:);
    elseif isfield(S,'length'), L = S.length(:);
    else, error('Track struct must include field: length_m or L or length'); end
    if ~isnan(uF), L = L*uF; end

    % RADIUS
    if     isfield(S,'signed_radius_m'), R = S.signed_radius_m(:);
    elseif isfield(S,'radius_m'),        R = S.radius_m(:);
    elseif isfield(S,'R'),               R = S.R(:);
    elseif isfield(S,'radius'),          R = S.radius(:);
    else,                                 R = zeros(size(L));
    end
    if isfield(S,'turn_dir')
        dir = upper(string(S.turn_dir(:)));
        sgn = ones(size(R)); sgn(dir=="L")=+1; sgn(dir=="R")=-1; R = abs(R).*sgn;
    end
    if ~isnan(uF) && (isfield(S,'radius') || isfield(S,'R')), R = R*uF; end

    % TYPE
    if isfield(S,'type'), type = lower(string(S.type(:)));
    else, type = repmat("straight", numel(L),1); type(R~=0) = "corner";
    end

    track.length_m = L(:);
    track.R        = R(:);
    track.type     = type(:);
end

% Treat huge radius as straight
track.R(~isfinite(track.R)) = 0;
track.R(abs(track.R) > 1e6) = 0;
is_str = (track.type=="straight");
track.R(is_str) = 0;
end

function tf = hascol(T, name)
tf = any(strcmpi(name, T.Properties.VariableNames));
end

function n = name_of(T, vec)
% try to recover name of the last-used variable (best-effort; harmless if empty)
n = '';
try
    vnames = T.Properties.VariableNames;
    for k=1:numel(vnames)
        if isequal(T.(vnames{k}), vec), n = vnames{k}; return; end
    end
end
end

function tf = endsWithi(s, pat)
tf = false;
if ischar(s) || isstring(s)
    tf = endsWith(string(s), string(pat), 'IgnoreCase', true);
end
end

function col = col_any(T, names)
col = [];
for k=1:numel(names)
    if any(strcmpi(names{k}, T.Properties.VariableNames))
        col = T.(names{k}); return;
    end
end
end

function [s_c, x_c, y_c, psi_c] = compute_track_xy(track, start_xy, start_psi, ds)
% Integrate centerline; positive R => left turn, negative => right
x = start_xy(1); y = start_xy(2); psi = start_psi;
N = numel(track.length_m);

s_c = zeros(0,1); x_c = zeros(0,1); y_c = zeros(0,1); psi_c = zeros(0,1);
s_acc = 0;

for i=1:N
    L  = track.length_m(i);
    R  = track.R(i);              % signed radius (0 for straight)
    n  = max(1, ceil(L/ds));
    dS = L / n;

    if abs(R) < 1e-9
        % Straight
        for k=1:n
            x = x + dS*cos(psi);
            y = y + dS*sin(psi);
            s_acc = s_acc + dS;
            x_c(end+1,1)=x; y_c(end+1,1)=y; psi_c(end+1,1)=psi; s_c(end+1,1)=s_acc; %#ok<AGROW>
        end
    else
        % Arc (left/right by sign of R): dpsi = dS/R
        for k=1:n
            dpsi = dS / R;
            psi_mid = psi + 0.5*dpsi;
            x = x + dS*cos(psi_mid);
            y = y + dS*sin(psi_mid);
            psi = psi + dpsi;
            s_acc = s_acc + dS;
            x_c(end+1,1)=x; y_c(end+1,1)=y; psi_c(end+1,1)=psi; s_c(end+1,1)=s_acc; %#ok<AGROW>
        end
    end
end
end

function [s, v] = extract_sv(R)
% Find distance & speed arrays in R (common names)
cand_s = {'s','s_vec','s_all','S','s_cum','distance_m'};
cand_v = {'v','v_vec','v_all','V','speed','speed_mps'};

s = []; v = [];
for k=1:numel(cand_s), if isfield(R,cand_s{k}), s = R.(cand_s{k}); break; end, end
for k=1:numel(cand_v), if isfield(R,cand_v{k}), v = R.(cand_v{k}); break; end, end
if isempty(s) || isempty(v), error('R must contain s (m) and v (m/s) arrays.'); end
s = s(:); v = v(:);
end

