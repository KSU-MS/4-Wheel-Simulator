function plot_track_velocity_corners(track, R, opts)
% PLOT_TRACK_VELOCITY_CORNERS
% XY centerline colored by speed, with corner entry/min/exit and ΔV callouts.

if nargin < 3, opts = struct; end
ds     = getf(opts,'ds_plot_m',0.25);
cmap   = getf(opts,'colormap_name','turbo');
arrows = getf(opts,'show_arrows',true);
mph    = getf(opts,'units_mph',false);

% --- Normalize track & build XY ---
T = normalize_track(track);
[s_xy, x_xy, y_xy, psi_xy, seg_idx_xy] = centerline_xy(T, ds);

% --- Get distance & speed from R (tolerant) ---
[s_sim, v_sim] = extract_sv_any(R);
v_xy  = interp1(s_sim, v_sim, s_xy, 'linear', 'extrap');
v_lab = v_xy; lab_unit = 'm/s';
if mph, v_xy = v_xy*2.23693629; v_lab=v_xy; lab_unit='mph'; end

% --- Plot ---
figure('Color','w','Name','Velocity Heat Trace'); hold on; axis equal;
scatter(x_xy, y_xy, 14, v_xy, 'filled'); colormap(cmap);
cb = colorbar; ylabel(cb, ['Speed (' lab_unit ')']);
xlabel('X [m]'); ylabel('Y [m]'); title('Track Velocity Map'); grid on; box on;

if arrows
    step = max(1, round(3.0/ds));
    quiver(x_xy(1:step:end), y_xy(1:step:end), cos(psi_xy(1:step:end)), ...
           sin(psi_xy(1:step:end)), 0.5, 'Color',[0.2 0.2 0.2], 'LineWidth',0.8);
end

% --- Corner entry/min/exit/ΔV ---
corner_mask = strcmpi(T.type, "corner") | (T.R ~= 0);
for i = find(corner_mask').'
    in_seg = (seg_idx_xy == i); if ~any(in_seg), continue; end
    vseg   = v_lab(in_seg);
    ventry = vseg(1); vexit = vseg(end); vmin = min(vseg); dV = ventry - vmin;

    kseg = find(in_seg); kmid = kseg(round(numel(kseg)/2));
    text(x_xy(kmid), y_xy(kmid), ...
        sprintf('In %.1f\nMin %.1f\nOut %.1f\n\\DeltaV %.1f %s', ...
            ventry, vmin, vexit, dV, lab_unit), ...
        'Color','k','FontSize',8,'FontWeight','bold', ...
        'HorizontalAlignment','center','VerticalAlignment','bottom', ...
        'BackgroundColor','w','Margin',0.1,'EdgeColor',[0.2 0.2 0.2]);
end

% Console table
fprintf('\nCorner entry/min/exit speeds:\n');
fprintf('%6s  %10s  %10s  %10s  %10s  %10s  %10s\n','Idx','s_start[m]','s_end[m]', ...
        ['In[' lab_unit ']'],['Min[' lab_unit ']'],['Out[' lab_unit ']'],['ΔV[' lab_unit ']']);
s_cum = [0; cumsum(T.length_m(:))];
for i = find(corner_mask').'
    in_seg = (seg_idx_xy==i);
    if any(in_seg)
        vseg = v_lab(in_seg);
        fprintf('%6d  %10.1f  %10.1f  %10.1f  %10.1f  %10.1f  %10.1f\n', ...
            i, s_cum(i), s_cum(i+1), vseg(1), min(vseg), vseg(end), vseg(1)-min(vseg));
    end
end
end

% ================= helpers =================
function out = getf(s, f, d)
if isstruct(s) && isfield(s,f) && ~isempty(s.(f)), out = s.(f); else, out = d; end
end

function T = normalize_track(track)
if istable(track)
    L = pick(track, {'length_m','L','length','len','segment_length'});
    R = pick(track, {'signed_radius_m','radius_m','R','radius','corner_radius_m'}, 0*L);
    if any(strcmpi('turn_dir', track.Properties.VariableNames))
        dir = upper(string(track.turn_dir)); sgn = ones(size(R));
        sgn(dir=="L")=+1; sgn(dir=="R")=-1; R = abs(R).*sgn;
    end
    if any(strcmpi('type', track.Properties.VariableNames)), tp = lower(string(track.type));
    else, tp = repmat("straight", numel(L),1); tp(R~=0)="corner";
    end
else
    S = track;
    if isfield(S,'length_m'), L=S.length_m(:);
    elseif isfield(S,'L'),    L=S.L(:);
    elseif isfield(S,'length'), L=S.length(:);
    else, error('track needs length_m or L');
    end
    if     isfield(S,'signed_radius_m'), R=S.signed_radius_m(:);
    elseif isfield(S,'radius_m'),        R=S.radius_m(:);
    elseif isfield(S,'R'),               R=S.R(:);
    elseif isfield(S,'radius'),          R=S.radius(:);
    else, R=zeros(size(L));
    end
    if isfield(S,'turn_dir')
        dir = upper(string(S.turn_dir(:))); sgn = ones(size(R));
        sgn(dir=="L")=+1; sgn(dir=="R")=-1; R=abs(R).*sgn;
    end
    if isfield(S,'type'), tp=lower(string(S.type(:)));
    else, tp=repmat("straight",numel(L),1); tp(R~=0)="corner";
    end
end
R(~isfinite(R)) = 0;
T.length_m = L(:);
T.R        = R(:);
T.type     = tp(:);
end

function v = pick(T, names, def)
v = []; 
for k=1:numel(names)
    if any(strcmpi(names{k},T.Properties.VariableNames)), v = T.(names{k}); break; end
end
if isempty(v)
    if nargin<3, error('Missing track column: %s', strjoin(names,',')); else, v = def; end
end
end

function [s_c, x_c, y_c, psi_c, seg_idx] = centerline_xy(T, ds)
x=0; y=0; psi=0; N=numel(T.length_m);
s_acc=0; s_c=[]; x_c=[]; y_c=[]; psi_c=[]; seg_idx=[];
for i=1:N
    L=T.length_m(i); R=T.R(i); n=max(1,ceil(L/ds)); dS=L/n;
    for kk = 1:n         % <-- FIX: use a normal loop variable
        if abs(R)<1e-9
            x = x + dS*cos(psi); y = y + dS*sin(psi);
        else
            dpsi = dS/R; psi_mid = psi + 0.5*dpsi;
            x = x + dS*cos(psi_mid); y = y + dS*sin(psi_mid); psi = psi + dpsi;
        end
        s_acc = s_acc + dS; %#ok<NASGU>
        x_c(end+1,1)=x; y_c(end+1,1)=y; psi_c(end+1,1)=psi; %#ok<AGROW>
        s_c(end+1,1)=sum(T.length_m(1:i-1)) + dS*kk;        %#ok<AGROW>
        seg_idx(end+1,1)=i;                                 %#ok<AGROW>
    end
end
end

function [s, v] = extract_sv_any(R)
[s, v] = try_direct_fields(R);
if ~isempty(s) && ~isempty(v), s=s(:); v=v(:); return; end

[s, v] = try_time_speed(R);
if ~isempty(s) && ~isempty(v), s=s(:); v=v(:); return; end

[s, v] = try_cells_or_nested(R);
if ~isempty(s) && ~isempty(v), s=s(:); v=v(:); return; end

[s, v] = try_from_accel(R);
if ~isempty(s) && ~isempty(v), s=s(:); v=v(:); return; end

[s, v] = heuristic_pick(R);
if ~isempty(s) && ~isempty(v), s=s(:); v=v(:); return; end

error('R must contain distance+speed, or time+speed to reconstruct distance.');
end

function [s, v] = try_direct_fields(R)
s = []; v = [];
cand_s = {'s','s_vec','s_all','S','distance','distance_m'};
cand_v = {'v','v_vec','v_all','V','speed','speed_mps','vx'};
for c = cand_s, if isfield(R,c{1}), s = R.(c{1}); break; end, end
for c = cand_v, if isfield(R,c{1}), v = R.(c{1}); break; end, end
end

function [s, v] = try_time_speed(R)
s = []; v = [];
tcands = {'t','t_vec','time','time_s'};
for tname = tcands
    if isfield(R, tname{1})
        t = R.(tname{1});
        v = find_speed(R);
        if ~isempty(v) && numel(v)==numel(t)
            s = cumtrapz(t(:), v(:)); return;
        end
    end
end
end

function v = find_speed(R)
vcands = {'v','v_vec','speed','speed_mps','vx'};
v = [];
for c = vcands
    if isfield(R, c{1}), v = R.(c{1}); return; end
end
end

function [s, v] = try_cells_or_nested(R)
s = []; v = [];
fn = fieldnames(R);
cell_v = {}; cell_s = {}; cell_t = {};
for k=1:numel(fn)
    val = R.(fn{k});
    if iscell(val)
        if contains(lower(fn{k}),'v'), cell_v = val; end
        if contains(lower(fn{k}),'s'), cell_s = val; end
        if contains(lower(fn{k}),'t'), cell_t = val; end
    elseif isstruct(val)
        try, [s,v] = extract_sv_any(val); return; catch, end
    end
end
if ~isempty(cell_v)
    v = vertcat(cell_v{:});
    if ~isempty(cell_s)
        s = vertcat(cell_s{:}); return;
    elseif ~isempty(cell_t)
        t = vertcat(cell_t{:}); s = cumtrapz(t, v); return;
    end
end
end

function [s, v] = try_from_accel(R)
s = []; v = [];
tcands = {'t','t_vec','time','time_s'};
acands = {'ax','ax_vec','a_long','a_x'};
t = []; ax = [];
for tn = tcands, if isfield(R, tn{1}), t = R.(tn{1}); break; end, end
for an = acands, if isfield(R, an{1}), ax = R.(an{1}); break; end, end
if ~isempty(t) && ~isempty(ax) && numel(t)==numel(ax)
    v = cumtrapz(t(:), ax(:)); s = cumtrapz(t(:), v); 
end
end

function [s, v] = heuristic_pick(R)
s = []; v = [];
bestS = 0; bestV = 0;
fn = fieldnames(R);
for i=1:numel(fn)
    val = R.(fn{i});
    if isnumeric(val) && isvector(val) && numel(val) > 10
        vec = val(:);
        scoreV = double(all(isfinite(vec))) + 2*double(median(vec) > 0 & median(vec) < 100);
        scoreS = double(all(isfinite(vec))) + 2*double(issorted(vec)) + double(max(vec) > 10);
        if scoreV > bestV, v = vec; bestV = scoreV; end
        if scoreS > bestS, s = vec; bestS = scoreS; end
    end
end
if isempty(s)
    t = [];
    for i=1:numel(fn)
        val = R.(fn{i});
        if isnumeric(val) && isvector(val) && numel(val) > 10 && ...
           (contains(lower(fn{i}),'t') || contains(lower(fn{i}),'time'))
            t = val(:); break;
        end
    end
    if ~isempty(t) && ~isempty(v) && numel(t)==numel(v)
        s = cumtrapz(t, v);
    end
end
end
