function veh = load_vehicle(filename)
T = readtable(filename, 'TextType','string');
for i = 1:height(T)
    key = T.param(i);
    val = T.value(i);
    % convert "true/false" strings to logicals
    if strcmpi(val,"true")
        veh.(key) = true;
    elseif strcmpi(val,"false")
        veh.(key) = false;
    else
        vnum = str2double(val);
        if ~isnan(vnum)
            veh.(key) = vnum;
        else
            veh.(key) = val;
        end
    end
end
% derive tire struct
veh.tire.mu_long_0 = veh.mu_long_0;
veh.tire.mu_lat_0  = veh.mu_lat_0;
veh.tire.Fz_ref    = veh.fz_ref_N;
veh.tire.load_sens = veh.load_sens_exp;
end
