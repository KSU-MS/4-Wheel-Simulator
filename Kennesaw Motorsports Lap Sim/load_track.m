function track = load_track(filename)
T = readtable(filename, 'TextType','string');
track.type   = string(T.type);
track.radius = double(T.radius_m);
track.length = double(T.length_m);
end
