function motor = load_motor(filename)
T = readtable(filename, 'TextType','string');
for i = 1:height(T)
    key = T.param(i);
    val = T.value(i);
    vnum = str2double(val);
    if ~isnan(vnum)
        motor.(key) = vnum;
    else
        motor.(key) = val;
    end
end
end
