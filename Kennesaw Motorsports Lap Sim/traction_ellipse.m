function Fx_cap = traction_ellipse(Fx_max, Fy_max, Fy_req)
Fx_cap = zeros(size(Fx_max));
for i=1:numel(Fx_max)
    if Fy_max(i) <= 0
        Fx_cap(i) = Fx_max(i);
    else
        usage = min(1, (Fy_req(i)/max(1e-6,Fy_max(i)))^2 );
        Fx_cap(i) = max(0, Fx_max(i) * sqrt( max(0, 1 - usage) ));
    end
end
end
