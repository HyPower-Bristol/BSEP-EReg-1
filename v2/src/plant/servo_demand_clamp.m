function y = fcn(u)
if u < 0
    y = 0;
else 
    if u > 180
        y = 180;
    else
        y = u;
    end
end
