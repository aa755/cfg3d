function points = unitspace(ps, pe, step)
% ps = [0 0 0];
% pe = [1 1 1];
% step = 0.1;

    function side = oneside_unitspace(central_pt, p1)
        leng = norm(p1 - central_pt);
        numpts = floor(leng / step);
        if numpts == 0
            side = [];
            return
        end
        endpt = central_pt + (p1 - central_pt)/leng*step*numpts;
        side = [linspace(central_pt(1), endpt(1), numpts);
            linspace(central_pt(2), endpt(2), numpts);
            linspace(central_pt(3), endpt(3), numpts)];
        
        side = side';
    end

points = [oneside_unitspace(ps,pe)];
% center_pt = (ps + pe) / 2;
% points = oneside_unitspace(center_pt, ps);
% points = [points; oneside_unitspace(center_pt, pe)];
% plot3(points(:,1),points(:,2),points(:,3),'.');

end