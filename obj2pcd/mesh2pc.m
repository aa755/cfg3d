function mesh2pc(filename, num_positions, height, radius, with_noise)
DEBUG = 0;
%sprintf('C:\Users\abhishek\Desktop\sketchup\%s',filename)
[OBJ, units] = read_wobj(sprintf('%s',filename));
OBJ.vertices = OBJ.vertices * units;
FV.vertices=OBJ.vertices;
points = [];
figure;
hold on

material_map = containers.Map();
for i=1:length(OBJ.material)
    if strcmp(OBJ.material(i).type, 'newmtl')
        mname = OBJ.material(i).data;
    elseif strcmp(OBJ.material(i).type, 'Kd')
        % urgb = r<<16 | g<<8 | b
        rgb_data = OBJ.material(i).data*256;
        ur = uint32(rgb_data(1));
        ug = uint32(rgb_data(2));
        ub = uint32(rgb_data(3));
        color = bitshift(ur, 16) + bitshift(ug,8) + ub;
        color = typecast(uint32(color), 'single');
        material_map(mname) = color;
    end
end

for i=1:length(OBJ.objects)
    if strcmp(OBJ.objects(i).type, 'usemtl')
        face_color = material_map(OBJ.objects(i).data);
    elseif strcmp(OBJ.objects(i).type, 'f')
        FV.faces = OBJ.objects(i).data.vertices;
        face = OBJ.objects(i).data.vertices;
        face_size = size(face);
        for k=1:face_size(1)
            vts = face(k,1:3);
            p1 = OBJ.vertices(vts(1),1:3);
            p2 = OBJ.vertices(vts(2),1:3);
            p3 = OBJ.vertices(vts(3),1:3);
            points = [points; trimesh2pc(p1, p2, p3, face_color)];
        end
        % patch(FV,'facecolor',[1 0 0]); camlight
    end
end

if DEBUG
    p1 = [1 0 0];
    p2 = [0 1 0];
    p3 = [0 0 1];
    points = trimesh2pc(p1, p2, p3, 0.0);
end

colors = hsv(num_positions);
    function shaked_pt = shakept(origins, victim_pt)
        df = origins - repmat(victim_pt, [num_positions,1]);
        [closest_dist, closest_index] = min(sqrt(sum(df.^2,2)));
        origin = origins(closest_index,1:3);
        dev = normrnd(0, (closest_dist^2) * 0.0025);
        shaked_pt = (victim_pt - origin) / closest_dist * (closest_dist + dev) + origin;
        plot3(shaked_pt(1),shaked_pt(2),shaked_pt(3),'Color',colors(closest_index,:),'Marker','.','MarkerSize',4);
    end

origins = zeros(num_positions, 3);
scene_center = mean(points);
for i=1:num_positions
    x = scene_center(1) + radius*cos(2*i*pi/num_positions);
    y = scene_center(2) + radius*sin(2*i*pi/num_positions);
    z = height;
    origins(i, 1:3) = [x y z];
end
plot3(origins(:,1),origins(:,2),origins(:,3),'+');
    
if with_noise
    points_size = size(points);
    shaked_points = zeros(points_size);
    for i=1:points_size(1)
        shaked_points(i,1:3) = shakept(origins, points(i,1:3));
    end
    points(:,1:3) = shaked_points(:,1:3);
else
    
    plot3(points(:,1),points(:,2),points(:,3),'.');
end

h = gcf;
pcdwrite(sprintf('%s.pcd',filename), points);
csvwrite(sprintf('%s.csv',filename), points);
saveas(h, sprintf('%s.fig',filename));
saveas(h, sprintf('%s.png',filename));

end