% vertices: 3 by 3, each row is a point
function points = trimesh2pc(p1, p2, p3, face_color)

a = p2 - p3;
b = p1 - p2;
h = norm(cross(a,b)) / norm(a);

UNIT = 0.01;
numpts = int32(h/UNIT);
side1 = [linspace(p1(1), p2(1), numpts);
        linspace(p1(2), p2(2), numpts);
        linspace(p1(3), p2(3), numpts)];

side2 = [linspace(p1(1), p3(1), numpts);
        linspace(p1(2), p3(2), numpts);
        linspace(p1(3), p3(3), numpts)];

points = [];

for i=1:numpts
    hpts = unitspace(side1(:,i), side2(:,i), UNIT);
    points = [points; hpts];
end

size_points = size(points);
points = [points repmat(face_color, [size_points(1) ,1])];


% 
%     function shaked_pt = shakept(origin, victim_pt)
%         dist = norm(victim_pt - origin);
%         dev = normrnd(0, (dist^2) * 0.0025);
%         shaked_pt = (victim_pt - origin) / dist * (dist + dev) + origin;
%     end
% origin = [-5 -5 -5]
% points_size = size(points);
% shaked_points = zeros(points_size);
% for i=1:points_size(1)
%     shaked_points(i,:) = shakept(origin, points(i,:));
% end
% plot3(shaked_points(:,1),shaked_points(:,2),shaked_points(:,3),'.');
% 
% points = shaked_points;

% h = plot3(points(:,1),points(:,2),points(:,3),'.');
end