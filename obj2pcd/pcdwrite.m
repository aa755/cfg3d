function pcdwrite(filename, pc)
pcsize = size(pc);
fid = fopen(filename, 'w');
fprintf(fid, '# .PCD v.7 - Point Cloud Data file format\n');
fprintf(fid,'FIELDS x y z rgb\n');
fprintf(fid,'SIZE 4 4 4 4\n');
fprintf(fid,'TYPE F F F F\n');
fprintf(fid,'COUNT 1 1 1 1\n');
fprintf(fid,'WIDTH %d\n', pcsize(1));
fprintf(fid,'HEIGHT 1\n');
fprintf(fid,'VIEWPOINT 0 0 0 1 0 0 0\n');
fprintf(fid,'POINTS %d\n', pcsize(1));
fprintf(fid,'DATA ascii\n');
for i=1:pcsize(1)
    fprintf(fid,'%f %f %f %e\n', pc(i,:));
end
fclose(fid);

% function pcdwrite(filename, pc)
% pcsize = size(pc);
% fid = fopen(filename, 'w');
% fprintf(fid, '# .PCD v.7 - Point Cloud Data file format\n');
% fprintf(fid,'FIELDS x y z\n');
% fprintf(fid,'SIZE 4 4 4\n');
% fprintf(fid,'TYPE F F F\n');
% fprintf(fid,'COUNT 1 1 1\n');
% fprintf(fid,'WIDTH %d\n', pcsize(1));
% fprintf(fid,'HEIGHT 1\n');
% fprintf(fid,'VIEWPOINT 0 0 0 1 0 0 0\n');
% fprintf(fid,'POINTS %d\n', pcsize(1));
% fprintf(fid,'DATA ascii\n');
% for i=1:pcsize(1)
%     fprintf(fid,'%f %f %f\n', pc(i,:));
% end
% fclose(fid);