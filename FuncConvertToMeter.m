function pc_meters = FuncConvertToMeter(pc_mm)
x = pc_mm.Location(:,1);
y = pc_mm.Location(:,2);
z = pc_mm.Location(:,3);
x_new = x/1000.0;
y_new = y/1000.0;
z_new = z/1000.0;
% Create a new pointCloud object with the converted coordinates
xyz_meters = [x_new, y_new, z_new];
pc_meters = pointCloud(xyz_meters);
end