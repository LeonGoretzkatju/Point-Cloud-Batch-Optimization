function Low_PointCloud  = FuncDownsamplePoints(PointCloud, Rate)

if Rate == 1
    Low_PointCloud = PointCloud;
else
    xyz = PointCloud.Location;
    x = xyz(:, 1);
    y = xyz(:, 2);
    z = xyz(:, 3);

    num_points = size(xyz, 1);
    index = 1:Rate:num_points;
    low_xyz = [x(index), y(index), z(index)];
    Low_PointCloud = pointCloud(low_xyz);
end
end