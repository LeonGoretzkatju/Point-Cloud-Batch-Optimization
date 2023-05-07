function global_point_clouds = FuncCreateGlobalMapPoints(Pose, Downsample_pointclouds)
    n_downsample_points = length(Downsample_pointclouds);
    global_point_clouds = pointCloud(zeros(0, 3));
    for i = 1:n_downsample_points
        posei = Pose{i};
        pose_i_t = posei(1:3,4);
        pose_i_t_meter = pose_i_t/1000.0;
        posei(1:3,4) = pose_i_t_meter;
        pointCloud_i = Downsample_pointclouds{i};
        xyi = pointCloud_i.Location(:,1:2)'; % Extract x and y values
        zi = pointCloud_i.Location(:,3)'; % Extract z values as the new Oddi
        P = [xyi;zi;ones(1,size(zi,2))];
        Pwi = inv(posei)*P;
        Pwi_T = Pwi';
        Pwi_T_xyz = [Pwi_T(:,1), Pwi_T(:,2), Pwi_T(:,3)];
        pointCloud_wi = pointCloud(Pwi_T_xyz);
        global_point_clouds = pcmerge(global_point_clouds, pointCloud_wi, 0.001);
    end
end