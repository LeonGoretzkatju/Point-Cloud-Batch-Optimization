function Map = FuncInitialiseGridMap3D_New_Vector(Map, Pose_Vector, PointCloudData)
    Size_i = Map.Size_i;
    Size_j = Map.Size_j;
    Grid = Map.Grid;
    Scale = Map.Scale;
    Origin = Map.Origin;
    nPointClouds = length(PointCloudData); % Number of point clouds
    for i = 1:nPointClouds
        yaw = Pose_Vector(i,4);
        pitch = Pose_Vector(i,5);
        roll = Pose_Vector(i,6);
        Rz = FuncRZ(yaw);
        Ry = FuncRY(pitch);
        Rx = FuncRX(roll);
        Riw = Rz*Ry*Rx;
        tiw = Pose_Vector(i,1:3)';
        pointCloud_i = PointCloudData{i};
        xyi = pointCloud_i.Location(:,1:2)'; % Extract x and y values
        zi = pointCloud_i.Location(:,3)'; % Extract z values as the new Oddi
        xyzi = [xyi;zi];
        Pwi = Riw'*(xyzi-tiw);
        for j = 1:size(Pwi,2)
            XY3 = (Pwi(1:2,j)-Origin)/Scale+1;
            u = XY3(1,:);
            v = XY3(2,:);
            ui = fix(u);
            vi = fix(v);
            zi = Pwi(3,j);
            if (ui >= 1 && ui <= Size_j) && (vi >= 1 && vi <= Size_i)
                Grid(vi,ui) = zi;
            end
        end
    end
    Map.Grid = Grid;
end