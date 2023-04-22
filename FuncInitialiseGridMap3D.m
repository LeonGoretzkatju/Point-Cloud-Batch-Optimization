function Map = FuncInitialiseGridMap3D(Map, Pose, PointCloudData)
    Size_i = Map.Size_i;
    Size_j = Map.Size_j;
    Grid = Map.Grid;
    N = Map.N;
    Scale = Map.Scale;
    Origin = Map.Origin;
    nPointClouds = length(PointCloudData); % Number of point clouds
    for i = 1:nPointClouds
        posei = Pose{i};
        pointCloud_i = PointCloudData{i};
        xyi = pointCloud_i.Location(:,1:2)'; % Extract x and y values
        zi = pointCloud_i.Location(:,3)'; % Extract z values as the new Oddi
        P = [xyi;zi;ones(1,size(zi,2))];
        Pwi = inv(posei)*P;
        length_cow = size(Pwi,2);
        for j = 1:size(Pwi,2)
            XY3 = (Pwi(1:2,j)-Origin)*Scale;
            u = XY3(1,:);
            v = XY3(2,:);
            ui = fix(u);
            vi = fix(v);
            zi = Pwi(3,j);
            if (ui >= 1 && ui <= Size_i) && (vi >= 1 && vi <= Size_j)
                Grid(ui,vi) = zi;
            end
        end
    end
    Map.Grid = Grid;
    Map.N = N;
end