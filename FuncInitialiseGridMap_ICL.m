function Map = FuncInitialiseGridMap_ICL(Map,Pose_GT_ICL,D,K,DepthScale)
    Grid = Map.Grid;
    Scale = Map.Scale;
    Origin = Map.Origin;
    nD = numel(D);
    [nv, nu] = size(D{1});
    KI = inv(K);
    
    for i = 1:nD
        Di = D{i};
        T = euler_angles_translation_to_se3(Pose_GT_ICL(i,1:3),Pose_GT_ICL(i,4:6)');
        Ri = T(1:3,1:3);
        Ti = T(1:3,4);
        Di_col = reshape(Di, [], 1);
        [j, k] = meshgrid(1:nu, 1:nv);
        x = [j(:), k(:), ones(numel(j), 1)] .* Di_col;
        X = KI * x' / DepthScale;
        X = Ri*X + Ti;    
        ID = round((X(1:2, :) - Origin) / Scale) + 1;
%         ID = ID(:, ID(1, :) >= 1 & ID(1, :) <= size(Grid, 2) & ID(2, :) >= 1 & ID(2, :) <= size(Grid, 1));
        %find the X(3,:) that correspond to the valid ID
%         X = X(:, ID(1, :) >= 1 & ID(1, :) <= size(Grid, 2) & ID(2, :) >= 1 & ID(2, :) <= size(Grid, 1));
        GridIndices = sub2ind(size(Grid), ID(2, :), ID(1, :));
        Grid(GridIndices) = X(3,:);
    end
    Map.Grid = Grid;
end