function [Map,ID] = FuncInitialiseGridMap_Simu23(Map,Pose,D,K)
    Grid = Map.Grid;
    Scale = Map.Scale;
    Origin = Map.Origin;
    nD = length(D);
    [nv, nu] = size(D{1});
    KI = inv(K);
    X_all = zeros(3, nv * nu * nD);
    Local_Scan_Set = cell(1, nD);
    Local_Scan = zeros(3, nv * nu);
    
    for i = 1:nD
        Di = D{i};
        Ri = RMatrixYPR22(Pose(i,1),Pose(i,2),Pose(i,3));
        Ti = Pose(i, 4:6)';
        
        Di_col = reshape(Di, [], 1);
        [j, k] = meshgrid(1:nu, 1:nv);
        x = [j(:), k(:), ones(numel(j), 1)] .* Di_col;
        X = Ri' * KI * x' + Ti;

        ID = round((X(1:2, :) - Origin) / Scale) + 1;
        GridIndices = sub2ind(size(Grid), ID(2, :), ID(1, :));
        Grid(GridIndices) = X(3,:);
        
        X_all(:, ((i - 1) * nv * nu + 1):(i * nv * nu)) = X;
        Local_Scan(:, 1:(nv * nu)) = X;
        Local_Scan_Set{i} = Local_Scan;
    end
    Map.Grid = Grid;
end