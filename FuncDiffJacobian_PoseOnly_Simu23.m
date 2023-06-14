function [ErrorS,MSE_Error,JP] = FuncDiffJacobian_PoseOnly_Simu23(Map,Pose,D,K,MODE_MAP)
    Size_i = Map.Size_i;
    Size_j = Map.Size_j;
    Scale = Map.Scale;
    Origin = Map.Origin;
    nD = length(D);
    [nv, nu] = size(D{1});
    KI = inv(K);
    
    cell_ErrorS = cell(1,nD);
    nPts = 0;
    cell_JPID1 = cell(1,nD);
    cell_JPID2 = cell(1,nD);
    cell_JPVal = cell(1,nD);

    for i = 1:nD
        Di = D{i};
        Ri = RMatrixYPR22(Pose(i,1),Pose(i,2),Pose(i,3));
        Ti = Pose(i, 4:6)';
        
        Di_col = reshape(Di, [], 1);
        [j, k] = meshgrid(1:nu, 1:nv);
        x = [j(:), k(:), ones(numel(j), 1)] .* Di_col;
        x_local = KI * x';
        X = Ri' * x_local + Ti;

        XY3 = round((X(1:2, :) - Origin) / Scale) + 1;
        if strcmp(MODE_MAP,'CONTINUOUS')==1
            Md = Map.DgridG(XY3(2,:),XY3(1,:)); % interpolant of map grid 
        end
        zk = X(3,:);
        Ek = Md - zk;
        cell_ErrorS{i} = Ek';
        dMdXY3 = [Map.DgridGu(XY3(2,:),XY3(1,:));Map.DgridGv(XY3(2,:),XY3(1,:))];

        u = XY3(1,:); % x of grid map
        v = XY3(2,:); % y of grid map
        u1 = fix(u); % Rounding to zero direction
        v1 = fix(v);

        nPtsk = length(zk);
        IDk = nPts+1:nPts+nPtsk; %ID numer from 1 to nPtsk
        nPts = nPts+nPtsk;
    end
end