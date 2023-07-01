function [ErrorS,MSE_Error,JD] = FuncDiffJacobian_MapOnly_ICL(Map,Pose_GT_ICL,D,K,MODE_MAP,DepthScale)
    Size_i = Map.Size_i;
    Size_j = Map.Size_j;
    Scale = Map.Scale;
    Origin = Map.Origin;
    nD = length(D);
    [nv, nu] = size(D{1});
    KI = inv(K);
    
    cell_ErrorS = cell(1,nD);
    nPts = 0;
    cell_JDID1 = cell(1,nD);
    cell_JDID2 = cell(1,nD);
    cell_JDVal = cell(1,nD);

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

        XY3 = round((X(1:2, :) - Origin) / Scale) + 1;
        if strcmp(MODE_MAP,'CONTINUOUS')==1
            Md = Map.DgridG(XY3(2,:),XY3(1,:)); % interpolant of map grid 
        else
            Md = Map.DgridG(round(XY3(2,:)),round(XY3(1,:))); 
        end
        zk = X(3,:);
        Ek = Md - zk;
        cell_ErrorS{i} = Ek';

        u = XY3(1,:); % x of grid map
        v = XY3(2,:); % y of grid map
        u1 = fix(u); % Rounding to zero direction
        v1 = fix(v);

        nPtsk = length(zk);
        IDk = nPts+1:nPts+nPtsk; %ID numer from 1 to nPtsk
        nPts = nPts+nPtsk;

        dEdM = [(v1+1-v).*(u1+1-u);(v-v1).*(u1+1-u);(v1+1-v).*(u-u1);(v-v1).*(u-u1)];
        dEdMID2 = [Size_j*(v1-1)+u1;Size_j*v1+u1;Size_j*(v1-1)+u1+1;Size_j*v1+u1+1];
        dEdMID1 = repmat(IDk,4,1); 

        cell_JDID1{i} = reshape(dEdMID1',[],1);
        cell_JDID2{i} = reshape(dEdMID2',[],1);
        cell_JDVal{i} = reshape(dEdM',[],1);
    end
    ErrorS = vertcat(cell_ErrorS{:});
    JDID1 = vertcat(cell_JDID1{:});
    JDID2 = vertcat(cell_JDID2{:});
    JDVal = vertcat(cell_JDVal{:});

    ErrorS = double(ErrorS);
    Sum_Error = ErrorS'*ErrorS;
    MSE_Error = Sum_Error/(length(ErrorS));

    JDVal = double(JDVal);
    JDID1 = double(JDID1);
    JDID2 = double(JDID2);
    JD = sparse(JDID1,JDID2,JDVal,nPts,Size_i*Size_j);
end