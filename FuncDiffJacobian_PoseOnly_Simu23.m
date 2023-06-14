function [ErrorS,MSE_Error,JP,IS] = FuncDiffJacobian_PoseOnly_Simu23(Map,Pose,D,K,MODE_MAP)
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
        Rx = FuncRX_Simu23(Pose(i,3));
        Ry = FuncRY_Simu23(Pose(i,2));
        Rz = FuncRZ_Simu23(Pose(i,1));
        dRXdG = FuncdRXdG_Simu23(Pose(i,3));
        dRYdB = FuncdRYdB_Simu23(Pose(i,2));
        dRZdA = FuncdRZdA_Simu23(Pose(i,1));

        dXY3dA = ((Rx * Ry * dRZdA)' * x_local + Ti)/Scale;
        dXY3dB = ((Rx * dRYdB * Rz)' * x_local + Ti)/Scale;
        dXY3dG = ((dRXdG * Ry * Rz)' * x_local + Ti)/Scale;
        dXY3dTix = [1;0;0]/Scale;
        dXY3dTiy = [0;1;0]/Scale;
        dXY3dTiz = [0;0;1]/Scale;
        dMdtx = sum(dMdXY3 .* dXY3dTix(1:2,1), 1);
        dMdty = sum(dMdXY3 .* dXY3dTiy(1:2,1), 1);
        dMdtz = sum(dMdXY3 .* dXY3dTiz(1:2,1), 1);
        dMdA = sum(dMdXY3.*dXY3dA(1:2,:));
        dMdB = sum(dMdXY3.*dXY3dB(1:2,:));
        dMdG = sum(dMdXY3.*dXY3dG(1:2,:));
        dMdR = [dMdA;dMdB;dMdG];
        dMdTi = [dMdtx;dMdty;dMdtz];
        dMdP = [dMdR;dMdTi];

        xyzk_length = size(dMdXY3,2);
        dZdA = dXY3dA(3,:);
        dZdB = dXY3dB(3,:);
        dZdG = dXY3dG(3,:);
        dZdR = [dZdA;dZdB;dZdG];
        dZdTi = repmat([dXY3dTix(3,1);dXY3dTiy(3,1);dXY3dTiz(3,1)], 1, xyzk_length);
        dZdP = [dZdR;dZdTi];

        dEdP = dMdP-dZdP;

        nPtsk = length(zk);
        IDk = nPts+1:nPts+nPtsk; %ID numer from 1 to nPtsk
        nPts = nPts+nPtsk;
        dEdPID1 = repmat(IDk,6,1);
        dEdPID2 = repmat(6*(i-1)+1:6*i,nPtsk,1)';
    
        cell_JPID1{i} = reshape(dEdPID1',[],1);
        cell_JPID2{i} = reshape(dEdPID2',[],1);
        cell_JPVal{i} = reshape(dEdP',[],1);
    end
    ErrorS = vertcat(cell_ErrorS{:});
    JPID1 = vertcat(cell_JPID1{:});
    JPID2 = vertcat(cell_JPID2{:});
    JPVal = vertcat(cell_JPVal{:});
    ErrorS = double(ErrorS);
    IS = GetInformationMfromS(ErrorS);
    Sum_Error = ErrorS'*IS*ErrorS;
    MSE_Error = Sum_Error/(length(ErrorS));
    JPVal = double(JPVal);
    JP = sparse(JPID1,JPID2,JPVal);
end