function [ErrorS,MSE_Error,Sum_Error,JP,IS,JD] = FuncDiffJacobian_All_Simu23(Map,Pose,D,K,MODE_MAP,MODE_DERIVATIVES)
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

        %   The mode of derivative 
        if strcmp (MODE_DERIVATIVES, 'PARSING')==1
            Map = FuncGradient_Matrix_M(XY3(2,:),XY3(1,:),Map); % calculate gradient of M and N, given(x,y and Map)
            dMdXY3 = [Map.DgridGu; Map.DgridGv]; % derivative of Grid Map with P_m 
        else
            if strcmp(MODE_MAP,'CONTINUOUS')==1
                dMdXY3 = [Map.DgridGu(XY3(2,:),XY3(1,:));Map.DgridGv(XY3(2,:),XY3(1,:))]; % derivative of Grid Map with x,y/N
                dMdXY3du = Map.DgridGu(XY3(2,:),XY3(1,:));
                dMdXY3dv = Map.DgridGv(XY3(2,:),XY3(1,:));
            else
                dMdXY3 = [Map.DgridGu(round_XY3)';Map.DgridGv(round_XY3)'];
            end
        end

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

        Rx = FuncRX_Simu23(Pose(i,3));
        Ry = FuncRY_Simu23(Pose(i,2));
        Rz = FuncRZ_Simu23(Pose(i,1));
        dRXdG = FuncdRXdG_Simu23(Pose(i,3));
        dRYdB = FuncdRYdB_Simu23(Pose(i,2));
        dRZdA = FuncdRZdA_Simu23(Pose(i,1));

        dXY3dA = ((Rx * Ry * dRZdA)' * x_local)/Scale;
        dXY3dB = ((Rx * dRYdB * Rz)' * x_local)/Scale;
        dXY3dG = ((dRXdG * Ry * Rz)' * x_local)/Scale;
        dXY3dTix = [1;0;0]/Scale;
        dXY3dTiy = [0;1;0]/Scale;
        dXY3dTiz = [0;0;1]/Scale;
%         dMdA = sum(dMdXY3.*dXY3dA(1:2,:));
%         dMdB = sum(dMdXY3.*dXY3dB(1:2,:));
%         dMdG = sum(dMdXY3.*dXY3dG(1:2,:));
        dMdA = dMdXY3du.*dXY3dA(1,:) + dMdXY3dv.*dXY3dA(2,:);
        dMdB = dMdXY3du.*dXY3dB(1,:) + dMdXY3dv.*dXY3dB(2,:);
        dMdG = dMdXY3du.*dXY3dG(1,:) + dMdXY3dv.*dXY3dG(2,:);

        dMdR = [dMdA;dMdB;dMdG];
        dMdTi = [dMdXY3/Scale;zeros(1,size(dMdXY3,2))];
        dMdP = [dMdR;dMdTi];

        xyzk_length = size(dMdXY3,2);
        dZdA = dXY3dA(3,:);
        dZdB = dXY3dB(3,:);
        dZdG = dXY3dG(3,:);
        dZdR = [dZdA;dZdB;dZdG];
        dZdTi = repmat([dXY3dTix(3,1);dXY3dTiy(3,1);dXY3dTiz(3,1)], 1, xyzk_length);
        dZdP = [dZdR;dZdTi];

        dEdP = dMdP-dZdP;
        dEdPID1 = repmat(IDk,6,1);
        dEdPID2 = repmat(6*(i-1)+1:6*i,nPtsk,1)';
    
        cell_JPID1{i} = reshape(dEdPID1',[],1);
        cell_JPID2{i} = reshape(dEdPID2',[],1);
        cell_JPVal{i} = reshape(dEdP',[],1);

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

    JPID1 = vertcat(cell_JPID1{:});
    JPID2 = vertcat(cell_JPID2{:});
    JPVal = vertcat(cell_JPVal{:});
    IS = GetInformationMfromS(ErrorS);
    JPVal = double(JPVal);
    JP = sparse(JPID1,JPID2,JPVal);
end