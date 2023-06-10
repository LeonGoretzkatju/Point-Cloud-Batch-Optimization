function [ErrorS,MSE_Error,JP,IS,JD] = FuncDiffJacobianStepTest_New_Vector(Map,Pose_Vector,Scan,MODE_MAP)
    Size_i = Map.Size_i;
    Size_j = Map.Size_j;
    Scale = Map.Scale;
    Origin = Map.Origin;
    
    nD = length(Scan); % the timestamp
    nPts = 0;
    cell_ErrorS = cell(1,nD);
    
    cell_JPID1 = cell(1,nD);
    cell_JPID2 = cell(1,nD);
    cell_JPVal = cell(1,nD);
    cell_JDID1 = cell(1,nD);
    cell_JDID2 = cell(1,nD);
    cell_JDVal = cell(1,nD);
    for k = 1:nD
        yaw = Pose_Vector(k,4);
        pitch = Pose_Vector(k,5);
        roll = Pose_Vector(k,6);
        Rz = FuncRZ(yaw);
        Ry = FuncRY(pitch);
        Rx = FuncRX(roll);
        Scan_k = Scan{k};
        xyk = Scan_k.Location(:,1:2)'; % Extract x and y values
        zk = Scan_k.Location(:,3)'; % Extract z values as the new Oddi
        xyzk = [xyk;zk];

        Rkw = Rz*Ry*Rx;
        tkw = Pose_Vector(k,1:3)';
        Pwk = Rkw'*(xyzk-tkw);
        XY3 = (Pwk(1:2,:)-Origin)/Scale+1;    
        if strcmp(MODE_MAP,'CONTINUOUS')==1
            Md = Map.DgridG(XY3(2,:),XY3(1,:)); % interpolant of map grid 
        end
        Ek = Md - zk;
%         Ek = zk - Md;
        cell_ErrorS{k} = Ek';
        euler_angles = [yaw,pitch,roll];
        dMdXY3 = [Map.DgridGu(XY3(2,:),XY3(1,:));Map.DgridGv(XY3(2,:),XY3(1,:))];
        Ryaw = FuncRZ(euler_angles(1));
        Rpitch = FuncRY(euler_angles(2));
        Rroll = FuncRX(euler_angles(3));
    
        [dRyaw,dRpitch,dRroll] = DiffRotationMatrix(euler_angles);
%         dXY3dRyaw = Rroll'*Rpitch'*(dRyaw)'*(xyzk-tkw)/Scale;
%         dXY3dRpitch = Rroll'*(dRpitch)'*Ryaw'*(xyzk-tkw)/Scale;
%         dXY3dRroll = (dRroll)'*Rpitch'*Ryaw'*(xyzk-tkw)/Scale;
        dXY3dRyaw = (dRyaw*Rpitch*Rroll)'*(xyzk-tkw)/Scale;
        dXY3dRpitch = (Ryaw*dRpitch*Rroll)'*(xyzk-tkw)/Scale;
        dXY3dRroll = (Ryaw*Rpitch*dRroll)'*(xyzk-tkw)/Scale;
        Rkw_T = Rkw';
        dXY3dtx = -Rkw_T(1:3,1)/Scale;
        dXY3dty = -Rkw_T(1:3,2)/Scale;
        dXY3dtz = -Rkw_T(1:3,3)/Scale;

        dMdRyaw = sum(dMdXY3.*dXY3dRyaw(1:2,:));
        dMdRpitch = sum(dMdXY3.*dXY3dRpitch(1:2,:));
        dMdRroll = sum(dMdXY3.*dXY3dRroll(1:2,:));
        dMdR = [dMdRyaw;dMdRpitch;dMdRroll];

        xyzk_length = size(xyzk,2);
        dMdtx = sum(dMdXY3 .* dXY3dtx(1:2,1), 1);
        dMdty = sum(dMdXY3 .* dXY3dty(1:2,1), 1);
        dMdtz = sum(dMdXY3 .* dXY3dtz(1:2,1), 1);
        dMdT = [dMdtx;dMdty;dMdtz];
        dMdP = [dMdT;dMdR];
        
        dZdRyaw = dXY3dRyaw(3,:);
        dZdRpitch = dXY3dRpitch(3,:);
        dZdRroll = dXY3dRroll(3,:);
        dZdR = [dZdRyaw;dZdRpitch;dZdRroll];
        dZdT = repmat([dXY3dtx(3,1);dXY3dty(3,1);dXY3dtz(3,1)], 1, xyzk_length);
        dZdP = [dZdT;dZdR];


        dEdP = dMdP-dZdP;
%         dEdP = dZdP - dMdP;
%         dEdP = dMdP;
        nPtsk = length(zk);
        IDk = nPts+1:nPts+nPtsk; %ID numer from 1 to nPtsk
        nPts = nPts+nPtsk;
        dEdPID1 = repmat(IDk,6,1);
        dEdPID2 = repmat(6*(k-1)+1:6*k,nPtsk,1)';
    
        cell_JPID1{k} = reshape(dEdPID1',[],1);
        cell_JPID2{k} = reshape(dEdPID2',[],1);
        cell_JPVal{k} = reshape(dEdP',[],1);
            
%         u = max(XY3(1,:), 1); % x of grid map
%         v = max(XY3(2,:), 1); % y of grid map
        u = XY3(1,:); % x of grid map
        v = XY3(2,:); % y of grid map
        u1 = fix(u); % Rounding to zero direction
        v1 = fix(v);
        
        %limit the range of u1 at [1,Size_j-1]
        u1(u1<1) = 1;
        u1(u1>Size_j-1) = Size_j-1;
        %limit the range of v1 at [1,Size_i-1]
        v1(v1<1) = 1;
        v1(v1>Size_i-1) = Size_i-1;
        
        dEdM = [(v1+1-v).*(u1+1-u);(v-v1).*(u1+1-u);(v1+1-v).*(u-u1);(v-v1).*(u-u1)];
        dEdMID2 = [Size_j*(v1-1)+u1;Size_j*v1+u1;Size_j*(v1-1)+u1+1;Size_j*v1+u1+1];
        dEdMID1 = repmat(IDk,4,1);
    
        cell_JDID1{k} = reshape(dEdMID1',[],1);
        cell_JDID2{k} = reshape(dEdMID2',[],1);
        cell_JDVal{k} = reshape(dEdM',[],1);
    end
    ErrorS = vertcat(cell_ErrorS{:});
    JPID1 = vertcat(cell_JPID1{:});
    JPID2 = vertcat(cell_JPID2{:});
    JPVal = vertcat(cell_JPVal{:});

    JDID1 = vertcat(cell_JDID1{:});
    JDID2 = vertcat(cell_JDID2{:});
    JDVal = vertcat(cell_JDVal{:});

    ErrorS = double(ErrorS);
    IS = GetInformationMfromS(ErrorS);
    Sum_Error = ErrorS'*IS*ErrorS;
    MSE_Error = Sum_Error/(length(ErrorS));
    JPVal = double(JPVal);
    JP = sparse(JPID1,JPID2,JPVal);

    JDVal = double(JDVal);
    JDID1 = double(JDID1);
    JDID2 = double(JDID2);
    JD = sparse(JDID1,JDID2,JDVal,nPts,Size_i*Size_j);
end