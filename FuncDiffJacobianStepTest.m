function [ErrorS,ErrorO,Sum_Error,MSE_Error,IS,IO,JP,JD,JO] = FuncDiffJacobianStepTest(Map,Pose,Odom,Scan,MODE_DERIVATIVES,MODE_MAP)

Size_i = Map.Size_i;
Size_j = Map.Size_j;
Scale = Map.Scale;
Origin = Map.Origin;

nD = length(Scan); % the timestamp
nPts = 0;
Cnti = 1;
cell_ErrorS = cell(1,nD);
cell_ErrorO = cell(1,nD);

cell_JPID1 = cell(1,nD);
cell_JPID2 = cell(1,nD);
cell_JPVal = cell(1,nD);
cell_JDID1 = cell(1,nD);
cell_JDID2 = cell(1,nD);
cell_JDVal = cell(1,nD);

cell_JOID1 = cell(1,nD);
cell_JOID2 = cell(1,nD);
cell_JOVal = cell(1,nD);
for k = 1:nD
    posek = Pose{k};
    pose_k_t = posek(1:3,4);
    pose_k_t_meter = pose_k_t/1000.0;
    posek(1:3,4) = pose_k_t_meter;
    [euler_angles, ~] = se3_to_euler_angles_translation(posek);
    Scan_k = Scan{k};
    xyk = Scan_k.Location(:,1:2)'; % Extract x and y values
    zk = Scan_k.Location(:,3)'; % Extract z values as the new Oddi
    XY1_Span = [xyk;zk];
    P = [xyk;zk;ones(1,size(zk,2))];
    Pwk = inv(posek)*P;
    XY3 = (Pwk(1:2,:)-Origin)/Scale+1;    
    if strcmp(MODE_MAP,'CONTINUOUS')==1
        Md = Map.DgridG(XY3(1,:),XY3(2,:)); % interpolant of map grid 
    else
        Md = Map.DgridG(round(XY3(1,:)),round(XY3(2,:)));
    end
    Ek = Md - zk;
    cell_ErrorS{k} = Ek';
    
%   The mode of derivative 
    if strcmp (MODE_DERIVATIVES, 'PARSING')==1
        Map = FuncGradient_Matrix_M(XY3(2,:),XY3(1,:),Map); % calculate gradient of M and N, given(x,y and Map)
        dMdXY3 = [Map.DgridGu; Map.DgridGv]; % derivative of Grid Map with P_m 
        
    else
        if strcmp(MODE_MAP,'CONTINUOUS')==1
%             dMdXY3 = [Map.DgridGu(XY3(2,:),XY3(1,:));Map.DgridGv(XY3(2,:),XY3(1,:))]; % derivative of Grid Map with x,y/N
            dMdXY3 = [Map.DgridGv(XY3(1,:),XY3(2,:));Map.DgridGu(XY3(1,:),XY3(2,:))];
        else
            dMdXY3 = [Map.DgridGu(round_XY3)';Map.DgridGv(round_XY3)'];
        end
    end
    Ryaw = FuncRZ(euler_angles(1));
    Rpitch = FuncRY(euler_angles(2));
    Rroll = FuncRX(euler_angles(3));

    [dRyaw,dRpitch,dRroll] = DiffRotationMatrix(euler_angles);
    dXY3dRyaw = (dRyaw*Rpitch*Rroll)'*XY1_Span/Scale;
    dXY3dRpitch = (Ryaw*dRpitch*Rroll)'*XY1_Span/Scale;
    dXY3dRroll = (Ryaw*Rpitch*dRroll)'*XY1_Span/Scale;
    dMdRyaw = sum(dMdXY3.*dXY3dRyaw(1:2,:));
    dMdRpitch = sum(dMdXY3.*dXY3dRpitch(1:2,:));
    dMdRroll = sum(dMdXY3.*dXY3dRroll(1:2,:));
    dZdRyaw = dXY3dRyaw(3,:);
    dZdRpitch = dXY3dRpitch(3,:);
    dZdRroll = dXY3dRroll(3,:);
    dZdR = [dZdRyaw;dZdRpitch;dZdRroll];
    dZdT = [zeros(1,size(dMdXY3,2));...
        zeros(1,size(dMdXY3,2));ones(1,size(dMdXY3,2))]/Scale;
    dMdR = [dMdRyaw;dMdRpitch;dMdRroll];
    dMdT = [dMdXY3(1,:);dMdXY3(2,:);zeros(1,size(dMdXY3,2))]/Scale;
    dMdP = [dMdT;dMdR];
    dZdP = [dZdT;dZdR];
%     dEdP = dZdP-dMdP;
    dEdP = dMdP-dZdP;

    nPtsk = length(zk);
    IDk = nPts+1:nPts+nPtsk; %ID numer from 1 to nPtsk
    nPts = nPts+nPtsk;
    
    dEdPID1 = repmat(IDk,6,1);
    dEdPID2 = repmat(6*(k-1)+1:6*k,nPtsk,1)';

    cell_JPID1{k} = reshape(dEdPID1',[],1);
    cell_JPID2{k} = reshape(dEdPID2',[],1);
    cell_JPVal{k} = reshape(dEdP',[],1);

%     u = XY3(1,:); % x of grid map
%     v = XY3(2,:); % y of grid map

    u = max(XY3(1,:), 1); % x of grid map
    v = max(XY3(2,:), 1); % y of grid map

    u1 = fix(u); % Rounding to zero direction
    v1 = fix(v);
    
    dEdM = [(v1+1-v).*(u1+1-u);(v-v1).*(u1+1-u);(v1+1-v).*(u-u1);(v-v1).*(u-u1)];
    dEdMID2 = [Size_j*(u1-1)+v1;Size_j*u1+v1;Size_j*(u1-1)+v1+1;Size_j*u1+v1+1];
    dEdMID1 = repmat(IDk,4,1);

    
    height = size(dEdMID2,1);
    width = size(dEdMID2,2);
    area = height * width;
    area_max = Size_i*Size_j;
%     disp("area");
%     disp(area);
%     disp("area_max: Size_i*Size_j");
%     disp(area_max);
    

    % Make sure dEdMID2 values are within the bounds of the sparse matrix
    max_u_index = Size_i * Size_j;
    dEdMID2 = min(dEdMID2, max_u_index);

    height = size(dEdMID2,1);
    width = size(dEdMID2,2);
    area = height * width;
    area_max = Size_i*Size_j;
%     disp("area new");
%     disp(area);
%     disp("area_max: Size_i*Size_j");
%     disp(area_max);

    cell_JDID1{k} = reshape(dEdMID1',[],1);
    cell_JDID2{k} = reshape(dEdMID2',[],1);
    cell_JDVal{k} = reshape(dEdM',[],1);

    if k < nD
        posek1 = Pose{k+1};
        [euler_anglesk, translationsk] = se3_to_euler_angles_translation(posek);
        [euler_anglesk1, translationsk1] = se3_to_euler_angles_translation(posek1);
        translationsk = translationsk / 1000.0;
        translationsk1 = translationsk1 / 1000.0;
        RZ = FuncRZ(euler_anglesk(1));
        RY = FuncRY(euler_anglesk(2));
        RX = FuncRX(euler_anglesk(3));
        R = FuncR(RZ,RY,RX);
        t = translationsk;
        dRZdA = FuncdRZdA(euler_anglesk(1));
        dRYdB = FuncdRYdB(euler_anglesk(2));
        dRXdG = FuncdRXdG(euler_anglesk(3));
        dRdA = FuncR(dRZdA,RY,RX);
        dRdB = FuncR(RZ,dRYdB,RX);
        dRdG = FuncR(RZ,RY,dRXdG);
        t2 = translationsk1;
        RZ2 = FuncRZ(euler_anglesk1(1));
        RY2 = FuncRY(euler_anglesk1(2));
        RX2 = FuncRX(euler_anglesk1(3));
        R2 = FuncR(RZ2,RY2,RX2);  

        dT = R'*(translationsk1-translationsk);
        DeltaR = R'*R2;
        Transk = Odom{k};
        OdomR = Transk(1:3,1:3);
        OdomT = Transk(1:3,4)/1000.0;
        Euler_DeltaR = Rotation_to_Euler(DeltaR);
        Euler_OdomR = Rotation_to_Euler(OdomR);
        dEuler = Euler_DeltaR;
        EOk = [dT;dEuler] - [OdomT;Euler_OdomR];
        cell_ErrorO{k} = EOk;
        Ri = R'*R2;
        dRZ2dA2 = FuncdRZdA(euler_anglesk1(1));
        dRY2dB2 = FuncdRYdB(euler_anglesk1(2));
        dRX2dG2 = FuncdRXdG(euler_anglesk1(3));
        dR2dA2 = FuncR(dRZ2dA2,RY2,RX2);
        dR2dB2 = FuncR(RZ2,dRY2dB2,RX2);
        dR2dG2 = FuncR(RZ2,RY2,dRX2dG2);
        dRidA2 = R'*dR2dA2;
        dRidB2 = R'*dR2dB2;
        dRidG2 = R'*dR2dG2;
        dRidA = dRdA'*R2;
        dRidB = dRdB'*R2;
        dRidG = dRdG'*R2;
        ddA2 = FuncdRi(dRidA2,Ri);
        ddB2 = FuncdRi(dRidB2,Ri);
        ddG2 = FuncdRi(dRidG2,Ri);
        ddA = FuncdRi(dRidA,Ri);
        ddB = FuncdRi(dRidB,Ri);
        ddG = FuncdRi(dRidG,Ri);
        R_T = R';
        a = (6*k-5:6*k)';
        b = (6*k+1:6*k+6)';
        cell_JOID1{k} = [Cnti;Cnti+1;Cnti+2;Cnti;Cnti+1;Cnti+2;Cnti;Cnti+1;...
            Cnti+2;Cnti;Cnti+1;Cnti+2;Cnti;Cnti+1;Cnti+2;Cnti;Cnti+1;Cnti+2;...
            Cnti;Cnti+1;Cnti+2;Cnti;Cnti+1;Cnti+2;Cnti;Cnti+1;Cnti+2;Cnti+3;...
            Cnti+4;Cnti+5;Cnti+3;Cnti+4;Cnti+5;Cnti+3;Cnti+4;Cnti+5;Cnti+3;...
            Cnti+4;Cnti+5;Cnti+3;Cnti+4;Cnti+5;Cnti+3;Cnti+4;Cnti+5];
        cell_JOID2{k} = [a(1);a(1);a(1);a(2);a(2);a(2);a(3);a(3);a(3);a(4);...
            a(4);a(4);a(5);a(5);a(5);a(6);a(6);a(6);b(1);b(1);b(1);b(2);b(2);...
            b(2);b(3);b(3);b(3);a(4);a(4);a(4);a(5);a(5);a(5);a(6);a(6);a(6);...
            b(4);b(4);b(4);b(5);b(5);b(5);b(6);b(6);b(6)];
        cell_JOVal{k} = [-R_T(:,1);-R_T(:,2);-R_T(:,3);dRdA'*(t2-t);dRdB'*(t2-t);...
            dRdG'*(t2-t);R_T(:,1);R_T(:,2);R_T(:,3);ddA;ddB;ddG;ddA2;ddB2;ddG2];
        Cnti = Cnti+6;
    end
end
ErrorS = vertcat(cell_ErrorS{:});
ErrorO = vertcat(cell_ErrorO{:});

JPID1 = vertcat(cell_JPID1{:});
JPID2 = vertcat(cell_JPID2{:});
JPVal = vertcat(cell_JPVal{:});

JDID1 = vertcat(cell_JDID1{:});
JDID2 = vertcat(cell_JDID2{:});
JDVal = vertcat(cell_JDVal{:});

JOID1 = vertcat(cell_JOID1{:});
JOID2 = vertcat(cell_JOID2{:});
JOVal = horzcat(cell_JOVal{:});

IS = GetInformationMfromS(ErrorS);
IO = GetInformationMfromO(Odom);

ErrorS = double(ErrorS);
% Sum_Error = ErrorS' * IS * ErrorS + ErrorO'*IO*ErrorO;
Sum_Error = ErrorS' * IS * ErrorS;
% MSE_Error = Sum_Error/(length(ErrorS)+length(ErrorO));
MSE_Error = Sum_Error/(length(ErrorS));

JDVal = double(JDVal);
JDID1 = double(JDID1);
JDID2 = double(JDID2);
JD = sparse(JDID1,JDID2,JDVal,nPts,Size_i*Size_j);

JPVal = double(JPVal);
JP = sparse(JPID1,JPID2,JPVal);
JO = sparse(JOID1,JOID2,JOVal);
end