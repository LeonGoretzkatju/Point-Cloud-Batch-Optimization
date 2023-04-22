%% Code edited by Yingyu Wang 
function [ErrorS,ErrorO,Sum_Error,MSE_Error,JP,JD,JO,IS,IO]= FuncDiffJacobian(Map,Pose,Scan,Odom,MODE_DERIVATIVES,MODE_MAP)

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
    Rk = FuncRMatrix2D(Pose(k,3)); % the R of the robot
    Tk = Pose(k,1:2)'; % x,y of the robot
    XY1 = Scan{k}.xy'; % x,y of the scan
    Oddk = Scan{k}.Odd'; % Odds value of the scan

    XY2 = Rk'*XY1+Tk; % convert robot cordinate to the world cordinate
    XY3 = (XY2-Origin)/Scale+1; % convert the world cordinate to map, related to the formula(9) in paper    
    
   if strcmp(MODE_MAP,'CONTINUOUS')==1
        Md = Map.DgridG(XY3(2,:),XY3(1,:)); % interpolant of map grid 
        MN = Map.NgridG(XY3(2,:),XY3(1,:)); % interpolant of hit number 
   else
        Md = Map.DgridG(round(XY3(2,:)),round(XY3(1,:))); 
        round_XY3 = [round(XY3(2,:));round(XY3(1,:))]';
        round_XY3 = (round_XY3(:,2)-1)*Size_i+round_XY3(:,1);
        MN = Map.N(round_XY3)'; 
       
   end
    Ek = Md./MN-Oddk; %  diff between each point
    cell_ErrorS{k} = Ek';
    
%   The mode of derivative 
    if strcmp (MODE_DERIVATIVES, 'PARSING')==1
        Map = FuncGradient_Matrix_M(XY3(2,:),XY3(1,:),Map); % calculate gradient of M and N, given(x,y and Map)
        dMdXY3 = [Map.DgridGu; Map.DgridGv]./MN; % derivative of Grid Map with P_m 
        
    else
        if strcmp(MODE_MAP,'CONTINUOUS')==1
            dMdXY3 = [Map.DgridGu(XY3(2,:),XY3(1,:));Map.DgridGv(XY3(2,:),XY3(1,:))]./MN; % derivative of Grid Map with x,y/N
        else
            dMdXY3 = [Map.DgridGu(round_XY3)';Map.DgridGv(round_XY3)']./MN;
        end
    end
    
    dR = FuncdR2D(Pose(k,3)); % the derivative of rotation wirh theta
    dXY3dR = dR'*XY1/Scale;    
    dMdR = sum(dMdXY3.*dXY3dR(1:2,:));
    dMdT = dMdXY3/Scale;    
    dMdP = [dMdT;dMdR];
    
    nPtsk = length(Oddk);
    IDk = nPts+1:nPts+nPtsk;
    nPts = nPts+nPtsk;
    
    dEdPID1 = repmat(IDk,3,1);
    dEdPID2 = repmat(3*(k-1)+1:3*k,nPtsk,1)';
    
    cell_JPID1{k} = reshape(dEdPID1',[],1);
    cell_JPID2{k} = reshape(dEdPID2',[],1);
    cell_JPVal{k} = reshape(dMdP',[],1);
        
    u = XY3(1,:); % x of grid map
    v = XY3(2,:); % y of grid map
    u1 = fix(u); % Rounding to zero direction
    v1 = fix(v);
    
    dEdM = [(v1+1-v).*(u1+1-u);(v-v1).*(u1+1-u);(v1+1-v).*(u-u1);(v-v1).*(u-u1)]./MN;
    dEdMID2 = [Size_j*(v1-1)+u1;Size_j*v1+u1;Size_j*(v1-1)+u1+1;Size_j*v1+u1+1];
    dEdMID1 = repmat(IDk,4,1);

    cell_JDID1{k} = reshape(dEdMID1',[],1);
    cell_JDID2{k} = reshape(dEdMID2',[],1);
    cell_JDVal{k} = reshape(dEdM',[],1);

    %%
    if k<nD
        P1 = Pose(k,:)'; % pose of k
        P2 = Pose(k+1,:)'; % pose of k+1
        R1 = Rk; % rotation of k
        dT = R1*(P2(1:2)-P1(1:2));
        dPhi = P2(3)-P1(3);
        while dPhi>pi || dPhi<-pi
            dPhi = wrap(dPhi);
        end
        EOk = [dT;dPhi]-Odom(k,:)';
        while EOk(3)>pi || EOk(3)<-pi
            EOk(3) = wrap(EOk(3));
        end
        cell_ErrorO{k} = EOk;
        %%
        dTdT1 = -R1;
        dTdT2 = R1;
        dTdPhi1 = dR*(P2(1:2)-P1(1:2));
        dPhid1 = -1;
        dPhid2 = 1;
        
        a = (3*k-2:3*k)';
        b = (3*k+1:3*k+3)';

        cell_JOID1{k} = [Cnti;Cnti;  Cnti+1;Cnti+1;  Cnti;Cnti;  Cnti+1;Cnti+1;  Cnti;Cnti+1;  Cnti+2;Cnti+2];
        cell_JOID2{k} = [a(1:2);  a(1:2);  b(1:2);  b(1:2);  a(3);a(3);  a(3);b(3)];
        cell_JOVal{k} = [dTdT1(1,:),dTdT1(2,:),  dTdT2(1,:),dTdT2(2,:),  dTdPhi1(1,1),dTdPhi1(2,1),  dPhid1,dPhid2]; 
        Cnti = Cnti+3;
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


%%
[IS,IO] = FuncGetI(Odom,ErrorS); % covariance matrix 
Sum_Error = ErrorS'*IS*ErrorS+ErrorO'*IO*ErrorO;
MSE_Error = Sum_Error/(length(ErrorS)+length(ErrorO));


%%
JPVal = double(JPVal);
JP = sparse(JPID1,JPID2,JPVal);
% clearvars JPID1 JPID2 JPVal
JDVal = double(JDVal);
JDID1 = double(JDID1);
JDID2 = double(JDID2);
JD = sparse(JDID1,JDID2,JDVal,nPts,Size_i*Size_j);
% clearvars JDID1 JDID2 JDVal nPts
JO = sparse(JOID1,JOID2,JOVal);

end
