function [ErrorS_Sel,JP_Sel,JD_Sel]= FuncDiffSelectJacobian(Select_Map,Pose,Select_Scan)

Size_i = Select_Map.Size_i;
Size_j = Select_Map.Size_j;
Scale = Select_Map.Scale;
Origin = Select_Map.Origin;



%%
nD = length(Select_Scan); % the timestamp
nPts = 0;
cell_ErrorS_Sel = cell(1,nD);
cell_JPID1 = cell(1,nD);
cell_JPID2 = cell(1,nD);
cell_JPVal = cell(1,nD);
cell_JDID1 = cell(1,nD);
cell_JDID2 = cell(1,nD);
cell_JDVal = cell(1,nD);
for k = 1:nD

    Rk = FuncRMatrix2D(Pose(k,3)); % the R of the robot
    Tk = Pose(k,1:2)'; % x,y of the robot
    XY1 = Select_Scan{k}.xy'; % x,y of the scan
    Oddk = Select_Scan{k}.Odd'; % Odds value of the scan

    XY2 = Rk'*XY1+Tk; % convert robot cordinate to the world cordinate
    XY3 = (XY2-Origin)/Scale+1; % convert the world cordinate to map 

    % interpolation of grid of selected map
    [Md,out_index] = FuncInterSelectMap(XY3(2,:),XY3(1,:),Select_Map);
    MN = FuncInterSelectN(XY3(2,:),XY3(1,:),Select_Map,out_index); % interpolant of hit number
    % remove out
    Oddk(out_index) = [];
    Ek = Md./MN-Oddk; %  diff between each point
    cell_ErrorS_Sel{k} = Ek';
    Select_Map = FuncGradientSelectMap(XY3(2,:),XY3(1,:),Select_Map,out_index); % calculate gradient of M and N, given(x,y and Map)
    dMdXY3 = [Select_Map.DgridGu; Select_Map.DgridGv]./MN; % derivative of Grid Map with P_m 

    dR = FuncdR2D(Pose(k,3)); % the derivative of rotation wirh theta

    % remove out point
    Y = XY1(1,:);
    X = XY1(2,:);
    Y(out_index) = [];
    X(out_index) = [];

    XY1 = [Y;X];
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
    % remove out point
    u(out_index) = [];
    v(out_index) = [];

    u1 = fix(u); % Rounding to zero direction
    v1 = fix(v);
    
    dEdM = [(v1+1-v).*(u1+1-u);(v-v1).*(u1+1-u);(v1+1-v).*(u-u1);(v-v1).*(u-u1)]./MN;
    dEdMID2 = [Size_j*(v1-1)+u1;Size_j*v1+u1;Size_j*(v1-1)+u1+1;Size_j*v1+u1+1];
    dEdMID1 = repmat(IDk,4,1);

    cell_JDID1{k} = reshape(dEdMID1',[],1);
    cell_JDID2{k} = reshape(dEdMID2',[],1);
    cell_JDVal{k} = reshape(dEdM',[],1);
    
end
%%

ErrorS_Sel = vertcat(cell_ErrorS_Sel{:});
JPID1 = vertcat(cell_JPID1{:});
JPID2 = vertcat(cell_JPID2{:});
JPVal = vertcat(cell_JPVal{:});

JDID1 = vertcat(cell_JDID1{:});
JDID2 = vertcat(cell_JDID2{:});
JDVal = vertcat(cell_JDVal{:});
JP_Sel = sparse(JPID1,JPID2,JPVal);
JD_Sel = sparse(JDID1,JDID2,JDVal,nPts,Size_i*Size_j);
Remove_Point = Select_Map.Remove_Point;
% The index of removed points (the order of variables of the map)
index_remove = (Remove_Point(:,1)-1)*Size_j+Remove_Point(:,2);
JD_Sel(:,index_remove) = [];

end

