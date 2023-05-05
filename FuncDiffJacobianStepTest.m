function [cell_ErrorS,dMdXYSet,dEdPSet,JP] = FuncDiffJacobianStepTest(Map,Pose,Scan,Odom,MODE_DERIVATIVES,MODE_MAP)

Size_i = Map.Size_i;
Size_j = Map.Size_j;
Scale = Map.Scale;
Origin = Map.Origin;

nD = length(Scan); % the timestamp
nPts = 0;
Cnti = 1;

cell_ErrorS = cell(1,nD);

cell_JPID1 = cell(1,nD);
cell_JPID2 = cell(1,nD);
cell_JPVal = cell(1,nD);
cell_JDID1 = cell(1,nD);
cell_JDID2 = cell(1,nD);
cell_JDVal = cell(1,nD);
dMdXYSet = {};
dEdPSet = {};
for k = 1:nD
    posek = Pose{k};
    [euler_angles, translations] = se3_to_euler_angles_translation(posek);
    Scan_k = Scan{k};
    xyk = Scan_k.Location(:,1:2)'; % Extract x and y values
    XY1 = xyk;
    zk = Scan_k.Location(:,3)'; % Extract z values as the new Oddi
    XY1_Span = [xyk;zk];
    P = [xyk;zk;ones(1,size(zk,2))];
    Pwk = inv(posek)*P;
    XY3 = (Pwk(1:2,:)-Origin)*Scale;    
    if strcmp(MODE_MAP,'CONTINUOUS')==1
        Md = Map.DgridG(XY3(2,:),XY3(1,:)); % interpolant of map grid 
    else
        Md = Map.DgridG(round(XY3(2,:)),round(XY3(1,:)));
    end
    Ek = Md - zk;
    cell_ErrorS{k} = Ek';
    
%   The mode of derivative 
    if strcmp (MODE_DERIVATIVES, 'PARSING')==1
        Map = FuncGradient_Matrix_M(XY3(2,:),XY3(1,:),Map); % calculate gradient of M and N, given(x,y and Map)
        dMdXY3 = [Map.DgridGu; Map.DgridGv]; % derivative of Grid Map with P_m 
        
    else
        if strcmp(MODE_MAP,'CONTINUOUS')==1
            dMdXY3 = [Map.DgridGu(XY3(2,:),XY3(1,:));Map.DgridGv(XY3(2,:),XY3(1,:))]; % derivative of Grid Map with x,y/N
        else
            dMdXY3 = [Map.DgridGu(round_XY3)';Map.DgridGv(round_XY3)'];
        end
    end
    dMdXYSet{end+1} = dMdXY3;
    [dRyaw,dRpitch,dRroll] = DiffRotationMatrix(euler_angles);
    dXY3dRyaw = dRyaw'*XY1_Span*Scale;
    dXY3dRpitch = dRpitch'*XY1_Span*Scale;
    dXY3dRroll = dRroll'*XY1_Span*Scale;
    dMdRyaw = sum(dMdXY3.*dXY3dRyaw(1:2,:));
    dMdRpitch = sum(dMdXY3.*dXY3dRpitch(1:2,:));
    dMdRroll = sum(dMdXY3.*dXY3dRroll(1:2,:));
    dZdRyaw = dXY3dRyaw(3,:);
    dZdRpitch = dXY3dRpitch(3,:);
    dZdRroll = dXY3dRroll(3,:);
    dZdR = [dZdRyaw;dZdRpitch;dZdRroll];
    dZdT = Scale*[zeros(1,size(dMdXY3,2));...
        zeros(1,size(dMdXY3,2));ones(1,size(dMdXY3,2))];
    dMdR = [dMdRyaw;dMdRpitch;dMdRroll];
    dMdT = Scale*[dMdXY3(1,:);dMdXY3(2,:);zeros(1,size(dMdXY3,2))];
    dMdP = [dMdT;dMdR];
    dZdP = [dZdT;dZdR];
    dEdP = [dZdT-dMdT;dZdR-dMdR];
    dEdPSet{end+1} = dEdP;

    nPtsk = length(zk);
    IDk = nPts+1:nPts+nPtsk; %ID numer from 1 to nPtsk
    nPts = nPts+nPtsk;
    
    dEdPID1 = repmat(IDk,6,1);
    dEdPID2 = repmat(6*(k-1)+1:6*k,nPtsk,1)';

    cell_JPID1{k} = reshape(dEdPID1',[],1);
    cell_JPID2{k} = reshape(dEdPID2',[],1);
    cell_JPVal{k} = reshape(dEdP',[],1);

end
JPID1 = vertcat(cell_JPID1{:});
JPID2 = vertcat(cell_JPID2{:});
JPVal = vertcat(cell_JPVal{:});
JPVal = double(JPVal);
JP = sparse(JPID1,JPID2,JPVal);
end