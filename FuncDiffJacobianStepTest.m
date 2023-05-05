function [cell_ErrorS,dMdXYSet] = FuncDiffJacobianStepTest(Map,Pose,Scan,Odom,MODE_DERIVATIVES,MODE_MAP)

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
for k = 1:nD
    posek = Pose{k};
    Scan_k = Scan{k};
    xyk = Scan_k.Location(:,1:2)'; % Extract x and y values
    zk = Scan_k.Location(:,3)'; % Extract z values as the new Oddi
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

end