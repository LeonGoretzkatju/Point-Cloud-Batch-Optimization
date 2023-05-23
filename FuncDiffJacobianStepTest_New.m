function [ErrorS,MSE_Error] = FuncDiffJacobianStepTest(Map,Pose,Odom,Scan,MODE_DERIVATIVES,MODE_MAP)
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
        [euler_angles, ~] = se3_to_euler_angles_translation(posek);
        Scan_k = Scan{k};
        xyk = Scan_k.Location(:,1:2)'; % Extract x and y values
        zk = Scan_k.Location(:,3)'; % Extract z values as the new Oddi
        XY1_Span = [xyk;zk];
        P = [xyk;zk;ones(1,size(zk,2))];
        Pwk = inv(posek)*P;
        XY3 = (Pwk(1:2,:)-Origin)/Scale+1;    
        if strcmp(MODE_MAP,'CONTINUOUS')==1
            Md = Map.DgridG(XY3(2,:),XY3(1,:)); % interpolant of map grid 
        end
        Ek = Md - zk;
        cell_ErrorS{k} = Ek';
        dMdXY3 = [Map.DgridGu(XY3(2,:),XY3(1,:));Map.DgridGv(XY3(2,:),XY3(1,:))];
    end
    ErrorS = vertcat(cell_ErrorS{:});
    ErrorS = double(ErrorS);
    IS = GetInformationMfromS(ErrorS);
    Sum_Error = ErrorS'*IS*ErrorS;
    MSE_Error = Sum_Error/(length(ErrorS));
end