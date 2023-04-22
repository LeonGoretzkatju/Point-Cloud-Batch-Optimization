function [Map,Pose,Scan,index] = FuncRemoveOutlier(Map,Pose,Scan,MODE_MAP)


Scale = Map.Scale;
Origin = Map.Origin;

Pose = Pose';

nn = length(Scan);

parfor i = 1:nn
    posei = Pose(:,i);
    xy = Scan{i}.xy';
    Oddi = Scan{i}.Odd;
    
    Ri = FuncRMatrix2D(posei(3));
    Si = Ri'*xy+posei(1:2);
    XY3 = (Si-Origin)/Scale+1;
    
    
    if strcmp(MODE_MAP,'CONTINUOUS')==1
        Md = Map.DgridG(XY3(2,:),XY3(1,:)); % interpolant of map grid 
        MN = Map.NgridG(XY3(2,:),XY3(1,:)); % interpolant of hit number 
    else
        Md = Map.DgridG(round(XY3(2,:)),round(XY3(1,:))); % interpolant of map grid 
        round_XY3 = [round(XY3(2,:));round(XY3(1,:))]';
        round_XY3 = (round_XY3(:,2)-1)*Size_i+round_XY3(:,1);
        MN = Map.N(round_XY3)'; % interpolant of hit number 
       
    end
    Ek{i} = abs(Md./MN-Oddi'); %  diff between each point
    

    Error_i(i) = sum(Ek{i})./size(Ek{i},2);

end

mean_error = sum(Error_i)/size(Error_i,2);

index = find((Error_i>(2*mean_error))==1);

if isempty(index)==0
    Pose(:,index)=[]; 
    Scan(index) = [];
    Map = FuncInitialiseGridMap(Map,Pose',Scan);
    num_index = size(index,2);
    fprintf('Removed %d Poses and Scans \n', num_index);
    fprintf('The indexs of Outliers are %d \n', index);

end
    
end