function [Map,Pose] = FuncUpdate(Map,Pose,DeltaP,DeltaD)

DeltaP2 = reshape(DeltaP,3,[])';
Pose(2:end,:) = Pose(2:end,:)+DeltaP2;

Size_i = Map.Size_i;
Size_j = Map.Size_j;

DeltaD2 = reshape(DeltaD,Size_j,Size_i)';

Map.Grid = Map.Grid+DeltaD2;

end


