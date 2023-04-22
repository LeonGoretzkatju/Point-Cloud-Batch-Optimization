function [Map] = FuncCreateGridMap(Size_i,Size_j,Scale,Origin)

Map.Size_i = Size_i;
Map.Size_j = Size_j;
Map.Grid = zeros(Size_i,Size_j);
Map.N = zeros(Size_i,Size_j);
Map.Scale = Scale;
Map.Origin = Origin;

end