function Map = FuncUpdateMapOnly(Map, DeltaD)
    Size_i = Map.Size_i;
    Size_j = Map.Size_j;
    DeltaD2 = reshape(DeltaD, Size_i, Size_j);
    Map.Grid = Map.Grid + DeltaD2;
end