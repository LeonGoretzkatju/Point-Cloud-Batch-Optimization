function Map = AddNoiseToMap(Map)
Grid = Map.Grid;
Size_i = Map.Size_i;
Size_j = Map.Size_j;
Grid = Grid + 0.5*rand(Size_i,Size_j);
Map.Grid = Grid;
end