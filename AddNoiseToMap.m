function Map = AddNoiseToMap(Map,Noise_Level)
Grid = Map.Grid;
Size_i = Map.Size_i;
Size_j = Map.Size_j;
Grid = Grid + Noise_Level*rand(Size_i,Size_j);
Map.Grid = Grid;
end