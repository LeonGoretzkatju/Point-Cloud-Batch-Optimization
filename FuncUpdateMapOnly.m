function Map = FuncUpdateMapOnly(Map, DeltaD)
% Size_i = Map.Size_i;
% Size_j = Map.Size_j;
% 
% DeltaD2 = reshape(DeltaD,Size_j,Size_i)';
% 
% Map.Grid = Map.Grid+DeltaD2;
    % Find outliers in DeltaD2 with absolute values larger than 0.08
%     outliers = abs(DeltaD2) >= 0.05;

    % Set outlier elements to zero
%     DeltaD2(outliers) = 0;
% Map.Grid = Map.Grid + DeltaD2;
Size_i = Map.Size_i;
Size_j = Map.Size_j;

DeltaD2 = reshape(DeltaD,Size_j,Size_i)';

Map.Grid = Map.Grid+DeltaD2;
end