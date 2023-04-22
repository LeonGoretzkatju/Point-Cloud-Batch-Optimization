function [Select_Map,Pose] = FuncSelectUpdate(Select_Map,Pose,DeltaP,DeltaD)

DeltaP2 = reshape(DeltaP,3,[])';
Pose(2:end,:) = Pose(2:end,:)+DeltaP2;

Select_Point = Select_Map.Select_Point; % point cordinate (x,y)
selected_index = (Select_Point(:,1)-1)*Select_Map.Size_j+Select_Point(:,2); % variable index
% sort the order from small to big
sort_index = sort(selected_index);
% convert to matrix index 
matrix_i = ceil(sort_index./Select_Map.Size_j);
matrix_j = sort_index - (matrix_i-1).*Select_Map.Size_j;
index_matrix = (matrix_j-1).*Select_Map.Size_i + matrix_i;
% The order of index is for matrix (first column - second column - ...) 
Select_Map.Grid(index_matrix) = Select_Map.Grid(index_matrix) + DeltaD;

% check = sparse(matrix_i,matrix_j,DeltaD,Select_Map.Size_i,Select_Map.Size_j);
% check = full(check);

end


