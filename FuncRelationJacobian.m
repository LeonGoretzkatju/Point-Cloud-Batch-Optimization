function [JR_Low,JR_Select,ErrorRelation] = FuncRelationJacobian(Map,Select_Map,Original_Map,index_low,Rate)

Select_Point = Select_Map.Select_Point;
select_index_matrix_sort = (Select_Point(:,2)-1) * Select_Map.Size_i + Select_Point(:,1);

% find the hit number value of select map
Val_N_Select = Select_Map.N(select_index_matrix_sort);
zero_index = find(Val_N_Select==0);
zero_ij = select_index_matrix_sort(zero_index);
original_val = Original_Map.N(zero_ij);
Val_N_Select(zero_index) = original_val;
% find the grid value of select map
Val_M_Select = Select_Map.Grid(select_index_matrix_sort);
zero_index = find(Val_M_Select==0);
zero_ij = select_index_matrix_sort(zero_index);
original_val = Original_Map.Grid(zero_ij);
Val_M_Select(zero_index) = original_val;
% find the hit number value of low resolution map
matrix_index_low = (index_low(:,2)-1)*Map.Size_i + index_low(:,1);
Val_N_Low = Map.N(matrix_index_low);

% find the grid value of low resolution map
Val_M_Low = Map.Grid(matrix_index_low);

% calculate the normalization value (the index order is matrix )
Val_Normal_Select = Val_M_Select./Val_N_Select;
Val_Normal_Select = -Val_Normal_Select./Rate;
Val_Normal_Low = Val_M_Low./Val_N_Low;

% calculate the index of variables
Select_Point = Select_Map.Select_Point;
select_index_variables = (Select_Point(:,1)-1)*Select_Map.Size_j + Select_Point(:,2);
low_index_variables = index_low(:,1)*Map.Size_j + index_low(:,2);

id_row_low = 1:length(low_index_variables);
id_row_select = ones(Rate*Rate*length(id_row_low),1);
ErrorRelation = zeros(length(id_row_low),1);
for i=1:length(id_row_low)
    id_row_select(((i-1)*Rate*Rate+1):(i*Rate*Rate),1) = id_row_low(i) * id_row_select(((i-1)*Rate*Rate+1):(i*Rate*Rate),1);
    ErrorRelation(i) = Val_Normal_Low(i) + sum(Val_Normal_Select(((i-1)*Rate*Rate+1):(i*Rate*Rate),1),1);
end

JR_Low = sparse(id_row_low',low_index_variables,Val_Normal_Low,length(id_row_low),Map.Size_i*Map.Size_j);

JR_Select = sparse(id_row_select,select_index_variables,Val_Normal_Select,length(id_row_low),Select_Map.Size_i*Select_Map.Size_j);
% remove elements 
Remove_Point = Select_Map.Remove_Point;
% The index of removed points (the order of variables of the map)
index_remove = (Remove_Point(:,1)-1)*Select_Map.Size_j+Remove_Point(:,2);
JR_Select(:,index_remove) = [];


end

