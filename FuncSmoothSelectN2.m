function Select_Map = FuncSmoothSelectN2(Select_Map,Lambda,HH2_Select)

Size_i = Select_Map.Size_i;
Size_j = Select_Map.Size_j;
HH_Select = Lambda*HH2_Select;

[i,j,Val] = find(Select_Map.N); % Find the index and value of all non-zero elements
ni = length(i); % the number of non-zero elements
ID1 = (1:ni)';
ID2 = Size_j*(i-1)+j; %variable order
A1 = sparse(ID1,ID2,1,ni,Size_i*Size_j);
Val = sparse(Val);

Remove_Point = Select_Map.Remove_Point; % point cordinate (x,y)
remove_index = (Remove_Point(:,1)-1)*Size_j+Remove_Point(:,2); % variable index
% sort the order from small to big
remove_sort_index = sort(remove_index);

% select points
Select_Point = Select_Map.Select_Point;
selected_index = (Select_Point(:,1)-1)*Size_j+Select_Point(:,2); % variable index
% sort the order from small to big
select_sort_index = sort(selected_index);

A1(:,remove_sort_index) = [];
I = A1'*A1+HH_Select;
E = A1'*Val;

DeltaN = I\E;

% recover full map N
column_index = ones(length(select_sort_index),1);
Recover_Delta = sparse(select_sort_index,column_index,DeltaN,Size_i*Size_j,1);
Recover_Delta = full(Recover_Delta);
Select_Map.N = reshape(Recover_Delta,Size_j,Size_i)';
end