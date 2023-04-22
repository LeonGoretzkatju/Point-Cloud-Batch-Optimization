function HH_Select = FuncSelectMapConst(Select_Map)

% Size_i = Select_Map.Size_i;
Size_j = Select_Map.Size_j;
% the matrix index of variables in the map 

Sort_Select_Variables = Select_Map.Sort_Select_Variables;

% convert to matrix index 
matrix_i = ceil(Sort_Select_Variables./Size_j);
matrix_j = Sort_Select_Variables - (matrix_i-1).*Size_j;
matrix_ij = [matrix_i,matrix_j];

% define the right and below cell in the map(matrix)
matrix_j_right = matrix_j+1;
matrix_i_right = matrix_i;
sort_index_right = [matrix_i_right,matrix_j_right];
matrix_i_below = matrix_i+1;
matrix_j_below = matrix_j;
sort_index_below = [matrix_i_below,matrix_j_below];


% if the right/below hand has variables
find_right = ismember(sort_index_right,matrix_ij,'rows');
find_below = ismember(sort_index_below,matrix_ij,'rows');

ID1 = [];
ID2 = [];
Val = [];
nCnt = 0;

for k=1:length(matrix_i)
    ij = matrix_ij(k,:);
    ij_below = [ij(1)+1,ij(2)];
    ij0 = k; 

    if find_right(k)==1
        nCnt = nCnt+1;
        ij1 = k+1;
        ID1 = [ID1;nCnt;nCnt];
        ID2 = [ID2;ij0;ij1];
        Val = [Val;1;-1];
    end
    if find_below(k) ==1
        nCnt = nCnt+1;
        find_index = ij_below==matrix_ij;
        find_index = find_index(:,1).*find_index(:,2);
        ij2 = find(find_index==1);
        ID1 = [ID1;nCnt;nCnt];
        ID2 = [ID2;ij0;ij2];
        Val = [Val;1;-1];
    end
end

J = sparse(ID1,ID2,Val);
HH_Select = J'*J;

end