function Select_Map = FuncSmoothSelectGrid(Select_Map,Lambda)

Size_i = Select_Map.Size_i;
Size_j = Select_Map.Size_j;
Grid = Select_Map.Grid;
[i,j,Val] = find(Grid); % Find the index and value of all non-zero elements
ni = length(i); % the number of non-zero elements
ID1 = (1:ni)';
ID2 = Size_j*(i-1)+j;
A1 = sparse(ID1,ID2,1,ni,Size_i*Size_j);
Val = sparse(Val);

HH2 = FuncMapConst(Select_Map);
HH = HH2*Lambda;

I = A1'*A1+HH;
E = A1'*Val;

DeltaN = I\E;

DeltaN = full(DeltaN);

Select_Map.Smoothing_Grid = reshape(DeltaN,Size_j,Size_i)';


end
