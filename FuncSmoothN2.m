function [Map] = FuncSmoothN2(Map,Lambda,HH2)

Size_i = Map.Size_i;
Size_j = Map.Size_j;

[i,j,Val] = find(Map.N); % Find the index and value of all non-zero elements
ni = length(i); % the number of non-zero elements
ID1 = (1:ni)';
ID2 = Size_j*(i-1)+j;
A1 = sparse(ID1,ID2,1,ni,Size_i*Size_j);
Val = sparse(Val);

HH = HH2*Lambda;
I = A1'*A1+HH;
E = A1'*Val;

DeltaN = I\E;

DeltaN = full(DeltaN);

Map.N = reshape(DeltaN,Map.Size_j,Map.Size_i)';

end