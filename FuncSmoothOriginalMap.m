function [Map] = FuncSmoothOriginalMap(Map,Lambda,SMOOTHING_MODE,KERNEL_SIZE)

Size_i = Map.Size_i;
Size_j = Map.Size_j;

[i,j,Val] = find(Map.Grid); % Find the index and value of all non-zero elements
ni = length(i); % the number of non-zero elements
ID1 = (1:ni)';
ID2 = Size_j*(i-1)+j;
A1 = sparse(ID1,ID2,1,ni,Size_i*Size_j);
Val = sparse(Val);

if strcmp(SMOOTHING_MODE,'CONV')==1
    HH2 = FuncMapConv2(Map,KERNEL_SIZE);
else
    HH2 = FuncMapConst(Map); 
end
HH = HH2*Lambda;

I = A1'*A1+HH;
E = A1'*Val;

DeltaN = I\E;

DeltaN = full(DeltaN);

Map.Grid = reshape(DeltaN,Map.Size_j,Map.Size_i)';

end