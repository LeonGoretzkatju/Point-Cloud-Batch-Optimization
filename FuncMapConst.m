function HH = FuncMapConst(Map)

Size_i = Map.Size_i;
Size_j = Map.Size_j;

ID1 = [];
ID2 = [];
Val = [];

nCnt = 0;
for i = 1:Size_i
    for j = 1:Size_j
        ij0 = Size_j*(i-1)+j;
        ij1 = Size_j*(i-1)+j+1;
        ij2 = Size_j*i+j;
        
        if j+1<=Size_j
            nCnt = nCnt+1;
            ID1 = [ID1;nCnt;nCnt];
            ID2 = [ID2;ij0;ij1];
            Val = [Val;1;-1];
        end
        
        if i+1<=Size_i  
            nCnt = nCnt+1;
            ID1 = [ID1;nCnt;nCnt];
            ID2 = [ID2;ij0;ij2];
            Val = [Val;1;-1];
        end
    end
end

J = sparse(ID1,ID2,Val);
HH = J'*J;

