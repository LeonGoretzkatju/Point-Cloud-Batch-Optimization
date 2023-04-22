function HH = FuncMapConv2(Map,KERNEL_SIZE)

Size_i = Map.Size_i;
Size_j = Map.Size_j;



a = 1./(KERNEL_SIZE*KERNEL_SIZE);


ID1 = [];
ID2 = [];
Value = [];
nCnt = 0;
loc = 0;

for i=1:Size_i
    for j=1:Size_j
        nCnt = nCnt+1;
        ij = [];

        min_i = (i-(KERNEL_SIZE-1)/2);
        max_i = (i+(KERNEL_SIZE-1)/2);
        min_j = (j-(KERNEL_SIZE-1)/2);
        max_j = (j+(KERNEL_SIZE-1)/2);

        if min_i < 1
            min_i = 1;
        end
        if max_i > Size_i
            max_i = Size_i;
        end
        if min_j <1
            min_j = 1;
        end
        if max_j > Size_j
            max_j = Size_j;
        end
       
        B_1 = min_i:max_i;
        B_1 = (B_1-1).*Size_j;
        B_2 = min_j:max_j;

        for m=1:length(B_1)
            for n=1:length(B_2)
                ij(m,n) = B_1(m) + B_2(n);
            end
        end
               



        ij = reshape(ij',1,[])';
        Val = (-a) * ones(length(ij),1);



        loc = loc+1;
        index = find(ij==loc);
        Val(index) = Val(index)+1;



%         Val((KERNEL_SIZE-1)/2 * KERNEL_SIZE + (KERNEL_SIZE-1)/2+1) = ...
%             Val((KERNEL_SIZE-1)/2 * KERNEL_SIZE + (KERNEL_SIZE-1)/2+1)+1;

        
        row = nCnt * ones(length(ij),1);
        ID1 = [ID1; row];
        ID2 = [ID2;ij];
        Value = [Value; Val];
    end
    
    
       
    
end

J = sparse(ID1,ID2,Value);

HH = J'*J;

