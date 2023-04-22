function HH = FuncMapCov(Map)

Size_i = Map.Size_i;
Size_j = Map.Size_j;

KERNEL_SIZE = 3;

a = 1./(KERNEL_SIZE*KERNEL_SIZE);


ID1 = [];
ID2 = [];
Value = [];
nCnt = 0;
for i=1:Size_i
    for j=1:Size_j
        nCnt = nCnt+1;
        Val = (-a) * ones(KERNEL_SIZE*KERNEL_SIZE,1);
       
        if  i>(KERNEL_SIZE -1)/2  && i<=(Size_i-(KERNEL_SIZE-1)/2) ...
                && j> (KERNEL_SIZE -1)/2 && j<= (Size_j-(KERNEL_SIZE-1)/2)
            
            ij11 = (i-2) * Size_j + (j-1);
            ij12 = (i-2) * Size_j + j;
            ij13 = (i-2) * Size_j + (j+1);
            ij21 = (i-1) * Size_j + (j-1);
            ij22 = (i-1) * Size_j + j;
            ij23 = (i-1) * Size_j + (j+1);
            ij31 = i * Size_j + (j-1);
            ij32 = i * Size_j + j;
            ij33 = i * Size_j + (j+1);
            
            Val(5) = Val(5)+1;
                          
            
        elseif i==1 && j==1
            ij11 = (i-1) * Size_j + j ;
            ij12 = (i-1) * Size_j + j+1;
            ij13 = (i-1) * Size_j + j+2;
            ij21 = i * Size_j + j;
            ij22 = i * Size_j + j+1;
            ij23 = i * Size_j + j+2;
            ij31 = (i+1) * Size_j + j;
            ij32 = (i+1) * Size_j + (j+1);
            ij33 = (i+1) * Size_j + (j+2);
            
            Val(1) = Val(1)+1;
            
        elseif i==1 && j== Size_j
            ij11 = (i-1) * Size_j + j-2 ;
            ij12 = (i-1) * Size_j + j-1;
            ij13 = (i-1) * Size_j + j;
            ij21 = i * Size_j + j-2;
            ij22 = i * Size_j + j-1;
            ij23 = i * Size_j + j;
            ij31 = (i+1) * Size_j + j-2;
            ij32 = (i+1) * Size_j + j-1;
            ij33 = (i+1) * Size_j + j;
            
            Val(3) = Val(3)+1;
            
            
       elseif i==Size_i && j==1
            ij11 = (i-3) * Size_j + j ;
            ij12 = (i-3) * Size_j + j+1;
            ij13 = (i-3) * Size_j + j+2;
            ij21 = (i-2) * Size_j + j;
            ij22 = (i-2) * Size_j + j+1;
            ij23 = (i-2) * Size_j + j+2;
            ij31 = (i-1) * Size_j + j;
            ij32 = (i-1) * Size_j + j+1;
            ij33 = (i-1) * Size_j + j+2;
            
            Val(7) = Val(7)+1;
            
        elseif i == Size_i && j == Size_j 
              ij11 = (i-3) * Size_j + j-2 ;
              ij12 = (i-3) * Size_j + j-1;
              ij13 = (i-3) * Size_j + j;
              ij21 = (i-2) * Size_j + j-2;
              ij22 = (i-2) * Size_j + j-1;
              ij23 = (i-2) * Size_j + j;
              ij31 = (i-1) * Size_j +j-2;
              ij32 = (i-1) * Size_j + j-1;
              ij33 = (i-1) * Size_j + j;
            
              Val(9) = Val(9)+1;
              
              
        elseif i==1 && j> (KERNEL_SIZE -1)/2 && j<= (Size_j-(KERNEL_SIZE-1)/2)
            ij11 = (i-1) * Size_j + (j-1);
            ij12 = (i-1) * Size_j + (j);
            ij13 = (i-1) * Size_j + (j+1);
            ij21 = (i) * Size_j + (j-1);
            ij22 = (i) * Size_j + (j);
            ij23 = (i) * Size_j + (j+1);
            ij31 = (i+1) * Size_j + (j-1);
            ij32 = (i+1) * Size_j + j;
            ij33 = (i+1) * Size_j + j+1;
            
            Val(2) = Val(2)+1;
            
            
        elseif i==Size_i && j> (KERNEL_SIZE -1)/2 && j<= (Size_j-(KERNEL_SIZE-1)/2)
            
            ij11 = (i-3) * Size_j + (j-1);
            ij12 = (i-3) * Size_j + (j);
            ij13 = (i-3) * Size_j + (j+1);
            ij21 = (i-2) * Size_j + (j-1);
            ij22 = (i-2) * Size_j + (j);
            ij23 = (i-2) * Size_j + (j+1);
            ij31 = (i-1) * Size_j + (j-1);
            ij32 = (i-1) * Size_j + j;
            ij33 = (i-1) * Size_j + (j+1);
            
            Val(8) = Val(8)+1;
            
            
        elseif i>(KERNEL_SIZE -1)/2  && i<=(Size_i-(KERNEL_SIZE-1)/2) ...
                &&j==1
            
            ij11 = (i-2) * Size_j + j;
            ij12 = (i-2) * Size_j + j+1;
            ij13 = (i-2) * Size_j +j+2;
            ij21 = (i-1) * Size_j + j;
            ij22 = (i-1) * Size_j + j+1;
            ij23 = (i-1) * Size_j + j+2;
            ij31 = i * Size_j + j;
            ij32 = i * Size_j +j+1;
            ij33 = i * Size_j + j+2;
            Val(4) = Val(4)+1;
            
            
        elseif i>(KERNEL_SIZE -1)/2  && i<=(Size_i-(KERNEL_SIZE-1)/2) ...
                &&j==Size_j
            ij11 = (i-2) * Size_j + j-2;
            ij12 = (i-2) * Size_j + j-1;
            ij13 = (i-2) * Size_j +j;
            ij21 = (i-1) * Size_j + j-2;
            ij22 = (i-1) * Size_j + j-1;
            ij23 = (i-1) * Size_j + j;
            ij31 = i * Size_j + j-2;
            ij32 = i * Size_j +j-1;
            ij33 = i * Size_j + j;
            Val(6) = Val(6)+1;
            
            
            
            
        else
            
            fprintf('Missing some elements in the derivatives of the smoothing term, the index of missing element is %d %d \n',i,j);
            
            
        end
        ID1 = [ID1; nCnt;nCnt;nCnt;nCnt;nCnt;nCnt;nCnt;nCnt;nCnt];

        ij = [ij11;ij12;ij13;ij21;ij22;ij23;ij31;ij32;ij33];       
        ID2 = [ID2;ij];
        Value = [Value; Val];
    end
       
    
end


J = sparse(ID1,ID2,Value);

HH = J'*J;

