function HH = FuncMapSmooth(Map,KERNEL_SIZE)


AVERAGE_KERNEL = 1/(KERNEL_SIZE*KERNEL_SIZE);

H = AVERAGE_KERNEL*ones(KERNEL_SIZE,KERNEL_SIZE);


A = Map.Grid;
size_i = Map.Size_i;
size_j = Map.Size_j;


HH = zeros(size_i,size_j);


for i =1:size_i
    for j=1:size_j
        
        if i<= (size_i-KERNEL_SIZE+1) && j<= (size_j-KERNEL_SIZE+1)
            HH(i,j) = sum(sum(A(i:(i+KERNEL_SIZE-1),j:(j+KERNEL_SIZE-1)) .* H))/(KERNEL_SIZE*KERNEL_SIZE);
        elseif i> (size_i-KERNEL_SIZE+1) && j<= (size_j-KERNEL_SIZE+1)
            
            HH(i,j) = sum(sum(A((size_i-KERNEL_SIZE+1):(size_i),j:(j+KERNEL_SIZE-1)) .* H))/(KERNEL_SIZE*KERNEL_SIZE);
        
        elseif i<= (size_i-KERNEL_SIZE+1) && j> (size_j-KERNEL_SIZE+1)
            HH(i,j) = sum(sum(A(i:(i+KERNEL_SIZE-1),(size_j-KERNEL_SIZE+1):(size_j)) .* H))/(KERNEL_SIZE*KERNEL_SIZE);
            
        else
            HH(i,j) = sum(sum(A((size_i-KERNEL_SIZE+1):(size_i),(size_j-KERNEL_SIZE+1):(size_j)) .*H))/(KERNEL_SIZE*KERNEL_SIZE);
            
        end
        
    end
end

end