function Pic = ShowPicture(Map)

Grid = Map.Grid;
Size_i  = Map.Size_i;
Size_j = Map.Size_j;

Pic_Size_i = Size_i-1;
Pic_Size_j = Size_j-1;

Pic = zeros(Pic_Size_i,Pic_Size_j);

for i=1:Pic_Size_i
    for j=1:Pic_Size_j
        Pic(i,j) = (Grid(i,j) + Grid(i,j+1) + Grid(i+1,j) + Grid(i+1,j+1))/4;
    end
end

end