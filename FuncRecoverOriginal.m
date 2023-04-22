function Original_Map = FuncRecoverOriginal(Map,Original_Size_i,Original_Size_j)
Size_i = Map.Size_i;
Size_j = Map.Size_j;
Rate = Original_Size_i/Size_i;
Grid = Map.Grid;
N = Map.N;

Original_Map.Grid = zeros(Original_Size_i,Original_Size_j);
Original_Map.N = zeros(Original_Size_i,Original_Size_j);

for i=1:Size_i
    for j=1:Size_j
        Original_Map.Grid(((i-1)*Rate+1):Rate*i,((j-1)*Rate+1):Rate*j) = Grid(i,j);
        Original_Map.N(((i-1)*Rate+1):Rate*i,((j-1)*Rate+1):Rate*j) = N(i,j);
    end
end

DTgrid = griddedInterpolant(Original_Map.Grid); % Interpolant
Original_Map.DgridG = DTgrid;
NTgrid = griddedInterpolant(Original_Map.N); % Interpolant
Original_Map.NgridG = NTgrid;
[Gdu,Gdv] = gradient(Original_Map.Grid);
Gdugrid = griddedInterpolant(Gdu);
Gdvgrid = griddedInterpolant(Gdv);
Original_Map.DgridGu = Gdugrid;
Original_Map.DgridGv = Gdvgrid;


end