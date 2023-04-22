function Map = FuncMapGrid(Map,MODE_DERIVATIVES,MODE_MAP)

    
DTgrid = griddedInterpolant(Map.Grid); % Interpolant
Map.DgridG = DTgrid;


if strcmp(MODE_MAP,'CONTINUOUS')==1
    NTgrid = griddedInterpolant(Map.N); % Interpolant
else
    NTgrid = Map.N; % Interpolant
end

Map.NgridG = NTgrid;

if strcmp(MODE_DERIVATIVES,'DEFAULT')==1
    
    [Gdu,Gdv] = gradient(Map.Grid);
    if strcmp(MODE_MAP,'CONTINUOUS')==1
        Gdugrid = griddedInterpolant(Gdu);
        Gdvgrid = griddedInterpolant(Gdv);
        Map.DgridGu = Gdugrid;
        Map.DgridGv = Gdvgrid;
    else
        Map.DgridGu = Gdu;
        Map.DgridGv = Gdv;
    end

end

end