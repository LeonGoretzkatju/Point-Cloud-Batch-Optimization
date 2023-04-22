clc;
clear;

%%
load depth.mat;
load rgb.mat;

%%
nD = length(D);

for i=1:nD
    [Gdu,Gdv] = gradient(D{i});
    
    DTgrid = griddedInterpolant(D{i});
    Gdugrid = griddedInterpolant(Gdu);
    Gdvgrid = griddedInterpolant(Gdv);
    
    Dgrid{i}.G = DTgrid;
    Dgrid{i}.Gu = Gdugrid;
    Dgrid{i}.Gv = Gdvgrid;
    
    
    [GIu,GIv] = gradient(double(I{i}));
    
    ITgrid = griddedInterpolant(double(I{i}));
    GIugrid = griddedInterpolant(GIu);
    GIvgrid = griddedInterpolant(GIv);
    
    Igrid{i}.G = ITgrid;
    Igrid{i}.Gu = GIugrid;
    Igrid{i}.Gv = GIvgrid;
end

% save Dgrid.mat Dgrid;
% save Igrid.mat Igrid; %Image grid is not ready. RGB and Greyscale