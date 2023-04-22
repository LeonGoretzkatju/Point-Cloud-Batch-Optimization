function Map = FuncGradientMap(Map)

Size_i = Map.Size_i;
Size_j = Map.Size_j;
Grid = Map.Grid;
remove_index_matrix = Map.remove_index_matrix;
Grid(remove_index_matrix) = nan;

% test
% grid_nan_index = find(isnan(Grid));
% Grid(grid_nan_index) = Original_Map.Grid(grid_nan_index);

% create variables Gdv and Gdu
Gdv = zeros(Size_i,Size_j);
Gdu = zeros(Size_i,Size_j);

for i=1:Size_i

    if i==1
        Gdv(i,:) = Grid(i+1,:) - Grid(i,:);
    elseif i==Size_i
        Gdv(i,:) = Grid(i,:) - Grid(i-1,:);
    else 
        Gdv(i,:) = (Grid(i+1,:) - Grid(i-1,:))/2;
    end

end

for j=1:Size_j

    if j==1
        Gdu(:,j) = Grid(:,j+1) - Grid(:,j);
    elseif j==Size_j
        Gdu(:,j) = Grid(:,j) - Grid(:,j-1);
    else 
        Gdu(:,j) = (Grid(:,j+1) - Grid(:,j-1))/2;
    end
end


% [Gdu_nan_i,Gdu_nan_j] = find(isnan(Gdu));
% [Gdv_nan_i,Gdv_nan_j] = find(isnan(Gdv));
% 
% ori_Gdu = Original_Map.DgridGu(Gdu_nan_i,Gdu_nan_j);
% ori_Gdv = Original_Map.DgridGv(Gdv_nan_i,Gdv_nan_j);
% 
% 
% 
% 
% Gdu_nan_index = (Gdu_nan_j-1)*Size_i + Gdu_nan_i;
% Gdv_nan_index = (Gdv_nan_j-1)*Size_i + Gdv_nan_i;
% 
% Gdu(Gdu_nan_index) = ori_Gdu;
% Gdv(Gdv_nan_index) = ori_Gdv;

Map.DgridGu = Gdu;
Map.DgridGv = Gdv;

end