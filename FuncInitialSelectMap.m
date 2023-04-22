function Select_Map = FuncInitialSelectMap(Size_i,Size_j,Scale,Origin,Pose,Select_Scan,index_select)

%% Create Selected Map
Select_Map.Size_i = Size_i;
Select_Map.Size_j = Size_j;
Select_Map.Grid = zeros(Size_i,Size_j);
Select_Map.N = zeros(Size_i,Size_j);
Select_Map.Scale = Scale;
Select_Map.Origin = Origin;

Grid = Select_Map.Grid;
N = Select_Map.N;

Pose = Pose';

nn = length(Select_Scan); % the timestamp

parfor i = 1:nn
    posei = Pose(:,i);
    xy = Select_Scan{i}.xy';
    Oddi = Select_Scan{i}.Odd;
    
    Ri = FuncRMatrix2D(posei(3));
    Si = Ri'*xy+posei(1:2); % R *[X,Y]+t
    XY3 = (Si-Origin)/Scale+1; % 
    u = XY3(1,:);
    v = XY3(2,:);
%     ui = round(u);
%     vi = round(v);
    ui = fix(u);
    vi = fix(v);

    a_0 = u - ui;
    a_1 = ui+1-u;
    b_0 = v - vi;
    b_1 = vi+1-v;
    Interp = [a_1.*b_1;a_0.*b_1;a_1.*b_0;a_0.*b_0]';
    repmat_oddi = repmat(Oddi,1,4);
    four_grid = Interp.*repmat_oddi;
    four_grid_reshape = reshape(four_grid,size(four_grid,1)*size(four_grid,2),1);
    four_N_reshape = reshape(Interp,size(four_grid,1)*size(four_grid,2),1);
    
    four_grid_u = [ui';ui'+1;ui';ui'+1];
    four_grid_v = [vi';vi';vi'+1;vi'+1];

    UpdateOi = sparse(four_grid_v,four_grid_u,four_grid_reshape,Size_i,Size_j);
    UpdateNi = sparse(four_grid_v,four_grid_u,four_N_reshape,Size_i,Size_j); % all odds are 1

    
%     UpdateOi = sparse(vi,ui,Oddi,Size_i,Size_j);
%     UpdateNi = sparse(vi,ui,1,Size_i,Size_j); % all odds are 1
    UpdateOi = full(UpdateOi);
    UpdateNi = full(UpdateNi);
    
    Grid = Grid+UpdateOi; % update the grid value
    N = N+UpdateNi; % all update is +1
end

%%
% show selected resolution map
figure(3)
ab = exp(Grid);
PMat3 = ab./(ab+1);
fig_3 = 1-PMat3;
imshow(fig_3);

full_index = sparse(index_select(:,1),index_select(:,2),1,Size_i,Size_j);
[full_x,full_y] = find(full_index==0);
remove_index_matrix = find(full_index==0);
index_remove = [full_x,full_y];
select_index_matrix = find(full_index==1);

% Creat Select_Map structure
Select_Map.Grid = Grid;
Select_Map.N = N;
Select_Map.Select_Point = index_select;
Select_Map.Remove_Point = index_remove;
Select_Map.select_index_matrix = select_index_matrix;
Select_Map.remove_index_matrix = remove_index_matrix;

Select_Point = Select_Map.Select_Point;
Select_Variables = (Select_Point(:,1)-1)*Select_Map.Size_j + Select_Point(:,2);
Sort_Select_Variables = sort(Select_Variables);
Select_Map.Sort_Select_Variables = Sort_Select_Variables;

end