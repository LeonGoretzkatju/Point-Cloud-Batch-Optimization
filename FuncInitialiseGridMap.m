function Map = FuncInitialiseGridMap(Map,Pose,Scan)

Size_i = Map.Size_i;
Size_j = Map.Size_j;
Grid = Map.Grid;
N = Map.N;
Scale = Map.Scale;
Origin = Map.Origin;

Pose = Pose';

nn = length(Scan); % the timestamp

for i = 1:nn
    posei = Pose(:,i);
    xy = Scan{i}.xy';
    Oddi = Scan{i}.Odd;
    
    Ri = FuncRMatrix2D(posei(3));
    Si = Ri'*xy+posei(1:2); % R *[X,Y]+t
    XY3 = (Si-Origin)/Scale+1; % 

    u = XY3(1,:);
    v = XY3(2,:);
%     ui = round(u);
%     vi = round(v);
    ui = fix(u);
    vi = fix(v);

%     a_0 = u - ui;
%     a_1 = ui+1-u;
%     b_0 = v - vi;
%     b_1 = vi+1-v;
%     Interp = [a_1.*b_1;a_0.*b_1;a_1.*b_0;a_0.*b_0]';
%     repmat_oddi = repmat(Oddi,1,4);
%     four_grid = Interp.*repmat_oddi;
%     four_grid_reshape = reshape(four_grid,size(four_grid,1)*size(four_grid,2),1);
%     four_N_reshape = reshape(Interp,size(four_grid,1)*size(four_grid,2),1);
%     
%     four_grid_u = [ui';ui'+1;ui';ui'+1];
%     four_grid_v = [vi';vi';vi'+1;vi'+1];
% 
%     UpdateOi = sparse(four_grid_v,four_grid_u,four_grid_reshape,Size_i,Size_j);
%     UpdateNi = sparse(four_grid_v,four_grid_u,four_N_reshape,Size_i,Size_j); % all odds are 1

    UpdateOi = sparse(vi,ui,Oddi,Size_i,Size_j);
    UpdateNi = sparse(vi,ui,1,Size_i,Size_j); % all odds are 1
    UpdateOi = full(UpdateOi);
    UpdateNi = full(UpdateNi);
    
    Grid = Grid+UpdateOi; % update the grid value
    N = N+UpdateNi; % all update is +1
end

%%
Map.Grid = Grid;
Map.N = N;
end