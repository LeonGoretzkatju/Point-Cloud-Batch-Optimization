function [Map] = FuncInitialiseGridMap_New(Map,Pose,Scan,MAX_RANGE)

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
    [x,y] = find(Scan{i}>0|Scan{i}<0);
    xy = [x,y];
    index = (y-1) * 2 * MAX_RANGE + x;
    Oddi = Scan{i}(index);
%     x = x - MAX_RANGE;
%     y = y- MAX_RANGE;
    xy = xy - MAX_RANGE;
%     Scan{i}.xy = xy;
%     Scan{i}.Odd = Oddi;
    xy = xy';

    
    Ri = FuncRMatrix2D(posei(3));
    Si = Ri'*xy+posei(1:2); % R *[X,Y]+t
    XY3 = (Si-Origin)/Scale+1; % 
%     XY3 = XY3( ~ isnan(XY3));
%     Oddi = Oddi( ~ isnan(XY3(1,:)));
    u = XY3(1,:);
    v = XY3(2,:);
    ui = round(u);
    vi = round(v);
    
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