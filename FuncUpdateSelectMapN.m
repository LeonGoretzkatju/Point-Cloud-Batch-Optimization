function Select_Map = FuncUpdateSelectMapN(Select_Map,Pose,Select_Scan)

Size_i = Select_Map.Size_i;
Size_j = Select_Map.Size_j;
N = zeros(Size_i,Size_j);
Scale = Select_Map.Scale;
Origin = Select_Map.Origin;

Pose = Pose';

nn = length(Select_Scan);

parfor i = 1:nn
    posei = Pose(:,i);
    xy = Select_Scan{i}.xy';    
    Ri = FuncRMatrix2D(posei(3));
    Si = Ri'*xy+posei(1:2);
    XY3 = (Si-Origin)/Scale+1;
    
    u = XY3(1,:);
    v = XY3(2,:);
    ui = fix(u);
    vi = fix(v);
    
    UpdateNi = sparse(vi,ui,1,Size_i,Size_j);
    UpdateNi = full(UpdateNi);
    N = N+UpdateNi;
end

%%
Select_Map.N = N;
end