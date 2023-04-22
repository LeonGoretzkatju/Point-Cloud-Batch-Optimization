function [Map] = FuncInitialiseGridMap2(Map,Pose,D,K,IniD)

Size_i = Map.Size_i;
Size_j = Map.Size_j;
Grid = Map.Grid;
Scale = Map.Scale;
Origin = Map.Origin;
KI = inv(K);

nD = length(D);
[nv,nu] = size(D{1});

%%%%%%%%%%%%%%%%%%%
Grid(:,:) = IniD;
%%%%%%%%%%%%%%%%%%%

%%

for i=1:nD
    Di = D{i};
    Ri = RMatrixYPR22(Pose(i,1),Pose(i,2),Pose(i,3));
    Ti = Pose(i,4:6)';
    for j=1:nu
        for k=1:nv
            x = [j;k;1]*Di(k,j);
            X = Ri'*KI*x+Ti;
            
            ID = round((X(1:2)-Origin)/Scale)+1;
            if Grid(ID(2),ID(1))==IniD %%0
                Grid(ID(2),ID(1)) = X(3);
            end
        end
    end
end

%%
Map.Grid = Grid;

end