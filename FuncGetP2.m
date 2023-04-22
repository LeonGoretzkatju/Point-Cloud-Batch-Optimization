function P2 = FuncGetP2(Map,Pose,V)

%%
PP1 = sum(Pose(1:end,:).^2,'all');

% [ID1,ID2,Val] = find(V);
DD = reshape(Map.Grid',[],1);
% DD2 = DD(ID1);
PP2 = DD'*DD;

P2 = sqrt(PP1+PP2);