function [DeltaD,Sum_Delta] = FuncDeltaFeatureOnly(JP,JD,Error,HH,Map)

V = JD'*JD;
ED = -JD'*Error;

XH0 = reshape(Map.Grid',[],1);
EH = -HH*XH0;

II = V+HH;
EE = ED+EH;
DeltaD = II\EE;

Sum_Delta = DeltaD'*DeltaD;

%%
% [a,b] = size(V);
% [ID1,ID2,Val] = find(V);
% Val2 = 1./Val;
% VI = sparse(ID1,ID2,Val2,a,b);
% 
% 
% DeltaD = VI*ED;
% Sum_Delta = DeltaD'*DeltaD;

%%
% U = JP'*JP;
% V = JD'*JD;
% W = JP'*JD;
% 
% Error = sparse(Error);
% 
% EP = -JP'*Error;
% ED = -JD'*Error;
% 
% %%
% [a,b] = size(V);
% [ID1,ID2,Val] = find(V);
% Val2 = 1./Val;
% VI = sparse(ID1,ID2,Val2,a,b);
% 
% %%
% S = U-W*VI*W';
% ES = EP-W*VI*ED;
% 
% %%
% DeltaP = S\ES;
% DeltaD = VI*(ED-W'*DeltaP);
% 
% Sum_Delta = DeltaP'*DeltaP+DeltaD'*DeltaD;

end
