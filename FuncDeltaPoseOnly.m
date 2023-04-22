function [DeltaP,Sum_Delta] = FuncDeltaPoseOnly(JP,JD,Error)

U = JP'*JP;
EP = -JP'*Error;

DeltaP = U\EP;
Sum_Delta = DeltaP'*DeltaP;

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
