function [DeltaP,DeltaD,Sum_Delta] = FuncDeltaLMSBA(U,V,W,EP,ED,Lambda)

% JP = JP(:,7:end);
% 
% U = JP'*JP;
% V = JD'*JD;
% W = JP'*JD;
% 
% Error = sparse(Error);
% 
% EP = -JP'*Error;
% ED = -JD'*Error;

%%
SizeU = size(U,1);
LambdaU = sparse(1:SizeU,1:SizeU,Lambda);
U = U+LambdaU;

SizeV = size(V,1);
LambdaV = sparse(1:SizeV,1:SizeV,Lambda);
V = V+LambdaV;

%%
II = [U,W;
      W',V];
EE = [EP;ED];

Delta = II\EE;
nP = SizeU;
DeltaP = Delta(1:nP);
DeltaD = Delta(nP+1:end);

Sum_Delta = Delta'*Delta;

%%
% [a,b] = size(V);
% [ID1,ID2,Val] = find(V);
% Val = Val+Lambda;
% Val2 = 1./Val;
% VI = sparse(ID1,ID2,Val2,a,b);
% 
% %% Test
% IDT = ID1-ID2;
% c = find(IDT);
% if size(c)
%     fprintf('V is not diagonal. ID is %d\n', c);
% end
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
