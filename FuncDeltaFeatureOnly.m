function [DeltaD,Sum_Delta] = FuncDeltaFeatureOnly(JD,Error,HH,Map)

V = JD'*JD;
ED = -JD'*Error;



XH0 = reshape(Map.Grid',[],1);
EH = -HH*XH0;

II = V+HH;
[aa]=size(II,1);
II = II+sparse(1:aa,1:aa,1)*1e-8;
EE = ED+EH;
DeltaD = II\EE;

Sum_Delta = DeltaD'*DeltaD;

end
