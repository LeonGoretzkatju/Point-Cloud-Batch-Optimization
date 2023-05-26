function [DeltaD,Sum_Delta] = FuncDeltaFeatureOnly(JP,JD,Error,HH,Map)

V = JD'*JD;
ED = -JD'*Error;

XH0 = reshape(Map.Grid',[],1);
EH = -HH*XH0;

II = V+HH;
EE = ED+EH;
DeltaD = II\EE;

Sum_Delta = DeltaD'*DeltaD;

end
