function [DeltaP,DeltaD,Sum_Delta] = FuncSelMapDelta(JO,ErrorO,IO,JP_Sel,JD_Sel,ErrorS_Sel,Select_Map,HH_Select,Lambda_O)

JO = JO(:,4:end);
JP_Sel = JP_Sel(:,4:end);
Sigma_S = 1;%0.5;
nS = length(JP_Sel);
IS_Sel = sparse(1:nS,1:nS,1./Sigma_S.^2);

U = JP_Sel'*IS_Sel*JP_Sel + Lambda_O*JO'*IO*JO;
W = JP_Sel'*IS_Sel*JD_Sel;


V_1 = JD_Sel'*IS_Sel*JD_Sel;
V = V_1+HH_Select;
ErrorS_Sel = sparse(ErrorS_Sel);
ErrorO = sparse(ErrorO);

EP = -JP_Sel'*IS_Sel*ErrorS_Sel-Lambda_O*JO'*IO*ErrorO;

ED_Sel = -JD_Sel'*IS_Sel*ErrorS_Sel;
% calculate EH_Sel
sort_select_map = reshape(Select_Map.Grid',[],1);
Sort_Select_Variables = Select_Map.Sort_Select_Variables;
XH0_Sel = sort_select_map(Sort_Select_Variables);
EH_Sel = -HH_Select*XH0_Sel;
ED = ED_Sel+EH_Sel;
%%
% create sparse zero matrix 
II = [U,W;
      W',V];
  
EE = [EP;ED];

Delta = II\EE;

nP = size(JP_Sel,2);
DeltaP = Delta(1:nP);
DeltaD = Delta(nP+1:end);

Sum_Delta = Delta'*Delta;

end
