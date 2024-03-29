function [DeltaP,DeltaD,Sum_Delta] = FuncDelta(JP,JD,JO,ErrorS,ErrorO,HH,Map,IS,IO,Lambda,Lambda_O)

THRESHOLD = 4;

JP = JP(:,4:end);
JO = JO(:,4:end);

[Size_i,Size_j] = size(Map.Grid);

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Test 2021-01-11
if Lambda ==0
    index_JD = (JD~=0);
    sum_JD = sum(index_JD,1);
    index_JD = find(sum_JD<THRESHOLD);
    index_Recover = find(sum_JD >=THRESHOLD);

    JD(:,index_JD) = [];

    HH(:,index_JD) = [];
    HH(index_JD,:) = [];
end



U = JP'*IS*JP+Lambda_O*JO'*IO*JO;
V = JD'*IS*JD;
spy(U);
spy(V);

W = JP'*IS*JD;


ErrorS = sparse(ErrorS);
ErrorO = sparse(ErrorO);

EP = -JP'*IS*ErrorS-Lambda_O*JO'*IO*ErrorO;
ED = -JD'*IS*ErrorS;

XH0 = reshape(Map.Grid',[],1);

if Lambda==0

    XH0(index_JD,:) = [];
end

EH = -HH*XH0;

%%


II = [U,W;
      W',V+HH];
  
EE = [EP;ED+EH];
Delta = II\EE;

nP = size(JP,2);
DeltaP = Delta(1:nP);
DeltaD = Delta(nP+1:end);

if Lambda==0
    y_Recover = ones(1,length(index_Recover));
    DeltaD = sparse(index_Recover,y_Recover,DeltaD',Size_i*Size_j,1); 
    Delta = [DeltaP;DeltaD];
end

clearvars II EE

Sum_Delta = Delta'*Delta;

end
