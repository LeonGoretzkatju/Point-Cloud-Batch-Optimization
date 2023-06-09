function [DeltaP,DeltaD,Sum_Delta] = FuncDelta3DPoseSmooth(JP,JS,JD,ErrorS,Map,IS,Lambda)
    JP = JP(:,7:end);

    HH = JS'*JS*Lambda;
    
    U = JP'*IS*JP;
    V = JD'*IS*JD;
    W = JP'*IS*JD;

    ErrorS = sparse(ErrorS);
    
    EP = -JP'*IS*ErrorS;
    ED = -JD'*IS*ErrorS;
    
    XH0 = reshape(Map.Grid',[],1);
    
    EH = -HH*XH0;
    
    II = [U,W;
          W',V+HH];

%     Feature_Only = V+HH;
%     Error_Feature_Only = ED+EH;
%     DeltaD = Feature_Only\Error_Feature_Only;
%     DeltaP = U\EP;
      
    EE = [EP;ED+EH];
    Delta = II\EE;    
    
    nP = size(JP,2);
    DeltaP = Delta(1:nP);
    DeltaD = Delta(nP+1:end);
    
    clearvars II EE
    
    Sum_Delta = DeltaP'*DeltaP;
end