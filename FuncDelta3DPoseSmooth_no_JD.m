function [DeltaP, DeltaD, Sum_Delta] = FuncDelta3DPoseSmooth_no_JD(JP,JS,ErrorS,Map,IS,Lambda)
    JP = JP(:,7:end);

    HH = JS'*JS*Lambda;
    
    U = JP'*IS*JP;
%     W = JP'*IS*JD;
    W = sparse(size(U,1),size(HH,2));

    ErrorS = sparse(ErrorS);
    
    EP = -JP'*IS*ErrorS;
    
    XH0 = reshape(Map.Grid',[],1);
    
    EH = -HH*XH0;
    
    II = [U,W;
          W',HH];
      
    EE = [EP;EH];
    Delta = II\EE;
    
    nP = size(JP,2);
    DeltaP = Delta(1:nP);
    DeltaD = Delta(nP+1:end);
    
    clearvars II EE
    
    Sum_Delta = Delta'*Delta;
end