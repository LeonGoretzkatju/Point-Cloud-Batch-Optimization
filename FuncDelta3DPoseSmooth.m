function [DeltaP,DeltaD,Sum_Delta] = FuncDelta3DPoseSmooth(JP,JS,ErrorS,Map,IS,Lambda)
    JP = JP(:,7:end);

    HH = JS'*JS*Lambda;
    
    U = JP'*JP;
    V = HH;
    
    rows = size(U,1);
    cols = size(V,2);
    W = sparse(rows,cols);
    
    
    ErrorS = sparse(ErrorS);
    
    EP = -JP'*IS*ErrorS;
%     ED = -JD'*IS*ErrorS;
    
    XH0 = reshape(Map.Grid',[],1);
    
    EH = -HH*XH0;
    
    II = [U,W;
          W',V];
      
    EE = [EP;EH];
    Delta = II\EE;
    
    nP = size(JP,2);
    DeltaP = Delta(1:nP);
    DeltaD = Delta(nP+1:end);
    
    clearvars II EE
    
    Sum_Delta = Delta'*Delta;
end