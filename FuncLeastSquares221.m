function [Map,Pose,Reason,Info] = FuncLeastSquares221(Map,Pose,Scan,Odom)

[ErrorS,ErrorO,Sum_Error,MSE_Error,JP,JD,JO,IS,IO]= FuncDiffJacobian(Map,Pose,Scan,Odom);

fprintf('Initial Error is %.8f\n', MSE_Error);
Sum_Delta = 22;
MaxIter = 18;
MinError = 1e-8;
MinDelta = 1e-10;

HH2 = FuncMapConst(Map);
Lambda = 0.01;
% Lambda = 1.0;

%%
Iter = 1;
while Sum_Error>MinError && Sum_Delta>MinDelta && Iter<=MaxIter
    HH = HH2*Lambda;
    [DeltaP,DeltaD,Sum_Delta] = FuncDelta(JP,JD,JO,ErrorS,ErrorO,HH,Map,IS,IO);
    [Map,Pose] = FuncUpdate(Map,Pose,DeltaP,DeltaD);
    Map = FuncUpdateMapN(Map,Pose,Scan);
%     Map = FuncSmoothN2(Map,10);
%     [Map] = FuncInitialiseGridMap(Map,Pose,Scan);
    Map = FuncMapGrid(Map);
    [ErrorS,ErrorO,Sum_Error,MSE_Error,JP,JD,JO,IS,IO]= FuncDiffJacobian(Map,Pose,Scan,Odom);
    
%     if Lambda>=2e-8 && mod(Iter,8)==0
%         Lambda = Lambda/100;
%     end
    
    fprintf('Iterations %d Error %.8f\n', Iter,MSE_Error);
    Iter = Iter+1;
    
    %%
    figure(2);
    ab = exp(Map.Grid);
    PMat2 = ab./(ab+1);
    imshow(1-PMat2);
    
end

if Sum_Error<MinError
    Reason = 1;
elseif Sum_Delta<MinDelta
    Reason = 2;
elseif Iter>MaxIter
    Reason = 3;
else
    Reason = 4;
end

Info = sparse([]);
if Iter>0
    Jacobian = [JP,JD];
    Info = Jacobian'*Jacobian;
end
