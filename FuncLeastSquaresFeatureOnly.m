
function [Map,Pose,Reason,Info] = FuncLeastSquaresFeatureOnly(Map,Pose,D,K)

% [Error,Sum_Error,MSE_Error,Map]= FuncDiff(Map,Pose,D,K);
[Error,Sum_Error,MSE_Error,JP,JD]= FuncDiffJacobian(Map,Pose,D,K);
fprintf('Initial Error is %.8f\n', MSE_Error);
Sum_Delta = 22;
MaxIter = 7;
MinError = 1e-8;
MinDelta = 1e-10;

HH = FuncMapConst(Map);

HH = HH*100;

%%
Iter = 0;
while Sum_Error>MinError && Sum_Delta>MinDelta && Iter<=MaxIter
%     [JP,JD] = FuncJacobian(Map,Pose,D,Dgrid,K);
    [DeltaD,Sum_Delta] = FuncDeltaFeatureOnly(JP,JD,Error,HH,Map);
    [Map,Pose] = FuncUpdateFeatureOnly(Map,Pose,DeltaD);
    Map = FuncMapGrid(Map);
    [Error,Sum_Error,MSE_Error,JP,JD]= FuncDiffJacobian(Map,Pose,D,K);
    Iter = Iter+1;
    fprintf('Iterations %d Error %.8f\n', Iter,MSE_Error);    
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