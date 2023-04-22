function [Map,Pose,Reason,Info] = FuncLeastSquaresLMSBA(Map,Pose,D,K)

MaxIter = 88;
Factor = 2;
t = 1e-3;
e1 = 1e-12;
e2 = 1e-12;
e3 = 1e-12;
e4 = 0;
Stop = 0;
Reason = 0;
Iter = 0;

% [Error,Sum_Error,MSE_Error,Map]= FuncDiff(Map,Pose,D,Dgrid,K);
[Error,Sum_Error,MSE_Error,JP,JD]= FuncDiffJacobian(Map,Pose,D,K);
HH = FuncMapConst(Map);

fprintf('Initial Error is %.8f\n', MSE_Error);
ErrorPre = Sum_Error;

% [JP,JD] = FuncJacobian(Map,Pose,D,Dgrid,K);
XH0 = reshape(Map.Grid',[],1);
EH = -HH*XH0;
JP = JP(:,7:end);
U = JP'*JP;
V = JD'*JD+HH;
W = JP'*JD;
Error = sparse(Error);
EP = -JP'*Error;
ED = -JD'*Error+EH;
Lambda = t*max(max(diag(U)),max(diag(V)));
g = max(max(abs(EP)),max(abs(ED)));

if g<=e1
    Stop = 1;
    Reason = 1;
end

while Stop~=1 && Iter<=MaxIter
    Iter = Iter+1;
    P = -1;
    while Stop~=1 && P<=0
    [DeltaP,DeltaD,Sum_Delta] = FuncDeltaLMSBA(U,V,W,EP,ED,Lambda);
    P2 = FuncGetP2(Map,Pose,V);
    if Sum_Delta<=e2*(P2+e2)
        Stop = 1;
        Reason = 2;
    else
        [Map,Pose] = FuncUpdate(Map,Pose,DeltaP,DeltaD);
        Map = FuncMapGrid(Map);
%         [Error,Sum_Error,MSE_Error,Map]= FuncDiff(Map,Pose,D,Dgrid,K);
        [Error,Sum_Error,MSE_Error,JP,JD]= FuncDiffJacobian(Map,Pose,D,K);
        Delta = [DeltaP;DeltaD];
        P = (ErrorPre-Sum_Error)/(Delta'*(Lambda*Delta+[EP;ED]));
        if P>0
            if sqrt(ErrorPre)-sqrt(Sum_Error)<e4*sqrt(ErrorPre)
                Stop = 1;
                Reason = 3;
            end
%             [JP,JD] = FuncJacobian(Map,Pose,D,Dgrid,K);
            XH0 = reshape(Map.Grid',[],1);
            EH = -HH*XH0;
            JP = JP(:,7:end);
            U = JP'*JP;
            V = JD'*JD+HH;
            W = JP'*JD;
            Error = sparse(Error);
            EP = -JP'*Error;
            ED = -JD'*Error+EH;
            g = max(max(abs(EP)),max(abs(ED)));
            if Stop ==1 || g<=e1
                Stop = 1;
                Reason = 1;
            end
            Lambda = Lambda*max(1/3,1-(2*P-1)^3);
            Factor = 2;
        else    
            DeltaP = -DeltaP;
            DeltaD = -DeltaD;
            [Map,Pose] = FuncUpdate(Map,Pose,DeltaP,DeltaD);
            Map = FuncMapGrid(Map);
            Lambda = Factor*Lambda;
            Factor = Factor*2;
        end
    end
    end
    if sqrt(Sum_Error)<=e3
        Stop = 1;
        Reason = 4;
    end
    if P>0
        fprintf('Iterations %d Error %.8f\n', Iter, MSE_Error);
        ErrorPre = Sum_Error;
    end
end
    
Info = sparse([]);
if Iter>0
    Jacobian = [JP,JD];
    Info = Jacobian'*Jacobian;
end