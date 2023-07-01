function [Map,Pose,Reason,Info] = FuncLeastSquares_Simu23(Map,Pose,D,K,MODE_DERIVATIVES,MODE_MAP,FILE_DIRECTORY)
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
    tic;
    [Error,MSE_Error,Sum_Error,JP,IS,JD] = FuncDiffJacobian_All_Simu23(Map,Pose,D,K,MODE_MAP,MODE_DERIVATIVES);
    Iter_time = toc;
    fprintf('Initial Error is %.8f Time Use %f\n\n', MSE_Error, Iter_time);
    txt=strcat(FILE_DIRECTORY,'/result.txt'); 
    fid=fopen(txt,'a');
    fprintf(fid,'Initial Error is %.8f Time Use %f\n\n', MSE_Error, Iter_time);
    fclose(fid);

    figure;
    xlabel('Iteration');
    ylabel('MSE Error');
    title('MSE Error vs Iteration');
    hold on;

    HH = FuncMapConst(Map);
    ErrorPre = Sum_Error;
    
    XH0 = reshape(Map.Grid',[],1);
    EH = -HH*XH0;
    JP = JP(:,7:end);
    U = JP'*IS*JP;
    V = JD'*IS*JD+HH;
    W = JP'*IS*JD;
    Error = sparse(Error);
    EP = -JP'*IS*Error;
    ED = -JD'*IS*Error+EH;
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
                [Map] = FuncMapGrid(Map,MODE_DERIVATIVES,MODE_MAP);
        %         [Error,Sum_Error,MSE_Error,Map]= FuncDiff(Map,Pose,D,Dgrid,K);
                tic;
                [Error,MSE_Error,Sum_Error,JP,IS,JD] = FuncDiffJacobian_All_Simu23(Map,Pose,D,K,MODE_MAP,MODE_DERIVATIVES);
                Iter_time = toc;
                fprintf('MSE Error is %.8f Time Use %f\n\n', MSE_Error, Iter_time);
                txt=strcat(FILE_DIRECTORY,'/result.txt'); 
                fid=fopen(txt,'a');
                fprintf(fid,'MSE Error is %.8f Time Use %f\n\n', MSE_Error, Iter_time);
                fclose(fid);
    
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
                    U = JP'*IS*JP;
                    V = JD'*IS*JD+HH;
                    W = JP'*IS*JD;
                    Error = sparse(Error);
                    EP = -JP'*IS*Error;
                    ED = -JD'*IS*Error+EH;
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
                    [Map] = FuncMapGrid(Map,MODE_DERIVATIVES,MODE_MAP);
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
%             fprintf('Iterations %d Error %.8f\n', Iter, MSE_Error);
            ErrorPre = Sum_Error;
        end
        plot(Iter, MSE_Error, 'b.');
        if Iter>1
            plot([Iter-1,Iter],[MSE_Error_Pre,MSE_Error],'r-');
        end
        MSE_Error_Pre = MSE_Error;
        drawnow;
    end
        
    Info = sparse([]);
    if Iter>0
        Jacobian = [JP,JD];
        Info = Jacobian'*Jacobian;
    end
end