function [Map,Pose,Reason,Info] = FuncLeastSquares_Simu23_GN(Map,Pose,D,K,MODE_DERIVATIVES,MODE_MAP,FILE_DIRECTORY)
    MaxIter = 50;
    MinError = 1e-5;
    MinDelta = 1e-10;
    DOWN_TIME = 38;
    Sum_Delta = 22;
    Iter = 0;
    tic;
    [Error,MSE_Error,Sum_Error,JP,IS,JD] = FuncDiffJacobian_All_Simu23(Map,Pose,D,K,MODE_MAP,MODE_DERIVATIVES);
    Iter_time = toc;
    fprintf('Initial Error is %.8f Time Use %f\n\n', MSE_Error, Iter_time);
    txt=strcat(FILE_DIRECTORY,'/result1.txt'); 
    fid=fopen(txt,'a');
    fprintf(fid,'Initial Error is %.8f Time Use %f\n\n', MSE_Error, Iter_time);
    fclose(fid);

    while MSE_Error>MinError && Sum_Delta>MinDelta && Iter<=MaxIter 
        tic;
        if Iter<=8
            Lambda = 2.0;
        elseif Iter > 8 && Iter <= DOWN_TIME
            Lambda = 0.1;
        elseif Iter > DOWN_TIME && Iter<=(DOWN_TIME+28) 
            Lambda = 0.01;
        else
            Lambda = 0.001;
        end
        HH2 = FuncMapConst(Map); 
        HH = HH2*Lambda;
        [DeltaP,DeltaD,Sum_Delta] = FuncDelta3D(JP,JD,Error,HH,Map,IS,Lambda);
        [Map,Pose] = FuncUpdate(Map,Pose,DeltaP,DeltaD);
        [Map] = FuncMapGrid(Map,MODE_DERIVATIVES,MODE_MAP);
        tic;
        [Error,MSE_Error,Sum_Error,JP,IS,JD] = FuncDiffJacobian_All_Simu23(Map,Pose,D,K,MODE_MAP,MODE_DERIVATIVES);
        Iter_time = toc;
        fprintf('MSE Error is %.8f Time Use %f Iter Use %d \n\n', MSE_Error, Iter_time, Iter);
        txt=strcat(FILE_DIRECTORY,'/result.txt'); 
        fid=fopen(txt,'a');
        fprintf(fid,'MSE Error is %.8f Time Use %f Iter Use %d \n\n', MSE_Error, Iter_time, Iter);
        fclose(fid);
        Iter = Iter + 1;
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
    
    if Iter>0
        Jacobian = [JP,JD];
        Info = Jacobian'*Jacobian;
    end
end