function [Map,Pose,Reason,Info] = FuncLeastSquares(Map,Pose,Scan)
tic;
[Error,Sum_Error,MSE_Error,JP,JD,IS]= FuncDiffJacobian_No_Odom(Map,Pose,Scan);
Iter_time = toc;
fprintf('Initial Error is %.8f Time Use %d\n\n', MSE_Error, Iter_time);
txt=strcat('./b0_result/result.txt'); 
fid=fopen(txt,'a');
fprintf(fid,'Initial Error is %.8f Time Use %d\n\n', MSE_Error, Iter_time);
fclose(fid);

Sum_Delta = 22;
MaxIter = 150;
MinError = 1e-8;
MinDelta = 1e-10;

% XH0 = reshape(Map.Grid',[],1);
HH2 = FuncMapConst(Map);
% HH = HH*0.01;


%%
Iter = 1;
while Sum_Error>MinError && Sum_Delta>MinDelta && Iter<=MaxIter
    tic;
    if Iter<=18
        Lambda = 0.1;
    elseif Iter<=36
        Lambda = 0.01;
    else
        Lambda = 0.0001;
    end

    HH = HH2*Lambda;
    [DeltaP,DeltaD,Sum_Delta] = FuncDelta_No_Odom(JP,JD,Error,HH,Map,IS);
    [Map,Pose] = FuncUpdate(Map,Pose,DeltaP,DeltaD);
    Map = FuncUpdateMapN(Map,Pose,Scan);
    Map = FuncMapGrid(Map);
    [Error,Sum_Error,MSE_Error,JP,JD,IS]= FuncDiffJacobian_No_Odom(Map,Pose,Scan);
%     Lambda = Lambda/1.5;
%     if Lambda>=2e-8
%         Lambda = Lambda/2;
%     end
    full_Sum_Delta = full(Sum_Delta);
    Iter_time = toc;
    fprintf('Iterations %d Lambda %3f Error %.8f Delta %.8f Iter Time Use %d\n\n', Iter,Lambda,MSE_Error,full_Sum_Delta,Iter_time);
    
%     txt=strcat('./carpark_result/result.txt'); 
    fid=fopen(txt,'a');
    fprintf(fid,'Iterations %d Lambda %3f Error %.8f Delta %.8f\n Iter Time Use %d\n\n', Iter,Lambda,MSE_Error,full_Sum_Delta,Iter_time);
    fclose(fid);

    Iter = Iter+1;
    figure(2);
    ab = exp(Map.Grid);
    PMat2 = ab./(ab+1);
    fig_2 = 1-PMat2;
    imshow(fig_2);
    filename=['./b0_result/fig_',num2str(Iter-1),'.jpg'];
    imwrite(fig_2,filename,'jpg');
    name_map=['./b0_result/Map_',num2str(Iter-1),'.mat'];
    name_pose=['./b0_result/Pose_',num2str(Iter-1),'.mat'];
        
    save (name_map,'Map');
    save (name_pose,'Pose');
  
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
