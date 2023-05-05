function [Pose,Reason,Info,index] = FuncLeastSquaresBatch(Map,Pose,Scan,...
    Odom,MODE_DERIVATIVES,FILE_DIRECTORY,MAX_ITER,MODE_MAP,...
   Low_Scan,Size_i,Size_j,Scale,Origin,DOWN_TIME,Lambda_O,...
   Lambda_N,MULTI_MODE,EVALUTION_GT,Rate,KERNEL_SIZE)
tic;
[ErrorS,ErrorO,Sum_Error,MSE_Error,...
    JP,JD,JO,IS,IO]= FuncDiffJacobian(Map,Pose,Low_Scan,Odom,MODE_DERIVATIVES,...
    MODE_MAP);
Iter_time = toc;
fprintf('Initial Error is %.8f Time Use %f\n\n', MSE_Error, Iter_time);
txt=strcat(FILE_DIRECTORY,'/result.txt'); 
fid=fopen(txt,'a');
fprintf(fid,'Initial Error is %.8f Time Use %f\n\n', MSE_Error, Iter_time);
fclose(fid);
total_time = Iter_time;
Sum_Delta = 22;
MaxIter = MAX_ITER;
MinError = 1e-8;
MinDelta = 1e-10;
tem_MSE = 10;
Over_Num = 0;

Iter = 1;
Iter_minError = 10;
index = [];
while MSE_Error>MinError && Sum_Delta>MinDelta && Iter<=MaxIter 
    tic;
    if Iter<=18
        Lambda = 0.000001;
    elseif Iter<=DOWN_TIME 
        Lambda = 0.0000001;
%     elseif Iter<=DOWN_TIME 
%         Lambda = 0.000001;
    elseif Iter<=(DOWN_TIME+8) 
        Lambda = 0.001;
%     elseif Iter<=(DOWN_TIME+8)
%         Lambda = 0.00001;
    else
        Lambda = 0.0001;
    end


    HH2 = FuncMapConst(Map); 
    HH = HH2*Lambda;

    if Iter < (DOWN_TIME+1)
       [DeltaP,DeltaD,Sum_Delta] = FuncDelta(JP,JD,JO,ErrorS,ErrorO,HH,Map,IS,IO,Lambda,Lambda_O);
       [Map,Pose] = FuncUpdate(Map,Pose,DeltaP,DeltaD);
       Map = FuncUpdateMapN(Map,Pose,Low_Scan);
       Map = FuncSmoothN2(Map,10,HH2);
       Map = FuncMapGrid(Map,MODE_DERIVATIVES,MODE_MAP);   
       [ErrorS,ErrorO,Sum_Error,MSE_Error,JP,JD,JO,IS,IO]= FuncDiffJacobian(Map,Pose,Low_Scan,Odom,MODE_DERIVATIVES,MODE_MAP);

    elseif Iter ==(DOWN_TIME+1)
            if strcmp(EVALUTION_GT,'TRUE')==1
                FuncEval(Pose,Pose_GT,txt);
            end
        if strcmp(MULTI_MODE,'TRUE')==1
            % selected map part
            Low_Map = FuncCreateGridMap(round(Size_i/Rate),round(Size_j/Rate),Scale*Rate,Origin);
            Low_Map = FuncInitialiseGridMap(Low_Map,Pose,Low_Scan);
            tic;           
            [p_1,p_2,p_3,p_4,index_select] = FuncCalBound(Low_Map,Rate,KERNEL_SIZE);
            local_index = FuncInvProj(Map,Pose,p_1,p_2,p_3,p_4);
            Select_Scan = FuncSelectScan(Scan,local_index);
            Select_Map = FuncInitialSelectMap(Size_i,Size_j,Scale,Origin,Pose,Select_Scan,index_select);
            HH2_Select = FuncSelectMapConst(Select_Map);
            HH_Select = HH2_Select*Lambda; 
            Select_Map = FuncSmoothSelectN2(Select_Map,Lambda_N,HH2_Select);
            select_time = toc;
            fprintf('The Select Time is %f \n\n',select_time);
            txt=strcat(FILE_DIRECTORY,'/result.txt'); 
            fid=fopen(txt,'a');
            fprintf(fid,'The Select Time is %f \n\n',select_time);
            fclose(fid);
            [ErrorS_Sel,JP_Sel,JD_Sel] = FuncDiffSelectJacobian(Select_Map,Pose,Select_Scan);
            [DeltaP,DeltaD,Sum_Delta] = FuncSelMapDelta(JO,ErrorO,IO,JP_Sel,JD_Sel,ErrorS_Sel,Select_Map,HH_Select,Lambda_O);
            [Select_Map,Pose] = FuncSelectUpdate(Select_Map,Pose,DeltaP,DeltaD);
            if strcmp(EVALUTION_GT,'TRUE')==1
                FuncEval(Pose,Pose_GT,txt);
            end
           
            Select_Map = FuncUpdateSelectMapN(Select_Map,Pose,Select_Scan);
            Select_Map = FuncSmoothSelectN2(Select_Map,Lambda_N,HH2_Select);

        else
            Map = FuncCreateGridMap(Size_i,Size_j,Scale,Origin);
            Low_Scan = Scan;
            Map = FuncInitialiseGridMap(Map,Pose,Low_Scan);
            HH2 = FuncMapConst(Map);
            HH = HH2*Lambda;
            Map = FuncSmoothN2(Map,10,HH2);
            Map = FuncMapGrid(Map,MODE_DERIVATIVES,MODE_MAP);
            [ErrorS,ErrorO,Sum_Error,MSE_Error,JP,JD,JO,IS,IO]= FuncDiffJacobian(Map,Pose,Low_Scan,Odom,MODE_DERIVATIVES,MODE_MAP);
            [DeltaP,DeltaD,Sum_Delta] = FuncDelta(JP,JD,JO,ErrorS,ErrorO,HH,Map,IS,IO,Lambda,Lambda_O);
            [Map,Pose] = FuncUpdate(Map,Pose,DeltaP,DeltaD);
            Map = FuncUpdateMapN(Map,Pose,Low_Scan);
            if strcmp(EVALUTION_GT,'TRUE')==1
                FuncEval(Pose,Pose_GT,txt);
            end
        end

    elseif (Iter > (DOWN_TIME+1))
        
        if strcmp(MULTI_MODE,'TRUE')==1

            HH_Select = HH2_Select*Lambda;
            [ErrorS_Sel,JP_Sel,JD_Sel] = FuncDiffSelectJacobian(Select_Map,Pose,Select_Scan);
            [DeltaP,DeltaD,Sum_Delta] = FuncSelMapDelta(JO,ErrorO,IO,JP_Sel,JD_Sel,ErrorS_Sel,Select_Map,HH_Select,Lambda_O);
            [Select_Map,Pose] = FuncSelectUpdate(Select_Map,Pose,DeltaP,DeltaD);
            Select_Map = FuncUpdateSelectMapN(Select_Map,Pose,Select_Scan);
            Select_Map = FuncSmoothSelectN2(Select_Map,Lambda_N,HH2_Select);

            if strcmp(EVALUTION_GT,'TRUE')==1
                FuncEval(Pose,Pose_GT,txt);
            end     
        else
            Map = FuncSmoothN2(Map,10,HH2);
            Map = FuncMapGrid(Map,MODE_DERIVATIVES,MODE_MAP);
            [ErrorS,ErrorO,Sum_Error,MSE_Error,JP,JD,JO,IS,IO]= FuncDiffJacobian(Map,Pose,Low_Scan,Odom,MODE_DERIVATIVES,MODE_MAP);
            [DeltaP,DeltaD,Sum_Delta] = FuncDelta(JP,JD,JO,ErrorS,ErrorO,HH,Map,IS,IO,Lambda,Lambda_O);
            [Map,Pose] = FuncUpdate(Map,Pose,DeltaP,DeltaD);
            Map = FuncUpdateMapN(Map,Pose,Low_Scan);
            if strcmp(EVALUTION_GT,'TRUE')==1
                FuncEval(Pose,Pose_GT,txt);
            end
        end
    end
    
    if MSE_Error <= tem_MSE
        tem_MSE = MSE_Error;
        Over_Num = 0;
        Iter_minError = Iter;
    else
        Over_Num = Over_Num +1; 
    end
    
    full_Sum_Delta = full(Sum_Delta);
    Iter_time = toc;
    fprintf('Iterations %d Lambda %3f Error %.8f Delta %.8f Iter Time Use %f\n\n', Iter,Lambda,MSE_Error,full_Sum_Delta,Iter_time);
    fid=fopen(txt,'a');
    fprintf(fid,'Iterations %d Lambda %3f Error %.8f Delta %.8f\n Iter Time Use %f\n\n', Iter,Lambda,MSE_Error,full_Sum_Delta,Iter_time);
    fprintf('The Iter Number of Minimum MSE Error is %d\n\n', Iter_minError);
    total_time = total_time + Iter_time;
    fprintf('The Total Time Use %f\n\n', total_time);
    fprintf(fid,'The Total Time is %f\n\n', total_time);
    fclose(fid);
    %%
    figure(2);
    if (Iter >= (DOWN_TIME+1)) && strcmp(MULTI_MODE,'TRUE')==1       
        ab = exp(Select_Map.Grid);
    else
        ab = exp(Map.Grid);
    end
    PMat2 = ab./(ab+1);
    fig_2 = 1-PMat2;
    imshow(fig_2);
    
    % update the iteration time
    Iter = Iter+1;
    filename=[FILE_DIRECTORY,'/fig_',num2str(Iter-1),'.jpg'];
    imwrite(fig_2,filename,'jpg');
    name_map=[FILE_DIRECTORY,'/Map_',num2str(Iter-1),'.mat'];
    name_pose=[FILE_DIRECTORY,'/Pose_',num2str(Iter-1),'.mat'];
    if strcmp(MULTI_MODE,'TRUE')==1 && (Iter-1) >= (DOWN_TIME+1)
        save (name_map,'Select_Map');
    else
        save (name_map,'Map');
    end
    save (name_pose,'Pose');    
    
end

if Sum_Error<MinError
    Reason = 1;
elseif Sum_Delta<MinDelta
    Reason = 2;
elseif Iter>MaxIter
    Reason = 3;
elseif Over_Num >5
    Reason = 4;   
else
    Reason = 5;
end
S
if Iter>0
    if strcmp(MULTI_MODE,'TRUE')==1
        Jacobian = [JP_Sel,JD_Sel];
    else
        Jacobian = [JP,JD];
    end
    Info = Jacobian'*Jacobian;
    name_info=[FILE_DIRECTORY,'/Info_',num2str(Iter-1),'.mat'];
    save (name_info, 'Info');
end