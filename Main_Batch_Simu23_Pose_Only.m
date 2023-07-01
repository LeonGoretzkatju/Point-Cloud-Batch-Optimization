close all;
clc;
clear all;

load depth_Simu23.mat
load Pose_GT_Simu23.mat
load cal_Simu.mat

MODE_DERIVATIVES = 'DEFAULT'; % DEFAULT or PARSING
MODE_MAP = 'CONTINUOUS'; % CONTINUOUS or DISCRETE

Pose = Pose_GT;

Size_i = 600;
Size_j = 600;
Scale = 0.01;
Origin = [-3;-3];

Map = FuncCreateGridMap(round(Size_i),round(Size_j),Scale,Origin);
[Map,ID] = FuncInitialiseGridMap_Simu23(Map,Pose,D,K);

[Map] = FuncMapGrid(Map,MODE_DERIVATIVES,MODE_MAP);
tic;
[ErrorS,MSE_Error,JD] = FuncDiffJacobian_MapOnly_Simu23(Map,Pose,D,K,MODE_MAP);
Iter_time = toc;
fprintf('Initial Error is %.8f Time Use %f\n\n', MSE_Error, Iter_time);

Lambda = 0.1;
HH2 = FuncMapConst(Map); 
HH = HH2*Lambda;
Iter = 0;
% this step is to obtain the more accurate map and set it fixed for the
% pose only
while Iter <= 0
    [DeltaD,Sum_Delta] = FuncDeltaFeatureOnly(JD,ErrorS,HH,Map);
    Map = FuncUpdateMapOnly(Map,DeltaD);
    [Map] = FuncMapGrid(Map,MODE_DERIVATIVES,MODE_MAP);
    tic;
    [ErrorS,MSE_Error,JD] = FuncDiffJacobian_MapOnly_Simu23(Map,Pose,D,K,MODE_MAP);
    Iter_time = toc;
    fprintf('MSE Error is %.8f Time Use %f\n\n', MSE_Error, Iter_time);
    Iter = Iter+1;
end

%then we start the pose only bundle adjustment
% Noise_Level = 3.0;
nP = size(Pose,1);
Pose(2:end,1:3) = Pose(2:end,1:3)+(rand(nP-1,3)- 0.5)*0.1;
Pose(2:end,4:6) = Pose(2:end,4:6)+(rand(nP-1,3)-0.5)*0.0;
% [Map,ID] = FuncInitialiseGridMap_Simu23(Map,Pose,D,K);
% Map = AddNoiseToMap(Map, Noise_Level);
% [a,b,c] = find(Map.Grid);
% figure(2);
% plot3(a,b,c,'b.','MarkerSize',0.5);
% title("Before Optimization");

tic;
[ErrorS,MSE_Error,JP,IS] = FuncDiffJacobian_PoseOnly_Simu23(Map,Pose,D,K,MODE_MAP);
Iter_time = toc;
fprintf('Initial Error is %.8f Time Use %f\n\n', MSE_Error, Iter_time);
Iter = 0;
MaxIter = 355;
figure;
xlabel('Iteration');
ylabel('MSE Error');
title('MSE Error vs Iteration');
hold on;

while Iter<=MaxIter
    [DeltaP_PoseOnly,Sum_Delta_PoseOnly] = FuncDelta3DPoseOnly(JP,ErrorS,IS);
    [Pose] = FuncUpdate3DPoseOnly(Pose,DeltaP_PoseOnly);
    tic;
    [ErrorS,MSE_Error,JP,IS] = FuncDiffJacobian_PoseOnly_Simu23(Map,Pose,D,K,MODE_MAP);
    Iter_time = toc;
    fprintf('MSE Error is %.8f Time Use %f Iteration Use %d \n\n', MSE_Error, Iter_time, Iter);
    Iter = Iter+1;
    
    % plot the MSE_Error vs Iteration
    plot(Iter, MSE_Error, 'b.');
    % connect MSE_Error
    if Iter>1
        plot([Iter-1,Iter],[MSE_Error_Pre,MSE_Error],'r-');
    end
    MSE_Error_Pre = MSE_Error;
    drawnow;
end

[a,b,c] = find(Map.Grid);
figure(2);
plot3(a,b,c,'b.','MarkerSize',0.5);
title("After Optimization");