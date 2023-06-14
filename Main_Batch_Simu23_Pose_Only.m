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

[a,b,c] = find(Map.Grid);
figure(1);
plot3(a,b,c,'b.','MarkerSize',0.5);

[Map,Gdugrid,Gdvgrid] = FuncMapGrid(Map,MODE_DERIVATIVES,MODE_MAP);
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
    [Map,Gdugrid,Gdvgrid] = FuncMapGrid(Map,MODE_DERIVATIVES,MODE_MAP);
    tic;
    [ErrorS,MSE_Error,JD] = FuncDiffJacobian_MapOnly_Simu23(Map,Pose,D,K,MODE_MAP);
    Iter_time = toc;
    fprintf('MSE Error is %.8f Time Use %f\n\n', MSE_Error, Iter_time);
    Iter = Iter+1;
end

%then we start the pose only bundle adjustment
tic;
[ErrorS,MSE_Error,JP,IS] = FuncDiffJacobian_PoseOnly_Simu23(Map,Pose,D,K,MODE_MAP);
Iter_time = toc;
fprintf('Initial Error is %.8f Time Use %f\n\n', MSE_Error, Iter_time);
Iter = 0;
MaxIter = 5;
while Iter<=MaxIter
    [DeltaP_PoseOnly,Sum_Delta_PoseOnly] = FuncDelta3DPoseOnly(JP,ErrorS,IS);
    [Pose] = FuncUpdate3DPoseOnly(Pose,DeltaP_PoseOnly);
    tic;
    [ErrorS,MSE_Error,JP,IS] = FuncDiffJacobian_PoseOnly_Simu23(Map,Pose,D,K,MODE_MAP);
    Iter_time = toc;
    fprintf('MSE Error is %.8f Time Use %f\n\n', MSE_Error, Iter_time);
    Iter = Iter+1;
end


