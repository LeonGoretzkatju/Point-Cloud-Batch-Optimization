close all;
clc;
clear all;

load depth_Simu23.mat
load Pose_GT_Simu23.mat
load cal_Simu.mat

FILE_DIRECTORY = 'Results/test';
MODE_DERIVATIVES = 'DEFAULT'; % DEFAULT or PARSING
MODE_MAP = 'CONTINUOUS'; % CONTINUOUS or DISCRETE

Pose = Pose_GT;

Size_i = 600;
Size_j = 600;
Scale = 0.01;
Origin = [-3;-3];

Map = FuncCreateGridMap(round(Size_i),round(Size_j),Scale,Origin);

nP = size(Pose,1);
Pose(2:end,1:3) = Pose(2:end,1:3)+(rand(nP-1,3)- 0.5)*0.1;
Pose(2:end,4:6) = Pose(2:end,4:6)+(rand(nP-1,3)-0.5)*0.1;
[Map,ID] = FuncInitialiseGridMap_Simu23(Map,Pose,D,K);

Noise_Level = 0.1;
% [Map,ID] = FuncInitialiseGridMap_Simu23(Map,Pose,D,K);
Map = AddNoiseToMap(Map, Noise_Level);

% Map = FuncLinearMap(Map,Pose,D,K);
[Map] = FuncMapGrid(Map,MODE_DERIVATIVES,MODE_MAP);

%% Least Square Method using LM Solver
% [Map,Pose,Reason,Info] = FuncLeastSquares_Simu23_LMS(Map,Pose,D,K, ...
%     MODE_DERIVATIVES,MODE_MAP,FILE_DIRECTORY);

%% Leaset Square Method using Gauss-Newton Method with GNC
[Map,Pose,Reason,Info] = FuncLeastSquares_Simu23_GN(Map,Pose,D,K, ...
    MODE_DERIVATIVES,MODE_MAP,FILE_DIRECTORY);

fprintf('Reason is %d\n', Reason);
[a,b,c] = find(Map.Grid);
figure(2);
plot3(a,b,c,'b.','MarkerSize',0.5);
title("After Optimization");