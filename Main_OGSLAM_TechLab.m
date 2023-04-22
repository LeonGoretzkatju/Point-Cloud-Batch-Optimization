%% BundleAdjustment.m
%%
close all;
clc;
clear;

%% Load Data
load Scan_TechLab.mat;
load Pose_GT_TechLab.mat;
load Odom_TechLab.mat;

%% Create Grid Map
Size_i = 600;
Size_j = 600;
Scale = 0.01;
Origin = [-4.5;-5]; %left/right hand coordinate frame?
[Map] = FuncCreateGridMap(Size_i,Size_j,Scale,Origin);

%% Initialise Grid Map
% N_T = 2.0;
% N_R = 0.5;
% N_T = 0.0;
% N_R = 0.0;
Pose = Pose_GT;
nP = size(Pose,1);
% Pose(2:end,1:2) = Pose(2:end,1:2)+(rand(nP-1,2)-0.5)*N_T;
% Pose(2:end,3) = Pose(2:end,3)+(rand(nP-1,1)-0.5)*N_R;

%%
% [Pose] = FuncGetInitialPose(Odom);

%%
[Map] = FuncInitialiseGridMap(Map,Pose,Scan);
Map = FuncSmoothN2(Map,10);
Map = FuncMapGrid(Map);

Diff1 = Pose-Pose_GT;

%% Show Initialisation
figure(1);
ab = exp(Map.Grid);
PMat = ab./(ab+1);
imshow(1-PMat);

%% Least Squares
tic;

% [Map,Pose,Reason,Info] = FuncLeastSquaresLMSBA(Map,Pose,D,K);

% [Map,Pose,Reason,Info] = FuncLeastSquares(Map,Pose,Scan);
[Map,Pose,Reason,Info] = FuncLeastSquares22(Map,Pose,Scan,Odom);

% [Map,Pose,Reason,Info] = FuncLeastSquaresPoseOnly(Map,Pose,D,K);

% [Map,Pose,Reason,Info] = FuncLeastSquaresFeatureOnly(Map,Pose,D,K);

%%
BATime = toc;
fprintf('BA Time Use %d\n\n', BATime);

%% Print Reason
fprintf('Reason is %d\n', Reason);
    
%% Show Result
figure(2);
ab = exp(Map.Grid);
PMat2 = ab./(ab+1);
imshow(1-PMat2);

%%
Diff2 = Pose-Pose_GT;
Diff = [Diff1;Diff2];
