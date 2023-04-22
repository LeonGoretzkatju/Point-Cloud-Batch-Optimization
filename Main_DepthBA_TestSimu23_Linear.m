%% BundleAdjustment.m
%%
close all;
clc;
clear;

%% Load Data
% load depth_Simu.mat;
% load Pose_GT_Simu.mat;
% load cal_Simu.mat;
% load Dgrid_Simu.mat;

%% Load Data
% load depth_Simu23.mat;
% load Pose_GT_Simu23.mat;
% load cal_Simu.mat;
% load Dgrid_Simu23.mat;

%% Load Data
load depth_Simu23_n001.mat;
load Pose_GT_Simu23.mat;
load cal_Simu.mat;
load Dgrid_Simu23_n001.mat;

%% Load Data
% load depth_Simu23_sn1001.mat;
% load Pose_GT_Simu23.mat;
% load cal_Simu.mat;
% load Dgrid_Simu23_sn1001.mat;

%% Load Data
% load depth_Simu23_n001f.mat;
% load Pose_GT_Simu23.mat;
% load cal_Simu.mat;
% load Dgrid_Simu23_n001f.mat;

%%
% D2{1} = D{1};
% D2{2} = D{2};
% D = D2;
% 
% DgridT{1} = Dgrid{1};
% DgridT{2} = Dgrid{2};
% Dgrid = DgridT;
% 
% Pose_GT2 = Pose_GT([1,2],:);
% Pose_GT = Pose_GT2;

%% Create Grid Map
% Size_i = 1300;
% Size_j = 1600;
% Scale = 0.002;
% Origin = [-1.4;0.3];
% [Map] = FuncCreateGridMap(Size_i,Size_j,Scale,Origin);

%% Create Grid Map
% Size_i = 600;
% Size_j = 600;
% Scale = 0.01;
% Origin = [-3;-3];
% [Map] = FuncCreateGridMap(Size_i,Size_j,Scale,Origin);

%% Create Grid Map
% Size_i = 800;
% Size_j = 800;
% Scale = 0.005;
% Origin = [-2;-2];
% [Map] = FuncCreateGridMap(Size_i,Size_j,Scale,Origin);

%% Create Grid Map
Size_i = 600;
Size_j = 600;
Scale = 0.01;
Origin = [-3;-3];
[Map] = FuncCreateGridMap(Size_i,Size_j,Scale,Origin);

%% Initialise Grid Map
Pose = Pose_GT;
nP = size(Pose,1);

% Map = FuncLinearMap(Map,Pose,D,K);
% Pose(:,4:6) = Pose(:,4:6)+0.5;
% Pose(2:end,1:3) = Pose(2:end,1:3)+(rand(nP-1,3)-0.5)*0.3;
% Pose(2:end,4:6) = Pose(2:end,4:6)+(rand(nP-1,3)-0.5)*0.5;

Pose(2:end,1:3) = Pose(2:end,1:3)+(rand(nP-1,3)-0.5)*0.1;
Pose(2:end,4:6) = Pose(2:end,4:6)+(rand(nP-1,3)-0.5)*0.2;

% [Map] = FuncInitialiseGridMap(Map,Pose,D,K);

Map = FuncLinearMap(Map,Pose,D,K);

Map = FuncMapGrid(Map);

% Pose = Pose_GT;
% Pose(:,4:6) = Pose(:,4:6)+(rand(2,3)-0.5)*0.5;
% Pose(:,4:6) = Pose(:,4:6)+0.5;

Diff1 = Pose-Pose_GT;

%% Test Initialisation 

% figure(1);
% surf(Map.Grid,'EdgeColor','none');

%% Test Initialisation (Better)

[a,b,c] = find(Map.Grid);
figure(1);
plot3(a,b,c,'b.','MarkerSize',0.5);

%% Least Squares
tic;

[Map,Pose,Reason,Info] = FuncLeastSquaresLMSBA(Map,Pose,D,K);

% [Map,Pose,Reason,Info] = FuncLeastSquares(Map,Pose,D,K);

% [Map,Pose,Reason,Info] = FuncLeastSquaresPoseOnly(Map,Pose,D,K);

% [Map,Pose,Reason,Info] = FuncLeastSquaresFeatureOnly(Map,Pose,D,K);

%%
BATime = toc;
fprintf('BA Time Use %d\n\n', BATime);

%% Print Reason
fprintf('Reason is %d\n', Reason);
    
%% Test Initialisation (Better)

[a,b,c] = find(Map.Grid);
figure(2);
plot3(a,b,c,'r.','MarkerSize',0.5);

%%
Diff2 = Pose-Pose_GT;
Diff = [Diff1;Diff2];
