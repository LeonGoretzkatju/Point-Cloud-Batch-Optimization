%% BundleAdjustment.m
%%
close all;
clc;
clear;

%% Load Data
load depth.mat;
load Pose_GT.mat;
load cal.mat;
load Dgrid.mat

%%
D2{1} = D{2};
D2{2} = D{1};
D = D2;

DgridT{1} = Dgrid{2};
DgridT{2} = Dgrid{1};
Dgrid = DgridT;

Pose_GT2 = Pose_GT([2,1],:);
Pose_GT = Pose_GT2;

%% Create Grid Map
% Size_i = 1300;
% Size_j = 1600;
% Scale = 0.002;
% Origin = [-1.4;0.3];
% [Map] = FuncCreateGridMap(Size_i,Size_j,Scale,Origin);

%% Create Grid Map
Size_i = 650;
Size_j = 800;
Scale = 0.004;
Origin = [-1.4;0.3];
[Map] = FuncCreateGridMap(Size_i,Size_j,Scale,Origin);

%% Initialise Grid Map
Pose = Pose_GT;
% Pose(:,4:6) = Pose(:,4:6)+(rand(2,3)-0.5)*0.2;

[Map] = FuncInitialiseGridMap(Map,Pose,D,K);

% Pose = Pose_GT;
% Pose(:,4:6) = Pose(:,4:6)+(rand(2,3)-0.5)*0.5;
Pose(:,4:6) = Pose(:,4:6)+0.5;

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

% [Map,Pose,Reason,Info] = FuncLeastSquares(Map,Pose,D,Dgrid,K);

[Map,Pose,Reason,Info] = FuncLeastSquaresPoseOnly(Map,Pose,D,Dgrid,K);

% [Map,Pose,Reason,Info] = FuncLeastSquaresFeatureOnly(Map,Pose,D,Dgrid,K);

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
