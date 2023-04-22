%% Occupancy-SLAM++
%%
close all;
clc;
clear;
addpath('Data')
%% Load Data

% CarPark Dataset
load Data/CarPark_Data/Scan_CarPark.mat;
load Data/CarPark_Data/Pose_GT_CarPark.mat;
load Data/CarPark_Data/Odom_CarPark.mat;

% C5_1 Dataset
% load Data/C51_Data/Pose_GT_C51.mat;
% load Data/C51_Data/Scan_C51.mat;
% load Data/C51_Data/Odom_C51.mat;

% C4 Dataset

% load Data/C4_Data/Scan_C4.mat;
% load Data/C4_Data/Odom_C4.mat;
% load Data/C4_Data/Pose_GT_C4.mat;


% Museum Dataset
% load Data/b0_Scan_Matching/Pose_b0_Sampling.mat
% load Data/b0_Scan_Matching/Scan_b0_Sampling.mat
% load Data/b0_Scan_Matching/b0_Odom_Generated.mat
% load Data/b0_Data/Pose_GT_b0.mat
% load Data/b0_Data/Scan_b0.mat

% load ('./Data/Simu3640_Data/Scan_Simu364_Noise.mat');
% load ('./Data/Simu3640_Data/Odom_Simu364_GT.mat');
% load ('./Data/Simu3640_Data/Pose_GT_Simu364.mat');
% load ('./Data/Simu3720_Data/Simu3720_Carto_Sync_2.mat');


% load ../Multi_OG_SLAM/Data/Simu312_Data/Scan_Simu312_GT.mat
% load ../Multi_OG_SLAM/Data/Simu312_Data/Pose_GT_Simu312.mat
% load ../Multi_OG_SLAM/Data/Simu312_Data/Odom_Simu312_GT.mat





% load Data/Intel_Data/Scan_Intel_10.mat;
% load Data/Intel_Data/Pose_GT_Intel.mat;
% load Data/Intel_Data/Odom_Intel.mat;

% load Pose_GT_Simu380_1x_sync.mat
% load Scan_Simu380_sync.mat
% load Odom_Simu380_sync.mat

% load Pose_GT_C51.mat
% load Scan_C51.mat
% load Odom_C51.mat


% Pose = Pose_Carto;
%% Definition of configurations
MODE_DERIVATIVES = 'DEFAULT'; % DEFAULT or PARSING

MAX_ITER = 55;

FILE_DIRECTORY = 'Results/test';

MODE_MAP = 'CONTINUOUS'; % CONTINUOUS or DISCRETE

Rate = 2; % Down Sampling Rate of Map at first

DOWN_TIME = 50; % Down Sampling Times

MULTI_MODE = 'TRUE'; % MULTI-RESOLUTION MODE TRUE OR FALSE

Lambda_N = 5;

EVALUTION_GT = 'TRUE'; % IF EVALUTE THE OPTIMIZED POSES USING GT

KERNEL_SIZE = 6;

DATA_MODE = 'REAL'; % REAL or SIMU

% the weight of odometry input
Lambda_O = 1;

MODE_POSE = 'TRUE'; %If initial poses need to be calculated from odom

%% Create Grid Map
% CarPark Dataset
Size_i = 650;
Size_j = 750;
Scale = 0.1;
Origin = [-20;-30];

% C5_1 Dataset
% Size_i = 1000;
% Size_j = 800;
% Scale = 0.05;
% Origin = [-15;-20];

% Size_i = 1200;
% Size_j = 1000;
% Scale = 0.05;
% Origin = [-20;-25];

% C4 Dataset
% Size_i = 2500;
% Size_j = 2500;
% Scale = 0.1;
% Origin = [-75;-100];

% Museum Dataset
% Size_i = 1600;
% Size_j = 1800;
% Scale = 0.05;
% Origin = [-60; -50];

% Size_i = 2000;
% Size_j = 2200;
% Scale = 0.05;
% Origin = [-70; -60];

% Size_i = 720;
% Size_j = 950;
% Scale = 0.1;
% Origin = [-62.5;-50];


% Size_i = 1000;
% Size_j = 1000;
% Scale = 0.05;
% Origin = [-25;-25]; 



% Size_i = 1100;
% Size_j = 1300;
% Scale = 0.05;
% Origin = [-30;-28]; %left/right hand coordinate frame?

% Size_i = 700;
% Size_j = 800;
% Scale = 0.05;
% Origin = [-20;-20]; %left/right hand coordinate frame?
% Size_i = 1000;
% Size_j = 800;
% Scale = 0.05;
% Origin = [-15;-20]; %left/right hand coordinate frame?
% simu 532
% Size_i = 1106;
% Size_j = 1105;
% Scale = 0.05;
% Origin = [-27; -26]; %left/right hand coordinate frame?

% simu380
% Size_i = 1102;
% Size_j = 1116;
% Scale = 0.05;
% Origin = [-25.1295; -26.259];

% simu404
% Size_i = 1500;
% Size_j = 1500;
% Scale = 0.05;
% Origin = [-55.1295; -56.259];

% N_T = 2;
% N_R = 0.5; 
% nP = size(Pose,1);
% 
% Pose(2:end,1:2) = Pose(2:end,1:2)+(rand(nP-1,2)-0.5)*N_T;
% Pose(2:end,3) = Pose(2:end,3)+(rand(nP-1,1)-0.5)*N_R;

% N_T = 0.1;
% N_R = 0.004; 
% nP = size(Odom,1);
% 
% noise_dxdy = randn(nP-1,2)*N_T;
% noise_dphi = randn(nP-1,1)*N_R;
% noise_dxdy(noise_dxdy > 3*N_T) =N_T;
% noise_dxdy(noise_dxdy < -3*N_T) = -N_T;
% noise_dphi(noise_dphi > 3*N_R) =N_R;
% noise_dphi(noise_dphi < -3*N_R) = -N_R;
% 
% 
% Odom(2:end,1:2) = Odom(2:end,1:2)+ noise_dxdy;
% Odom(2:end,3) = Odom(2:end,3)+noise_dphi;

% Pose_GT = Pose;
if strcmp(MODE_POSE,'TRUE')==1
    [Pose,Odom] = FuncPosefromOdom(Odom,Pose_GT,DATA_MODE);
end



% save ('./Results/CarPark_Result_Extra/Initial_Pose.mat','Pose');
% load Data/CarPark_Data/Odom_CarPark.mat;
% Pose_Noise = Pose;
Pose = Pose_GT;
% Scan = FuncLowSampleScan(Scan,2);
% Pose_GT = Pose;
Low_Scan = FuncLowSampleScan(Scan,Rate);
Map = FuncCreateGridMap(round(Size_i/Rate),round(Size_j/Rate),Scale*Rate,Origin);

%% Initialise Grid Map
% N_T = 5;
% N_R = 0.5;
% N_T = 0.1;
% N_R = 0.05; 
% Pose = Pose_GT;
% nP = size(Pose,1);
% Pose(2:end,1:2) = Pose(2:end,1:2)+(rand(nP-1,2)-0.5)*N_T;
% Pose(2:end,3) = Pose(2:end,3)+(rand(nP-1,1)-0.5)*N_R;
Map = FuncInitialiseGridMap(Map,Pose,Low_Scan);
HH2 = FuncMapConst(Map);
Map = FuncSmoothN2(Map,10,HH2);
Map = FuncMapGrid(Map,MODE_DERIVATIVES,MODE_MAP);     
% Pose_GT = Pose;
%% Show Initialisation
% Pic = ShowPicture(Map);
figure(1);
ab = exp(Map.Grid);
% ab = exp(Pic);
PMat = ab./(ab+1);
imshow(1-PMat);

%% Least Squares
if Lambda_O==0
    Odom = zeros(size(Pose,1)-1,3);
end

[Pose,Reason,Info,index] = FuncLeastSquares22(Map,Pose,Scan,Odom,MODE_DERIVATIVES,FILE_DIRECTORY,...
    MAX_ITER,MODE_MAP,Low_Scan,Size_i,Size_j,Scale,Origin,DOWN_TIME,...
    Lambda_O,Lambda_N,MULTI_MODE,EVALUTION_GT,Pose_GT,Rate,KERNEL_SIZE);
%% Print Reason
fprintf('Reason is %d\n\n', Reason);
    
%% Show Result
% Show the remapped result
figure(3);
Map = FuncCreateGridMap(Size_i,Size_j,Scale,Origin);
Map = FuncInitialiseGridMap(Map,Pose,Scan);
ab = exp(Map.Grid);
PMat2 = ab./(ab+1);
imshow(1-PMat2);

%%
txt=strcat(FILE_DIRECTORY,'/result.txt');
Pose_GT(:,index)=[]; 
FuncEval(Pose,Pose_GT,txt);