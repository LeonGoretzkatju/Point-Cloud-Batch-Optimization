% close all;
% clc;
% clear all;
load Trans_plane.mat
addpath("plane_seq2\")
MODE_DERIVATIVES = 'DEFAULT'; % DEFAULT or PARSING

MAX_ITER = 55;

FILE_DIRECTORY = 'Results/test';

MODE_MAP = 'CONTINUOUS'; % CONTINUOUS or DISCRETE

Rate = 1; % Down Sampling Rate of Map at first

DOWN_TIME = 50; % Down Sampling Times

MULTI_MODE = 'TRUE'; % MULTI-RESOLUTION MODE TRUE OR FALSE

Lambda_N = 5;

EVALUTION_GT = 'FALSE'; % IF EVALUTE THE OPTIMIZED POSES USING GT

KERNEL_SIZE = 6;

DATA_MODE = 'REAL'; % REAL or SIMU

% the weight of odometry input
Lambda_O = 0.0;

MODE_POSE = 'TRUE'; %If initial poses need to be calculated from odom

%% Create Grid Map
% CarPark Dataset
Size_i = 245;
Size_j = 150;
Scale = 0.02;
Origin = [-1.3219;-4.0403];
% Origin = [-1;-4];

% Create an empty cell array named Poses
Trans_original = {};

% Use a loop to store the poses in the Poses cell array
num_of_planes = 3;
for i = 1:num_of_planes-1
    pose_name = sprintf('pose%d', i);
    Trans_original{i} = eval(pose_name);
end

Trans = Trans_Preprocess(Trans_original);

if strcmp(MODE_POSE,'TRUE')==1
    Pose = FuncPosefromTrans(Trans,num_of_planes);
end
Pose_Rearrange = {Pose{2},Pose{3}};
pointclouds = {};
Downsample_pointclouds = {};
for i = 2:num_of_planes
    ptCloud = pcread("plane_seq2\" + "plane" + i + ".pcd");
    pointclouds{end+1} = ptCloud;
end
for i = 1:numel(pointclouds)
    Downsample_points = pointclouds{i};
%     ptCloud_down_10 = pcdownsample(pointclouds{i},'gridAverage',25);
%     Downsample_points = ptCloud_down_10;
    x = Downsample_points.Location(:,1);
    y = Downsample_points.Location(:,2);
    z = Downsample_points.Location(:,3);
    x_new = x/1000.0;
    y_new = y/1000.0;
    z_new = z/1000.0;
    % Create a new pointCloud object with the converted coordinates
    xyz_meters = [x_new, y_new, z_new];
    Downsample_points_new = pointCloud(xyz_meters);
    Downsample_pointclouds{end+1} = Downsample_points_new;
end
% global_point_clouds = FuncCreateGlobalMapPoints(Pose, Downsample_pointclouds);
% figure;
% pcshow(global_point_clouds);
Noise_Level = 100.0;
Map = FuncCreateGridMap(round(Size_i),round(Size_j),Scale,Origin);
Map = FuncInitialiseGridMap3D_New(Map,Pose_Rearrange,Downsample_pointclouds);
% Map = AddNoiseToMap(Map,Noise_Level);
% Pose_Noise = AddNoise(Pose);

% [a,b,c] = find(Map.Grid);
% figure(1);
% plot3(a,b,c,'b.','MarkerSize',0.5);

% HH2 = FuncMapConst(Map);
[Map,Gdugrid,Gdvgrid] = FuncMapGrid(Map,MODE_DERIVATIVES,MODE_MAP);
tic;
[ErrorS,MSE_Error,JP,IS,JD] = FuncDiffJacobianStepTest_New(Map,Pose,Trans,...
    Downsample_pointclouds,MODE_DERIVATIVES,...
    MODE_MAP);
Iter_time = toc;
fprintf('Initial Error is %.8f Time Use %f\n\n', MSE_Error, Iter_time);
Iter = 0;
Iter_minError = 10;
MaxIter = 50;
MinError = 1e-8;
MinDelta = 1e-10;
index = [];
%pose only, feature only
%no outlier rejection, record
%bigger smmoothing term, 十倍 二十倍 等
%有一些急剧变化的点 girdient maybe large
%lambda larger, 0.1 0.2 ... 
%same lambda, 哪些超过delta bound，画出这些点在map的位置， 如果不是分布在边缘，可能会有bug
Lambda = 0.1;
HH2 = FuncMapConst(Map); 
HH = HH2*Lambda;
while Iter <= 0
    [DeltaD,Sum_Delta] = FuncDeltaFeatureOnly(JP,JD,ErrorS,HH,Map);
    Map = FuncUpdateMapOnly(Map,DeltaD);
    [Map,Gdugrid,Gdvgrid] = FuncMapGrid(Map,MODE_DERIVATIVES,MODE_MAP);
    last_MSE_Error = MSE_Error;
    tic;
    [ErrorS,MSE_Error,JP,IS,JD] = FuncDiffJacobianStepTest_New(Map,Pose,Trans,...
    Downsample_pointclouds,MODE_DERIVATIVES,...
    MODE_MAP);
    Iter_time = toc;
    fprintf('MSE Error is %.8f Time Use %f\n\n', MSE_Error, Iter_time);
    Iter = Iter+1;
%     if abs(MSE_Error - last_MSE_Error) <= 0.000001
%         disp("reach the global minima");
%         break;
%     end
end
% [a,b,c] = find(Map.Grid);
% figure(1);
% plot3(a,b,c,'b.','MarkerSize',0.5);
[i, j] = ndgrid(1:Size_i, 1:Size_j); % Generate grid indices
% x = (i - 1) * Scale + Origin(1); % Convert i indices to x coordinates
% y = (j - 1) * Scale + Origin(2); % Convert j indices to y coordinates
% z = Map.Grid; % Use Grid values as z coordinates

x = i*Scale;
y = j*Scale;
z = Map.Grid;

points = [y(:), x(:), z(:)];
points = points(z(:) > 0, :);
% Create a pointCloud object
global_points = pointCloud(points);

% Visualize the point cloud
figure; % Create a new figure
pcshow(global_points); % Display the 3D point cloud
hold on;
xlabel('X'); % Label the x-axis
ylabel('Y'); % Label the y-axis
zlabel('Z'); % Label the z-axis
title('Grid Visualization as 3D Point Cloud'); % Set the title for the figure

posei = Pose{2};
T_reconstructed = posei;
Riw = T_reconstructed(1:3,1:3);
tiw = T_reconstructed(1:3,4);
pointCloud_i = Downsample_pointclouds{2};
xyi = pointCloud_i.Location(:,1:2)'; % Extract x and y values
zi = pointCloud_i.Location(:,3)'; % Extract z values as the new Oddi
xyzi = [xyi;zi];
Pwi = Riw'*(xyzi-tiw);
Pwi_xy = Pwi(1:2,:);
Pwi_z = Pwi(3,:);
XY3 = (Pwi_xy-Origin);
Pwi_new = [XY3;Pwi_z];
Pwi_T = Pwi_new';
Pwi_T_xyz = [Pwi_T(:,1), Pwi_T(:,2), Pwi_T(:,3)];
pointCloud_wi = pointCloud(Pwi_T_xyz);
pcshow(pointCloud_wi);