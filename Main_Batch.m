close all;
clc;
clear;
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
Size_i = 300;
Size_j = 480;
Scale = 0.01;
Origin = [-1.314;-4.033];

% Create an empty cell array named Poses
Trans_original = {};

% Use a loop to store the poses in the Poses cell array
num_of_planes = 6;
for i = 1:num_of_planes-1
    pose_name = sprintf('pose%d', i);
    Trans_original{i} = eval(pose_name);
end

Trans = Trans_Preprocess(Trans_original);

if strcmp(MODE_POSE,'TRUE')==1
    Pose = FuncPosefromTrans(Trans,num_of_planes);
end
pointclouds = {};
Downsample_pointclouds = {};
for i = 1:num_of_planes
    ptCloud = pcread("plane_seq2\" + "plane" + i + ".pcd");
    pointclouds{end+1} = ptCloud;
end
gridStep = 25.0;
for i = 1:numel(pointclouds)
%     ptCloud_down_10 = pcdownsample(pointclouds{i},'gridAverage',gridStep);
%     Downsample_points = FuncDownsamplePoints(ptCloud_down_10, Rate);
    Downsample_points = pointclouds{i};
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
Map = FuncCreateGridMap(round(Size_i),round(Size_j),Scale,Origin);
Map = FuncInitialiseGridMap3D(Map,Pose,Downsample_pointclouds);
% Convert the grid to 3D points
% [i, j] = ndgrid(1:Size_i, 1:Size_j); % Generate grid indices
% x = (i - 1) * Scale + Origin(1); % Convert i indices to x coordinates
% y = (j - 1) * Scale + Origin(2); % Convert j indices to y coordinates
% z = Map.Grid; % Use Grid values as z coordinates
% 
% points = [x(:), y(:), z(:)];
% 
% % Remove points with zero occupancy values to improve visualization
% points = points(z(:) > 0, :);
% 
% % Create a pointCloud object
% global_points = pointCloud(points);
% 
% % Visualize the point cloud
% figure; % Create a new figure
% pcshow(global_points); % Display the 3D point cloud
% xlabel('X'); % Label the x-axis
% ylabel('Y'); % Label the y-axis
% zlabel('Z'); % Label the z-axis
% title('Grid Visualization as 3D Point Cloud'); % Set the title for the figure
HH2 = FuncMapConst(Map);
Map = FuncSmoothN2(Map,10,HH2);
[Map,Gdugrid,Gdvgrid] = FuncMapGrid(Map,MODE_DERIVATIVES,MODE_MAP);

%% Least Squares
if Lambda_O==0
    Odom = zeros(size(Pose,1)-1,3);
end
tic;
[ErrorS,ErrorO,Sum_Error,MSE_Error,IS,IO,JP,JD,JO] = FuncDiffJacobianStepTest(Map,Pose,Trans,...
    Downsample_pointclouds,MODE_DERIVATIVES,...
    MODE_MAP);
Iter_time = toc;
fprintf('Initial Error is %.8f Time Use %f\n\n', MSE_Error, Iter_time);
% fprintf('Initial Error is %.8f Time Use %f\n\n', MSE_Error, Iter_time);
total_time = Iter_time;
Sum_Delta = 22;
MaxIter = MAX_ITER;
MinError = 1e-8;
MinDelta = 1e-10;
tem_MSE = 10;
Over_Num = 0;

Iter = 0;
Iter_minError = 10;
index = [];
while Iter <= 2
    HH2 = FuncMapConst(Map); 
    Lambda = 0.000;
    HH = HH2*Lambda;
%     [DeltaP,DeltaD,Sum_Delta] = FuncDelta3D(JP,JD,JO,ErrorS,ErrorO,HH,Map,IS,IO,Lambda,Lambda_O);
    [DeltaD,Sum_Delta] = FuncDeltaFeatureOnly(JP,JD,ErrorS,HH,Map);
    %     [DeltaP_PoseOnly,Sum_Delta_PoseOnly] = FuncDelta3DPoseOnly(JP,ErrorS,IS);
%     Pose_Vector = FuncParamPose(Pose);
%     [Map,Pose_Vector_new] = FuncUpdate3D(Map,Pose_Vector,DeltaP,DeltaD);
    Map = FuncUpdateMapOnly(Map,DeltaD);
%     [Pose_Vector_new_PoseOnly] = FuncUpdate3DPoseOnly(Pose_Vector,DeltaP_PoseOnly);
%     Pose = FuncInverParamPose(Pose_Vector_new);
%     Map = FuncInitialiseGridMap3D(Map,Pose,Downsample_pointclouds);
    [Map,Gdugrid,Gdvgrid] = FuncMapGrid(Map,MODE_DERIVATIVES,MODE_MAP);
    tic;
    [ErrorS,ErrorO,Sum_Error,MSE_Error,IS,IO,JP,JD,JO] = FuncDiffJacobianStepTest(Map,Pose,Trans,...
        Downsample_pointclouds,MODE_DERIVATIVES,...
        MODE_MAP);
    Iter_time = toc;
    fprintf('MSE Error is %.8f Time Use %f\n\n', MSE_Error, Iter_time);
    Iter = Iter+1;
end
% Convert the grid to 3D points
[i, j] = ndgrid(1:Size_i, 1:Size_j); % Generate grid indices
x = (i - 1) * Scale + Origin(1); % Convert i indices to x coordinates
y = (j - 1) * Scale + Origin(2); % Convert j indices to y coordinates
z = Map.Grid; % Use Grid values as z coordinates

points = [x(:), y(:), z(:)];

% Remove points with zero occupancy values to improve visualization
points = points(z(:) > 0.5, :);

% Create a pointCloud object
global_points = pointCloud(points);

% Visualize the point cloud
figure; % Create a new figure
pcshow(global_points); % Display the 3D point cloud
xlabel('X'); % Label the x-axis
ylabel('Y'); % Label the y-axis
zlabel('Z'); % Label the z-axis
title('Grid Visualization as 3D Point Cloud'); % Set the title for the figure