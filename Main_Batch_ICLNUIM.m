clear all;
clc;
close all;
K = [481.20,0.0,319.50;
    0.0,-480.0,239.50;
    0.0,0.0,1.0];
% Trajectory_name = "ICLRT_GT.txt";
% [Pose_GT_ICL] = extract_rot_trans(Trajectory_name);
% save Pose_GT_ICLNUIM Pose_GT_ICL

load Pose_GT_ICLNUIM.mat

addpath("../living_room_traj2_frei_png.tar/depth/");
DepthScale = 5000.0;

D = cell(1,50);
for i = 1:numel(D)
    Depth = imread("../living_room_traj2_frei_png.tar/depth/" + (i) + ".png");
    D{i} = double(Depth);
end
nD = numel(D);
[nv, nu] = size(D{1});
KI = inv(K);
X_all = zeros(3, nv * nu * nD);

for i = 1:nD
    Di = D{i};
    T = euler_angles_translation_to_se3(Pose_GT_ICL(i,1:3),Pose_GT_ICL(i,4:6)');
    Ri = T(1:3,1:3);
    Ti = T(1:3,4);
    Di_col = reshape(Di, [], 1);
    [j, k] = meshgrid(1:nu, 1:nv);
    x = [j(:), k(:), ones(numel(j), 1)] .* Di_col;
    X = KI * x' / DepthScale;
    X = Ri*X + Ti;    
    X_all(:, ((i - 1) * nv * nu + 1):(i * nv * nu)) = X;
end

Pwi_T_xyz = X_all';
pointCloud_wi = pointCloud(Pwi_T_xyz);
figure;
pcshow(pointCloud_wi);

Scale = 0.01;
Size_i = 280;
Size_j = 470;
Origin = [-1.9558;0.1235];

MODE_DERIVATIVES = 'PARSING'; % DEFAULT or PARSING
MODE_MAP = 'DISCRETE'; % CONTINUOUS or DISCRETE

Map = FuncCreateGridMap(round(Size_i),round(Size_j),Scale,Origin);
Map = FuncInitialiseGridMap_ICL(Map,Pose_GT_ICL,D,K,DepthScale);

[i, j] = ndgrid(1:Size_i, 1:Size_j); % Generate grid indices
x = i*Scale;
y = j*Scale;
z = Map.Grid;

points = [y(:), x(:), z(:)];
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

[Map] = FuncMapGrid(Map,MODE_DERIVATIVES,MODE_MAP);
tic;
[ErrorS,MSE_Error,JD] = FuncDiffJacobian_MapOnly_ICL(Map,Pose_GT_ICL,D,K,MODE_MAP,DepthScale);
Iter_time = toc;
fprintf('Initial Error is %.8f Time Use %f\n\n', MSE_Error, Iter_time);

Lambda = 0.001;
HH2 = FuncMapConst(Map); 
HH = HH2*Lambda;
Iter = 0;
while Iter <= 1
    [DeltaD,Sum_Delta] = FuncDeltaFeatureOnly(JD,ErrorS,HH,Map);
    Map = FuncUpdateMapOnly(Map,DeltaD);
    [Map] = FuncMapGrid(Map,MODE_DERIVATIVES,MODE_MAP);
    tic;
    [ErrorS,MSE_Error,JD] = FuncDiffJacobian_MapOnly_ICL(Map,Pose_GT_ICL,D,K,MODE_MAP,DepthScale);
    Iter_time = toc;
    fprintf('MSE Error is %.8f Time Use %f\n\n', MSE_Error, Iter_time);
    Iter = Iter+1;
end
[i, j] = ndgrid(1:Size_i, 1:Size_j); % Generate grid indices
x = i*Scale;
y = j*Scale;
z = Map.Grid;

points = [y(:), x(:), z(:)];
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