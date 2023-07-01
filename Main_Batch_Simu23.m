% close all;
% clc;
% clear;
% 
% load depth_Simu23.mat
% load Pose_GT_Simu23.mat
% load cal_Simu.mat
% 
% Pose = Pose_GT;
% 
% nD = length(D);
% [nv,nu] = size(D{1});
% KI = inv(K);
% X_all = [];
% for i=1:nD
%     Di = D{i};
%     Ri = RMatrixYPR22(Pose(i,1),Pose(i,2),Pose(i,3));
%     Ti = Pose(i,4:6)';
%     for j=1:nu
%         for k=1:nv
%             x = [j;k;1]*Di(k,j);
%             X = Ri'*KI*x+Ti;
%             X_all = [X_all,X];
%         end
%     end
% end
% X_all_T = X_all';
% Pwi_T_xyz = [X_all_T(:,1), X_all_T(:,2), X_all_T(:,3)];
% pointCloud_wi = pointCloud(Pwi_T_xyz);
% figure;
% pcshow(pointCloud_wi);
close all;
clc;
clear all;

load depth_Simu23.mat
load Pose_GT_Simu23.mat
load cal_Simu.mat

MODE_DERIVATIVES = 'DEFAULT'; % DEFAULT or PARSING
MODE_MAP = 'CONTINUOUS'; % CONTINUOUS or DISCRETE

Pose = Pose_GT;

% nD = length(D);
% [nv, nu] = size(D{1});
% KI = inv(K);
% X_all = zeros(3, nv * nu * nD);
% Local_Scan_Set = {};
% Local_Scan = zeros(3, nv * nu);
% 
% for i = 1:nD
%     Di = D{i};
%     Ri = RMatrixYPR22(Pose(i,1),Pose(i,2),Pose(i,3));
%     Ti = Pose(i, 4:6)';
%     
%     Di_col = reshape(Di, [], 1);
%     [j, k] = meshgrid(1:nu, 1:nv);
%     x = [j(:), k(:), ones(numel(j), 1)] .* Di_col;
%     X = Ri' * KI * x' + Ti;
%     
%     X_all(:, ((i - 1) * nv * nu + 1):(i * nv * nu)) = X;
%     Local_Scan(:, 1:(nv * nu)) = X;
%     Local_Scan_Set{end+1} = Local_Scan;
% end
% 
% Pwi_T_xyz = X_all';
% pointCloud_wi = pointCloud(Pwi_T_xyz);
% figure;
% pcshow(pointCloud_wi);
% 
% Local_Scan_1 = Local_Scan_Set{1};
% pointcloud_Scan_1 = pointCloud(Local_Scan_1');
% figure;
% pcshow(pointcloud_Scan_1);

Size_i = 600;
Size_j = 600;
Scale = 0.01;
Origin = [-3;-3];

Noise_Level = 100000.0;

Map = FuncCreateGridMap(round(Size_i),round(Size_j),Scale,Origin);
[Map,ID] = FuncInitialiseGridMap_Simu23(Map,Pose,D,K);
Map = AddNoiseToMap(Map, Noise_Level);

[a,b,c] = find(Map.Grid);
figure(1);
plot3(a,b,c,'b.','MarkerSize',0.5);
title("Before Optimization");

[Map] = FuncMapGrid(Map,MODE_DERIVATIVES,MODE_MAP);
tic;
[ErrorS,MSE_Error,JD] = FuncDiffJacobian_MapOnly_Simu23(Map,Pose,D,K,MODE_MAP);
Iter_time = toc;
fprintf('Initial Error is %.8f Time Use %f\n\n', MSE_Error, Iter_time);

Lambda = 1.0;
HH2 = FuncMapConst(Map); 
HH = HH2*Lambda;
Iter = 0;
while Iter <= 1
    [DeltaD,Sum_Delta] = FuncDeltaFeatureOnly(JD,ErrorS,HH,Map);
    Map = FuncUpdateMapOnly(Map,DeltaD);
    [Map] = FuncMapGrid(Map,MODE_DERIVATIVES,MODE_MAP);
    tic;
    [ErrorS,MSE_Error,JD] = FuncDiffJacobian_MapOnly_Simu23(Map,Pose,D,K,MODE_MAP);
    Iter_time = toc;
    fprintf('MSE Error is %.8f Time Use %f\n\n', MSE_Error, Iter_time);
    Iter = Iter+1;
end

[a,b,c] = find(Map.Grid);
figure(2);
plot3(a,b,c,'b.','MarkerSize',0.5);
title("After Optimization");