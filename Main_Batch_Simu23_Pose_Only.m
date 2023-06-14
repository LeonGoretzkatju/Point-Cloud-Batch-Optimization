close all;
clc;
clear all;

load depth_Simu23.mat
load Pose_GT_Simu23.mat
load cal_Simu.mat

MODE_DERIVATIVES = 'DEFAULT'; % DEFAULT or PARSING
MODE_MAP = 'CONTINUOUS'; % CONTINUOUS or DISCRETE

Pose = Pose_GT;