% load all.mat
close all;
clc;
clear;
load Trans_plane.mat
num_of_planes = 6;
addpath("plane_seq2\")
pointclouds = {};
for i = 1:num_of_planes
    ptCloud = pcread("plane_seq2\" + "plane" + i + ".pcd");
    pointclouds{end+1} = ptCloud;
end