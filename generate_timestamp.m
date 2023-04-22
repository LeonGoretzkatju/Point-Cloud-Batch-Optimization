clear all;
clc;
load Scan_Test10.mat;
load Odom_Simu76.mat;

timestamps=fopen('/Users/yingyuwang/Downloads/rosbag_generate/timestamps.txt','a');
odometry=fopen('/Users/yingyuwang/Downloads/rosbag_generate/odometry.txt','a');

size = size(Odom,1);
for i = 0:size+1
%     tem = [i,i];
%     timestamp = [timestamp; tem];
    if i < size+1
        fprintf(timestamps,'%d %d \n',i,i);
        if i < size-1
            fprintf(odometry,'%d %d %.15f %.15f %.15f\n',i+1,i+1,Odom(i+1,1),Odom(i+1,2),Odom(i+1,3));
        elseif i == size-1
            fprintf(odometry,'%d %d %.15f %.15f %.15f',i+1,i+1,Odom(i+1,1),Odom(i+1,2),Odom(i+1,3));
        end
    else
        fprintf(timestamps,'%d %d',i,i);
    end
end

fclose(timestamps);
fclose(odometry);