clear all;
clc;
close all;

q1 = [0.999998 ,0.00108385, -0.00139558, -5.38901e-05];
q1=quatnormalize(q1); %单位化  
R1=quat2dcm(q1);%q1的第一位是实部

for i = 1:5:10
    i
end

% IDk = 1:100;
% dEdMID1 = repmat(IDk,4,1);
% cell_JDID1{1} = reshape(dEdMID1',[],1);
% vertcat(cell_JDID1{:})
% JD = sparse(10,10);
% load Trans_plane.mat
% vi = [1 2 3 4 5 6];
% ui = [7 8 9 10 11 12];
% Oddi = [1 1 1 1 1 1];
% Grid = sparse(vi,ui,Oddi);
% a = 1/(1-1/(1.05)^20)*20000;
% sum_value = 0;
% for i = 0:11
%     sum_value = sum_value + 1000*1/(1.01)^i;
% end
% Income = [-1000 840 708 388 178 50];
% for i = 0:5
%     sum_value = sum_value + Income(i+1)/(1.2)^i;
% end
% p = [42 42 42 42 42 -150];
% r = roots(p);
% result = 1/0.8899-1;
% A = [1 2;3 4];
% B = [3 4;5 6];
% C = sum(A.*B)
% IDk = 4:6;
% dEdPID1 = repmat(IDk,100,1)';
% a = [1,2,3;2,3,4]
% b = [1,3,2;3,2,4]
% d = a./b
% c = sum(a.*b);
% JPID1 = 1:100;
% JPID2 = 101:200;
% JPVal = 1:100;
% JP = sparse(JPID1,JPID2,JPVal);
% c = 0.3;
% fix(c);
% [euler1, t1] = se3_to_euler_angles_translation(pose1);
% [euler2, t2] = se3_to_euler_angles_translation(pose2);
% RZ = FuncRZ(euler1(1));
% RY = FuncRY(euler1(2));
% RX = FuncRX(euler1(3));
% R1 = FuncR(RZ,RY,RX);
% RZ2 = FuncRZ(euler2(1));
% RY2 = FuncRY(euler2(2));
% RX2 = FuncRX(euler2(3));
% R2 = FuncR(RZ2,RY2,RX2);
% DeltaR = R1'*R2;
% Euler_DeltaR = Rotation_to_Euler(DeltaR)
% DeltaRZ = FuncRZ(Euler_DeltaR(1));
% DeltaRY = FuncRY(Euler_DeltaR(2));
% DeltaRX = FuncRX(Euler_DeltaR(3));
% Euler_R_Delta = DeltaRZ * DeltaRY * DeltaRX;
% Euler_new = euler1+Euler_DeltaR'
% euler2
% RZ_new = FuncRZ(Euler_new(1));
% RY_new = FuncRY(Euler_new(2));
% RX_new = FuncRX(Euler_new(3));
% R_new = FuncR(RZ_new, RY_new, RX_new);
% R_tt = R1*Euler_R_Delta;
% R2;
% Theory_DeltaR = euler2 - euler1
% Size_i = 490;
% Size_j = 300;
% vi = 1:100;
% ui = 101:200;
% Oddi = 5:104;
% UpdateOi = sparse(vi,ui,Oddi,Size_i,Size_j);
% Map.Grid = zeros(15,13);
% Grid = Map.Grid;
% Grid(1,3) = 1;
% Grid(2,4) = 2;
% XH0 = reshape(Grid',[],1);