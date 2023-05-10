clear all;
clc;
close all;
load Trans_plane.mat
IDk = 4:6;
dEdPID1 = repmat(IDk,100,1)';
a = [1,2,3;2,3,4];
b = [1,3,2;3,2,4];
c = sum(a.*b);
JPID1 = 1:100;
JPID2 = 101:200;
JPVal = 1:100;
JP = sparse(JPID1,JPID2,JPVal);
c = 0.3;
fix(c);
[euler1, t1] = se3_to_euler_angles_translation(pose1);
[euler2, t2] = se3_to_euler_angles_translation(pose2);
RZ = FuncRZ(euler1(1));
RY = FuncRY(euler1(2));
RX = FuncRX(euler1(3));
R1 = FuncR(RZ,RY,RX);
RZ2 = FuncRZ(euler2(1));
RY2 = FuncRY(euler2(2));
RX2 = FuncRX(euler2(3));
R2 = FuncR(RZ2,RY2,RX2);
DeltaR = R1'*R2;
Euler_DeltaR = Rotation_to_Euler(DeltaR)
DeltaRZ = FuncRZ(Euler_DeltaR(1));
DeltaRY = FuncRY(Euler_DeltaR(2));
DeltaRX = FuncRX(Euler_DeltaR(3));
Euler_R_Delta = DeltaRZ * DeltaRY * DeltaRX;
Euler_new = euler1+Euler_DeltaR'
euler2
RZ_new = FuncRZ(Euler_new(1));
RY_new = FuncRY(Euler_new(2));
RX_new = FuncRX(Euler_new(3));
R_new = FuncR(RZ_new, RY_new, RX_new);
R_tt = R1*Euler_R_Delta;
R2;
% Theory_DeltaR = euler2 - euler1