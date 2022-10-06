close all;
clc;
clear;

addpath LPnL-Bar-ENull;
addpath others;
addpath RPnL;
addpath ASPnL;
addpath P3Lmethods;



%Type1 Room Camera Pose Estimation

T1p1 = [159 148;313 630;341 218;159 148;313 630;341 218;159 148;313 630;341 218;159 148;313 630;341 218;159 148;313 630;341 218];
T1p2 = [3 63;314 475;620 63;3 63;314 475;620 63;3 63;314 475;620 63;3 63;314 475;620 63;3 63;314 475;620 63];


T1P1_w = [256 256 326;256 115 256;271 256 256];
T1P2_w = [256 256 371;256 160 256;371 256 256];
T1P3_w = [256 256 331;256 120 256;276 256 256];
T1P4_w = [256 256 366;256 155 256;365 256 256];
T1P5_w = [256 256 340;256 130 256;285 256 256];
T1P6_w = [256 256 355;256 143 256;343 256 256];
T1P7_w = [256 256 345;256 135 256;290 256 256];
T1P8_w = [256 256 350;256 140 256;315 256 256];
T1P9_w = [256 256 351;256 141 256;323 256 256];
T1P10_w = [256 256 363;256 151 256;335 256 256];


P1_w = [T1P1_w;T1P3_w;T1P5_w;T1P7_w;T1P9_w];
P2_w = [T1P2_w;T1P4_w;T1P6_w;T1P8_w;T1P10_w];

T1A = [512 0 320;0 512 320;0   0   1];

for i = 1:length(T1p1)
    temp1 = T1A\[T1p1(i,:) 1].';
%     temp1 = (temp1/temp1(3)).';
    temp1 = temp1';
    iT1p1(i,:) = temp1(:,1:2);
    temp2 = T1A\[T1p2(i,:) 1].';
%     temp2 = (temp2/temp2(3)).';
    temp2 = temp2';
    iT1p2(i,:) = temp2(:,1:2);
end

% [T1R, T1T, T1err] = LPnL_Bar_ENull(T1p1', T1p2', T1P1_w', T1P2_w');

[T1R, T1T, T1err] = LPnL_Bar_ENull(iT1p1', iT1p2', P1_w', P2_w');
errorEpln=reprojection_error_usingRT(P1_w,T1p1,T1R,T1T,T1A);
fprintf('error EPnL: %.3f\n',errorEpln);


[T1R1, T1T1] = ASPnL(iT1p1', iT1p2', P1_w', P2_w');
%compute error
errorAspnl=reprojection_error_usingRT(P1_w,T1p1,T1R1,T1T1,T1A);
fprintf('error Aspnl: %.3f\n',errorAspnl);


sol =  P3L(iT1p1', iT1p2', P1_w', P2_w');
for i = 1:size(sol,2)
    errorP3L(i)=reprojection_error_usingRT(P1_w,T1p1,sol(i).R,sol(i).T,T1A);
end

sol =  myP3L(iT1p1', iT1p2', P1_w', P2_w');
for i = 1:size(sol,2)
    errormyP3L(i)=reprojection_error_usingRT(P1_w,T1p1,sol(i).R,sol(i).T,T1A);
end





