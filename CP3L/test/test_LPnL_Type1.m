close all;
clc;
clear;

addpath LPnL-Bar-ENull;
addpath others;
addpath RPnL;



%Type1 Room Camera Pose Estimation

T1p1 = [522 367;522 359;103 361;531 370;94 372];
T1p2 = [103 370;528 5;96 2;621 392;14 392];


T1P1_w = [256 0 256;256 5 256;256 5 512;261 0 256;261 0 512];
T1P2_w = [256 0 512;256 220 256;256 215 512;296 0 256;291 0 512];


T1A = [197.4 0 320;0 197.4 320;0   0   1];

for i = 1:length(T1p1)
    temp1 = inv(T1A)*[T1p1(i,:) 1].';
    temp1 = (temp1/temp1(3)).';
    iT1p1(i,:) = temp1(:,1:2);
    temp2 = inv(T1A)*[T1p2(i,:) 1].';
    temp2 = (temp2/temp2(3)).';
    iT1p2(i,:) = temp2(:,1:2);
end


[T1R, T1T, T1err] = LPnL_Bar_ENull(iT1p1', iT1p2', T1P1_w', T1P2_w');
errorEpln=reprojection_error_usingRT(T1P1_w,T1p1,T1R,T1T,T1A);
fprintf('error EPnL: %.3f\n',errorEpln);


[T1R1, T1T1] = ASPnL(iT1p1', iT1p2', T1P1_w', T1P2_w');
%compute error
errorAspnl=reprojection_error_usingRT(T1P1_w,T1p1,T1R1,T1T1,T1A);
fprintf('error Aspnl: %.3f\n',errorAspnl);






