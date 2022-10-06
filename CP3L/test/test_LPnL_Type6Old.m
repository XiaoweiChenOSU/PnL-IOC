close all;
clc;
clear;

addpath LPnL-Bar-ENull;
addpath others;
addpath RPnL;




%Type1 Room Camera Pose Estimation

% T1p1 = [626 621;602 19];
% T1p2 = [108 621;402 19];
% 
% 
% T1P1_w = [256 0 256;256 256 266];
% T1P2_w = [256 0 476;256 256 351];

T1p1 = [626 621;626 19;626 621;24 621];
T1p2 = [24 621;24 19;626 19;24 19];

T1P1_w = [256 0 256;256 256 256;256 0 256;256 0 512];
T1P2_w = [256 0 512;256 256 512;256 256 256;256 256 512];


T1A = [202.61578599 0 320;0 202.61578599 320;0 0 1];

for i = 1:length(T1p1)
    temp1 = inv(T1A)*[T1p1(i,:) 1].';
    temp1 = (temp1/temp1(3)).';
    iT1p1(i,:) = temp1(:,1:2);
    temp2 = inv(T1A)*[T1p2(i,:) 1].';
    temp2 = (temp2/temp2(3)).';
    iT1p2(i,:) = temp2(:,1:2);
end


% [T1R, T1T, T1err] = LPnL_Bar_ENull(iT1p1', iT1p2', T1P1_w', T1P2_w');
% errorEpln=reprojection_error_usingRT(T1P1_w,T1p1,T1R,T1T,T1A);
% fprintf('error EPnL: %.3f\n',errorEpln);
[T1R1, T1T1,errASPnL] = ASPnL(iT1p1', iT1p2', T1P1_w', T1P2_w');
%compute error
errorAspnl=reprojection_error_usingRT(T1P1_w,T1p1,T1R1,T1T1,T1A);
fprintf('error Aspnl: %.3f\n',errorAspnl);


[T1R2, T1T2,errorRPnL] = RPnL(iT1p1', iT1p2', T1P1_w', T1P2_w');


[T1R3, T1T3] = SRPnL(iT1p1', iT1p2', T1P1_w', T1P2_w');