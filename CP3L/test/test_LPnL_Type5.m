close all;
clc;
clear;

addpath LPnL-Bar-ENull;
addpath others;
addpath ASPnL;
addpath SRPnL;



%Type2 Room Camera Pose Estimation

T1p1 = [314 183;318 385;318 403;326 408;324 181];
T1p2 = [277 161;318 282;87 544;589 573;384 145];


T1P1_w = [256 256 261;256 25 256;256 0 256;271 0 256;266 256 256];
T1P2_w = [256 256 306;256 155 256;256 0 441;456 0 256;331 256 256];


T1A = [179.5 0 320;0 179.5 320;0 0 1]; 

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




