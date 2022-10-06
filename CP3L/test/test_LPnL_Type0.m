close all;
clc;
clear;

addpath LPnL-Bar-ENull;
addpath others;
addpath ASPnL;
addpath RPnL;
addpath SRPnL;

%Type0 Room Camera Pose Estimation

p1 = [507 507;508 470;119 468;491 119;515 514;521 111;104 104;72 562];
p2 = [321 510;510 343;115 267;344 117;563 563;574 59;31 32;13 621];



% 
P1T_w = [256 0 256;256 25 256;256 30 512;256 256 271;261 0 256;261 256 256;261 256 512;281 0 512];
P1_w = [256 0 296;256 85 256;256 50 512;256 256 291;269 0 256;271 256 256;281 256 512;291 0 512];
P2_w = [256 0 381;256 110 256;256 160 512;256 256 366;286 0 256;286 256 256;291 256 512;301 0 512];




A = [197.4 0 320;0 197.4 320;0   0   1];

for i = 1:length(p1)
    temp1 = inv(A)*[p1(i,:) 1].';
    temp1 = (temp1/temp1(3)).';
    ip1(i,:) = temp1(:,1:2);
    temp2 = inv(A)*[p2(i,:) 1].';
    temp2 = (temp2/temp2(3)).';
    ip2(i,:) = temp2(:,1:2);
end


[R, T, err] = LPnL_Bar_ENull(ip1', ip2', P1_w', P2_w');
errorEpln=reprojection_error_usingRT(P1T_w,p1,R,T,A);
fprintf('error EPnL: %.3f\n',errorEpln);


[R1, T1] = ASPnL(ip1', ip2', P1_w', P2_w');
%compute error
errorAspnl=reprojection_error_usingRT(P1T_w,p1,R1,T1,A);
fprintf('error Aspnl: %.3f\n',errorAspnl);


[R2, T2] = SRPnL(ip1', ip2', P1_w', P2_w');
errorSRPnL=reprojection_error_usingRT(P1T_w,p1,R2,T2,A);
fprintf('error SRPnL: %.3f\n',errorSRPnL);


