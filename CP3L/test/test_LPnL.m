close all;
clc;
clear;

addpath LPnL-Bar-ENull;
addpath others;
addpath RPnL;
addpath SRPnL;
addpath P3Lmethods;
addpath ASPnL;



%Type1 Room Camera Pose Estimation

T1p1 = [159 148;313 630;341 218];
T1p2 = [3 63;314 475;620 63];


T1P1_w = [256 256 326;256 115 256;271 256 256];
T1P2_w = [256 256 371;256 160 256;371 256 256];

x3d_h = [T1P1_w;T1P2_w];
x2d_h = [T1p1;T1p2];


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

% [T1R, T1T, T1err] = LPnL_Bar_ENull(iT1p1', iT1p2', T1P1_w', T1P2_w');
% errorEpln=reprojection_error_usingRT(T1P1_w,T1p1,T1R,T1T,T1A);
% fprintf('error EPnL: %.3f\n',errorEpln);


% [T1R1, T1T1,errASPnL] = ASPnL(iT1p1', iT1p2', T1P1_w', T1P2_w');
%compute error
% errorAspnl=reprojection_error_usingRT(T1P1_w,T1p1,T1R1,T1T1,T1A);
% fprintf('error Aspnl: %.3f\n',errorAspnl);


% [T1R2, T1T2,errorRPnL] = RPnL(iT1p1', iT1p2', T1P1_w', T1P2_w');
% for i = 1:size(T1R2,3)
%     errorRPnL(i)=reprojection_error_usingRT(T1P1_w,T1p1,T1R2(:,:,i),T1T2(:,i),T1A);
% end


% [T1R3, T1T3] = SRPnL(iT1p1', iT1p2', T1P1_w', T1P2_w');
% for i = 1:size(T1R3,3)
%     errorSRPnL(i)=reprojection_error_usingRT(T1P1_w,T1p1,T1R3(:,:,i),T1T3(:,i),T1A);
% end


solP3L =  P3L(iT1p1', iT1p2', T1P1_w', T1P2_w');
% for i = 1:size(solP3L,2)
%     errorP3L(i)=reprojection_error_usingRT(T1P1_w,T1p1,solP3L(i).R,solP3L(i).T,T1A);
% end

solmyP3L =  myP3L(iT1p1', iT1p2', T1P1_w', T1P2_w');
% for i = 1:size(solmyP3L,2)
%     errormyP3L(i)=reprojection_error_usingRT(T1P1_w,T1p1,solmyP3L(i).R,solmyP3L(i).T,T1A);
% end

% [REpnp,TEpnp,Xc,best_solution]=efficient_pnp(x3d_h,x2d_h,T1A)
% errorEpnp=reprojection_error_usingRT(x3d_h,x2d_h,REpnp,TEpnp,T1A);







