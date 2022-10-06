close all;
clc;
clear;

addpath LPnL-Bar-ENull;
addpath others;
addpath RPnL;




%Type1 Room Camera Pose Estimation



T1p1 = [75.93167688320756, 1.1230627628499974;520.975224359587, 318.23706199837716;75.93167688320756, 1.1230627628499974;520.975224359587, 318.23706199837716;75.93167688320756, 1.1230627628499974;520.975224359587, 0.9082217062624522];
T1p2 = [75.93167688320756, 318.3391826185565;520.975224359587, 0.9082217062624522;520.975224359587, 0.9082217062624522;75.93167688320756, 318.3391826185565;520.975224359587, 318.23706199837716;75.93167688320756, 318.3391826185565];

T1P1_w = [256 232 256;512 221 256;256 232 256;512 41 256;256 232 256;512 221 256];
T1P2_w = [256 41 256;512 41 256;512 221 256;256 41 256;512 41 256;256 41 256];





T1A = [512 0 160;0 512 320;0 0 1];

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
[T1R1, T1T1,errASPnL] = ASPnL(iT1p1', iT1p2', T1P1_w', T1P2_w');
%compute error
errorAspnl=reprojection_error_usingRT(T1P1_w,T1p1,T1R1,T1T1,T1A);
fprintf('error Aspnl: %.3f\n',errorAspnl);


[T1R2, T1T2,errorRPnL] = RPnL(iT1p1', iT1p2', T1P1_w', T1P2_w');


[T1R3, T1T3] = SRPnL(iT1p1', iT1p2', T1P1_w', T1P2_w');


[REpnp,TEpnp,Xc,best_solution]=efficient_pnp(T1P1_w,T1p1,T1A)
errorEpnp=reprojection_error_usingRT(T1P1_w,T1p1,REpnp,TEpnp,T1A);