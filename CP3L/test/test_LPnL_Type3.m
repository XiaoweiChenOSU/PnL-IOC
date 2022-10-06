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




% T1P1_w = [256 256 326;256 115 256;271 256 256];
T1P1_w = [256 256 256;256 256 256;256 256 256];
T1P2_w = [256 256 371;256 160 256;371 256 256];








T1A = [512 0 320;0 512 320;0   0   1];

% R = [0.691791089481684,0.0105610713676920,-0.722020465281498;-0.269880973920502,-0.923650477233471,-0.272091998820441;-0.669768130347709,0.383090406663946,-0.636122937719264];
% T = [3.58733792203451;339.006099989604;445.999564598126];

% R = [0.70710678,0.0000007,-0.70710678;-0.27628863,-0.92050485,-0.27628863;-0.65089522,0.39073113,-0.65089522];
R = [0.706022365110961,0.00208931738729643,-0.708186454767373;-0.277261054637476,-0.919355709937976,-0.279126111615798;-0.651658443929364,0.393421800815988,-0.648506406366402];
T = [-2.12132034;341.15198673;442.84175724];


% ############## Camera Rotation Matrix ##################
% [[ 0.70710678  0.          0.70710678]
%  [ 0.27628863  0.92050485 -0.27628863]
%  [-0.65089522  0.39073113  0.65089522]]
% 
% 
% ############## Camera Translation Matrix ##################
% [  -2.12132034 -341.15198673  442.84175724]



% 
% T1p1 = [159 148;313 630;341 218];
% T1p2 = [3 63;314 475;620 63];
% 
% T1p1 = [159 148;313 630;341 218];
T1p1 = [315, 233;315, 233;315, 233];
T1p2 = [3 63;314 475;620 63];

x3d_h = [T1P1_w(:,1:3);T1P2_w(:,1:3)];
x2d_h = [T1p1;T1p2];


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


[T1R, T1T, T1err] = LPnL_Bar_ENull(T1p1', T1p2', T1P1_w', T1P2_w');

% [T1R, T1T, T1err] = LPnL_Bar_ENull(iT1p1', iT1p2', T1P1_nw', T1P2_nw');
% errorEpln=reprojection_error_usingRT(T1P1_nw,T1p1,T1R,T1T,T1A);
% fprintf('error EPnL: %.3f\n',errorEpln);
% 
% % 
[T1R1, T1T1,errASPnL] = ASPnL(iT1p1', iT1p2', T1P1_w', T1P2_w');
% pv = -[315,233,1]';
% PW = [256,256,256];
% % PW1 = [256, 256, 326];
% % pv1 = -[159,148,1]';
% PW1 = [371, 256, 256];
% pv1 = -[620,63,1]';


pv = -[3,63,1]';
PW = [256,256,371];
% 
T1A = [512 0 320;0 512 -320;0   0   1];
A = [T1A pv];%;T1A pv1
f = 512;
u0 = 320;
v0 = 320;
% 
% 
% 
B = [f*T1R1(1,1)*PW(1)+T1R1(3,1)*u0*PW(1)+f*T1R1(1,2)*PW(2)+T1R1(3,2)*u0*PW(2)+f*T1R1(1,3)*PW(3)+T1R1(3,3)*u0*PW(3);...
    f*T1R1(2,1)*PW(1)+T1R1(3,1)*v0*PW(1)+f*T1R1(2,2)*PW(2)+T1R1(3,2)*v0*PW(2)+f*T1R1(2,3)*PW(3)+T1R1(3,3)*v0*PW(3);...
    T1R1(3,1)*PW(1)+T1R1(3,2)*PW(2)+T1R1(3,3)*PW(3)];
%     ;...
%     f*T1R1(1,1)*PW1(1)+T1R1(3,1)*u0*PW1(1)+f*T1R1(1,2)*PW1(2)+T1R1(3,2)*u0*PW1(2)+f*T1R1(1,3)*PW1(3)+T1R1(3,3)*u0*PW1(3);...
%     f*T1R1(2,1)*PW1(1)+T1R1(3,1)*v0*PW1(1)+f*T1R1(2,2)*PW1(2)+T1R1(3,2)*v0*PW1(2)+f*T1R1(2,3)*PW1(3)+T1R1(3,3)*v0*PW1(3);...
%     T1R1(3,1)*PW1(1)+T1R1(3,2)*PW1(2)+T1R1(3,3)*PW1(3)];
% 
% 
% 
s = A\B;
s1 = lsqminnorm(A,B,1e-2)
p1 = T1p1';
p2 = T1p2';
nl = getProjNorm(p1,p2);
pv = 100*[315,233,1];
temp2 = T1A\pv.';

Ttemp = temp2 - T1R1*[256,256,256]';

TEST = T +  T1R1*[256,256,256]';


% %compute error
% % errorAspnl=reprojection_error_usingRT(T1P1_w,T1p1,T1R1,T1T1,T1A);
% % fprintf('error Aspnl: %.3f\n',errorAspnl);
% % 
% % 
[T1R2, T1T2,errorRPnL] = RPnL(iT1p1', iT1p2', T1P1_nw', T1P2_nw');
% for i = 1:size(T1R2,3)
%     errorRPnL(i)=reprojection_error_usingRT(T1P1_nw,T1p1,T1R2(:,:,i),T1T2(:,i),T1A);
% end
% % 
% % 
[T1R3, T1T3] = SRPnL(iT1p1', iT1p2', T1P1_nw', T1P2_nw');
% for i = 1:size(T1R3,3)
%     errorSRPnL(i)=reprojection_error_usingRT(T1P1_nw,T1p1,T1R3(:,:,i),T1T3(:,i),T1A);
% end
% % 
% % % 
solP3L =  P3L(iT1p1', iT1p2', T1P1_nw', T1P2_nw');
for i = 1:size(solP3L,2)
    errorP3L(i)=reprojection_error_usingRT(T1P1_nw,T1p1,solP3L(i).R,solP3L(i).T,T1A);
end
% 
solmyP3L =  myP3L(iT1p1', iT1p2', T1P1_nw', T1P2_nw');
for i = 1:size(solmyP3L,2)
    errormyP3L(i)=reprojection_error_usingRT(T1P1_nw,T1p1,solmyP3L(i).R,solmyP3L(i).T,T1A);
end

[REpnp,TEpnp,Xc,best_solution]=efficient_pnp(x3d_h,x2d_h,T1A);
errorEpnp=reprojection_error_usingRT(x3d_h,x2d_h,REpnp,TEpnp,T1A);







