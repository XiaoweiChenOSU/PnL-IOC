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

% T1p1 = [159 148;313 630;341 218];
% T1p2 = [3 63;314 475;620 63];


% T1p1 = [377.160272167267,193.182386305628;434.294483920141,617.427286764249;323.671428042403,184.841125502906];
% T1p2 = [458.024015909215,215.095257827910;262.992118341717,464.978766389276;599.427681322131,11.2326067979413];



T1P1_w = [266 286 226;266 115 206;271 266 266];
T1P2_w = [276 296 171;266 160 286;371 263 268];


for i = 1:3
    T1P1_nw(i,:) = [T1P1_w(i,:) 1];
    T1P2_nw(i,:) = [T1P2_w(i,:) 1];  
end





% for i = 1:3
% %     T1P1_nw(i,:) = [T1P1_w(i,:) 1] * [1 0 0 0;0 cos(pi/4) sin(pi/4) 0;0 -sin(pi/4) cos(pi/4) 0;0 0 0 1] * [cos(pi/4) 0 -sin(pi/4) 0;0 1 0 0;sin(pi/4) 0 cos(pi/4) 0;0 0 0 1] * [cos(pi/4) sin(pi/4) 0 0;-sin(pi/4) cos(pi/4) 0 0;0 0 1 0;0 0 0 1]
% %     T1P2_nw(i,:) = [T1P2_w(i,:) 1] * [1 0 0 0;0 cos(pi/4) sin(pi/4) 0;0 -sin(pi/4) cos(pi/4) 0;0 0 0 1] * [cos(pi/4) 0 -sin(pi/4) 0;0 1 0 0;sin(pi/4) 0 cos(pi/4) 0;0 0 0 1] * [cos(pi/4) sin(pi/4) 0 0;-sin(pi/4) cos(pi/4) 0 0;0 0 1 0;0 0 0 1]
%     T1P1_nw(i,:) = [T1P1_w(i,:) 1] * [1 0 0 0;0 cos(pi/4) sin(pi/4) 0;0 -sin(pi/4) cos(pi/4) 0;0 0 0 1];
%     T1P2_nw(i,:) = [T1P2_w(i,:) 1] * [1 0 0 0;0 cos(pi/4) sin(pi/4) 0;0 -sin(pi/4) cos(pi/4) 0;0 0 0 1];
%     T1P1_nw(i,:) = [T1P1_w(i,:) 1] * [1 0 0 0;0 cos(pi/8) sin(pi/8) 0;0 -sin(pi/8) cos(pi/8) 0;0 0 0 1] * [cos(pi/8) 0 -sin(pi/8) 0;0 1 0 0;sin(pi/8) 0 cos(pi/8) 0;0 0 0 1] * [cos(pi/8) sin(pi/8) 0 0;-sin(pi/8) cos(pi/8) 0 0;0 0 1 0;0 0 0 1]
%     T1P2_nw(i,:) = [T1P2_w(i,:) 1] * [1 0 0 0;0 cos(pi/8) sin(pi/8) 0;0 -sin(pi/8) cos(pi/8) 0;0 0 0 1] * [cos(pi/8) 0 -sin(pi/8) 0;0 1 0 0;sin(pi/8) 0 cos(pi/8) 0;0 0 0 1] * [cos(pi/8) sin(pi/8) 0 0;-sin(pi/8) cos(pi/8) 0 0;0 0 1 0;0 0 0 1]
% % %  
% %     T1P1_nw(i,:) = [T1P1_w(i,:) 1] * [1 0 0 0;0 cos(0) sin(0) 0;0 -sin(0) cos(0) 0;0 0 0 1] * [cos(0) 0 -sin(0) 0;0 1 0 0;sin(0) 0 cos(0) 0;0 0 0 1] * [cos(0) sin(0) 0 0;-sin(0) cos(0) 0 0;0 0 1 0;0 0 0 1];
% %     T1P2_nw(i,:) = [T1P2_w(i,:) 1] * [1 0 0 0;0 cos(0) sin(0) 0;0 -sin(0) cos(0) 0;0 0 0 1] * [cos(0) 0 -sin(0) 0;0 1 0 0;sin(0) 0 cos(0) 0;0 0 0 1] * [cos(0) sin(0) 0 0;-sin(0) cos(0) 0 0;0 0 1 0;0 0 0 1];
% %  
% end




T1A = [512 0 320;0 512 320;0   0   1];

% R = [0.691791089481684,0.0105610713676920,-0.722020465281498;-0.269880973920502,-0.923650477233471,-0.272091998820441;-0.669768130347709,0.383090406663946,-0.636122937719264];
% T = [3.58733792203451;339.006099989604;445.999564598126];

R = [0.70710678,0.0000007,-0.70710678;-0.27628863,-0.92050485,-0.27628863;-0.65089522,0.39073113,-0.65089522];
T = [-2.12132034;341.15198673;442.84175724];


% ############## Camera Rotation Matrix ##################
% [[ 0.70710678  0.          0.70710678]
%  [ 0.27628863  0.92050485 -0.27628863]
%  [-0.65089522  0.39073113  0.65089522]]
% 
% 
% ############## Camera Translation Matrix ##################
% [  -2.12132034 -341.15198673  442.84175724]




T1p1 = [];
T1p2 = [];
for i = 1:3
    P1 = T1P1_nw(i,:)';
    TD1 = (T1A*[R,T]*P1)';
    TD1 = (TD1/TD1(:,3));
    T1p1(i,:) = TD1(:,1:2);
    P2 = T1P2_nw(i,:)';
    TD2 = (T1A*[R,T]*P2)';
    TD2 = (TD2/TD2(:,3));
    T1p2(i,:) = TD2(:,1:2);
end



T1P1_nw = T1P1_nw(:,1:3);
T1P2_nw = T1P2_nw(:,1:3);

% T1p1 = [162.644201772959,123.369192749003;337.122742015488,635.039183816621;323.671428042403,184.841125502906];
% T1p2 = [11.9158551291277,35.9277468622570;335.304705047507,473.510691913258;599.427681322131,11.2326067979413];

x3d_h = [T1P1_nw(:,1:3);T1P2_nw(:,1:3)];
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


% [T1R, T1T, T1err] = LPnL_Bar_ENull(T1p1', T1p2', T1P1_w', T1P2_w');

[T1R, T1T, T1err] = LPnL_Bar_ENull(iT1p1', iT1p2', T1P1_nw', T1P2_nw');
errorEpln=reprojection_error_usingRT(T1P1_nw,T1p1,T1R,T1T,T1A);
fprintf('error EPnL: %.3f\n',errorEpln);

% 
[T1R1, T1T1,errASPnL] = ASPnL(iT1p1', iT1p2', T1P1_nw', T1P2_nw');
%compute error
% errorAspnl=reprojection_error_usingRT(T1P1_w,T1p1,T1R1,T1T1,T1A);
% fprintf('error Aspnl: %.3f\n',errorAspnl);
% 
% 
[T1R2, T1T2,errorRPnL] = RPnL(iT1p1', iT1p2', T1P1_nw', T1P2_nw');
for i = 1:size(T1R2,3)
    errorRPnL(i)=reprojection_error_usingRT(T1P1_nw,T1p1,T1R2(:,:,i),T1T2(:,i),T1A);
end
% 
% 
[T1R3, T1T3] = SRPnL(iT1p1', iT1p2', T1P1_nw', T1P2_nw');
for i = 1:size(T1R3,3)
    errorSRPnL(i)=reprojection_error_usingRT(T1P1_nw,T1p1,T1R3(:,:,i),T1T3(:,i),T1A);
end
% 
% % 
solP3L =  P3L(iT1p1', iT1p2', T1P1_nw', T1P2_nw');
for i = 1:size(solP3L,2)
    errorP3L(i)=reprojection_error_usingRT(T1P1_nw,T1p1,solP3L(i).R,solP3L(i).T,T1A);
end

solmyP3L =  myP3L(iT1p1', iT1p2', T1P1_nw', T1P2_nw');
for i = 1:size(solmyP3L,2)
    errormyP3L(i)=reprojection_error_usingRT(T1P1_nw,T1p1,solmyP3L(i).R,solmyP3L(i).T,T1A);
end

[REpnp,TEpnp,Xc,best_solution]=efficient_pnp(x3d_h,x2d_h,T1A)
errorEpnp=reprojection_error_usingRT(x3d_h,x2d_h,REpnp,TEpnp,T1A);







