close all;
clc;
clear;

addpath LPnL-Bar-ENull;
addpath others;
addpath RPnL;
addpath ASPnL;
addpath P3Lmethods;

syms b k

%Type1 Room Camera Pose Estimation

% T1p1 = [337 355;337 327;383 389];
T1p1 = [337 355;337 355;337 355];
T1p2 = [19 472;340 1;587 541];


% nl = []
% 
% for i = 1:3
%     equns = [k*T1p1(i,1)+b-T1p1(i,2)==0,k*T1p2(i,1)+b-T1p2(i,2)==0];
%     S = solve(equns,[k,b]);
%     nl(:,i) = [S.k,-1,S.b/512]'
% end
% 
% 
% nl = xnorm(nl);


% T1P1_w = [256 0 256;256 0 256;256 0 256];
T1P1_w = [256 0 256;256 10 256;271 0 256];
T1P2_w = [256 0 371;256 120 256;316 0 256];

% for i = 1:3
% %     T1P1_nw(i,:) = [T1P1_w(i,:) 1] * [1 0 0 0;0 cos(pi/4) sin(pi/4) 0;0 -sin(pi/4) cos(pi/4) 0;0 0 0 1] * [cos(pi/4) 0 -sin(pi/4) 0;0 1 0 0;sin(pi/4) 0 cos(pi/4) 0;0 0 0 1] * [cos(pi/4) sin(pi/4) 0 0;-sin(pi/4) cos(pi/4) 0 0;0 0 1 0;0 0 0 1]
% %     T1P2_nw(i,:) = [T1P2_w(i,:) 1] * [1 0 0 0;0 cos(pi/4) sin(pi/4) 0;0 -sin(pi/4) cos(pi/4) 0;0 0 0 1] * [cos(pi/4) 0 -sin(pi/4) 0;0 1 0 0;sin(pi/4) 0 cos(pi/4) 0;0 0 0 1] * [cos(pi/4) sin(pi/4) 0 0;-sin(pi/4) cos(pi/4) 0 0;0 0 1 0;0 0 0 1]
% % %     T1P1_nw(i,:) = [T1P1_w(i,:) 1] * [1 0 0 0;0 cos(pi/4) sin(pi/4) 0;0 -sin(pi/4) cos(pi/4) 0;0 0 0 1];
% %     T1P2_nw(i,:) = [T1P2_w(i,:) 1] * [1 0 0 0;0 cos(pi/4) sin(pi/4) 0;0 -sin(pi/4) cos(pi/4) 0;0 0 0 1];
%     T1P1_nw(i,:) = [T1P1_w(i,:) 1] * [1 0 0 0;0 cos(0) sin(0) 0;0 -sin(0) cos(0) 0;0 0 0 1] * [cos(0) 0 -sin(0) 0;0 1 0 0;sin(0) 0 cos(0) 0;0 0 0 1] * [cos(0) sin(0) 0 0;-sin(0) cos(0) 0 0;0 0 1 0;0 0 0 1]
%     T1P2_nw(i,:) = [T1P2_w(i,:) 1] * [1 0 0 0;0 cos(0) sin(0) 0;0 -sin(0) cos(0) 0;0 0 0 1] * [cos(0) 0 -sin(0) 0;0 1 0 0;sin(0) 0 cos(0) 0;0 0 0 1] * [cos(0) sin(0) 0 0;-sin(0) cos(0) 0 0;0 0 1 0;0 0 0 1]
% %  
% end



% R = [0.690114091724918,-0.0142297833406175,-0.723560677254349;0.335813825369856,-0.879355310526770,0.337584526509781;-0.641070678703337,-0.475953517808671,-0.602076933449867];
% T = [7.09655523226396;-143.068280845065;544.270983559964];

R = [0.32432927,-0.02837513,-0.94551858;0.04478246,-0.99796869,0.04531033;-0.94488362,-0.05703811,-0.32239975];
T = [272.94644862;97.0883716;497.28068964];





T1A = [512 0 0;0 512 0;0   0   1];



% T1p1 = [];
% T1p2 = [];
% for i = 1:3
%     P1 = T1P1_nw(i,:)';
%     TD1 = (T1A*[R,T]*P1)';
%     TD1 = (TD1/TD1(:,3))
%     T1p1(i,:) = TD1(:,1:2);
%     P2 = T1P2_nw(i,:)';
%     TD2 = (T1A*[R,T]*P2)';
%     TD2 = (TD2/TD2(:,3))
%     T1p2(i,:) = TD2(:,1:2);
% end
% 
% T1P1_nw = T1P1_nw(:,1:3);
% T1P2_nw = T1P2_nw(:,1:3);

x3d_h = [T1P1_w(:,1:3);T1P2_w(:,1:3)];
x2d_h = [T1p1;T1p2];


for i = 1:length(T1p1)
    temp1 = inv(T1A)*[T1p1(i,:) 1].';
    temp1 = (temp1/temp1(3)).';
    iT1p1(i,:) = temp1(:,1:2);
    temp2 = inv(T1A)*[T1p2(i,:) 1].';
    temp2 = (temp2/temp2(3)).';
    iT1p2(i,:) = temp2(:,1:2);
end






% 
% [T1R, T1T, T1err] = LPnL_Bar_ENull(iT1p1', iT1p2', T1P1_w', T1P2_w');
% errorEpln=reprojection_error_usingRT(T1P1_w,T1p1,T1R,T1T,T1A);
% fprintf('error EPnL: %.3f\n',errorEpln);
% 
% 
[T1R1, T1T1] = ASPnL(iT1p1', iT1p2', T1P1_w', T1P2_w');
% %compute error
% % errorAspnl=reprojection_error_usingRT(T1P1_w,T1p1,T1R1,T1T1,T1A);
% % fprintf('error Aspnl: %.3f\n',errorAspnl);
% 
% 
% [T1R2, T1T2,errorRPnL] = RPnL(iT1p1', iT1p2', T1P1_w', T1P2_w');
% for i = 1:size(T1R2,3)
%     errorRPnL(i)=reprojection_error_usingRT(T1P1_w,T1p1,T1R2(:,:,i),T1T2(:,i),T1A);
% end
% 
% 
% [T1R3, T1T3] = SRPnL(iT1p1', iT1p2', T1P1_w', T1P2_w');
% for i = 1:size(T1R3,3)
%     errorSRPnL(i)=reprojection_error_usingRT(T1P1_w,T1p1,T1R3(:,:,i),T1T3(:,i),T1A);
% end
% 
% 
% sol1 =  P3L(iT1p1', iT1p2', T1P1_w', T1P2_w');
% for i = 1:size(sol1,2)
%     errorP3L(i)=reprojection_error_usingRT(T1P1_w,T1p1,sol1(i).R,sol1(i).T,T1A);
% end
% 
% sol2 =  myP3L(iT1p1', iT1p2', T1P1_w', T1P2_w');
% for i = 1:size(sol2,2)
%     errormyP3L(i)=reprojection_error_usingRT(T1P1_w,T1p1,sol2(i).R,sol2(i).T,T1A);
% end
% 

[REpnp,TEpnp,Xc,best_solution]=efficient_pnp(x3d_h,x2d_h,T1A)
errorEpnp=reprojection_error_usingRT(x3d_h,x2d_h,REpnp,TEpnp,T1A);



errorSimu=reprojection_error_usingRT(x3d_h,x2d_h,R,T,T1A);



PW = [256, 0, 256];
p0 = [337, 355];

% pv = -[337, 355, 1]'
% A = [T1A pv];%;T1A pv1
% f = 512;
% u0 = 0;
% v0 = 0;
% 
% B = [f*T1R1(1,1)*PW(1)+T1R1(3,1)*u0*PW(1)+f*T1R1(1,2)*PW(2)+T1R1(3,2)*u0*PW(2)+f*T1R1(1,3)*PW(3)+T1R1(3,3)*u0*PW(3);...
%     f*T1R1(2,1)*PW(1)+T1R1(3,1)*v0*PW(1)+f*T1R1(2,2)*PW(2)+T1R1(3,2)*v0*PW(2)+f*T1R1(2,3)*PW(3)+T1R1(3,3)*v0*PW(3);...
%     T1R1(3,1)*PW(1)+T1R1(3,2)*PW(2)+T1R1(3,3)*PW(3)];
% 
% s = A\B;
% s1 = lsqminnorm(A,B,1)
% p1 = T1p1';
% p2 = T1p2';
% nl = getProjNorm(p1,p2);
% 
% pv = [337,355,1];
% 
% 
% temp2 = 172*(T1A\pv.');
% 
% Ttemp = temp2 - R*PW';
% 
% TEST = T +  R*[256,0,256]';


pv = -[337, 355, 1]';
PW1 = [256 0 371];
pv2 = -[19, 472, 1]';

oT = [0,0,0]'

A = [T1A pv oT;T1A oT pv2];%;T1A pv1
f = 512;
u0 = 0;
v0 = 0;


B = [f*T1R1(1,1)*PW(1)+T1R1(3,1)*u0*PW(1)+f*T1R1(1,2)*PW(2)+T1R1(3,2)*u0*PW(2)+f*T1R1(1,3)*PW(3)+T1R1(3,3)*u0*PW(3);...
    f*T1R1(2,1)*PW(1)+T1R1(3,1)*v0*PW(1)+f*T1R1(2,2)*PW(2)+T1R1(3,2)*v0*PW(2)+f*T1R1(2,3)*PW(3)+T1R1(3,3)*v0*PW(3);...
    T1R1(3,1)*PW(1)+T1R1(3,2)*PW(2)+T1R1(3,3)*PW(3);...
    f*T1R1(1,1)*PW1(1)+T1R1(3,1)*u0*PW1(1)+f*T1R1(1,2)*PW1(2)+T1R1(3,2)*u0*PW1(2)+f*T1R1(1,3)*PW1(3)+T1R1(3,3)*u0*PW1(3);...
    f*T1R1(2,1)*PW1(1)+T1R1(3,1)*v0*PW1(1)+f*T1R1(2,2)*PW1(2)+T1R1(3,2)*v0*PW1(2)+f*T1R1(2,3)*PW1(3)+T1R1(3,3)*v0*PW1(3);...
    T1R1(3,1)*PW1(1)+T1R1(3,2)*PW1(2)+T1R1(3,3)*PW1(3)];



s = A\B;
s1 = lsqminnorm(A,B,1e-2)

pv = [337,355,1];
temp2 = s(4)*(T1A\pv.');

Ttemp = temp2 - R*PW';

PW1 = [256 0 371];
pv1 = [19,472,1];
temp21 = s(5)*(T1A\pv1.');

Ttemp1 = temp21 - R*PW1';



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%







