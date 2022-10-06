close all;
clc;
clear;

addpath LPnL-Bar-ENull;
addpath others;
addpath RPnL;
addpath ASPnL;
addpath P3Lmethods;
addpath SRPnL;


%Type4 Room Camera Pose Estimation

% T1p1 = [279 297;279 297;279 297];
% T1p2 = [1 470;278 2;635 490];
% 
% 
% 
% 
% T1P1_w = [256 0 256;256 0 256;256 0 256];
% T1P2_w = [256 0 371;256 105 256;336 0 256];




% 
% T1p1 = [279 297;279 215;357 340];
% T1p2 = [1 470;278 2;635 490];
% T1p2 = [279 297;279 215;357 340];

T1p1 = [280 298;280 216;358 341];
T1p2 = [2.6774 471;279 17;636 491];
% 
% T1P1_w = [256 0 256;256 30 256;281 0 256];
% T1P2_w = [256 0 371;256 100 256;336 0 256];


% T1p1 = [280 298;280 298;280 298];
% T1p2 = [2.6774 471;279 17;636 491];
% 
T1P1_w = [256 0 256;256 0 256;256 0 256];
T1P2_w = [256 0 371;256 100 256;336 0 256];



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

% R = [0.32432927,-0.02837513,-0.94551858;0.04478246,-0.99796869,0.04531033;-0.94488362,-0.05703811,-0.32239975];
% T = [272.94644862;97.0883716;497.28068964];





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
[T1R1, T1T1, err1] = ASPnL(iT1p1', iT1p2', T1P1_w', T1P2_w');
% %compute error
% % errorAspnl=reprojection_error_usingRT(T1P1_w,T1p1,T1R1,T1T1,T1A);
% % fprintf('error Aspnl: %.3f\n',errorAspnl);
% 
% 
[T1R2, T1T2, errorRPnL] = RPnL(iT1p1', iT1p2', T1P1_w', T1P2_w');
% for i = 1:size(T1R2,3)
%     errorRPnL(i)=reprojection_error_usingRT(T1P1_w,T1p1,T1R2(:,:,i),T1T2(:,i),T1A);
% end
% 
% 
[T1R3, T1T3] = SRPnL(iT1p1', iT1p2', T1P1_w', T1P2_w');
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


% R  = REpnp;

%


% R = [ 0.51378347 -0.03592724  0.8571673 ;...
%   -0.03497143  0.99741529  0.06276735;...
%  -0.85720683 -0.0622252   0.51119906]

syms X Y Z t1 t2 t3 Z0 Z1 Z2 Z3
 

f = 512;


% R =  [0.5152   -0.0365   -0.8563
%     0.0315   -0.9976    0.0615
%    -0.8565   -0.0586   -0.5128];
% T = [188.185080790200;82.0738220787612;531.480639133733];


T1p1 = [280 298;280 216;358 341];
T1p2 = [2.6774 471;279 17;636 491];
% T1p2 = [1 470;278 2;635 490];
PW0 = [256, 0, 256];
% 
% temmP0 = T1A*[R T]*[PW0 1]';
% temmP0 = temmP0/temmP0(3);
p0 = [280 298 1];
% p0 = [279 297 1];

pc0 = T1A\p0.';

PW1 = [256, 0, 371];
p1 = [2.6774 471 1];
% p1 =[1 470 1];
pc1 = T1A\p1.';


PW2 = [256, Y, 256];
p2 = [279 17 1];
% p2 = [278 2 1];
pc2 = T1A\p2.';


PW3 = [X, 0, 256];
p3 = [636 491 1];
% p3 = [635 490 1];
pc3 = T1A\p3.';


% A = [
%      0 0 0 f 0 0 -p0(1) 0 0 0;...
%      0 0 0 0 f 0 -p0(2) 0 0 0;...
%      0 0 0 0 0 1 -1 0 0 0;...
%      0 0 f*R(1,3) f 0 0 0 -p1(1) 0 0;...
%      0 0 f*R(2,3) 0 f 0 0 -p1(2) 0 0;...
%      0 0 R(3,3) 0 0 1 0 -1 0 0;...
%      0 f*R(1,2) 0 f 0 0 0 0 -p2(1) 0;...
%      0 f*R(2,2) 0 0 f 0 0 0 -p2(2) 0;...
%      0 R(3,2) 0 0 0 1 0 0 -1 0;...
%      f*R(1,1) 0 0 f 0 0 0 0 0 -p3(1);...
%      f*R(2,1) 0 0 0 f 0 0 0 0 -p3(2);...
%      R(3,1) 0 0 0 0 1 0 0 0 -1;...
%     ]
% 
% b = [
%       f*R(1,1)*PW0(1)+f*R(1,2)*PW0(2)+f*R(1,3)*PW0(3);...
%       f*R(2,1)*PW0(1)+f*R(2,2)*PW0(2)+f*R(2,3)*PW0(3);...
%       R(3,1)*PW0(1)+R(3,2)*PW0(2)+R(3,3)*PW0(3);...
%       f*R(1,1)*PW1(1)+f*R(1,2)*PW1(2);...
%       f*R(2,1)*PW1(1)+f*R(2,2)*PW1(2);...
%       R(3,1)*PW1(1)+R(3,2)*PW1(2);...
%       f*R(1,1)*PW2(1)+f*R(1,3)*PW2(3);...
%       f*R(2,1)*PW2(1)+f*R(2,3)*PW2(3);...
%       R(3,1)*PW2(1)+R(3,3)*PW2(3);...
%       f*R(1,2)*PW3(2)+f*R(1,3)*PW3(3);...
%       f*R(2,2)*PW3(2)+f*R(2,3)*PW3(3);...
%       R(3,2)*PW3(2)+R(3,3)*PW3(3);...
%     ]
for i =1:size(T1R2,3)
    R = T1R2(:,:,i)
    A = [0 0 0 f 0 -p0(1);...
         0 0 0 0 f -p0(2);...
         0 0 f*R(1,3)-p1(1)*R(3,3) f 0 -p1(1);...
         0 0 f*R(2,3)-p1(2)*R(3,3) 0 f -p1(2);...
         0 f*R(1,2)-p2(1)*R(3,2) 0 f 0 -p2(1);...
         0 f*R(2,2)-p2(2)*R(3,2) 0 0 f -p2(2);...
         f*R(1,1)-p3(1)*R(3,1) 0 0 f 0 -p3(1);...
         f*R(2,1)-p3(2)*R(3,1) 0 0 0 f -p3(2);...
    %      0 0 1 0 0 0;...
    %      0 1 0 0 0 0;...
    %      1 0 0 0 0 0;
         ];
     b = [
          f*R(1,1)*PW0(1)+f*R(1,2)*PW0(2)+f*R(1,3)*PW0(3)-p0(1)*R(3,1)*PW0(1)-p0(1)*R(3,2)*PW0(2)-p0(1)*R(3,3)*PW0(3);...
          f*R(2,1)*PW0(1)+f*R(2,2)*PW0(2)+f*R(2,3)*PW0(3)-p0(2)*R(3,1)*PW0(1)-p0(2)*R(3,2)*PW0(2)-p0(2)*R(3,3)*PW0(3);...
          f*R(1,1)*PW1(1)+f*R(1,2)*PW1(2)-p1(1)*R(3,1)*PW1(1)-p1(1)*R(3,2)*PW1(2);...
          f*R(2,1)*PW1(1)+f*R(2,2)*PW1(2)-p1(2)*R(3,1)*PW1(1)-p1(2)*R(3,2)*PW1(2);...
          f*R(1,1)*PW2(1)+f*R(1,3)*PW2(3)-p2(1)*R(3,1)*PW2(1)-p2(1)*R(3,3)*PW2(3);...
          f*R(2,1)*PW2(1)+f*R(2,3)*PW2(3)-p2(2)*R(3,1)*PW2(1)-p2(2)*R(3,3)*PW2(3);...
          f*R(1,2)*PW3(2)+f*R(1,3)*PW3(3)-p3(1)*R(3,2)*PW3(2)-p3(1)*R(3,3)*PW3(3);...
          f*R(2,2)*PW3(2)+f*R(2,3)*PW3(3)-p3(2)*R(3,2)*PW3(2)-p3(2)*R(3,3)*PW3(3);...
    %       -371;
    %       -105;
         ]
     
     x =double(A\-b)
     x1 = double(linsolve(A,-b))
     s1 = lsqminnorm(A,double(b),1e-2)
     
     temmP0 = T1A*[R s1(3:5,:)]*[PW0 1]';
     
     temmP0 = temmP0/temmP0(3);
     
     
     x1 = [336 105 371 189.426121320767 80.7895147131118 531.310178777817]';
     re = double(A*x1 + b)
     M = [A b];
     [U, S, V] =  svd(M);
     v1 = V(:,7)/V(7,7)
     v2 = V(:,6)/V(7,6)
     v3 = V(:,5)/V(7,5)
     v4 = V(:,4)/V(7,4)
     v5 = V(:,3)/V(7,3)
     v6 = V(:,2)/V(7,2)
     v7 = V(:,1)/V(7,1)
end
% 
% b = -[
%     f*R(1,1)*PW0(1,1)+f*R(1,2)*PW0(1,2)+f*R(1,3)*PW0(1,3)-p0(1,1)*R(3,1)*PW0(1,1)-p0(1,1)*R(3,2)*PW0(1,2)-p0(1,1)*R(3,3)*PW0(1,3);...
%     f*R(2,1)*PW0(1,1)+f*R(2,2)*PW0(1,2)+f*R(2,3)*PW0(3)-p0(2)*R(3,1)*PW0(1)-p0(2)*R(3,2)*PW0(2)-p0(2)*R(3,3)*PW0(3);...
%     f*R(1,1)*PW1(1)+f*R(1,2)*PW1(2)-p1(1)*R(3,1)*PW1(1)-p1(1)*R(3,2)*PW1(2);...
%     f*R(2,1)*PW1(1)+f*R(2,2)*PW1(2)-p1(2)*R(3,1)*PW1(1)-p1(2)*R(3,2)*PW1(2);...
%     f*R(1,1)*PW2(1)+f*R(1,3)*PW2(3)-p2(1)*R(3,1)*PW2(1)-p2(1)*R(3,3)*PW2(3);...
%     f*R(2,1)*PW2(1)+f*R(2,3)*PW2(3)-p2(2)*R(3,1)*PW2(1)-p2(2)*R(3,3)*PW2(3);...
%     f*R(1,2)*PW3(2)+f*R(1,3)*PW3(3)-p3(1)*R(3,2)*PW3(2)-p3(1)*R(3,3)*PW3(3);...
%     f*R(2,2)*PW3(2)+f*R(2,3)*PW3(3)-p3(2)*R(3,2)*PW3(2)-p3(2)*R(3,3)*PW3(3);...
%     %       -371;
%     %       -105;
% ] 

% test = T1A*[R TEpnp]*[336, 0, 256, 1]'
% test = test/test(3)

x =double(A\-b)
x1 = double(linsolve(A,-b))
s1 = lsqminnorm(A,double(b),1e-2)

x1 = [336 105 371 188.185080790200 82.0738220787612 531.480639133733]';
x2 = [255.42079606010540074638972736025
 -0.76020517111166552412462608895346
   255.16739433640148391046923419923
   88.518912017786547333509217070973
  -24.120459593286381936433535979526
   348.77743499732129365919116815905];
x3 = [80.614476055477804310307226434622
 -230.19350017718557685954577668373
  3.8833093297492771906289411940027
  -127.6941471861899658335707715517
 -254.49542828213439670150559649348
 -47.573896445009219040488759392667];
x4 = [394.31570933310309556676822158337
  208.28698780402503711004702382644
 -581.34798993505816778401877699229
 -278.94238204975865670185861490753
   28.38972312127402183143898485102
  209.75671665065848642595577107436];
x5 = [48.320690623962678141665601796814
  490.46567849030188110338877374357
  237.86265092605963363601692351691
 -219.41311746649558655990788801342
  -282.4122181537062877442071137294
 -172.18645358631148880423610690976];
x6 = [ -121.027301828876965800067930575
  92.226190906172471798222593518812
 -161.48745895155289471654190467846
  289.92171329953276948535612241356
 -290.86158826854723233769266033905
  113.27873004650407937883085739537];
x7 = [-210.40218870910489416260216453062
 -1.3876867155805102748626106263665
  7.7447948481917708946289581218119
 -172.34163708099676484566453815641
 -14.528961970496725301204960898368
   191.1473127707136896564235684312];
x8 = [ 0.00017494687455979562948666070345406
   -0.0001617904790951058890481565348653
 -0.000015915484505974492464028573710292
    0.0017562713747497904420767617245266
    0.0038411170679574038061364371125669
     -0.00316408277242678961703105530302];
x =  -[-255.99999999999757053386203771831
 0.0000000000031723422829846712234692014682715
            -255.99999999999649047240790479727
            -89.235313208222730078261331737271
             23.357134276938625953172522983621
            -350.09070703889754395036985247908];

re = A*x + b

M = [A b];
[U, S, V] =  svd(M);
v1 = V(:,7)/V(7,7)
v2 = V(:,6)/V(7,6)
v3 = V(:,5)/V(7,5)
v4 = V(:,4)/V(7,4)
v5 = V(:,3)/V(7,3)
v6 = V(:,2)/V(7,2)
v7 = V(:,1)/V(7,1)


% eqns = [
%         f*t1-p0(1)*Z1 + (f*R(1,1)*PW0(1)+f*R(1,2)*PW0(2)+f*R(1,3)*PW0(3)) >= 0,...
%         f*t1-p0(1)*Z1 + (f*R(1,1)*PW0(1)+f*R(1,2)*PW0(2)+f*R(1,3)*PW0(3)) <= 0.5,...
%         f*t2-p0(2)*Z1  + (f*R(2,1)*PW0(1)+f*R(2,2)*PW0(2)+f*R(2,3)*PW0(3)) >= 0,...
%         f*t2-p0(2)*Z1  + (f*R(2,1)*PW0(1)+f*R(2,2)*PW0(2)+f*R(2,3)*PW0(3)) <= 0.5,...
%         t3-Z1 + (R(3,1)*PW0(1)+R(3,2)*PW0(2)+R(3,3)*PW0(3))>= 0,...
%         t3-Z1 + (R(3,1)*PW0(1)+R(3,2)*PW0(2)+R(3,3)*PW0(3))<= 0.1,...
%         f*R(1,3)*Z+f*t1-p1(1)*Z2 + (f*R(1,1)*PW1(1)+f*R(1,2)*PW1(2))>= 0,...
%         f*R(1,3)*Z+f*t1-p1(1)*Z2 + (f*R(1,1)*PW1(1)+f*R(1,2)*PW1(2))<= 0.5,...
%         f*R(2,3)*Z+f*t2-p1(2)*Z2 + (f*R(2,1)*PW1(1)+f*R(2,2)*PW1(2))>= 0,...
%         f*R(2,3)*Z+f*t2-p1(2)*Z2 + (f*R(2,1)*PW1(1)+f*R(2,2)*PW1(2))<= 0.5,...
%         R(3,3)*Z+t3-Z2 + (R(3,1)*PW1(1)+R(3,2)*PW1(2))>= 0,...
%         R(3,3)*Z+t3-Z2 + (R(3,1)*PW1(1)+R(3,2)*PW1(2))<= 0.1,...
%         f*R(1,2)*Y+f*t1-p2(1)*Z3+f*R(1,1)*PW2(1)+f*R(1,3)*PW2(3)>=0,...
%         f*R(1,2)*Y+f*t1-p2(1)*Z3+f*R(1,1)*PW2(1)+f*R(1,3)*PW2(3)<=0.5,...
%         f*R(2,2)*Y+f*t2-p2(2)*Z3+f*R(2,1)*PW2(1)+f*R(2,3)*PW2(3)>=0,...
%         f*R(2,2)*Y+f*t2-p2(2)*Z3+f*R(2,1)*PW2(1)+f*R(2,3)*PW2(3)<=0.5,...
%         R(3,2)*Y+t3-Z3+R(3,1)*PW2(1)+R(3,3)*PW2(3)>=0,...
%         R(3,2)*Y+t3-Z3+R(3,1)*PW2(1)+R(3,3)*PW2(3)<=0.1,...
%         f*R(1,1)*X+f*t1-p3(1)*Z4+f*R(1,2)*PW3(2)+f*R(1,3)*PW3(3)>=0,...
%         f*R(1,1)*X+f*t1-p3(1)*Z4+f*R(1,2)*PW3(2)+f*R(1,3)*PW3(3)<=0.5,...
%         f*R(2,1)*X+f*t2-p3(2)*Z4+f*R(2,2)*PW3(2)+f*R(2,3)*PW3(3)>=0,...
%         f*R(2,1)*X+f*t2-p3(2)*Z4+f*R(2,2)*PW3(2)+f*R(2,3)*PW3(3)<=0.5,...
%         R(3,1)*X+t3-Z4+R(3,2)*PW3(2)+R(3,3)*PW3(3)>=0,...
%         R(3,1)*X+t3-Z4+R(3,2)*PW3(2)+R(3,3)*PW3(3)<=0.1,...
%         sqrt((X-256).^2+(Z-256).^2)+sqrt((X-256).^2+Y.^2) > sqrt((Z-256).^2+Y.^2)
%        ]
% 
% 
% s = solve(eqns,[X Y Z t1 t2 t3 Z1 Z2 Z3 Z4],'ReturnConditions',true);


% M = [A b];
% [U, S, V] =  svd(M);
% 
% v1 = V(:,11)/V(11,11);
% v2 = V(:,10)/V(11,10);
% v3 = V(:,9)/V(11,9);
% v4 = V(:,8)/V(11,8);




% A = [
%      0 f 0 0 -p0(1) 0;...
%      0 0 f 0 -p0(2) 0;...
%      0 0 0 1 -1 0;...
%      f*R(1,3) f 0 0 0 -p1(1);...
%      f*R(2,3) 0 f 0 0 -p1(2);...
%      R(3,3) 0 0 1 0 -1;...
%     ]
% 
% b = [
%       f*R(1,1)*PW0(1)+f*R(1,2)*PW0(2)+f*R(1,3)*PW0(3);...
%       f*R(2,1)*PW0(1)+f*R(2,2)*PW0(2)+f*R(2,3)*PW0(3);...
%       R(3,1)*PW0(1)+R(3,2)*PW0(2)+R(3,3)*PW0(3);...
%       f*R(1,1)*PW1(1)+f*R(1,2)*PW1(2);...
%       f*R(2,1)*PW1(1)+f*R(2,2)*PW1(2);...
%       R(3,1)*PW1(1)+R(3,2)*PW1(2);...
%     ]




X = 371;
t1 = 188.2674;
t2 = 82.0641;
t3 = 531.4382;
Z1 = 181.7362;
Z2 = 122.5179;

ss1 = f*t1-p0(1)*Z1 + (f*R(1,1)*PW0(1)+f*R(1,2)*PW0(2)+f*R(1,3)*PW0(3));
ss2 = f*t2-p0(2)*Z1  + (f*R(2,1)*PW0(1)+f*R(2,2)*PW0(2)+f*R(2,3)*PW0(3));
ss3 = t3-Z1 + (R(3,1)*PW0(1)+R(3,2)*PW0(2)+R(3,3)*PW0(3));
ss4 = f*R(1,3)*X+f*t1-p1(1)*Z2 + (f*R(1,1)*PW1(1)+f*R(1,2)*PW1(2));
ss5 = f*R(2,3)*X+f*t2-p1(2)*Z2 + (f*R(2,1)*PW1(1)+f*R(2,2)*PW1(2));
ss6 =  R(3,3)*X+t3-Z2 + (R(3,1)*PW1(1)+R(3,2)*PW1(2)); 

A = [
     f 0 0 -p0(1) 0;...
     0 f 0 -p0(2) 0;...
     0 0 1 -1 0;...
     f 0 0 0 -p1(1);...
     0 f 0 0 -p1(2);...
     0 0 1 0 -1;...
    ]

b = [
      f*R(1,1)*PW0(1)+f*R(1,2)*PW0(2)+f*R(1,3)*PW0(3);...
      f*R(2,1)*PW0(1)+f*R(2,2)*PW0(2)+f*R(2,3)*PW0(3);...
      R(3,1)*PW0(1)+R(3,2)*PW0(2)+R(3,3)*PW0(3);...
      f*R(1,1)*PW1(1)+f*R(1,2)*PW1(2)+f*R(1,3)*PW1(3);...
      f*R(2,1)*PW1(1)+f*R(2,2)*PW1(2)+f*R(2,3)*PW1(3);...
      R(3,1)*PW1(1)+R(3,2)*PW1(2)+R(3,3)*PW1(3);...
    ]


% A = [
%      0 f 0 0 -p0(1) 0;...
%      0 0 f 0 -p0(2) 0;...
%      0 0 0 1 -1 0;...
%      f*R(1,1) f 0 0 0 -p3(1);...
%      f*R(2,1) 0 f 0 0 -p3(2);...
%      R(3,1) 0 0 1 0 -1;...
%     ]
% 
% b = [
%       f*R(1,1)*PW0(1)+f*R(1,2)*PW0(2)+f*R(1,3)*PW0(3);...
%       f*R(2,1)*PW0(1)+f*R(2,2)*PW0(2)+f*R(2,3)*PW0(3);...
%       R(3,1)*PW0(1)+R(3,2)*PW0(2)+R(3,3)*PW0(3);...
%       f*R(1,2)*PW3(2)+f*R(1,3)*PW3(3);...
%       f*R(2,2)*PW3(2)+f*R(2,3)*PW3(3);...
%       R(3,2)*PW3(2)+R(3,3)*PW3(3);...
%     ]

M = [A b];
[U, S, V] =  svd(M);
v1 = V(:,6)/V(6,6);
v2 = V(:,5)/V(6,5);
v3 = V(:,4)/V(6,4);






% A = [
%      0 f 0 0 -p0(1) 0;...
%      0 0 f 0 -p0(2) 0;...
%      0 0 0 1 -1 0;...
%      f*R(1,2) f 0 0 0 -p2(1);...
%      f*R(2,2) 0 f 0 0 -p2(2);...
%      R(3,2) 0 0 1 0 -1;...
%     ]
% 
% b = -[
%       f*R(1,1)*PW0(1)+f*R(1,2)*PW0(2)+f*R(1,3)*PW0(3);...
%       f*R(2,1)*PW0(1)+f*R(2,2)*PW0(2)+f*R(2,3)*PW0(3);...
%       R(3,1)*PW0(1)+R(3,2)*PW0(2)+R(3,3)*PW0(3);...
%       f*R(1,1)*PW2(1)+f*R(1,3)*PW2(3);...
%       f*R(2,1)*PW2(1)+f*R(2,3)*PW2(3);...
%       R(3,1)*PW2(1)+R(3,3)*PW2(3);...
%     ]


x =double(A\b)
x1 = double(linsolve(A,b))
s1 = lsqminnorm(A,double(b),1e-2)


temp0 = s1(4)*(T1A\p0.');
Ttemp0 = temp0 - R*PW0';





temp2 = s1(8)*(T1A\p1.');
PW1 = [256, 0, s1(3)];
Ttemp = temp2 - R*PW1';

temp3 = s1(9)*(T1A\p2.');
PW2 = [256, s1(2), 256];
Ttemp1 = temp3 - R*PW2';


temp4 = s1(10)*(T1A\p3.');
PW3 = [s1(1), 0, 256];
Ttemp2 = temp4 - R*PW3';











