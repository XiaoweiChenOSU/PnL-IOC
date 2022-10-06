close all;
clc;
clear;

addpath LPnL-Bar-ENull;
addpath others;

% p1 = [507 507;508 470;119 468;491 119;515 514;521 111;104 104;72 562;507 507];
% p2 = [321 510;510 343;115 267;344 117;563 563;574 59;31 32;13 621;31 32];
% 
% P1_w = [256 0 256;256 25 256;256 30 512;256 256 271;261 0 256;261 256 256;261 256 512;281 0 512;256 0 256];
% P2_w = [256 0 381;256 110 256;256 160 512;256 256 366;286 0 256;286 256 256;291 256 512;301 0 512;291 256 512];

p1 = [507 507;508 470;119 468;491 119;515 514;521 111;104 104;72 562];
p2 = [321 510;510 343;115 267;344 117;563 563;574 59;31 32;13 621];

P1_w = [256 0 256;256 25 256;256 30 512;256 256 271;261 0 256;261 256 256;261 256 512;281 0 512];
P2_w = [256 0 381;256 110 256;256 160 512;256 256 366;286 0 256;286 256 256;291 256 512;301 0 512];

% p1 = [320 233;320 233;320 233];
% p2 = [-704 832;1343 831;320 -2664];
% 
% P1_w = [256 0 256;256 0 256;256 0 256];
% P2_w = [256 0 384;324 0 256;256 10 256];

A = [197.4 0 320;0 197.4 320;0   0   1];

for i = 1:length(p1)
    temp1 = inv(A)*[p1(i,:) 1].'
    temp1 = (temp1/temp1(3)).'
    ip1(i,:) = temp1(:,1:2)
    temp2 = inv(A)*[p2(i,:) 1].'
    temp2 = (temp2/temp2(3)).'
    ip2(i,:) = temp2(:,1:2)
end


[R, T, err] = LPnL_Bar_ENull(ip1', ip2', P1_w', P2_w');
errorEpln=reprojection_error_usingRT(P1_w,p1,R,T,A);
fprintf('error EPnL: %.3f\n',errorEpln);
[R1, T1] = ASPnL(ip1', ip2', P1_w', P2_w');
%compute error
errorAspnl=reprojection_error_usingRT(P1_w,p1,R1,T1,A);
fprintf('error Aspnl: %.3f\n',errorAspnl);

% oImagePath = [pwd,'/layout_one.png'];
oImagePath = [pwd,'/type_0.png'];

oImage = imread(oImagePath);
[Ix , Iy, Iz] = size(oImage);

if Iz > 1
    oImage = rgb2gray(oImage);
end
oImage = mat2gray(oImage);


figure(1), imshow(oImage);





load('3d_data.mat');
load('2d_data.mat');


npw = [model,ones(16,1)];

A = [197.4 0 320;0 197.4 320;0   0   1];

L = [];
for i = 1:16
    P1 = npw(i,:)';
    p1=(A*[R,T]*P1)';
    u1=round(p1(:,1)./p1(:,3));
    v1=round(p1(:,2)./p1(:,3));
    for j2 = u1-10:u1+10
        for k2 = v1-10:v1+10
            oImage(k2,j2)=0;
        end
    end
    L = [L;u1 v1];
end


figure(1), imshow(oImage);

line([L(1,1) L(9,1)],[L(1,2) L(9,2)],'Color','red','LineWidth',2);  
line([L(2,1) L(10,1)],[L(2,2) L(10,2)],'Color','red','LineWidth',2); 
line([L(3,1) L(11,1)],[L(3,2) L(11,2)],'Color','red','LineWidth',2); 
line([L(4,1) L(12,1)],[L(4,2) L(12,2)],'Color','red','LineWidth',2); 
line([L(5,1) L(13,1)],[L(5,2) L(13,2)],'Color','red','LineWidth',2); 
line([L(6,1) L(14,1)],[L(6,2) L(14,2)],'Color','red','LineWidth',2); 
line([L(7,1) L(15,1)],[L(7,2) L(15,2)],'Color','red','LineWidth',2); 
line([L(8,1) L(16,1)],[L(8,2) L(16,2)],'Color','red','LineWidth',2); 
