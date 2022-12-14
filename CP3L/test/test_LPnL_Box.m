close all;
clc;
clear;

addpath LPnL-Bar-ENull;
addpath others;
addpath ASPnL;
addpath RPnL;

% p1 = [507 507;508 470;119 468;491 119;515 514;521 111;104 104;72 562;507 507];
% p2 = [321 510;510 343;115 267;344 117;563 563;574 59;31 32;13 621;31 32];
% 
% P1_w = [256 0 256;256 25 256;256 30 512;256 256 271;261 0 256;261 256 256;261 256 512;281 0 512;256 0 256];
% P2_w = [256 0 381;256 110 256;256 160 512;256 256 366;286 0 256;286 256 256;291 256 512;301 0 512;291 256 512];


% p1 = [491 1111;491 1111;491 1111;114 551;114 551;864 821;864 821;1031 406;1031 406;519 719;14 137;435 16];
% p2 = [114 551;864 821;519 719;14 137;435 405;435 405;1031 406;519 719;435 16;14 137;435 16;435 405];
% 
% P1_w = [0 7 11;0 6 12;6 7 12;7 7 0;0 7 0;0 0 12;7 0 12;9 4 12;9 0 10;9 7 9;9 7 0;9 0 0];
% P2_w = [0 7 9;0 5 12;9 7 12;9 7 0;0 5 0;0 0 11;8 0 12;9 6 12;9 0 8;9 7 5;9 6 0;7 0 0];




% p1 = [320 233;320 233;320 233];
% p2 = [-704 832;1343 831;320 -2664];
% 
% P1_w = [256 0 256;256 0 256;256 0 256];
% P2_w = [256 0 384;324 0 256;256 10 256];

% Eight line correspondence
p1 = [491 1111;491 1111;491 1111;864 821;864 821;1031 406;1031 406;519 719];
p2 = [114 551;864 821;519 719;435 405;1031 406;519 719;435 16;14 137];

P1_w = [0 7 11;0 6 12;6 7 12;0 0 12;7 0 12;9 4 12;9 0 10;9 7 9];
P2_w = [0 7 9;0 5 12;9 7 12;0 0 11;8 0 12;9 6 12;9 0 8;9 7 5];

% 
% P1_w = [0 7 2;0 4 12;6 7 12;0 0 8;2 0 12;9 4 12;9 0 6;9 7 8];
% P2_w = [0 7 1;0 3 12;5 7 12;0 0 7;1 0 12;9 3 12;9 0 5;9 7 7];


% Five line correspondence
% p1 = [491 1111;491 1111;491 1111;864 821;864 821];
% p2 = [114 551;864 821;519 719;435 405;1031 406];
% 
% P1_w = [0 7 11;0 6 12;1 7 12;0 0 12;1 0 12];
% P2_w = [0 7 0;0 0 12;9 7 12;0 0 11;9 0 12];


% Four line correspondence
% p1 = [491 1111;491 1111;491 1111;864 821];
% p2 = [114 551;864 821;519 719;435 405];
% 
% P1_w = [0 7 11;0 6 12;1 7 12;0 0 12];
% P2_w = [0 7 0;0 0 12;9 7 12;0 0 5];





[R, T, err] = LPnL_Bar_ENull(p1', p2', P1_w', P2_w')
[R1, T1, err1] = LPnL_Bar_LS(p1', p2', P1_w', P2_w')
[R2, T2] = ASPnL(p1', p2', P1_w', P2_w')
[R3, T3] = RPnL(p1', p2', P1_w', P2_w')


% oImagePath = [pwd,'/layout_one.png'];
oImagePath = [pwd,'/background.jpg'];

oImage = imread(oImagePath);
[Ix , Iy, Iz] = size(oImage);

if Iz > 1
    oImage = rgb2gray(oImage);
end
oImage = mat2gray(oImage);


figure(1), imshow(oImage);





% load('3d_data.mat');
% load('2d_data.mat');

Pw = [P1_w;P2_w]
npw = [Pw,ones(16,1)];

A = [1331.83166916386,-30.7045221050101,536.514369701359;0,1286.75858658220,340.016877078984;0,0,1];

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
