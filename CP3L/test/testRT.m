clear all; close all;

addpath 3Destimation;


% load('3d_data.mat');
% load('2d_data.mat')


% 
% %############## Camera Intrinsic Matrix ##################
% A = [[ 512    0  320]
%  [   0  512 320]
%  [   0    0    1]];
% 
% 
% %############## Camera Rotation Matrix ##################
% R = [0.706022365110961,0.00208931738729643,-0.708186454767373;-0.277261054637476,-0.919355709937976,-0.279126111615798;-0.651658443929364,0.393421800815988,-0.648506406366402];
% 
% 
% 
% 
% %############## Camera Translation Matrix ##################
% T =[8.71349934 331.11140587 444.01048618];
% 
% 
% testp = A*[R T']*[256, 120, 256,1]';
% testp = testp/testp(3);
% 
% 
% 
% I1 = rgb2gray(imread([pwd,'/roomType/3.1.png']));
% points1 = detectHarrisFeatures(I1);



oImagePath = [pwd,'/roomType/type_01.png'];

oImage = imread(oImagePath);
[Ix , Iy, Iz] = size(oImage);

if Iz > 1
    oImage = rgb2gray(oImage);
end
oImage = mat2gray(oImage);









% R = [0.691791089481684,0.0105610713676920,-0.722020465281498;-0.269880973920502,-0.923650477233471,-0.272091998820441;-0.669768130347709,0.383090406663946,-0.636122937719264];
% T = [3.58733792203451;339.006099989604;445.999564598126];
% 
% % R = [0.70710678, 0.0000010,  -0.70710678;-0.27628863, -0.92050485, -0.27628863;-0.65089522, 0.39073113, -0.65089522];
% % T = [2.12132034;341.15198673;442.84175724];
% % 
% T1P1_w = [256 256 326;256 115 256;271 256 256];
% T1P2_w = [256 256 371;256 160 256;371 256 256];
% 
% 
% 
% 
% % 
% T1A = [512 0 320;0 512 320;0 0 1];
% % 
% P1 = ([T1P1_w(1,:),ones(1,1)])';
% TD = (T1A*[R,T]*P1)';
% TD = TD/TD(:,3);
% 
% 
% T1p1 = [159 148;313 630;341 218];
% T1p2 = [3 63;314 475;620 63];
% T1p1 = [158.896262050777,147.415138189270;312.971185097641,630.923292305434;341.997432758494,230.360204571684];
% T1p2 = [2.98289673594146,62.6648323909242;313.689388506855,475.924422880694;626.226339392075,79.4644952342103];


% T1p1 = [337 355;337 355;337 355];
% T1p2 = [19, 472;339, 327;587, 541]
% T1p1 = [418.123968049086,147.987639835310;610.090262306951,542.548991571067;463.400849977882,208.262135006551];
% T1p2 = [400.832621154976,51.9334723654826;545.944199995071,428.190113547798;692.453627382171,85.8714694306361];
% T1p1 = [316 385;316 366;340 400];
% T1p2 = [47 551;313 17;632 584];

%Calculate the camera parameters

% the first one
% A1 =  intrisicPget(observe,model);
% [Rp1,Tp1,Xc1,sol1]=efficient_pnp(model,observe,A1);

% Pw = [268.250 116.31 367.63;
%     267.25 117.31 367.63;
%     267.25 117.31 368.63;
%     267.25 116.31 367.63];
% % npw = [model,ones(16,1)];
% 
% npw = [Pw,ones(4,1)];
% L = [];
% for i = 1:4
%     P1 = npw(i,:)';
% %     p1=(A1*[Rp1,Tp1]*P1)';
%     p1=([Rp1,Tp1]*P1)';
% %     u1=round(p1(:,1)./p1(:,3));
% %     v1=round(p1(:,2)./p1(:,3));
% %     for j2 = u1-5:u1+5
% %         for k2 = v1-5:v1+5
% %             oImage(k2,j2)=0;
% %         end
% %     end
% %     L = [L;u1 v1];
%     L = [L;p1];
% end


% T1p2 = [1 470;278 2;635 490];
% 
% u1=635;
% v1=490;
% for j2 = u1:u1+5
%     for k2 = v1:v1+5
%         oImage(k2,j2)=0;
%     end
% end
% 
% 

% 
% T1p1 = [306 228;313 630;341 218];
% T1p2 = [212 177;315 233;600 74];

% T1p1 = [377.160272167267,193.182386305628;434.294483920141,617.427286764249;323.671428042403,184.841125502906];
% T1p2 = [458.024015909215,215.095257827910;262.992118341717,464.978766389276;599.427681322131,11.2326067979413];

% T1p1 = [579 621;555 19];
% T1p2 = [414 621;402 19];
% 
% T1p1 = [316 385;316 366;340 400];
% T1p2 = [47 551;313 17;632 584];
% % T1p1 = [626 621;602 19];
% % T1p2 = [108 621;402 19];
% % 
% % T1p1 = [314 183;318 385;318 403;326 408;324 181];
% % T1p2 = [277 161;318 282;87 544;589 573;384 145];
% % T1p1 = [316 385;316 366;340 400];
% % T1p2 = [164 478;313 17;596 561];
% 
% % T1p1 = [306 228;313 630;341 218];
% % T1p2 = [212 177;315 233;600 74];
% 
% % T1p1 = [522 367;523 327;102 320;531 370;94 372];
% % T1p2 = [231 369;525 189;98 98;589 384;14 392];
% % T1p1 = [516 239;518 604;108 607;554 228;81 229];
% % T1p2 = [444 238;520 472;104 396;609 207;8 202];


% T1p1 = [626 621;626 19;626 621;24 621];
% T1p2 = [24 621;24 19;626 19;24 19];
% 
% 
% T1p1 = [337 355;337 327;383 389];
% T1p2 = [19 472;340 1;587 541];
% T1p1 = [337 355;337 327;383 389];
% T1p2 = [19 472;340 1;587 541];

u1=round(75.93167688320756);
v1=round(160.561531381425);
for j2 = 11-10:10+10
    for k2 = v1-10:v1+10
        oImage(k2,j2)=0;
    end
end
figure(1), imshow(oImage);

% T1p1 = [75.93167688320756, 1.1230627628499974;75.93167688320756, 1.1230627628499974;520.975224359587, 318.23706199837716;520.975224359587, 318.23706199837716;75.93167688320756, 1.1230627628499974;520.975224359587, 0.9082217062624522];
% T1p2 = [520.975224359587, 0.9082217062624522;75.93167688320756, 318.3391826185565;520.975224359587, 0.9082217062624522;75.93167688320756, 318.3391826185565;520.975224359587, 318.23706199837716;75.93167688320756, 318.3391826185565];

% T1p1 = [75.93167688320756, 1.1230627628499974;520.975224359587, 318.23706199837716];
% T1p2 = [75.93167688320756, 318.3391826185565;520.975224359587, 0.9082217062624522];
% 
% 
% T1p2 = round(T1p2);
% T1p1 = round(T1p1);


% pointS(1,1:5) = [256, 0, 306, 36.68728457194504, 630.6464130669883];
% pointS(2,1:5) = [256, 256, 306, 37.98274419214039, 30.553609359902225];
% pointS(3,1:5) = [512, 256, 306, 637.9327506436568, 7.280697470726068];
% pointS(4,1:5) = [512, 0, 281, 627.8651165545643, 600.084505502598];
% pointS(5,1:5) = [256, 256, 306, 37.98274419214039, 30.553609359902225];
% pointS(6,1:5) = [256, 0, 306, 36.68728457194504, 630.6464130669883];
% 
% 
% pointE(1,1:5) = [256, 256, 306, 37.98274419214039, 30.553609359902225];
% pointE(2,1:5) = [512, 256, 306, 637.9327506436568, 7.280697470726068];
% pointE(3,1:5) = [512, 0, 281, 627.8651165545643, 600.084505502598];
% pointE(4,1:5) = [256, 0, 306, 36.68728457194504, 630.6464130669883];
% pointE(5,1:5) = [512, 0, 281, 627.8651165545643, 600.084505502598];
% pointE(6,1:5) = [512, 256, 306, 637.9327506436568, 7.280697470726068];


% pointS(1,1:5) = [256, 0, 256, 93.70619386809568, 575.2244843975153];
% pointS(2,1:5) = [256, 256, 256, 92.86556007024905, 86.79395577579675];
% pointS(3,1:5) = [256, 0, 256, 93.70619386809568, 575.2244843975153];
% pointS(4,1:5) = [512, 0, 256, 599.4744419461035, 574.4126910202073];
% pointS(5,1:5) = [256, 0, 256, 93.70619386809568, 575.2244843975153];
% pointS(6,1:5) = [256, 256, 256, 92.86556007024905, 86.79395577579675];
% pointS(7,1:5) = [512, 256, 256, 581.2017659041953, 69.7891653683385];
% pointS(8,1:5) = [512, 0, 256, 599.4744419461035, 574.4126910202073];
% pointS(9,1:5) = [256, 0, pointS(5,3)+10, 36.68728457194504, 630.6464130669883];
% pointS(10,1:5) = [256, 256, pointS(6,3)+10, 37.98274419214039, 30.553609359902225];
% pointS(11,1:5) = [512, 256, pointS(7,3)+10, 637.9327506436568, 7.280697470726068];
% pointS(12,1:5) = [512, 0, pointS(8,3)+10, 627.8651165545643, 600.084505502598];
% 
% 
% pointE(1,1:5) = [256, 256, 256, 92.86556007024905, 86.79395577579675];
% pointE(2,1:5) = [512, 256, 256, 581.2017659041953, 69.7891653683385];
% pointE(3,1:5) = [512, 0, 256, 599.4744419461035, 574.4126910202073];
% pointE(4,1:5) = [512, 256, 256, 581.2017659041953, 69.7891653683385];
% pointE(5,1:5) = [256, 0, pointS(5,3)+10, 36.68728457194504, 630.6464130669883];
% pointE(6,1:5) = [256, 256, pointS(6,3)+10, 37.98274419214039, 30.553609359902225];
% pointE(7,1:5) = [512, 256, pointS(7,3)+10, 637.9327506436568, 7.280697470726068];
% pointE(8,1:5) = [512, 0, pointS(8,3)+10, 627.8651165545643, 600.084505502598];
% pointE(9,1:5) = [256, 256, pointS(6,3)+10, 37.98274419214039, 30.553609359902225];
% pointE(10,1:5) = [512, 256, pointS(7,3)+10, 637.9327506436568, 7.280697470726068];
% pointE(11,1:5) = [512, 0, pointS(8,3)+10, 627.8651165545643, 600.084505502598];
% pointE(12,1:5) = [256, 0, pointS(5,3)+10, 36.68728457194504, 630.6464130669883];
% 
% 
% 
% pointS(1,1:5) =  [0, 250, 0, 307.62973658891036, 239.93665957497137];
% pointS(2,1:5) = [0, 256, 0, 307.6591100935348, 233.97973007879574];
% pointS(3,1:5) = [25, 256, 0, 330.6555353372111, 218.53214425111884];
% 
% pointE(1,1:5) = [0, 25, 0, 305.86144048674487, 598.5460711398123];
% pointE(2,1:5) = [0, 256, 125, 57.20467738473769, 45.13290436456816];
% pointE(3,1:5) = [150, 256, 0, 626.9372812064411, 19.50826771415757];

% T1p1 = [2.960833119305801e+02,3.599202469977733e+02;2.960833119305801e+02,3.599202469977733e+02;2.960833119305801e+02,3.599202469977733e+02]

% pointS(1,1:5) = [0, 0, 0, 179.16413834071315, 387.0981022112845];
% pointS(2,1:5) = [0, 256, 0, 216.3971427866365, 113.18397391816666];
% pointS(3,1:5) = [256, 256, 0, 520.0667400636626, 177.48500863695858];
% pointS(4,1:5) = [256, 0, 0, 417.57184587195013, 453.2721780974126];
% 
% pointS(5,1:5) = [0, 0, 0, 179.16413834071315, 387.0981022112845];
% pointS(6,1:5) = [0, 256, 0, 216.3971427866365, 113.18397391816666];
% pointS(7,1:5) = [256, 256, 0, 520.0667400636626, 177.48500863695858];
% pointS(8,1:5) = [256, 0, 0, 417.57184587195013, 453.2721780974126];
% 
% 
% pointE(1,1:5) = [0, 256, 0, 216.3971427866365, 113.18397391816666];
% pointE(2,1:5) = [256, 256, 0, 520.0667400636626, 177.48500863695858];
% pointE(3,1:5) = [256, 0, 0, 417.57184587195013, 453.2721780974126];
% pointE(4,1:5) = [0, 0, 0, 179.16413834071315, 387.0981022112845];

% pointE(5,1:5) = [0, 0, pointS(5,3)+100, 13.921739362206893, 486.5315479655238];
% pointE(6,1:5) = [0, 256, pointS(6,3)+100, 160.25928833286872, 25.575232001077097];
% pointE(7,1:5) = [256, 256, pointS(7,3)+100, 621.2271544395212, 121.08980650181724];
% pointE(8,1:5) = [256, 0, pointS(8,3)+100, 475.8143239803294, 562.1024683876212];
% 
% pointE(5,1:5) = [0, 0, 100, 13.921739362206893, 486.5315479655238];
% pointE(6,1:5) = [0, 256, 50, 160.25928833286872, 25.575232001077097];
% pointE(7,1:5) = [256, 256, 50, 621.2271544395212, 121.08980650181724];
% pointE(8,1:5) = [256, 0, 75, 475.8143239803294, 562.1024683876212];
% 


pointS(1,1:5) = [0,256,0,339.207688624413,344.874419381720];
pointS(2,1:5) = [0,256,0,339.207688624413,344.8744193817204];
pointS(3,1:5) = [0,256,0,339.207688624413,344.874419381720];
pointE(1,1:5) = [231,256,0,639.634146552915,151.654921142347];
pointE(2,1:5) = [0,71,0,366.780511404217,638.973629764988];
pointE(3,1:5) = [0,256,255,26.7596395153649,167.137595389534];




T1p1 = pointS(1:3,4:5);

T1p2 = pointE(1:3,4:5);
% T1p2 = [5.782601299506542e+02,1.826739659136602e+02;2.830394688359937e+02,6.381490790235014e+02;2.032220754080868,2.241609576377190e+02]

line([T1p1(1,1) T1p2(1,1)],[T1p1(1,2) T1p2(1,2)],'Color','red','LineWidth',2);  
line([T1p1(2,1) T1p2(2,1)],[T1p1(2,2) T1p2(2,2)],'Color','red','LineWidth',2);
line([T1p1(3,1) T1p2(3,1)],[T1p1(3,2) T1p2(3,2)],'Color','red','LineWidth',2);
% line([T1p1(4,1) T1p2(4,1)],[T1p1(4,2) T1p2(4,2)],'Color','red','LineWidth',2);
% line([T1p1(5,1) T1p2(5,1)],[T1p1(5,2) T1p2(5,2)],'Color','blue','LineWidth',2);
% line([T1p1(6,1) T1p2(6,1)],[T1p1(6,2) T1p2(6,2)],'Color','red','LineWidth',2);
% line([T1p1(7,1) T1p2(7,1)],[T1p1(7,2) T1p2(7,2)],'Color','blue','LineWidth',2);
% line([T1p1(8,1) T1p2(8,1)],[T1p1(8,2) T1p2(8,2)],'Color','red','LineWidth',2);
% line([T1p1(9,1) T1p2(9,1)],[T1p1(9,2) T1p2(9,2)],'Color','red','LineWidth',2);
% line([T1p1(10,1) T1p2(10,1)],[T1p1(10,2) T1p2(10,2)],'Color','red','LineWidth',2);
% line([T1p1(11,1) T1p2(11,1)],[T1p1(11,2) T1p2(11,2)],'Color','red','LineWidth',2);
% line([T1p1(12,1) T1p2(12,1)],[T1p1(12,2) T1p2(12,2)],'Color','red','LineWidth',2);

% % % 
% 
% 
% 
% 
% % p1 = [507 507;119 468;491 119;515 514;521 111];
% % p2 = [321 510;115 267;344 117;563 563;574 59];
% % 
% % 
% % 
% % line([p1(1,1) p2(1,1)],[p1(1,2) p2(1,2)],'Color','red','LineWidth',2);  
% % line([p1(2,1) p2(2,1)],[p1(2,2) p2(2,2)],'Color','red','LineWidth',2);
% % line([p1(3,1) p2(3,1)],[p1(3,2) p2(3,2)],'Color','red','LineWidth',2);
% % line([p1(4,1) p2(4,1)],[p1(4,2) p2(4,2)],'Color','red','LineWidth',2);
% % line([p1(5,1) p2(5,1)],[p1(5,2) p2(5,2)],'Color','red','LineWidth',2);
% % line([p1(6,1) p2(6,1)],[p1(6,2) p2(6,2)],'Color','red','LineWidth',2);
% % line([p1(7,1) p2(7,1)],[p1(7,2) p2(7,2)],'Color','red','LineWidth',2);
% 
% % line([L(2,1) L(10,1)],[L(2,2) L(10,2)],'Color','red','LineWidth',2); 
% % line([L(3,1) L(11,1)],[L(3,2) L(11,2)],'Color','red','LineWidth',2); 
% % line([L(4,1) L(12,1)],[L(4,2) L(12,2)],'Color','red','LineWidth',2); 
% % line([L(5,1) L(13,1)],[L(5,2) L(13,2)],'Color','red','LineWidth',2); 
% % line([L(6,1) L(14,1)],[L(6,2) L(14,2)],'Color','red','LineWidth',2); 
% % line([L(7,1) L(15,1)],[L(7,2) L(15,2)],'Color','red','LineWidth',2); 
% % line([L(8,1) L(16,1)],[L(8,2) L(16,2)],'Color','red','LineWidth',2); 
%    
% 

    %Hessian matrix 
    Hes=[ 0,        0,     0,    0,     0,     0;
          0,        0,     0,    0,     0,     0;
          0,        0,     0,    0,     0,     0;
          0,        0,     0,    0,     0,     0;
          2,        0,     0,    0,     0,     0;
          0,        0,     0,    0,     0,     0;
          0,        0,     0,    0,     0,     0;
          0,        2,     0,    0,     0,     0;
          0,        1,     0,    0,     0,     0;
          0,        0,     2,    0,     0,     0;     
          0,        0,     0,    0,     0,     0;
          0,        0,     0,    0,     0,     0;
          0,        0,     0,    0,     0,     0;
          2*s4,     0,     0,    0,   0,     0;
          0,        0,     0,    0,   0,     0;
          0,    0,     0,   0,   0,     0;
          0,    2*s4,     0,   0,   0,     0;
          0,    0,     2*s4,   0,   0,     0;   
          0,        0,     0,    0,     0,     0;
          0,       0,     0,    0,     0,     0;
          0,        0,    0,    0,     0,     0;      
          2*s5,        0,    0,   0,     0,     0;
          0,  0,   0,   0,   0,     0;
          0,    2*s5,     0,   0,   0,     0;
          0,      0, 0,   0,  0,     0;
          0,        0, 2*s5,   0,   0,     0; 
          0,        0,     0,    0,     0,     0;
          0,       0,     0,    0,     0,     0;
          0,       0,     0,    0,     0,     0;
          2*s6,        0,    0,   0,     0,     0;
          0,    0,     0,   0,   0,     0;
          0,    2*s6,     0,   0,   0,     0;
          0,    0,   0,   0,   0,     0;
          0,        0,  2*s6,  0,   0,     0;
          ];   
      
      
        nLine = length(pn1);
        Q=zeros(2*nLine,10);
        N=zeros(2*nLine,3);
        n_c=xnorm(cross(pn1,pn2));
        nx=n_c(1,:)'; ny=n_c(2,:)'; nz=n_c(3,:)';
        Px=Pn1_w(1,:)'; Py=Pn1_w(2,:)'; Pz=Pn1_w(3,:)';
        Q(1:2:end,:)=[Px.*nx+Py.*ny+Pz.*nz,...
                  2*Py.*nz-2*Pz.*ny,...
                  2*Pz.*nx-2*Px.*nz,...
                  2*Px.*ny-2*Py.*nx,...
                  Px.*nx-Py.*ny-Pz.*nz,...
                  2.*Px.*ny+2*Py.*nx,...
                  2.*Px.*nz+2*Pz.*nx,...
                  Py.*ny-Px.*nx-Pz.*nz,...
                  2*Py.*nz+2*Pz.*ny,...
                  Pz.*nz-Px.*nx-Py.*ny];
        Px=Pn2_w(1,:)'; Py=Pn2_w(2,:)'; Pz=Pn2_w(3,:)';
        Q(2:2:end,:)=[Px.*nx+Py.*ny+Pz.*nz,...
                  2*Py.*nz-2*Pz.*ny,...
                  2*Pz.*nx-2*Px.*nz,...
                  2*Px.*ny-2*Py.*nx,...
                  Px.*nx-Py.*ny-Pz.*nz,...
                  2.*Px.*ny+2*Py.*nx,...
                  2.*Px.*nz+2*Pz.*nx,...
                  Py.*ny-Px.*nx-Pz.*nz,...
                  2*Py.*nz+2*Pz.*ny,...
                  Pz.*nz-Px.*nx-Py.*ny];
        N(1:2:end,:)=-n_c.';
        N(2:2:end,:)=-n_c.';


        if abs(det(N.'*N)) < 2e-10
            CC=(pinv(N.'*N)*N.')*Q;
        else
            CC=((N.'*N)\N.')*Q;
        end

        EE=Q-N*CC;
        GG=EE.'*EE;

        [solution,curErr] = RefineGaussNewton(Cayley(Rr),EE,GG);
        s1=solution(1);
        s2=solution(2);
        s3=solution(3);
        sr=[1,s1,s2,s3,s1^2,s1*s2,s1*s3,s2^2,s2*s3,s3^2].';
        factor=1/(1+s1^2+s2^2+s3^2); 
        Rr=factor*[1+s1^2-s2^2-s3^2,2*s1*s2-2*s3,2*s2+2*s1*s3;
                   2*s3+2*s1*s2,1-s1^2+s2^2-s3^2,2*s2*s3-2*s1;
                   2*s1*s3-2*s2,2*s1+2*s2*s3,1-s1^2-s2^2+s3^2];              
        Rt=factor*CC*sr; 
