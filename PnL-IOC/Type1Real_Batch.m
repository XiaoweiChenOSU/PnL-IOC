close all;
clc;
clear;

IniToolbox;

%Type1 Room Camera Pose Estimation

load('type1GroundTruth.mat');
[~,numImage] = size(groundTruth);

for j = 1:numImage
    
    R_truth = groundTruth(j).Rotation;
    T_truth = groundTruth(j).Translation;
    C_truth = -inv(R_truth)*T_truth;
    p = groundTruth(j).point;
    P = groundTruth(j).wpoint;

    A = groundTruth(j).intrinsics_matrix;

%     p1 = [p(1,:); p(1,:); p(1,:); p(2,:);p(2,:)];
%     p2 = [p(2,:);p(3,:);p(4,:);p(5,:);p(6,:)];

    
%     p1 = [roundn(p(1,:),-1); roundn(p(1,:),-1); roundn(p(1,:),-1); roundn(p(2,:),-1);roundn(p(2,:),-1)];
%     p2 = [roundn(p(2,:),-1);roundn(p(3,:),-1);roundn(p(4,:),-1);roundn(p(5,:),-1);roundn(p(6,:),-1)];

    p1 = [round(p(1,:)); round(p(1,:)); round(p(1,:)); round(p(2,:));round(p(2,:))];
    p2 = [round(p(2,:));round(p(3,:));round(p(4,:));round(p(5,:));round(p(6,:))];

    
% % 
%     I = imread('f0d1e88ba92440e1ae9a10336f1066a4_i2_5.jpg');
% 
%     imshow(I); hold on;
% 
%     line([p1(1,1),p2(1,1)],[p1(1,2),p2(1,2)],'Color','Red','LineWidth',3);
%     line([p1(2,1),p2(2,1)],[p1(2,2),p2(2,2)],'Color','Red','LineWidth',3);
%     line([p1(3,1),p2(3,1)],[p1(3,2),p2(3,2)],'Color','Red','LineWidth',3);
%     line([p1(4,1),p2(4,1)],[p1(4,2),p2(4,2)],'Color','Red','LineWidth',3);
%     line([p1(5,1),p2(5,1)],[p1(5,2),p2(5,2)],'Color','Red','LineWidth',3);
% % 


    P1_w = [P(1,:);P(1,:);P(1,:);P(2,:);P(2,:)];
    P2_w = [P(2,:);P(3,:);P(4,:);P(5,:);P(6,:)];



%     P2_w = [P(2,:);P(3,:);P(4,:);P(5,:);P(6,:)];


    for i = 1:length(p1)
        temp1 = inv(A)*[p1(i,:) 1].';
        temp1 = (temp1/temp1(3)).';
        ip1(i,:) = temp1(:,1:2);
        temp2 = inv(A)*[p2(i,:) 1].';
        temp2 = (temp2/temp2(3)).';
        ip2(i,:) = temp2(:,1:2);
    end 



    tic;
    [R13, T13, errs13] = ASPnLReal1(ip1', ip2', P1_w', P2_w',C_truth);
    toc;
    time01(j) = toc;


    tic;
    [R23, T23, errs23] = LPnL_Bar_ENullReal1(ip1', ip2', P1_w', P2_w',C_truth);
    toc;
    time02(j) = toc;


    tic;
    [R33, T33, errs33, aPointsN] = PnL_IOCReal1(ip1', ip2', P1_w', P2_w', A,C_truth);
    toc;
    time03(j) = toc;

    [Rr33, Tr33, errs33r, aPointsNr] = PnL_IOCReal1nr(ip1', ip2', P1_w', P2_w', A, C_truth);


    tic;
    [R07, T07, err07] = RPnLReal1(ip1', ip2', P1_w', P2_w',C_truth); 
    toc;
    time04(j) = toc;



    tic;
    [R08, T08, err08] = SRPnLReal1(ip1', ip2', P1_w', P2_w',C_truth);
    toc;
    time05(j) = toc;


    tempW2 = P2_w;
    tempW2(2,3) = aPointsN(1);
    tempW2(3,2) = aPointsN(2);
    tempW2(4,3) = aPointsN(3);
    tempW2(5,2) = aPointsN(4);

    Pa = [P1_w(1,:);tempW2];
    p2a =  [p1(1,:);p2];

    [errepASPnL(j),UrepASPnL{j}]=reprojection_error_usingRT(Pa,p2a,R13,T13,A);
    [errepLPnL(j),UrepLPnL{j}]=reprojection_error_usingRT(Pa,p2a,R23,T23,A);
    [errepRPnL(j),UrepRPnL{j}]=reprojection_error_usingRT(Pa,p2a,R07,T07,A);
    [errepSRPnL(j),UrepSRPnL{j}]=reprojection_error_usingRT(Pa,p2a,R08,T08,A);

    [errep(j),Urep{j}]=reprojection_error_usingRT(Pa,p2a,R33,T33,A);
    
    errR_ASPnL(j) = cal_rotation_err(R13,R_truth); 
    errT_ASPnL(j) = cal_translation_err(T13,T_truth);
    
    errR_LPnL(j) = cal_rotation_err(R23,R_truth); 
    errT_LPnL(j) = cal_translation_err(T23,T_truth);
    
    errR_RPnL(j) = cal_rotation_err(R07,R_truth); 
    errT_RPnL(j) = cal_translation_err(T07,T_truth);
    
    errR_SRPnL(j) = cal_rotation_err(R08,R_truth); 
    errT_SRPnL(j) = cal_translation_err(T08,T_truth);
    
    
    errR_IOC(j) = cal_rotation_err(R33,R_truth); 
    errT_IOC(j) = cal_translation_err(T33,T_truth);


    errRr_IOC(j) = cal_rotation_err(Rr33,R_truth); 
    errTr_IOC(j) = cal_translation_err(Tr33,T_truth);

    
end

errPnLIOC = mean(errep);
errASPnL = mean(errepASPnL);
errLPnL = mean(errepLPnL);
errRPnL = mean(errepRPnL);
errSRPnL = mean(errepSRPnL);


errPnLIOC_R = mean(errR_IOC);
errASPnL_R = mean(errR_ASPnL);
errLPnL_R = mean(errR_LPnL);
errRPnL_R = mean(errR_RPnL);
errSRPnL_R = mean(errR_SRPnL);

errPnLIOC_T = mean(errT_IOC);
errASPnL_T = mean(errT_ASPnL);
errLPnL_T = mean(errT_LPnL);
errRPnL_T = mean(errT_RPnL);
errSRPnL_T = mean(errT_SRPnL);


errPnLIOC_Rr = mean(errRr_IOC);
errPnLIOC_Tr = mean(errTr_IOC);


T_ASPNLT = mean(time01);
T_LPNLT = mean(time02);
T_RPNLT = mean(time04);
T_SRPNLT = mean(time05);
T_PNLIOCT = mean(time03);


S = 'Finish.';
disp(S);




