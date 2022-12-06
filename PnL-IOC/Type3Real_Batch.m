close all;
clc;
clear;

IniToolbox;

%Type3 Room Camera Pose Estimation

load('type3GroundTruth.mat');
[~,numImage] = size(groundTruth);

for j = 1:numImage
    
    R_truth = groundTruth(j).Rotation;
    T_truth = groundTruth(j).Translation;
    C_truth = -inv(R_truth)*T_truth;
    p = groundTruth(j).point;
    P = groundTruth(j).wpoint;
    A = groundTruth(j).intrinsics_matrix;

    Lheight = 2.9133239326302;


%     p1 = [p(1,:); p(1,:); p(1,:)];
%     p2 = [p(2,:); p(3,:); p(4,:)];

%     p1 = [roundn((p(1,:)),-1); roundn((p(1,:)),-1); roundn((p(1,:)),-1)];
%     p2 = [roundn((p(2,:)),-1);roundn((p(3,:)),-1);roundn((p(4,:)),-1)];

    p1 = [round(p(1,:));round(p(1,:));round(p(1,:))];
    p2 = [round(p(2,:));round(p(3,:));round(p(4,:))];

% 
%     I = imread('10c2796a0ad1486f835e992139f31b18_i0_4.jpg');
% 
%     imshow(I); hold on;
% 
%     line([p1(1,1),p2(1,1)],[p1(1,2),p2(1,2)],'Color','Red','LineWidth',3);
%     line([p1(2,1),p2(2,1)],[p1(2,2),p2(2,2)],'Color','Red','LineWidth',3);
%     line([p1(3,1),p2(3,1)],[p1(3,2),p2(3,2)],'Color','Red','LineWidth',3);
    
    %     P1_w = [0,Lheight,0;0,Lheight,0;0,Lheight,0];
% 
%     P2_w(1,:) = P1_w(1,:) + [1 0 0];
%     P2_w(2,:) = P1_w(2,:) - [0 1 0];
%     P2_w(3,:) = P1_w(3,:) + [0 0 1];
    
    P1_w = [P(1,:); P(1,:); P(1,:)];
    P2_w = [P(2,:);P(3,:);P(4,:)];
    cH = C_truth(2);


    for i = 1:length(p1)
        temp1 = inv(A)*[p1(i,:) 1].';
        temp1 = (temp1/temp1(3)).';
        ip1(i,:) = temp1(:,1:2);
        temp2 = inv(A)*[p2(i,:) 1].';
        temp2 = (temp2/temp2(3)).';
        ip2(i,:) = temp2(:,1:2);
    end 


    tic;
    [R23, T23, errs23] = SRPnLReal3(ip1', ip2', P1_w', P2_w',C_truth);
    toc;
    time02(j) = toc;

    tic;
    [R33, T33, aPointsN, errs33] = PnL_IOCReal3nr(ip1', ip2', P1_w', P2_w',C_truth);
    toc;
    time03(j) = toc;

    
    [Rr33, Tr33, errs33r, aPointsNr] = PnL_IOCReal3(ip1', ip2', P1_w', P2_w',C_truth);




    tic;
    [R08, T08, err08] = ASPnLReal3(ip1', ip2', P1_w', P2_w',C_truth);
    toc;
    time01(j) = toc;


    tempW2 = P2_w;
    tempW2(1,1) = aPointsN(1);
    tempW2(2,2) = aPointsN(2);
    tempW2(3,3) = aPointsN(3);

    Pa = [P1_w(1,:);tempW2];
    p2a =  [p1(1,:);p2];

    [errepASPnL(j),UrepASPnL{j}]=reprojection_error_usingRT(Pa,p2a,R08,T08,A);
    [errepSRPnL(j),UrepSRPnL{j}]=reprojection_error_usingRT(Pa,p2a,R23,T23,A);

    [errep(j),Urep{j}]=reprojection_error_usingRT(Pa,p2a,R33,T33,A);

    errR_ASPnL(j) = cal_rotation_err(R08,R_truth); 
    errT_ASPnL(j) = cal_translation_err(T08,T_truth);

    errR_SRPnL(j) = cal_rotation_err(R23,R_truth); 
    errT_SRPnL(j) = cal_translation_err(T23,T_truth);
    
    
    errR_IOC(j) = cal_rotation_err(R33,R_truth); 
    errT_IOC(j) = cal_translation_err(T33,T_truth);

    errRr_IOC(j) = cal_rotation_err(Rr33,R_truth); 
    errTr_IOC(j) = cal_translation_err(Tr33,T_truth);
        
end

errPnLIOC = mean(errep);
errASPnL = mean(errepASPnL);
errSRPnL = mean(errepSRPnL);


errPnLIOC_R = mean(errR_IOC);
errASPnL_R = mean(errR_ASPnL);
errSRPnL_R = mean(errR_SRPnL);

errPnLIOC_T = mean(errT_IOC);
errASPnL_T = mean(errT_ASPnL);
errSRPnL_T = mean(errT_SRPnL);

errPnLIOC_Rr = mean(errRr_IOC);
errPnLIOC_Tr = mean(errTr_IOC);

T_ASPNLT = mean(time01);
T_SRPNLT = mean(time02);
T_PNLIOCT = mean(time03);


S = 'Finish.';
disp(S);




