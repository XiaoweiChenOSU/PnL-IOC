close all;
clc;
clear;

IniToolbox;

%Type5 Room Camera Pose Estimation

load('type5GroundTruth.mat');
[~,numImage] = size(groundTruth);

for j = 1:numImage
    
    R_truth = groundTruth(j).Rotation;
    T_truth = groundTruth(j).Translation;
    p = groundTruth(j).point;
    P = groundTruth(j).wpoint;
    A = groundTruth(j).intrinsics_matrix;
    offset = 0.01;
    mul = 0.015;

%     offset = 0;
%     mul = 0;
    noise1 = [rand*mul-offset rand*mul-offset];
    noise2 = [rand*mul-offset rand*mul-offset];
    noise3 = [rand*mul-offset rand*mul-offset];
    noise4 = [rand*mul-offset rand*mul-offset];
    noise5 = [rand*mul-offset rand*mul-offset];
    noise6 = [rand*mul-offset rand*mul-offset];

%     noise =[rand*0.1 rand*0.1];
%     p1 = [p(1,:)+ noise;p(1,:)+noise;p(1,:)+noise;p(2,:);p(2,:)];
%     p2 = [p(2,:);p(3,:);p(4,:);p(5,:);p(6,:)];

    p1 = [p(1,:)+noise1; p(1,:)+noise1; p(1,:)+noise1; p(2,:)+noise2;p(2,:)+noise2];
    p2 = [p(2,:)+noise2;p(3,:)+noise3;p(4,:)+noise4;p(5,:)+noise5;p(6,:)+noise6];

    
    P1_w = [P(1,:); P(1,:); P(1,:); P(2,:);P(2,:)];
    P2_w = [P(2,:);P(3,:);P(4,:);P(5,:);P(6,:)];

%     D_l =  P2_wT - P1_w;
%     P2_w = P1_w + 10*D_l;
    
    
%     P1_w = [0,0,0;0,0,0;0,0,0;0,8,0;0,8,0];
% 
%     P2_w(1,:) = [0,8,0];
%     P2_w(2:2:5,:) = P1_w(2:2:5,1:3) + ones(2,1)*[0 0 5];
%     P2_w(3:2:5,:) = P1_w(3:2:5,1:3) + ones(2,1)*[5 0 0];

   





    for i = 1:length(p1)
        temp1 = inv(A)*[p1(i,:) 1].';
        temp1 = (temp1/temp1(3)).';
        ip1(i,:) = temp1(:,1:2);
        temp2 = inv(A)*[p2(i,:) 1].';
        temp2 = (temp2/temp2(3)).';
        ip2(i,:) = temp2(:,1:2);
    end 





    [R13, T13, errs13] = ASPnLReal5(ip1', ip2', P1_w', P2_w');
  


    [R23, T23, errs23] = LPnL_Bar_ENullReal5(ip1', ip2', P1_w', P2_w');


    errOta = errs13;
    errOts  = errs23;






    [R33, T33, errs33,aPointsN] = PnL_IOCReal5(ip1', ip2', P1_w', P2_w',A);




    tempW2 = P2_w;
    tempW2(2,3) = aPointsN(1);
    tempW2(3,1) = aPointsN(2);
    tempW2(4,3) = aPointsN(3);
    tempW2(5,1) = aPointsN(4);

    Pa = [P1_w(1,:);tempW2];
    p2a =  [p1(1,:);p2];


    




    [R07, T07, err07] = RPnLReal5(ip1', ip2', P1_w', P2_w'); 

    [R08, T08, err08] = SRPnLReal5(ip1', ip2', P1_w', P2_w');


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


S = 'Finish.';
disp(S);



