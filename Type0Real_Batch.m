close all;
clc;
clear;

% addpath LPnL-Bar-ENull;
% addpath others;
% addpath ASPnL;
% addpath RPnL;
% addpath SRPnL;
% addpath C2C;

%Type0 Room Camera Pose Estimation


%0f42320ef887416fb8ed9b91b6641cfc_i1_5.jpg

load('type0GroundTruth.mat');
[~,numImage] = size(groundTruth);

for j = 1:numImage
    
    R_truth = groundTruth(j).Rotation;
    T_truth = groundTruth(j).Translation;
    p = groundTruth(j).point;
    P = groundTruth(j).wpoint;
    A = groundTruth(j).intrinsics_matrix;
    Lheight = groundTruth(j).Lheight;
    Lwidth = groundTruth(j).Lwidth;
% % 
%     Lwidth =3.69283131691707;
% 

%     offset = 0.5;
%     mul = 1;  
   f = A(1,1);

 

    offset = 0.5;
    mul = 1;

%     offset = 0;
%     mul = 0;

    noise = 1;

    noise1 = [rand*mul-offset rand*mul-offset];
    noise2 = [rand*mul-offset rand*mul-offset];
    noise3 = [rand*mul-offset rand*mul-offset];
    noise4 = [rand*mul-offset rand*mul-offset];
    noise5 = [rand*mul-offset rand*mul-offset];
    noise6 = [rand*mul-offset rand*mul-offset];
    noise7 = [rand*mul-offset rand*mul-offset];
    noise8 = [rand*mul-offset rand*mul-offset];
    

    p1 = [p(1,:)+noise1;p(2,:)+noise2;p(3,:)+noise3;p(4,:)+noise4;p(1,:)+noise1;p(2,:)+noise2;p(3,:)+noise3;p(4,:)+noise4];
    p2 = [p(2,:)+noise2;p(3,:)+noise3;p(4,:)+noise4;p(1,:)+noise1;p(5,:)+noise5;p(6,:)+noise6;p(7,:)+noise7;p(8,:)+noise8];

    pointS = [P(1,:);P(2,:);P(3,:);P(4,:);P(1,:);P(2,:);P(3,:);P(4,:)];
    pointE = [P(2,:);P(3,:);P(4,:);P(1,:);P(5:8,:)];
    


    P1_w = pointS;
    P2_w = pointE;
    
    p2d = p2;



    for i = 1:length(p1)
        temp1 = inv(A)*[p1(i,:) 1].';
        temp1 = (temp1/temp1(3)).';
        ip1(i,:) = temp1(:,1:2);
        temp2 = inv(A)*[p2(i,:) 1].';
        temp2 = (temp2/temp2(3)).';
        ip2(i,:) = temp2(:,1:2);
    end 



    tp1 = ip1';
    tp2 = ip2';



 
    W1 = P1_w';
    W2 = P2_w';  
    P1N_w = [W1 W2(:,5:8) W2(:,5:6) W1(:,1:4)];
    P2N_w = [W2 W2(:,6:8) W2(:,5) W2(:,7:8) W2(:,6) W2(:,5) W2(:,8) W2(:,7)];



    [R01, T01, errs01] = ASPnL0(ip1', ip2', P1_w', P2_w');



    [R02, T02, errs02] = LPnL_Bar_ENull0(ip1', ip2', P1_w', P2_w');


    [R07, T07, err07] = RPnL0(ip1', ip2', P1_w', P2_w'); 


    [R08, T08, err08] = SRPnL0(ip1', ip2', P1_w', P2_w');



    [R33, T33, errs33, aPointsN] = PnL_IOCReal0(ip1', ip2', P1_w', P2_w');

    [R33r, T33r, errs33r, aPointsNr] = PnL_IOCReal0nr(ip1', ip2', P1_w', P2_w');

    [R_u, T_u, is_fail] = dlsu_full_type0Real(noise,f,A,P1_w,P2_w,p1,p2,R_truth,T_truth);



    if (is_fail)
        errRDU(j) = Inf;
        errTDU(j) = Inf;
        COUNTfAIL = sum(isinf(errRDU));
    else
        errRDU(j) = cal_rotation_err(R_u,R_truth);
        errTDU(j) = cal_translation_err(T_u,T_truth);
    end    

%     [R33, T33, errs33, aPointsN] = ASPnLAReal0(ip1', ip2', P1_w', P2_w');



    tempW2 = pointE;
    tempW2(5,2) = aPointsN(1);
    tempW2(6,2) = aPointsN(2);
    tempW2(7,2) = aPointsN(3);
    tempW2(8,2) = aPointsN(4);


    Pa = tempW2;
    p2a =  p2;

    [errepASPnL(j),UrepASPnL{j}]=reprojection_error_usingRT(Pa,p2a,R01,T01,A);
    [errepLPnL(j),UrepLPnL{j}]=reprojection_error_usingRT(Pa,p2a,R02,T02,A);
    [errepRPnL(j),UrepRPnL{j}]=reprojection_error_usingRT(Pa,p2a,R07,T07,A);
    [errepSRPnL(j),UrepSRPnL{j}]=reprojection_error_usingRT(Pa,p2a,R08,T08,A);

    [errep(j),Urep{j}]=reprojection_error_usingRT(Pa,p2a,R33,T33,A);

    
    errR_ASPnL(j) = cal_rotation_err(R01,R_truth); 
    errT_ASPnL(j) = cal_translation_err(T01,T_truth);
    
    errR_LPnL(j) = cal_rotation_err(R02,R_truth); 
    errT_LPnL(j) = cal_translation_err(T02,T_truth);
    
    errR_RPnL(j) = cal_rotation_err(R07,R_truth); 
    errT_RPnL(j) = cal_translation_err(T07,T_truth);
    
    errR_SRPnL(j) = cal_rotation_err(R08,R_truth); 
    errT_SRPnL(j) = cal_translation_err(T08,T_truth);
    
    
    errR_IOC(j) = cal_rotation_err(R33,R_truth); 
    errT_IOC(j) = cal_translation_err(T33,T_truth);

    errRr_IOC(j) = cal_rotation_err(R33r,R_truth); 
    errTr_IOC(j) = cal_translation_err(T33r,T_truth);

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

% errPnLIOC_Rr = mean(errRr_IOC);
% errPnLIOC_Tr = mean(errTr_IOC);


errDLSURm = mean(errRDU((isfinite(errRDU))));
errDLSURme = median(errRDU((isfinite(errRDU))));

errDLSUTm = mean(errTDU((isfinite(errTDU))));
errDLSUTme = median(errTDU((isfinite(errTDU))));


S = 'Finish.';
disp(S);




