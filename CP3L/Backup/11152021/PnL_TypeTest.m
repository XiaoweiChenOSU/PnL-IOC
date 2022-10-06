close all;
clc;
clear;

% addpath LPnL-Bar-ENull;
% addpath others;
% addpath ASPnL;
% addpath RPnL;
% addpath SRPnL;
% addpath C2C;

%Type3 Room Camera Pose Estimation
noise=5;
rateOutlier=0;
genN = 100;
Data = LayoutDataGen(3,noise,rateOutlier,genN);
ses = 0;
sres = 0;
stes = 0;
sea = 0;
srea = 0;
stea = 0;
serrp = 0;
for j = 1:genN
    pointS = Data{j}.pointS;
    pointE = Data{j}.pointE;
    pointSNoNoise = Data{j}.pointSt;
    pointENoNoise = Data{j}.pointEt;
    cH = Data{j}.Cc(2);
    
    p1 = pointS(:,4:5);
    p2 = pointE(:,4:5);
    
    p1real = pointSNoNoise(:,4:5);
    
    p2d = [p1(1,:);p2];
   
    A = [180 0 320;0 180 320;0   0   1];
    
    R = Data{j}.R;
    T = Data{j}.T;
    

    pno1 = pointSNoNoise(:,4:5);
    pno2 = pointENoNoise(:,4:5);
    
    P1_w = pointS(:,1:3);
    P2_w(1,:) = pointS(1,1:3) + [10 0 0];
    P2_w(2,:) = pointS(2,1:3) - [0 10 0];
    P2_w(3,:) = pointS(3,1:3) + [0 0 10];
    P2_w_real = pointE(:,1:3);

    

    

    for i = 1:length(p1)
        temp1 = inv(A)*[p1(i,:) 1].';
        temp1 = (temp1/temp1(3)).';
        ip1(i,:) = temp1(:,1:2);
        temp2 = inv(A)*[p2(i,:) 1].';
        temp2 = (temp2/temp2(3)).';
        ip2(i,:) = temp2(:,1:2);
        temp3 = inv(A)*[p1real(i,:) 1].';
        temp3 = (temp3/temp3(3)).';
        ip1r(i,:) = temp3(:,1:2);
    end 
    
    


    R_truth{j} = Data{j}.R;
    T_truth{j} = Data{j}.T;
    
    
    
    
    [R33, T33, aPoints, errs33] = ASPnLATest(ip1', ip2', P1_w', P2_w',cH,R,p2d,A);
%     [R33, T33, errs33] = ASPnLATestBack(ip1', ip2', P1_w', P2_w_real',cH,A,p2d,R);
    
    RA{j} = R33;
    TA{j} = T33;

    errOtAP(j) = errs33;
    
    tempW2 = P2_w;
    tempW2(1,1) = aPoints(1);
    tempW2(2,2) = aPoints(2);
    tempW2(3,3) = aPoints(3);
    
    Pa = [P1_w(1,:);tempW2];
    p2a =  [p1(1,:);p2];

    
    [reErr1,~] = reprojection_error_usingRT(Pa,p2a,R33,T33,A);
    
    
    errReAP(j) = reErr1;


     
    errRAP(j) = cal_rotation_err(R33,R_truth{j});
    

     
    errTAP(j) = cal_translation_err(T33,T_truth{j});
    
    
    
    ipo1 = [ip1;ip2];
    ipo2 = [ip2;ip2(2:3,:);ip2(1,:)];
    P1_ow = [P1_w;P2_w_real];
    P2_ow = [P2_w_real;P2_w_real(2:3,:);P2_w_real(1,:)];
    
    [R13, T13, errs13] = SRPnL(ipo1', ipo2', P1_ow', P2_ow');
    [R23, T23, errs23] = SRPnL3(ip1', ip2', P1_w', P2_w',cH);
    
    RS{j} = R23;
    TS{j} = T23;
    
    errOta(j) = errs13;
    errOts(j) = errs23;
    
    Pa = [P1_w(1,:);P2_w_real];
    p2a =  [p1(1,:);p2];
    
    [reErr1,~] = reprojection_error_usingRT(Pa,p2a,R13,T13,A);
    [reErr2,~] = reprojection_error_usingRT(P1_w(1,:),p1(1,:),R23,T23,A);
    
    
    errRea(j) = reErr1;
    errRes(j) = reErr2;

     
    errRa(j) = cal_rotation_err(R13,R_truth{j});
    errRs(j) = cal_rotation_err(R23,R_truth{j});
     
    errTa(j) = cal_translation_err(T13,T_truth{j});
    errTs(j) = cal_translation_err(T23,T_truth{j});


    
    
end

errAspnlRm = mean(errRa);
errLpnlRm = mean(errRs);

errAspnlRme = median(errRa);
errLpnlRme = median(errRs);


errAspnlTm = mean(errTa);
errLpnlTm = mean(errTs);

errAspnlTme = median(errTa);
errLpnlTme = median(errTs);

errAspnlOtm = mean(errOta);
errLpnlOtm = mean(errOts);

errAspnlOtme = median(errOta);
errLpnlOtme = median(errOts);


errAspnlRem = mean(errRea);
errLpnlRem = mean(errRes);

errAspnlReme = median(errRea);
errLpnlReme = median(errRes);


errAspnlARm = mean(errRAP);
errAspnlARme = median(errRAP);
errAspnlATm = mean(errTAP);
errAspnlATme = median(errTAP);
errAspnlAOtm = mean(errOtAP);
errAspnlAOtme = median(errOtAP);
errAspnlARem = mean(errReAP);
errAspnlAReme = median(errReAP);



S = 'Finish.';
disp(S);



