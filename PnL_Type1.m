close all;
clc;
clear;

% addpath LPnL-Bar-ENull;
% addpath others;
% addpath ASPnL;
% addpath RPnL;
% addpath SRPnL;
% addpath C2C;

%Type1 Room Camera Pose Estimation
noise=10;
rateOutlier=0;
genN = 100;
Data = LayoutDataGen(1,noise,rateOutlier,genN);
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
   
    A = [180 0 320;0 180 320;0   0   1];
    
    R = Data{j}.R;
    T = Data{j}.T;
    

    pno1 = pointSNoNoise(:,4:5);
    pno2 = pointENoNoise(:,4:5);
    
    P1_w = pointS(:,1:3);
    P2_w(1,:) = pointE(1,1:3);
    P2_w(2:2:5,:) = pointS(2:2:5,1:3) + ones(2,1)*[0 0 50];
    P2_w(3:2:5,:) = pointS(3:2:5,1:3) + ones(2,1)*[0 50 0];
    P2_w_real = pointE(:,1:3);
    WT2 = pointE(:,1:3);
    

    

    for i = 1:length(p1)
        temp1 = inv(A)*[p1(i,:) 1].';
        temp1 = (temp1/temp1(3)).';
        ip1(i,:) = temp1(:,1:2);
        temp2 = inv(A)*[p2(i,:) 1].';
        temp2 = (temp2/temp2(3)).';
        ip2(i,:) = temp2(:,1:2);
    end 
    
    
    
    for i = 1:length(pno1)
        temp1 = inv(A)*[pno1(i,:) 1].';
        temp1 = (temp1/temp1(3)).';
        ipno1(i,:) = temp1(:,1:2);
        temp2 = inv(A)*[pno2(i,:) 1].';
        temp2 = (temp2/temp2(3)).';
        ipno2(i,:) = temp2(:,1:2);
    end 
    

    R_truth{j} = Data{j}.R;
    T_truth{j} = Data{j}.T;
    
    
    tic;
    [R13, T13, errs13] = ASPnL1(ip1', ip2', P1_w', P2_w');
    toc;
    time01(j) = toc;
    
    tic;
    [R23, T23, errs23] = LPnL_Bar_ENull1(ip1', ip2', P1_w', P2_w');
    toc;
    time02(j) = toc;
   
    errOta(j) = errs13;
    errOts(j) = errs23;
    
    [reErr1,~] = reprojection_error_usingRT(P2_w_real,p2,R13,T13,A);
    [reErr2,~] = reprojection_error_usingRT(P2_w_real,p2,R23,T23,A);
    
    
    errRea(j) = reErr1;
    errRes(j) = reErr2;

     
    errRa(j) = cal_rotation_err(R13,R_truth{j});
    errRs(j) = cal_rotation_err(R23,R_truth{j});
     
    errTa(j) = cal_translation_err(T13,T_truth{j});
    errTs(j) = cal_translation_err(T23,T_truth{j});
    
    tic;
    [R33, T33, errs33] = PnL_IOC1(ip1', ip2', P1_w', P2_w',A);
    toc;
    time03(j) = toc;
    
    RA{j} = R33;
    TA{j} = T33;

    errOtAP(j) = errs33;

    
    [reErr1,~] = reprojection_error_usingRT(P2_w_real,p2,R33,T33,A);
    
    
    errReAP(j) = reErr1;


     
    errRAP(j) = cal_rotation_err(R33,R_truth{j});
    

     
    errTAP(j) = cal_translation_err(T33,T_truth{j});
    
    
    tic;
    [R07, T07] = RPnL1(ip1', ip2', P1_w', P2_w'); 
    toc;
    time07(j) = toc;
    errR07(j) = cal_rotation_err(R07,R_truth{j}); 
    errT07(j) = cal_translation_err(T07,T_truth{j});
    
    
    tic;
    [R08, T08] = SRPnL1(ip1', ip2', P1_w', P2_w');
    toc;
    time08(j) = toc;
    errR08(j) = cal_rotation_err(R08,R_truth{j}); 
    errT08(j) = cal_translation_err(T08,T_truth{j});
    Rsr{j} = R08;
    
%     tic;
%     [R09, T09] = ASPnLA11(ip1', ip2', P1_w', P2_w');
%     toc;
%     time09(j) = toc;
%     errR09(j) = cal_rotation_err(R09,R_truth{j}); 
%     errT09(j) = cal_translation_err(T09,T_truth{j});
%     R9{j} = R09;
%     
    
    
    
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


errRPNLRm = mean(errR07);
errRPNLRme = median(errR07);
errRPNLTm = mean(errT07);
errRPNLTme = median(errT07);


errSRPNLRm = mean(errR08);
errSRPNLRme = median(errR08);
errSRPNLTm = mean(errT08);
errSRPNLTme = median(errT08);


% errIOC09Rm = mean(errR08);
% errIOC09Rme = median(errR08);
% errIOC09Tm = mean(errT08);
% errIOC09Tme = median(errT08);

ti01 = mean(time01);
ti02 = mean(time02);
ti03 = mean(time03);
ti07 = mean(time07);
ti08 = mean(time08);

S = 'Finish.';
disp(S);




