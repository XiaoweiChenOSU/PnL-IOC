close all;
clc;
clear;

IniToolbox;


%Type0 Room Camera Pose Estimation
noise=1;
rateOutlier=0;
genN = 100;
Data = LayoutDataGen(0,noise,rateOutlier,genN);
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
%     pointS = [0,256,0,317.606525488722,352.591288736237;0,256,0,317.606525488722,352.591288736237;0,256,0,317.606525488722,352.591288736237];
%     pointE = [249,256,0,639.347400956776,156.038619719792;0,80,0,319.737606610151,637.705694522170;0,256,241,0.485572080676320,170.664678373069];
    cH = Data{j}.Cc(2);
%     cH = 126.803873198251;
    p1 = pointS(:,4:5);
    p2 = pointE(:,4:5);
   
    
    A = [180 0 320;0 180 320;0   0   1];
    f = 180;
    
    R = Data{j}.R;
    T = Data{j}.T;
    

    pno1 = pointSNoNoise(:,4:5);
    pno2 = pointENoNoise(:,4:5);
    P1_w = pointS(:,1:3);
    P2_w = [pointE(1:4,1:3); pointS(5:8,1:3) + ones(4,1)*[0 0 10]];
    
    P1_Ow = [pointS(:,1:3); pointS(1:2,1:3)];
    P2_Ow = [pointE(1:4,1:3); pointS(5:8,1:3) + ones(4,1)*[0 0 10];pointS(3:4,1:3)];
    
    P2_w_real = pointE(:,1:3);
    WT2 = pointE(:,1:3);
    
    p2d = p2;
    pn1 = [p1;p2];
    pn2 = [p2;p2(2:3,:);p2(1,:)];
    
    P1N_w = [pointS(:,1:3);pointE(:,1:3)];
    P2N_w = [pointE(:,1:3);pointE(2:3,1:3);pointE(1,1:3)];
    
    
    
    

    
%     P1N_w = [W1 W2(:,5:8) W2(:,5:6)];
%     P2N_w = [W2 W2(:,6:8) W2(:,5) W2(:,7:8)];

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
    
    
    tp1 = ip1';
    tp2 = ip2';
    
    po1 = [ip1;ip1(1:2,:)];
    po2 = [ip2;ip1(3:4,:)];
    
    pn1 = [tp1 tp2(:,5:8) tp2(:,5:6) tp1(:,1:4)];
    pn2 = [tp2 tp2(:,6:8) tp2(:,5) tp2(:,7:8) tp2(:,6) tp2(:,5) tp2(:,8) tp2(:,7)];
    
    W1 = P1_w';
    W2 = P2_w_real';  
    P1N_w = [W1 W2(:,5:8) W2(:,5:6) W1(:,1:4)];
    P2N_w = [W2 W2(:,6:8) W2(:,5) W2(:,7:8) W2(:,6) W2(:,5) W2(:,8) W2(:,7)];
    
    
    
    tic;
    [R01, T01, errs01] = ASPnL0(ip1', ip2', P1_w', P2_w');
    toc;
    time01(j) = toc;
    
    tic;
    [R02, T02, errs02] = LPnL_Bar_ENull0(ip1', ip2', P1_w', P2_w');
    toc;
    time02(j) = toc;

   
    errOta(j) = errs01;
    errOts(j) = errs02;
    
    [reErr1,~] = reprojection_error_usingRT(P2_w_real,p2,R01,T01,A);
    [reErr2,~] = reprojection_error_usingRT(P2_w_real,p2,R02,T02,A);
    
    
    errRea(j) = reErr1;
    errRes(j) = reErr2;

     
    errRa(j) = cal_rotation_err(R01,R_truth{j});
    errRs(j) = cal_rotation_err(R02,R_truth{j});
     
    errTa(j) = cal_translation_err(T01,T_truth{j});
    errTs(j) = cal_translation_err(T02,T_truth{j});
    
    
    
    tic;
    [R07, T07] = RPnL0(ip1', ip2', P1_w', P2_w'); 
    toc;
    time07(j) = toc;
    errR07(j) = cal_rotation_err(R07,R_truth{j}); 
    errT07(j) = cal_translation_err(T07,T_truth{j});
    
    tic;
    [R08, T08] = SRPnL0(ip1', ip2', P1_w', P2_w');
    toc;
    time08(j) = toc;
    errR08(j) = cal_rotation_err(R08,R_truth{j}); 
    errT08(j) = cal_translation_err(T08,T_truth{j});
    Rsr{j} = R08;
    
    

    
    tic;
    [R33, T33, errs33] = PnL_IOC0(ip1', ip2', P1_w', P2_w');
    toc;
    time03(j) = toc;
    errOtAP(j) = errs33;

    
    [reErr1,~] = reprojection_error_usingRT(P2_w_real,p2,R33,T33,A);
    
    
    errReAP(j) = reErr1;


     
    errRAP(j) = cal_rotation_err(R33,R_truth{j});
  
     
    errTAP(j) = cal_translation_err(T33,T_truth{j});


    tic;
    [R_u, T_u, is_fail] = dlsu_full_type0(noise,f,A,P1_w,P2_w,p1,p2,R,T);
    toc;
    timeDU(j) = toc;
    
    if (is_fail)
        errRDU(j) = Inf;
        errTDU(j) = Inf;
        COUNTfAIL = sum(isinf(errRDU));
    else
        errRDU(j) = cal_rotation_err(R_u,R_truth{j});
        errTDU(j) = cal_translation_err(T_u,T_truth{j});
    end    
end

errAspnlRm = mean(errRa);
errLpnlRm = mean(errRs);

errAspnlRme = median(errRa);
errLpnlRme = median(errRs);


errAspnlTm = mean(errTa);
errLpnlTm = mean(errTs);

errAspnlTme = median(errTa);
errLpnlTme = median(errTs);

% errAspnlOtm = mean(errOta);
% errLpnlOtm = mean(errOts);
% 
% errAspnlOtme = median(errOta);
% errLpnlOtme = median(errOts);


% errAspnlRem = mean(errRea);
% errLpnlRem = mean(errRes);
% 
% errAspnlReme = median(errRea);
% errLpnlReme = median(errRes);


errAsIOCARm = mean(errRAP);
errAsIOCARme = median(errRAP);
errAsIOCATm = mean(errTAP);
errAsIOCATme = median(errTAP);

% errAsIOCAOtm = mean(errOtAP);
% errAsIOCAOtme = median(errOtAP);
% errAsIOCARem = mean(errReAP);
% errAsIOCAReme = median(errReAP);


errDLSURm = mean(errRDU((isfinite(errRDU))));
errDLSURme = median(errRDU((isfinite(errRDU))));

errDLSUTm = mean(errTDU((isfinite(errTDU))));
errDLSUTme = median(errTDU((isfinite(errTDU))));


% errLiftARm = mean(errR03);
% errLiftARme = median(errR03);
% errLiftATm = mean(errT03);
% errLiftATme = median(errT03);

% errAlgLSRm = mean(errR04);
% errAlgLSRme = median(errR04);
% errAlgLSTm = mean(errT04);
% errAlgLSTme = median(errT04);
% 
% errDLTRm = mean(errR05);
% errDLTRme = median(errR05);
% errDLTTm = mean(errT05);
% errDLTTme = median(errT05);
% 
% errLBARRm = mean(errR06);
% errLBARRme = median(errR06);
% errLBARTm = mean(errT06);
% errLBARTme = median(errT06);


errRPNLRm = mean(errR07);
errRPNLRme = median(errR07);
errRPNLTm = mean(errT07);
errRPNLTme = median(errT07);

errSRPNLRm = mean(errR08);
errSRPNLRme = median(errR08);
errSRPNLTm = mean(errT08);
errSRPNLTme = median(errT08);

ti01 = mean(time01);
ti02 = mean(time02);
ti03 = mean(time03);
ti07 = mean(time07);
ti08 = mean(time08);

S = 'Finish.';
disp(S);




