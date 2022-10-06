close all;
clc;
clear;

% addpath LPnL-Bar-ENull;
% addpath others;
% addpath ASPnL;
% addpath RPnL;
% addpath SRPnL;
% addpath C2C;

%Type4 Room Camera Pose Estimation
noise=0;
rateOutlier=0;
genN = 10;
Data = LayoutDataGen(4,noise,rateOutlier,genN);
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
    
    p1r = pointS(:,4:5);
    p2r = pointS(:,4:5);
    p12 = p2r - p1r;
    cos12 = p12(1,:) * p12(2,:)'/(norm(p12(1,:)).*norm(p12(2,:)));
    cos23 = p12(2,:) * p12(3,:)'/(norm(p12(2,:)).*norm(p12(3,:)));
    cos13 = p12(1,:) * p12(3,:)'/(norm(p12(1,:)).*norm(p12(3,:)));
    
    ang(1) = acos(cos12);
    ang(2) = acos(cos23);
    ang(3) = acos(cos13);
    
    A = [180 0 320;0 180 320;0   0   1];
    
    R = Data{j}.R;
    T = Data{j}.T;
    
%     l1 = A*[R T]*[10 256 0 1]';
%     l2 = A*[R T]*[0 246 0 1]';
%     l3 = A*[R T]*[0 256 10 1]';
%     l0 = A*[R T]*[0 256 0 1]';
%     l1 = l1(1:2)/l1(3);
%     l2 = l2(1:2)/l2(3);
%     l3 = l3(1:2)/l3(3);
%     l0 = l0(1:2)/l0(3);
%     
%     p1 = l1 - l0;
%     p2 = l2 - l0;
%     p3 = l3 - l0;
%    
%     
%     cos121 = p1 * p2'/(norm(p1).*norm(p2));
%     cos231 = p2 * p3'/(norm(p2).*norm(p3));
%     cos131 = p1 * p3'/(norm(p1).*norm(p3));
%     
%     ang1(1) = acos(cos12);
%     ang1(2) = acos(cos23);
%     ang1(3) = acos(cos13);
    
    
    
%     l1 = A * R * [1 0 0]';
%     l2 = A * R * [0 1 0]';
%     l3 = A * R * [0 0 1]';
    
    

%     
%     cos121 = l1*l2;
%     cos231 = l2*l3;
%     cos131 = l1*l3;
    


    pno1 = pointSNoNoise(:,4:5);
    pno2 = pointENoNoise(:,4:5);
    P1_w = pointS(:,1:3);
    P2_w = pointS(:,1:3) + [10,0,0;0,20,0;0,0,30];
    P2_w_real = pointE(:,1:3);
    WT2 = pointE(:,1:3);
    
    pn1 = [p1;p2];
    pn2 = [p2;p2(2:3,:);p2(1,:)];
    
    P1N_w = [pointS(:,1:3);pointE(:,1:3)];
    P2N_w = [pointE(:,1:3);pointE(2:3,1:3);pointE(1,1:3)];


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
    
    for i = 1:length(pn1)
        temp1 = inv(A)*[pn1(i,:) 1].';
        temp1 = (temp1/temp1(3)).';
        ipn1(i,:) = temp1(:,1:2);
        temp2 = inv(A)*[pn2(i,:) 1].';
        temp2 = (temp2/temp2(3)).';
        ipn2(i,:) = temp2(:,1:2);
    end     
    
     [R13, T13, errs13] = ASPnL(ipn1', ipn2', P1N_w', P2N_w');
     [R23, T23, errs23] = SRPnL(ipn1', ipn2', P1N_w', P2N_w');
     
     errRa = cal_rotation_err(R13,R_truth{j});
     errRs = cal_rotation_err(R23,R_truth{j});
     
     errTa = cal_translation_err(T13,T_truth{j});
     errTs = cal_translation_err(T23,T_truth{j});
     s = Cayley(R23);
%      
     Xw = [P1N_w(1,:);pointE(:,1:3)];
     U = [p1(1,:);p2];
     [reErr1,~] = reprojection_error_usingRT(Xw,U,R13,T13,A);
     [reErr2,~] = reprojection_error_usingRT(Xw,U,R23,T23,A);

%     R_truth{j} = [0.706932007737452,-0.0157205736630635,-0.707106781186548;-0.515596990172036,-0.695815883496157,-0.500000000000000;-0.484155842845909,0.718048131978747,-0.500000000000000];
%     T_truth{j} = [1.12388295328195;217.625325021170;34.3165751568010];
%     
    
    
%     [R1,T1]=C2CTEST(p1, p2, A,P2_w_real)
%     [R, err] = ASPnLW(ip1', ip2', P1_w', P2_w_real')
    
    
    [Ro, R3, T3, Oaerrs1] = APSRPnL(ip1', ip2', P1_w', P2_w',A, p1,cH,p2,R_truth{j},T_truth{j},WT2,ipno1');
% 
%     [R5, Oaerrs3] = APSRPnLMCMC(ip1', ip2', P1_w', P2_w', cH, A);
%     
%     [R6, Oaerrs4] = APSRPnLMCMC1(ip1', ip2', P1_w', P2_w', cH, WT2);
%     
    errRhah1(j) = cal_rotation_err(R3,R_truth{j});
    errThah1 = cal_translation_err(T3,T_truth{j});
    
    errRhaho(j) = cal_rotation_err(Ro,R_truth{j});

    
    
%     [R2, T2, Oerrs, Rerrs] = WSRPnLf3ls(ip1', ip2', P1_w', P2_w',A, p1,cH,p2,R_truth{j},T_truth{j});
% %     [R2, F] = TSRPnL(ip1', ip2', P1_w', P2_w', R_truth{j});
%     WOerrs(j) =  Oerrs;
%     WRerrs(j) =  Rerrs;
%     errR2 = cal_rotation_err(R2,R_truth{j});
%     WRe(j) = errR2;
%     errT2 = cal_translation_err(T2,T_truth{j});
%     WTe(j) = errT2;
% % %     
    
    
    [R7, T7, Oaerrs1] = APSRPnLCopy(ip1', ip2', P1_w', P2_w',A, p1,cH,p2,R_truth{j},T_truth{j},WT2,ipno1');

    errRhah7(j) = cal_rotation_err(R7,R_truth{j});
    errThah7 = cal_translation_err(T7,T_truth{j});
    
    
    
%     l11 = A*[R3 T3]*[10 256 0 1]';
%     l21 = A*[R3 T3]*[0 246 0 1]';
%     l31 = A*[R3 T3]*[0 256 10 1]';
%     l01 = A*[R3 T3]*[0 256 0 1]';
%     l11 = l11(1:2)/l11(3);
%     l21 = l21(1:2)/l21(3);
%     l31 = l31(1:2)/l31(3);
%     l01 = l01(1:2)/l01(3);
%     
%     p11 = l11 - l01;
%     p21 = l21 - l01;
%     p31 = l31 - l01;
%    
%     
%     cos122 = p11' * p21/(norm(p11).*norm(p21));
%     cos232 = p21' * p31/(norm(p21).*norm(p31));
%     cos132 = p11' * p31/(norm(p11).*norm(p31));
%     
%     ang11(1) = acos(cos122);
%     ang11(2) = acos(cos232);
%     ang11(3) = acos(cos132);
    
    
%     sol1=myP3L(p1', p2', P1_w, P2_w);
    
%     sol1=myP3L(ip1', ip2', P1_w', P2_w_real');
%     
%     sol2=P3L(ip1', ip2', P1_w', P2_w');
% % 
%     [Rf,Tf,err] = opPnlMCMC(p1,p2,P1_w', P2_w', A, cH);
%     [Rf,ip,err] = opPnlMCMC(p1,p2,P1_w', P2_w', A, cH);
%     [ R,T,minErr] = TSRPnLMCMC(ip', ip2', P1_w', P2_w',cH,Rf)
    
    
% % 
%     errRhah2(j) = cal_rotation_err(Rf,R_truth{j});
%     errThah2 = cal_translation_err(Tf,T_truth{j});
%     errRhah3 = cal_rotation_err(R,R_truth{j});
%     errThah3 = cal_translation_err(T,T_truth{j});
    
    
    %aPoints,errp
    AOerrs1(j) =  Oaerrs1;
%     ARerrs1(j) =  Raerrs1;
    errR3 = cal_rotation_err(R3,R_truth{j});
    ARe(j) = errR3;
    errT3 = cal_translation_err(T3,T_truth{j});
    ATe(j) = errT3;
    
%     ferr(j) = errPf1;
%     ferrt1(j) = errPft1;
%     serrp = serrp + errp;
    
%     [R3, T3] = SRPnL(ip1', ip2', P1_w', P2_w');
    resR{j} = R3;
    resT{j} = T3;
    
       
    [Ro4,R4, T4, Oaerrs2] = APSRPnL(ipno1', ipno2', P1_w', P2_w',A, p1,cH,p2,R_truth{j},T_truth{j},WT2,ipno1');

%     serr(j) = errPf2;
%     ferrt2(j) = errPft2;
    
    AOerrs2(j) =  Oaerrs2;
%     ARerrs2(j) =  Raerrs2;
    
    errR4 = cal_rotation_err(R4,R_truth{j});
    ARe1(j) = errR4;
    errT4 = cal_translation_err(T4,T_truth{j});
    ATe1(j) = errT4;
%     
%     [F1, G] = P3LGA(A,p1,p2,P1_w', P2_w',cH,R3,T3,aPoints)

%     fserr = mean(errRhah1);
%     Sserr = mean(errRhah2);

%     fserrt = median(errRhah1);
%     Sserrt = median(errRhah2);


%     [R1,T1]=C2CTEST(p1, p2, A,P2_w_real)


%     [R2, T2, Oerrs, Rerrs] = WSRPnLf3ls(ip1', ip2', P1_w', P2_w',A, p1,cH,p2,R_truth{j},T_truth{j});
% %     [R2, F] = TSRPnL(ip1', ip2', P1_w', P2_w', R_truth{j});
%     WOerrs(j) =  Oerrs;
%     WRerrs(j) =  Rerrs;
%     errR2 = cal_rotation_err(R2,R_truth{j});
%     WRe(j) = errR2;
%     errT2 = cal_translation_err(T2,T_truth{j});
%     WTe(j) = errT2;


end




foerr = mean(errRhaho);
fserr = mean(errRhah1);
% Sserr = median(errRhah2);

fserrt = mean(errRhah1);
% Sserrt = median(errRhah2);

% BmeanOe = mean(WOerrs);
% BmedianOe = median(WOerrs);
% 
% BmeanREe = mean(WRerrs);
% BmedianREe = median(WRerrs);
% 
% 
% BmeanRe = mean(WRe);
% BmedianRe = median(WRe);
% 
% BmeanTe = mean(WTe);
% BmedianTe = median(WTe);


AmeanOe = mean(AOerrs1);
AmedianOe = median(AOerrs1);

% AmeanREe = mean(ARerrs1);
% AmedianREe = median(ARerrs1);


AmeanRe = mean(ARe);
AmedianRe = median(ARe);

AmeanTe = mean(ATe);
AmedianTe = median(ATe);


AmeanRe1 = mean(ARe1);
AmedianRe1 = median(ARe1);

AmeanTe1 = mean(ATe1);
AmedianTe1 = median(ATe1);





% avges = ses/genN;
% avgres = sres/genN;
% avgtes = stes/genN;
% 
% avgea = sea/genN;
% avgrea = srea/genN;
% avgtea = stea/genN;
% 
% avgErrp  = serrp/genN;

% avgew = sew/genN;
% avgrew = srew/genN;
% avgtew = stew/genN;
S = 'Finish.';
disp(S);
% 
% avgea = sea/genN;
% avgrea = srea/genN;
% avgtea = stea/genN;


% avgres = sres/genN;



