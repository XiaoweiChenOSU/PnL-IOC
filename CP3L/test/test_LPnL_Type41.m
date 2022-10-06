close all;
clc;
clear;

addpath LPnL-Bar-ENull;
addpath others;
addpath ASPnL;
addpath RPnL;
addpath SRPnL;

%Type4 Room Camera Pose Estimation

genN = 2;
Data = LayoutDataGen(4,0,0,genN);
ses = 0;
sres = 0;
stes = 0;
sea = 0;
srea = 0;
stea = 0;
for j = 1:genN
    pointS = Data{j}.pointS;
    pointE = Data{j}.pointE;
    p1 = pointS(:,4:5);
    p2 = pointE(:,4:5);
    P1_w = pointS(:,1:3);
    P2_w = pointE(:,1:3);

    A = [180 0 320;0 180 320;0   0   1];
    for i = 1:length(p1)
        temp1 = inv(A)*[p1(i,:) 1].';
        temp1 = (temp1/temp1(3)).';
        ip1(i,:) = temp1(:,1:2);
        temp2 = inv(A)*[p2(i,:) 1].';
        temp2 = (temp2/temp2(3)).';
        ip2(i,:) = temp2(:,1:2);
    end

%     R_truth = Data{j}.R;
%     [R1, T1, erra] = WASPnLf3ls(ip1', ip2', P1_w', P2_w');
%     srea = srea + erra;
%     T_truth = Data{j}.T;
%     errT1 = cal_translation_err(T1,T_truth);
%     stea = stea + errT1;

        

    R_truth = Data{j}.R;
    T_truth = Data{j}.T;
    [R2, T2] = WSRPnLf3ls(ip1', ip2', P1_w', P2_w',A, p2);
%     sres = sres + errs;
%     errT2 = cal_translation_err(T2,T_truth);
%     stes = stes + errT2;
    
%     [R3, T3] = SRPnL(ip1', ip2', P1_w', P2_w');
end


% avgew = sew/genN;
% avgrew = srew/genN;
% avgtew = stew/genN;

% disp()
% 
% avgea = sea/genN;
avgrea = srea/genN;
% avgtea = stea/genN;


avgres = sres/genN;



% A = [180 0 320;0 180 320;0   0   1];




% [R, T, err] = LPnL_Bar_ENull(ip1', ip2', P1_w', P2_w');
% errorEpln=reprojection_error_usingRT(P1_w,p1,R,T,A);
% fprintf('error EPnL: %.3f\n',errorEpln);

% ASPnL WSPnLType0 WSPnLType0New WSPnLType0NewTest
% 
% [R1, T1, err1] = WASPnL(ip1', ip2', P1_w', P2_w');
% 
% [R2, T2, err2] = ASPnL(ip1', ip2', P1_w', P2_w');
% 
% 
% [T1R4, T1T4] = SRPnL(ip1', ip2', P1_w', P2_w');

% P1w = P1';
% P2w = [P2_w(1:4,:);P1w(9:12,:);P1w(10:12,:);P1w(9,:)];
% ip1  = [ip1;ip2(5:8,:)];
% ip2 = [ip2;ip2(6:8,:);ip2(5,:)];
% 
% [R3, T3, err3] = ASPnL(ip1', ip2', P1w', P2w');
% 
% 
% [R4, T4] = SRPnL(ip1', ip2', P1w', P2w');

% R_truth = [[ 0.96793213  0.24133259 -0.06975647]
%             [ 0.24898738 -0.95850424  0.13883408]
%             [-0.03335669 -0.15175045 -0.98785583]];
% 
% 
% errR1 = cal_rotation_err(R1,R_truth);
% 
% T_truth = [-149.11312026   71.04161729  190.57902817];
% 
% errT1 = cal_translation_err(T1,T_truth');
% 
% errR2 = cal_rotation_err(R2,R_truth);
% 
% errT2 = cal_translation_err(T2,T_truth');
% 
% errR3 = cal_rotation_err(R3,R_truth);
% 
% errT3 = cal_translation_err(T3,T_truth');
% 
% errR4 = cal_rotation_err(R4,R_truth);
% 
% errT4 = cal_translation_err(T4,T_truth');
% 

% [T1R3, T1T3,errs] = WSPnLType0New(ip1', ip2', P1_w', P2_w');
%compute error
% pw = P1';
% p  = [p1;p2(5:8,:)];
% errorWAspnl=reprojection_error_usingRT(pw,p,R1,T1,A);
% fprintf('error Aspnl: %.3f\n',errorWAspnl);
% errorAspnl=reprojection_error_usingRT(pw,p,R2,T2,A);
% fprintf('error Aspnl: %.3f\n',errorAspnl);
% 
% 
% [R2, T2] = SRPnL(ip1', ip2', P1_w', P2_w');

% 
% x3d_h = [P1_w;P2_w];
% x2d_h = [p1;p2];
% 
% 
% [REpnp,TEpnp,Xc,best_solution]=efficient_pnp(x3d_h,x2d_h,A)
% errorEpnp=reprojection_error_usingRT(x3d_h,x2d_h,REpnp,TEpnp,A);


