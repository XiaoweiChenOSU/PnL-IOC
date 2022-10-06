close all;
clc;
clear;

addpath LPnL-Bar-ENull;
addpath others;
addpath RPnL;




%Type1 Room Camera Pose Estimation
% 
% T1p1 = [626 621;602 19];
% T1p2 = [108 621;402 19];
% % 
% % 
% T1P1_w = [256 0 256;256 256 266];
% T1P2_w = [256 0 476;256 256 351];

% T1p1 = [626 621;626 19;626 621;24 621];
% T1p2 = [24 621;24 19;626 19;24 19];
% 
% T1P1_w = [256 0 256;256 256 256;256 0 256;256 0 512];
% T1P2_w = [256 0 512;256 256 512;256 256 256;256 256 512];


% pointS(1,:) = [0, 120, 0, 142.68080919209623, 167.9639853803657];
% pointS(2,:) = [256, 120, 0, 497.31919080790374, 167.9639853803657];
% pointS(3,:) = [0, 240, 0, 145.49349646087646, 6.8006937226886635];
% pointS(4,:) = [256, 240, 0, 494.5065035391235, 6.8006937226886635];
% pointS(5,:) = [0, 120, 0, 142.68080919209623, 167.9639853803657];
% pointS(6,:) = [256, 120, 0, 497.31919080790374, 167.9639853803657];
% 
% pointE(1,:) = [0, 240, 0, 145.49349646087646, 6.8006937226886635];
% pointE(2,:) = [0, 120, 0, 142.68080919209623, 167.9639853803657];
% pointE(3,:) = [256, 240, 0, 494.5065035391235, 6.8006937226886635];
% pointE(4,:) = [256, 120, 0, 497.31919080790374, 167.9639853803657];
% pointE(5,:) = [256, 240, 0, 494.5065035391235, 6.8006937226886635];
% pointE(6,:) = [0, 240, 0, 145.49349646087646, 6.8006937226886635];

% 
% pointS(1,:) = [0, 130, 0, 149.48242397233435, 172.23735023714968];
% pointS(2,:) = [256, 100, 0, 522.6465334484185, 169.59660120688622];
% pointS(3,:) = [0, 250, 0, 133.1827708851854, 2.2338423727402414];
% pointS(4,:) = [256, 200, 0, 516.9274799911966, 6.611473700460635];
% pointS(5,:) = [0, 130, 0, 149.48242397233435, 172.23735023714968];
% pointS(6,:) = [256, 100, 0, 522.6465334484185, 169.59660120688622];
% 
% pointE(1,:) = [256, 100, 0, 522.6465334484185, 169.59660120688622];
% pointE(2,:) = [0, 250, 0, 133.1827708851854, 2.2338423727402414];
% pointE(3,:) = [256, 200, 0, 516.9274799911966, 6.611473700460635];
% pointE(4,:) = [0, 130, 0, 149.48242397233435, 172.23735023714968];
% pointE(5,:) = [0, 250, 0, 133.1827708851854, 2.2338423727402414];
% pointE(6,:) = [256, 200, 0, 516.9274799911966, 6.611473700460635];



T1p1 = [626 621;602 19];
T1p2 = [108 621;402 19];
% 
% 
T1P1_w = [256 0 256;256 256 266];
T1P2_w = [256 0 476;256 256 351];

% T1p1 = pointS(:,4:5);
% T1p2 = pointE(:,4:5);
% T1P1_w = pointS(:,1:3);
% T1P2_w= pointE(:,1:3);


% n_nc = xcross_mat(T1p1,T1p2);
% nc_nbar = xnorm(n_nc);
% 
% 
% for i = 1:3
%     nxi = nc_nbar(1,i);  nyi = nc_nbar(2,i);  nzi = nc_nbar(3,i);
%     if i == 1
%         MatN(i, i) = (nxi*Rw_c(1,1)+nyi*Rw_c(2,1)+nzi*Rw_c(3,1))/2;
%         MatN(i, 7) = (P1N_w(1,i)*(nxi*Rw_c(1,1)+nyi*Rw_c(2,1)+nzi*Rw_c(3,1))/2)+ ((P1N_w(2,i)+P2N_w(2,i))*(nxi*Rw_c(1,2)+nyi*Rw_c(2,2)+nzi*Rw_c(3,2))/2) + ((P1N_w(3,i)+P2N_w(3,i))*(nxi*Rw_c(1,3)+nyi*Rw_c(2,3)+nzi*Rw_c(3,3))/2);
%     elseif i == 2
%         MatN(i, i) = (nxi*Rw_c(1,2)+nyi*Rw_c(2,2)+nzi*Rw_c(3,2))/2;
%         MatN(i, 7) = ((P1N_w(1,i)+P2N_w(1,i))*(nxi*Rw_c(1,1)+nyi*Rw_c(2,1)+nzi*Rw_c(3,1))/2) + (P1N_w(2,i)*(nxi*Rw_c(1,2)+nyi*Rw_c(2,2)+nzi*Rw_c(3,2))/2) + ((P1N_w(3,i)+P2N_w(3,i))*(nxi*Rw_c(1,3)+nyi*Rw_c(2,3)+nzi*Rw_c(3,3))/2);      
%     elseif i == 3
%         MatN(i, i) = (nxi*Rw_c(1,3)+nyi*Rw_c(2,3)+nzi*Rw_c(3,3))/2;
%         MatN(i, 7) = ((P1N_w(1,i)+P2N_w(1,i))*(nxi*Rw_c(1,1)+nyi*Rw_c(2,1)+nzi*Rw_c(3,1))/2) + ((P1N_w(2,i)+P2N_w(2,i))*(nxi*Rw_c(1,2)+nyi*Rw_c(2,2)+nzi*Rw_c(3,2))/2) + (P1N_w(3,i)*(nxi*Rw_c(1,3)+nyi*Rw_c(2,3)+nzi*Rw_c(3,3))/2);      
%     end
%     MatN(i, 4) = nxi;
%     MatN(i, 5) = nyi;
%     MatN(i, 6) = nzi; 
% end
% 
% for i = 4:6                     
%     % apply the constraint scaleproduct(Pi^c, ni^c) = 0
%     nxi = nc_nbar(1,i);  nyi = nc_nbar(2,i);  nzi = nc_nbar(3,i); 
%     if i == 4
%         MatN(4, i-3) = (nxi*Rw_c(1,1)+nyi*Rw_c(2,1)+nzi*Rw_c(3,1))/2;
%         MatN(4, i-2) = (nxi*Rw_c(1,2)+nyi*Rw_c(2,2)+nzi*Rw_c(3,2))/2;
%         MatN(4, 7) = (P2N_w(1,i)*(nxi*Rw_c(1,1)+nyi*Rw_c(2,1)+nzi*Rw_c(3,1))/2)+ (P1N_w(2,i)*(nxi*Rw_c(1,2)+nyi*Rw_c(2,2)+nzi*Rw_c(3,2))/2) + ((P1N_w(3,i)+P2N_w(3,i))*(nxi*Rw_c(1,3)+nyi*Rw_c(2,3)+nzi*Rw_c(3,3))/2);    
%     elseif i == 5
%         MatN(5, i-3) = (nxi*Rw_c(1,2)+nyi*Rw_c(2,2)+nzi*Rw_c(3,2))/2;
%         MatN(5, i-2) = (nxi*Rw_c(1,3)+nyi*Rw_c(2,3)+nzi*Rw_c(3,3))/2;
%         MatN(5, 7) = ((P1N_w(1,i)+P2N_w(1,i))*(nxi*Rw_c(1,1)+nyi*Rw_c(2,1)+nzi*Rw_c(3,1))/2)+ (P2N_w(2,i)*(nxi*Rw_c(1,2)+nyi*Rw_c(2,2)+nzi*Rw_c(3,2))/2) + (P1N_w(3,i)*(nxi*Rw_c(1,3)+nyi*Rw_c(2,3)+nzi*Rw_c(3,3))/2);    
%     elseif i == 6
%         MatN(6, 1) = (nxi*Rw_c(1,1)+nyi*Rw_c(2,1)+nzi*Rw_c(3,1))/2;
%         MatN(6, i-3) = (nxi*Rw_c(1,3)+nyi*Rw_c(2,3)+nzi*Rw_c(3,3))/2;
%         MatN(6, 7) = (P1N_w(1,i)*(nxi*Rw_c(1,1)+nyi*Rw_c(2,1)+nzi*Rw_c(3,1))/2)+ ((P1N_w(2,i)+P2N_w(2,i))*(nxi*Rw_c(1,2)+nyi*Rw_c(2,2)+nzi*Rw_c(3,2))/2) + (P2N_w(3,i)*(nxi*Rw_c(1,3)+nyi*Rw_c(2,3)+nzi*Rw_c(3,3))/2);                   
%     end
%     MatN(i, 4) = nxi;
%     MatN(i, 5) = nyi;
%     MatN(i, 6) = nzi;
% end



T1A = [180 0 320;0 180 320;0 0 1];

for i = 1:length(T1p1)
    temp1 = inv(T1A)*[T1p1(i,:) 1].';
    temp1 = (temp1/temp1(3)).';
    iT1p1(i,:) = temp1(:,1:2);
    temp2 = inv(T1A)*[T1p2(i,:) 1].';
    temp2 = (temp2/temp2(3)).';
    iT1p2(i,:) = temp2(:,1:2);
end


% [T1R, T1T, T1err] = LPnL_Bar_ENull(iT1p1', iT1p2', T1P1_w', T1P2_w');
% errorEpln=reprojection_error_usingRT(T1P1_w,T1p1,T1R,T1T,T1A);
% fprintf('error EPnL: %.3f\n',errorEpln);
% [T1R1, T1T1,errASPnL] = ASPnL(iT1p1', iT1p2', T1P1_w', T1P2_w');
% %compute error
% errorAspnl=reprojection_error_usingRT(T1P1_w,T1p1,T1R1,T1T1,T1A);
% fprintf('error Aspnl: %.3f\n',errorAspnl);
% 
% 
% [T1R2, T1T2,errorRPnL] = RPnL(iT1p1', iT1p2', T1P1_w', T1P2_w');


[T1R3, T1T3,err] = SRPnL(iT1p1', iT1p2', T1P1_w', T1P2_w');