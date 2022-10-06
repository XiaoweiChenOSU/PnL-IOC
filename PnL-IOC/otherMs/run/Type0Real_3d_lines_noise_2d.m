%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This toolbox illustrates how to use the Uncertainty-aware PnPL
% algorithms described in:
%
%       A. Vakhitov, L. Ferraz, A. Agudo, F. Moreno-Noguer
%       Uncertainty-Aware Camera Pose Estimation from Points and Lines 
%       CVPR 2021
%
% 
% This program is free software: you can redistribute it and/or modify
% it under the terms of the version 3 of the GNU General Public License
% as published by the Free Software Foundation.
% 
% This program is distributed in the hope that it will be useful, but
% WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
% General Public License for more details.       
% You should have received a copy of the GNU General Public License
% along with this program. If not, see <http://www.gnu.org/licenses/>.
%
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% This script contains the 3D noise for points and lines experiments 
% from the main paper (Figure 3, bottom row)
% To reproduce, run the script

clear; clc;
IniToolbox;


load('type0GroundTruth1.mat');
[~,numImage] = size(groundTruth);


% experimental parameters
noise =  [0,1,2,3,4,5,6,7,8,9,10];
% nl = [0,1,2,3,4,5,6,7,8,9,10];

% nl_3d = [1,2,3,4,5,6,7,8,9,10]/40;
no = 1;
nl_3d = [0,0,0,0,0,0,0,0,0,0];

nlsamples = [0.1 0.1 0.1 0.1 0.1 0.1 0.1 0.1 0.1 0.1]; %percentage of samples for each sigma
npt = 4;
nln_gen = npt;
isotropic = false;
num = numImage;

% camera's parameters
width= 640;
height= 640;
% f= 800;
% K = [f 0 0
%      0 f 0
%      0 0 1];

avg_depth = 6;

% f= 180;
% K =  [180 0 320;0 180 320;0   0   1];


% compared methods
A= zeros(1,numImage);
B= zeros(num,1);

% name= {              'EPnPL',       'OPnPL',     'DLSLU',       'EPnPLU',       'DLSLU*',       'EPnPLU*',     'DLSU',     'EPnPU'};
% fun= {                @EPnPLS_GN,    @OPnPL,      @dlsu_full, @epnpu_full,     @dlsu_full,     @epnpu_full,     @dlsu_full, @epnpu_full};
% marker= {            '--',         '-',          '-',            '-',           '-.',            '-.',           '-.',       '-.'};
% color= {             'c',          'b',          'g',            'r',           'g',            'r',           'm',        'k'};


% 
% name= {  'DLSLU*',       'EPnPLU*'};
% fun= { @dlsu_full,     @epnpu_full};
% marker= {  '-.',            '-.'};
% color= {  'g',            'r'};


name= {  'DLSLU*'};
fun= { @dlsu_full};
marker= {  '-.'};
color= {  'g'};


method_list= struct('name', name, 'f', fun, 'mean_r', A, 'mean_t', A,...
                    'med_r', A, 'med_t', A, 'std_r', A, 'std_t', A, 'r', B, 't', B,...
                    'marker', marker, 'color', color, 'markerfacecolor', color);

% experiments
for j= 1:numImage
    
    R_truth = groundTruth(j).Rotation;
    T_truth = groundTruth(j).Translation;
    p = groundTruth(j).point;
    P = groundTruth(j).wpoint;
    K = groundTruth(j).intrinsics_matrix;
    Lheight = groundTruth(j).Lheight;
    Lwidth = groundTruth(j).Lwidth;

    f = K(1,1);


    offset = 0.5;
    mul = 1;

%     offset = 0;
%     mul = 0;

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

    nl = ones(1,10)*no;
   
    for k= 1:length(method_list)
        method_list(k).c = 0;
        method_list(k).e = 0;
        method_list(k).r = 0;
        method_list(k).t = 0;
    end
    
    %index_fail = [];
    index_fail = cell(1,length(name));
    


    R = R_truth;
    t = T_truth;


 



    for i = 1:length(p1)
        temp1 = inv(K)*[p1(i,:) 1].';
        temp1 = (temp1/temp1(3)).';
        ip1(:,i) = temp1';
        temp2 = inv(K)*[p2(i,:) 1].';
        temp2 = (temp2/temp2(3)).';
        ip2(:,i) = temp2';
    end


    XXw = P1_w(1:4,:)';


    for i = 1:4
        temp1 = inv(K)*[p1(i,:) 1].';
        temp1 = (temp1/temp1(3)).';
        Xc(:,i) = temp1';
    end


    % 3D covariances are zero
    Sigmas3D = zeros(3,3,npt);


    XXw_s = P1_w(1:4,:)';
    XXw_e = P2_w(1:4,:)';

    Sigmas3DLines = zeros(6,6,npt*2);

    xxn = p1(1:4,:)';

    xx_s = p1(1:4,:)';
    xx_e = p2(1:4,:)';
    
                     
    % project points
    [v, Cu, cov] = projectPoints_Covs(xxn, Xc, f, nl, nlsamples, K);         
    
    % project lines
    [XXw_s, XXw_e, xx_s_n, xx_e_n, ll, sigmas2dlines] = projectLines_Covs(nl, nlsamples, nln_gen, xx_s, xx_e, XXw_s, XXw_e, f, K);
            
    %call p3p to get R_est, t_est
    [R_est, t_est] = run_p3p(XXw, xxn, f, R, t);
    if size(R_est, 1) < 3
        continue;
    end
        
    % pose estimation
    for k= 1:length(method_list)            
        [is_fail, R1, t1, tcost] = run_pnpl_method(method_list(k), XXw, xxn, v, ...
                                                   f, Cu, cov, Sigmas3D, ...
                                                   avg_depth, R_est, t_est, ...
                                                   xx_s_n, xx_e_n, ...
                                                   XXw_s, XXw_e, ll, ...
                                                   Sigmas3DLines, sigmas2dlines);
        if (is_fail)
            index_fail{k} = [index_fail{k}, j];
            continue;
        end
        %choose the solution with smallest error 
        [error, y, best_id, ercorr] = choose_solution(R1, t1, R, t, XXw, Xc);
        
        method_list(k).c(j)= tcost * 1000;
        method_list(k).e(j)= ercorr;
        method_list(k).r(j)= y(1);            
        method_list(k).t(j)= y(2);

        showpercent(j,num);
        method_list = save_experiment(method_list, j, index_fail, num);
    end 
end
fprintf('\n');

for k= 1:length(method_list)
        method_list(k).meanR = mean(method_list(k).deleted_mean_r);
        method_list(k).meanT = mean(method_list(k).deleted_mean_t);
        method_list(k).medianR = mean(method_list(k).deleted_med_r);
        method_list(k).medianT = mean(method_list(k).deleted_med_t);
end


    
    % save result



% mkdir('results/');
% exp_lbl = 'results/ordinary3Dlines';
% save(exp_lbl, 'method_list', 'noise');
% save_pref = 'ord3d_lines';
% plot_lines(exp_lbl, save_pref);