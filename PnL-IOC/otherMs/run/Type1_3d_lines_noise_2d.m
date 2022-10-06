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

% experimental parameters
noise =  [0,1,2,3,4,5,6,7,8,9,10];
% nl = [0,1,2,3,4,5,6,7,8,9,10];

% nl_3d = [1,2,3,4,5,6,7,8,9,10]/40;

nl_3d = [0,0,0,0,0,0,0,0,0,0];

nlsamples = [0.1 0.1 0.1 0.1 0.1 0.1 0.1 0.1 0.1 0.1]; %percentage of samples for each sigma
npt = 2;
nln_gen = npt/2;
isotropic = false;
num = 100;

% camera's parameters
width= 640;
height= 640;
% f= 800;
% K = [f 0 0
%      0 f 0
%      0 0 1];

avg_depth = 6;

f= 180;
K =  [180 0 320;0 180 320;0   0   1];


% compared methods
A= zeros(size(noise));
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
for n= 1:length(noise)
    
%     npt= npts(i);
%     nln = npts(i);
%     fprintf('nln = %d (num sg = %d ): ', nln, length(nl));

    no=n-1;
    rateOutlier=0;
    Data = LayoutDataGen(1,no,rateOutlier,num);

    nl = ones(1,10)*no;
   
    for k= 1:length(method_list)
        method_list(k).c = zeros(1,num);
        method_list(k).e = zeros(1,num);
        method_list(k).r = zeros(1,num);
        method_list(k).t = zeros(1,num);
    end
    
    %index_fail = [];
    index_fail = cell(1,length(name));
    
    for j= 1:num

        pointS = Data{j}.pointS;
        pointE = Data{j}.pointE;
        pointSNoNoise = Data{j}.pointSt;
        pointENoNoise = Data{j}.pointEt;
        cH = Data{j}.Cc(2);
        p1 = pointS(:,4:5);
        p2 = pointE(:,4:5);

        R = Data{j}.R;
        t = Data{j}.T;


        P1_w = [pointS(1,1:3);pointE(1,1:3)];
        p1_i = [pointS(1,4:5);pointE(1,4:5)];
      



        for i = 1:length(p1)
            temp1 = inv(K)*[p1(i,:) 1].';
            temp1 = (temp1/temp1(3)).';
            ip1(:,i) = temp1';
            temp2 = inv(K)*[p2(i,:) 1].';
            temp2 = (temp2/temp2(3)).';
            ip2(:,i) = temp2';
        end


        XXw = P1_w';


        for i = 1:2
            temp1 = inv(K)*[p1_i(i,:) 1].';
            temp1 = (temp1/temp1(3)).';
            Xc(:,i) = temp1';
        end


        % 3D covariances are zero
        Sigmas3D = zeros(3,3,npt);


        XXw_s = P1_w(1,:)';
        XXw_e = P1_w(2,:)';

        Sigmas3DLines = zeros(6,6,npt*2);

        xxn = p1_i';

        xx_s = p1_i(1,:)';
        xx_e = p1_i(2,:)';
        
                         
        % project points
        [v, Cu, cov] = projectPoints_Covs(xxn, Xc, f, nl, nlsamples, K);         
        
        % project lines
        [XXw_s, XXw_e, xx_s_n, xx_e_n, ll, sigmas2dlines] = projectLines_Covs(nl, nlsamples, nln_gen, xx_s, xx_e, XXw_s, XXw_e, f, K);
                
        %call p3p to get R_est, t_est
%         [R_est, t_est] = run_p3p(XXw, xxn, f, R, t);
%         if size(R_est, 1) < 3
%             continue;
%         end

        R_est = R;
        t_est = t;
            
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
        end

        showpercent(j,num);
    end
    fprintf('\n');
    
    % save result
    method_list = save_experiment(method_list, n, index_fail, num);
end

mkdir('results/');
exp_lbl = 'results/ordinary3Dlines';
save(exp_lbl, 'method_list', 'noise');
save_pref = 'ord3d_lines';
plot_lines(exp_lbl, save_pref);