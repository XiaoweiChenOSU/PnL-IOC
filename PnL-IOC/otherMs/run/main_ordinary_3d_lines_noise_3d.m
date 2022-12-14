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
nl = [1,2,3,4,5,6,7,8,9,10];
% nl = [1,1,1,1,1,1,1,1,1,1];
% nl = [0,0,0,0,0,0,0,0,0,0];

% nl_3d = [1,2,3,4,5,6,7,8,9,10]/40;

nl_3d = [0,0,0,0,0,0,0,0,0,0];
nlsamples = [0.1 0.1 0.1 0.1 0.1 0.1 0.1 0.1 0.1 0.1]; %percentage of samples for each sigma
npts = [10:10:80];
% npts = [4:1:10];
isotropic = false;
num = 400;

% camera's parameters
width= 640;
height= 480;
f= 800;
K = [f 0 0
     0 f 0
     0 0 1];


% compared methods
A= zeros(size(npts));
B= zeros(num,1);

name= {              'EPnPL',       'OPnPL',     'DLSLU',       'EPnPLU',       'DLSLU*',       'EPnPLU*',     'DLSU',     'EPnPU'};
fun= {                @EPnPLS_GN,    @OPnPL,      @dlsu_full, @epnpu_full,     @dlsu_full,     @epnpu_full,     @dlsu_full, @epnpu_full};
marker= {            '--',         '-',          '-',            '-',           '-.',            '-.',           '-.',       '-.'};
color= {             'c',          'b',          'g',            'r',           'g',            'r',           'm',        'k'};


method_list= struct('name', name, 'f', fun, 'mean_r', A, 'mean_t', A,...
                    'med_r', A, 'med_t', A, 'std_r', A, 'std_t', A, 'r', B, 't', B,...
                    'marker', marker, 'color', color, 'markerfacecolor', color);

% experiments
for i= 1:length(npts)
    
    npt= npts(i);
    nln = npts(i);
    fprintf('nln = %d (num sg = %d ): ', nln, length(nl));
   
     for k= 1:length(method_list)
        method_list(k).c = zeros(1,num);
        method_list(k).e = zeros(1,num);
        method_list(k).r = zeros(1,num);
        method_list(k).t = zeros(1,num);
    end
    
    %index_fail = [];
    index_fail = cell(1,length(name));
    
    for j= 1:num
        
        % generate 3d coordinates in camera space
        Xc = [xrand(1,npt,[-2 2]); xrand(1,npt,[-2 2]); xrand(1,npt,[4 8])];            
        
        nln_gen = 2*nln;
        Xcs = [xrand(1,nln_gen,[-2 2]); xrand(1,nln_gen,[-2 2]); xrand(1,nln_gen,[4 8])];
        Xce = [xrand(1,nln_gen,[-2 2]); xrand(1,nln_gen,[-2 2]); xrand(1,nln_gen,[4 8])];
        avg_depth = 6;
        
        t= mean(Xc,2);
        R= rodrigues(randn(3,1));
        XXw= inv(R)*(Xc-repmat(t,1,npt));
        
        % generate 3d points
        [XXw, Sigmas3D] = generate_3d_points_with_covs(XXw, nl_3d, nlsamples, isotropic);
        
        XXw_s = inv(R)*(Xcs-repmat(t,1,nln_gen));
        XXw_e = inv(R)*(Xce-repmat(t,1,nln_gen));
        
        % generate 3d lines
        [XXw_s, XXw_e, Sigmas3DLines] = generate_3d_lines_with_covs(XXw_s, XXw_e, npt, nl_3d, isotropic);
                        
        % project points
        [xxn, v, Cu, cov] = project_points_compute_covs(Xc, f, nl, nlsamples);         
        
        % project lines
        [XXw_s, XXw_e, xx_s_n, xx_e_n, ll, sigmas2dlines] = project_lines_with_covs(nl, nlsamples, nln, Xcs, Xce, XXw_s, XXw_e, f);
                
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
        end

        showpercent(j,num);
    end
    fprintf('\n');
    
    % save result
    method_list = save_experiment(method_list, i, index_fail, num);
end

mkdir('results/');
exp_lbl = 'results/ordinary3Dlines';
save(exp_lbl, 'method_list', 'npts');
save_pref = 'ord3d_lines';
plot_lines(exp_lbl, save_pref);