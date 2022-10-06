function [R_F,t_F,is_fail] = dlsu_full_type0(noise,f,K,P1_w,P2_w,p1,p2,R,t)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
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

R_F = zeros(3,3);
t_F = zeros(3,1);

% experimental parameters

% nl_3d = [1,2,3,4,5,6,7,8,9,10]/40;

nl_3d = [0,0,0,0,0,0,0,0,0,0];

nlsamples = [0.1 0.1 0.1 0.1 0.1 0.1 0.1 0.1 0.1 0.1]; %percentage of samples for each sigma
npt = 4;
nln_gen = npt;
isotropic = false;

avg_depth = 6;

name= {  'DLSLU*'};
fun= { @dlsu_full};
method_list= struct('name', name, 'f', fun);



nl = ones(1,10)*noise;








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
    is_fail = true;
end
    
% pose estimation
[is_fail, R1, t1, tcost] = run_pnpl_method(method_list, XXw, xxn, v, ...
                                               f, Cu, cov, Sigmas3D, ...
                                               avg_depth, R_est, t_est, ...
                                               xx_s_n, xx_e_n, ...
                                               XXw_s, XXw_e, ll, ...
                                               Sigmas3DLines, sigmas2dlines);

if (is_fail)
    return;
end

[error, y, best_id, ercorr] = choose_solution(R1, t1, R, t, XXw, Xc);

R_F = R1(:,:,best_id);
t_F = t1(:,best_id);





