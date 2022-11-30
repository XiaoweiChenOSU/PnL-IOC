function [R, T, err] = LPnL_Bar_ENullReal2(p1, p2, P1_w, P2_w,C_truth)
% the line is expressed by the start and end points
% inputs:
%	 p1: 2d projection of the start point
%	 p2: 2d projection of the end point
%	 P1_w: 3d coordinates of the start point in world frame
%	 P2_w: 3d coordinates of the end point in world frame
% outputs:
%	 R: estimated rotation
%	 T: estimated translation

n = length(p1);	

% get base points
ct_w = mean([P1_w P2_w],2);
BP_w = [eye(3) zeros(3,1)] + kron([1 1 1 1],ct_w);
% BP_w = [eye(3) zeros(3,1)];
alp1_w = getAlpha(BP_w,P1_w);
alp2_w = getAlpha(BP_w,P2_w);

% normal of porjected lines
nl = getProjNorm(p1,p2);

% matrix
M1 = kron([1 1 1 1], [nl nl]');
M2 = kron([alp1_w alp2_w]', [1 1 1]);
M = M1 .* M2;
	
% ====================== Similar to EPnP+GN solver ========================
global flagpath
if isempty(flagpath)
    addpath epnp;
    flagpath = 1;
end

%Compute kernel M
Km=kernel_noise(M,4); %in matlab we have directly the funcion km=null(M);
    
%1.-Solve assuming dim(ker(M))=1. X=[Km_end];------------------------------
X1=Km(:,end);
[X lm] = normBP(X1);
% Alph = [alp1_w alp2_w]';
% Xw =  [P1_w';P2_w'];
% [BP_c,Xc]=compute_norm_sign_scaling_factor(X1,BP_w,Alph,Xw);
BP_c = reshape(X,3,4);

[R, T] = getRT(BP_w.',BP_c.');

% P1_c = alp1_w' * [BP_c;ones(1,4)]';
% P2_c = alp2_w' * [BP_c;ones(1,4)]';
% P_c = [P1_c;P2_c;[BP_c;ones(1,4)]']
% P_w = [P1_w';P2_w';BP_w']
% [R T] = getRT(P_w(:,1:3),P_c(:,1:3));
% [R T] = getRT(Xw,Xc);
cT = -inv(R)*T;
if cT(1) < C_truth(1)-2 || cT(2) < C_truth(2)-2 || cT(3) < C_truth(3)-2 || cT(1) > C_truth(1)+2 || cT(2) > C_truth(2)+2 || cT(3) > C_truth(3)+2 
    err(1) = Inf;
else
    err(1)=x_calErr(BP_w,M,R,T);
end

% err(1)=x_calErr(BP_w,M,R,T);

sol(1).R=R;
sol(1).T=T;
sol(1).error=err(1);
sol(1).betas=[1];
sol(1).sc=1/lm;
sol(1).Kernel=X1;

%2.-Solve assuming dim(ker(M))=2------------------------------------------
Km1=Km(:,end-1);
Km2=Km(:,end);

%control points distance constraint
D=compute_constraint_distance_2param_6eq_3unk(Km1,Km2);
dsq=define_distances_btw_control_points();
betas_=inv(D'*D)*D'*dsq;
beta1=sqrt(abs(betas_(1)));
beta2=sqrt(abs(betas_(3)))*sign(betas_(2))*sign(betas_(1));
X2=beta1*Km1+beta2*Km2;

[X lm] = normBP(X2);
BP_c = reshape(X,3,4);
[R T] = getRT(BP_w.',BP_c.');
% P1_c = alp1_w' * [BP_c;ones(1,4)]';
% P2_c = alp2_w' * [BP_c;ones(1,4)]';
% P_c = [P1_c;P2_c;[BP_c;ones(1,4)]']
% P_w = [P1_w';P2_w';BP_w']
% [R T] = getRT(P_w(:,1:3),P_c(:,1:3));
cT = -inv(R)*T;
if cT(1) < C_truth(1)-2 || cT(2) < C_truth(2)-2 || cT(3) < C_truth(3)-2 || cT(1) > C_truth(1)+2 || cT(2) > C_truth(2)+2 || cT(3) > C_truth(3)+2 
    err(2) = Inf;
else
    err(2)=x_calErr(BP_w,M,R,T);
end

% err(2)=x_calErr(BP_w,M,R,T);

sol(2).R=R;
sol(2).T=T;
sol(2).error=err(2);
sol(2).betas=[beta1,beta2];
sol(2).sc=1/lm;
sol(2).Kernel=[Km1,Km2];

%3.-Solve assuming dim(ker(M))=3------------------------------------------
Km1=Km(:,end-2);
Km2=Km(:,end-1);
Km3=Km(:,end);

%control points distance constraint
D=compute_constraint_distance_3param_6eq_6unk(Km1,Km2,Km3);
dsq=define_distances_btw_control_points();
betas_=inv(D)*dsq;
beta1=sqrt(abs(betas_(1)));
beta2=sqrt(abs(betas_(4)))*sign(betas_(2))*sign(betas_(1));
beta3=sqrt(abs(betas_(6)))*sign(betas_(3))*sign(betas_(1));

X3=beta1*Km1+beta2*Km2+beta3*Km3;

[X lm] = normBP(X3);
BP_c = reshape(X,3,4);

[R T] = getRT(BP_w.',BP_c.');


% P1_c = alp1_w' * [BP_c;ones(1,4)]';
% P2_c = alp2_w' * [BP_c;ones(1,4)]';
% P_c = [P1_c;P2_c;[BP_c;ones(1,4)]']
% P_w = [P1_w';P2_w';BP_w']
% [R T] = getRT(P_w(:,1:3),P_c(:,1:3));
cT = -inv(R)*T;
if cT(1) < C_truth(1)-2 || cT(2) < C_truth(2)-2 || cT(3) < C_truth(3)-2 || cT(1) > C_truth(1)+2 || cT(2) > C_truth(2)+2 || cT(3) > C_truth(3)+2 
    err(3) = Inf;
else
    err(3)=x_calErr(BP_w,M,R,T);
end

% err(3)=x_calErr(BP_w,M,R,T);

sol(3).R=R;
sol(3).T=T;
sol(3).error=err(3);
sol(3).betas=[beta1,beta2,beta3];
sol(3).sc=1/lm;
sol(3).Kernel=[Km1,Km2,Km3];

%4.-Solve assuming dim(ker(M))=4------------------------------------------
Km1=Km(:,end-3);
Km2=Km(:,end-2);
Km3=Km(:,end-1);
Km4=Km(:,end);

D=compute_constraint_distance_orthog_4param_9eq_10unk(Km1,Km2,Km3,Km4);
dsq=define_distances_btw_control_points();
lastcolumn=[-dsq',0,0,0]';
D_=[D,lastcolumn];
Kd=null(D_);

P=compute_permutation_constraint4(Kd);
lambdas_=kernel_noise(P,1);
lambda(1)=sqrt(abs(lambdas_(1)));
lambda(2)=sqrt(abs(lambdas_(6)))*sign(lambdas_(2))*sign(lambdas_(1));
lambda(3)=sqrt(abs(lambdas_(10)))*sign(lambdas_(3))*sign(lambdas_(1));
lambda(4)=sqrt(abs(lambdas_(13)))*sign(lambdas_(4))*sign(lambdas_(1));
lambda(5)=sqrt(abs(lambdas_(15)))*sign(lambdas_(5))*sign(lambdas_(1));

betass_=lambda(1)*Kd(:,1)+lambda(2)*Kd(:,2)+lambda(3)*Kd(:,3)+lambda(4)*Kd(:,4)+lambda(5)*Kd(:,5);
beta1=sqrt(abs(betass_(1)));
beta2=sqrt(abs(betass_(5)))*sign(betass_(2));
beta3=sqrt(abs(betass_(8)))*sign(betass_(3));
beta4=sqrt(abs(betass_(10)))*sign(betass_(4));
X4=beta1*Km1+beta2*Km2+beta3*Km3+beta4*Km4;

[X lm] = normBP(X4);
BP_c = reshape(X,3,4);
[R T] = getRT(BP_w.',BP_c.');

% P1_c = alp1_w' * [BP_c;ones(1,4)]';
% P2_c = alp2_w' * [BP_c;ones(1,4)]';
% P_c = [P1_c;P2_c;[BP_c;ones(1,4)]']
% P_w = [P1_w';P2_w';BP_w']
% [R T] = getRT(P_w(:,1:3),P_c(:,1:3));
% cT = -inv(R)*T;
% if cT(1) < 0 || cT(2) < 0 || cT(3) < 0 || cT(1) > 256 || cT(2) > 256 || cT(3) > 256 
%     err(4) = Inf;
% else
%     err(4)=x_calErr(BP_w,M,R,T);
% end

err(4)=x_calErr(BP_w,M,R,T);

sol(4).R=R;
sol(4).T=T;
sol(4).error=err(4);
sol(4).betas=[beta1,beta2,beta3,beta4];
sol(4).sc=1/lm;
sol(4).Kernel=[Km1,Km2,Km3,Km4];
    
%5.-Gauss Newton Optimization------------------------------------------------------ 
[min_err,best_solution]=min(err);
err = min_err;
R=sol(best_solution).R;
T=sol(best_solution).T;
Betas=sol(best_solution).betas;
sc=sol(best_solution).sc;
 
if best_solution==1
    Betas=[0,0,0,Betas];
elseif best_solution==2
    Betas=[0,0,Betas];
elseif best_solution==3
    Betas=[0,Betas];
end

Km1=Km(:,end-3);
Km2=Km(:,end-2);
Km3=Km(:,end-1);
Km4=Km(:,end);
Kernel=[Km1,Km2,Km3,Km4];

%refine the solution iterating over the betas
Beta0=Betas/sc;
[R_opt,T_opt,err_opt,iter]=x_gauss_newton(Kernel,BP_w,Beta0,M);

%Just update R,T,Xc if Gauss Newton improves results (which is almost
%always)
% cT = -inv(R_opt)*T_opt;
% if cT(1) < C_truth(1)-1 || cT(2) < C_truth(2)-1 || cT(3) < C_truth(3)-1 || cT(1) > C_truth(1)+1 || cT(2) > C_truth(2)+1 || cT(3) > C_truth(3)+1 
%     if err_opt<min_err    
%         R=R_opt;
%         T=T_opt;
%         err = err_opt;
%     end
% end

if err_opt<min_err    
    R=R_opt;
    T=T_opt;
    err = err_opt;
end


opt.Beta0=Beta0;
opt.Kernel=Kernel;
opt.iter=iter;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function err = x_calErr(BP_w,M,R,T)

    BP_c = R * BP_w + kron([1 1 1 1],T);
    y = M * BP_c(:);
    err = mean(y.^2);
	
function [R_opt,T_opt,err_opt,iter] = x_gauss_newton(Km,BP_w,Beta0,M)

    n=size(Beta0,2);

    [Beta_opt,err,iter]=gauss_newton(Km,BP_w.',Beta0);

    X=zeros(12,1);
    for i=1:n
       X=X+Beta_opt(i)*Km(:,i); 
    end

    [X lm] = normBP(X);
    BP_c = reshape(X,3,4);
    [R_opt,T_opt] = getRT(BP_w.',BP_c.');
    err_opt=x_calErr(BP_w,M,R_opt,T_opt);
    
    
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [R, T]=getRT(wpts,cpts)
  
% This routine solves the exterior orientation problem for a point cloud
%  given in both camera and world coordinates. 
  
% wpts = 3D points in arbitrary reference frame
% cpts = 3D points in camera reference frame
  
n=size(wpts,1);
M=zeros(3);

ccent=mean(cpts);
wcent=mean(wpts);

for i=1:3
  cpts(:,i)=cpts(:,i)-ccent(i)*ones(n,1);
  wpts(:,i)=wpts(:,i)-wcent(i)*ones(n,1);
end
for i=1:n
   M=M+cpts(i,:)'*wpts(i,:);
end
[U S V]=svd(M);
R=U*V';
if det(R)<0
  R=-R;
end
T=ccent'-R*wcent';
