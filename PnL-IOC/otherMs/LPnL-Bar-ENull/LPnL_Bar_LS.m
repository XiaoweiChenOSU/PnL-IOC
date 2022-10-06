function [R_cw, T_cw, err_cw] = LPnL_Bar_LS(p1, p2, P1_w, P2_w)
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
	alp1_w = getAlpha(BP_w,P1_w);
	alp2_w = getAlpha(BP_w,P2_w);

	% normal of porjected lines
	nl = getProjNorm(p1,p2);

	% matrix
	M1 = kron([1 1 1 1], [nl nl]');
	M2 = kron([alp1_w alp2_w]', [1 1 1]);
	M = M1 .* M2;
	
	MTM = M' * M;
% 	[XV XD] = xeig(MTM);
    [XV XD] = eig(MTM);
	X = normBP(XV(:,1));
	
    BP_c = reshape(X,3,4);
	[R_cw T_cw] = getRT(BP_w.',BP_c.');
    
    err_cw = x_calErr(BP_w,M,R_cw,T_cw)
    
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


function err = x_calErr(BP_w,M,R,T)

    BP_c = R * BP_w + kron([1 1 1 1],T);
    y = M * BP_c(:);
    err = mean(y.^2);

	
