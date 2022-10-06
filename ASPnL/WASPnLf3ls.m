function [R, T, err] = WASPnLf3ls(p1, p2, P1_w, P2_w)
% the line is expressed by the start and end points
% inputs:
%	 p1: 2d projection of the start point
%	 p2: 2d projection of the end point
%	 P1_w: 3d coordinates of the start point in world frame
%	 P2_w: 3d coordinates of the end point in world frame
% outputs:
%	 R: estimated rotation
%	 T: estimated translation

% 	nLine = length(p1);
% 	p1 = [p1; ones(1,nLine)];
% 	p2 = [p2; ones(1,nLine)];
    
    
    [Rsa, Tsa, errsa] = ASPnL(p1, p2, P1_w, P2_w);
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%        
    %According to the fact that n_i^C should be orthogonal to Pi^c and Vi^c, we 
    %have: scaleproduct(Vi^c, ni^c) = 0  and scaleproduct(Pi^c, ni^c) = 0.
    %where Vi^c = Rwc * Vi^w,  Pi^c = Rwc *(Pi^w - pos_cw) = Rwc * Pi^w - pos;
    %Using the above two constraints to construct linear equation system Mat about 
    %For type0,there will be four points which only have one unknown
    %axis, choose z as unknown axises, then the linear equation system Mat abou
    %[z1, z2, z3, z4, tx, ty, tz, 1].
    MatN = zeros(12, 8);

    % xiaowei
    %rotate all the vector including the by Rot.
    pos = Tsa;
    pn1 = [p1 p2(:,5:8)];
    pn2 = [p2 p2(:,6:8) p2(:,5)];
    nxs = pn1;
    nxe = pn2;
    n_nc = xcross_mat(nxs,nxe);
    nc_nbar = xnorm(n_nc);


    P1N_w = [P1_w P2_w(:,5:8)];
    P2N_w = [P2_w P2_w(:,6:8) P2_w(:,5)];
    Rw_c = Rsa;
    for i = 1:4
        j1 = i + 4;
        nxi = nc_nbar(1,j1);  nyi = nc_nbar(2,j1);  nzi = nc_nbar(3,j1);
        MatN(i, i) = (nxi*Rw_c(1,3)+nyi*Rw_c(2,3)+nzi*Rw_c(3,3))/2;
        MatN(i, 5) = nxi;
        MatN(i, 6) = nyi;
        MatN(i, 7) = nzi;
        MatN(i, 8) = ((P1N_w(1,j1)+P2N_w(1,j1))*(nxi*Rw_c(1,1)+nyi*Rw_c(2,1)+nzi*Rw_c(3,1))/2)+ ((P1N_w(2,j1)+P2N_w(2,j1))*(nxi*Rw_c(1,2)+nyi*Rw_c(2,2)+nzi*Rw_c(3,2))/2) + P1N_w(3,j1)*(nxi*Rw_c(1,3)+nyi*Rw_c(2,3)+nzi*Rw_c(3,3))/2;
    end
    
    

    for i = 5:9          
        % apply the constraint scaleproduct(Vi^c, ni^c) = 0
        if i>5 %because that if i=1, scaleproduct(Vi^c, ni^c) always be 0
            j = i + 3;
            nxi = nc_nbar(1,j);  nyi = nc_nbar(2,j);  nzi = nc_nbar(3,j);
            MatN(2*i-6, i-5) = nxi * Rw_c(1,3) + nyi * Rw_c(2,3) + nzi * Rw_c(3,3);
            if i-4 > 4
                MatN(2*i-6, 1) = -(nxi * Rw_c(1,3) + nyi * Rw_c(2,3) + nzi * Rw_c(3,3));
            else
                MatN(2*i-6, i-4) = -(nxi * Rw_c(1,3) + nyi * Rw_c(2,3) + nzi * Rw_c(3,3));
            end 
            MatN(2*i-6, 5) = (nxi * Rw_c(1,1) + nyi * Rw_c(2,1) + nzi * Rw_c(3,1))*(P1N_w(1,j)-P2N_w(1,j))+(nxi * Rw_c(1,2) + nyi * Rw_c(2,2) + nzi * Rw_c(3,2))*(P1N_w(2,j)-P2N_w(2,j));
        end
        % apply the constraint scaleproduct(Pi^c, ni^c) = 0
        if i < 9
            j = i + 4;
            nxi = nc_nbar(1,j);  nyi = nc_nbar(2,j);  nzi = nc_nbar(3,j); 
            MatN(2*i-5, i-4) = (nxi*Rw_c(1,3)+nyi*Rw_c(2,3)+nzi*Rw_c(3,3))/2;
            if i-3 > 4
                MatN(2*i-5, 1) = (nxi*Rw_c(1,3)+nyi*Rw_c(2,3)+nzi*Rw_c(3,3))/2;
            else
                MatN(2*i-5, i-3) = (nxi*Rw_c(1,3)+nyi*Rw_c(2,3)+nzi*Rw_c(3,3))/2;
            end
            MatN(2*i-5, 5) = nxi;
            MatN(2*i-5, 6) = nyi;
            MatN(2*i-5, 7) = nzi;
            MatN(2*i-5, 8) = ((P1N_w(1,j)+P2N_w(1,j))*(nxi*Rw_c(1,1)+nyi*Rw_c(2,1)+nzi*Rw_c(3,1))/2)+ ((P1N_w(2,j)+P2N_w(2,j))*(nxi*Rw_c(1,2)+nyi*Rw_c(2,2)+nzi*Rw_c(3,2))/2);    
        end
    end   
    %solve the linear system MatN * [costheta, sintheta, tx, ty, tz, 1]' = 0  using SVD,
    [UMatN, SMatN, VMatN] = svd(MatN);
    vecN = VMatN(:,5);% the last column of Vmat;
    vecN = vecN/vecN(5); %the condition that the last element of vec should be 1.      

    
    P1N_w(3,9:12) = vecN(1:4);
    P2N_w(3,5:8) = vecN(1:4);
    P2N_w(3,9:11) = vecN(2:4);
    P2N_w(3,12) = vecN(1);
    
    pn1 = pn1(1:2,:);
    pn2 = pn2(1:2,:);
    
    [Rea, Tea, errea] = ASPnL(pn1, pn2, P1N_w, P2N_w);
    
    if abs(errea) < abs(errsa)  
        R = Rea;
        T = Tea;
        err = errea;
    else
        R = Rsa;
        T = Tsa;
        err = errsa;
    end
end
    

	
    
    
    