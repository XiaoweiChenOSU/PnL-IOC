function [R, T, err] = WASPnLf8ls(p1, p2, P1_w, P2_w,po1,IA)
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
    MatN = zeros(12, 5);

    % xiaowei
    %rotate all the vector including the by Rot.
    pos = Tsa;
    nLine = length(p1);
    p1 = [p1; ones(1,nLine)];
	p2 = [p2; ones(1,nLine)];
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
            MatN(2*i-6, 8) = (nxi * Rw_c(1,1) + nyi * Rw_c(2,1) + nzi * Rw_c(3,1))*(P1N_w(1,j)-P2N_w(1,j))+(nxi * Rw_c(1,2) + nyi * Rw_c(2,2) + nzi * Rw_c(3,2))*(P1N_w(2,j)-P2N_w(2,j));
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
    
    
    
    f = IA(1,1);
    u0 = IA(1,3);
    v0 = IA(2,3);
    


    
    for i = 1:1
        if i > 1
            j = i -1;
            MatN(2*i+10,5) = f;
            MatN(2*i+10,7) = u0-po1(j,1);
            MatN(2*i+10,8) = (f*Rw_c(1,1) + u0*Rw_c(3,1) - po1(j,1)*Rw_c(3,1))*P1_w(1,j)+(f*Rw_c(1,2) + u0*Rw_c(3,2) - po1(j,1)*Rw_c(3,2))*P1_w(2,j) + (f*Rw_c(1,3) + u0*Rw_c(3,3) - po1(j,1)*Rw_c(3,3))*P1_w(3,j);
        end
        if i < 5
            MatN(2*i+11,6) = f;
            MatN(2*i+11,7) = v0-po1(i,2);
            MatN(2*i+11,8) = (f*Rw_c(2,1) + v0*Rw_c(3,1) - po1(i,2)*Rw_c(3,1))*P1_w(1,i)+(f*Rw_c(2,2) + v0*Rw_c(3,2) - po1(i,2)*Rw_c(3,2))*P1_w(2,i) + (f*Rw_c(2,3) + v0*Rw_c(3,3) - po1(i,2)*Rw_c(3,3))*P1_w(3,i);
        end
    end
    
    
    
    %solve the linear system MatN * [costheta, sintheta, tx, ty, tz, 1]' = 0  using SVD,
    [UMatN, SMatN, VMatN] = svd(MatN);
    vecN = VMatN(:,8);% the last column of Vmat;
    vecN = vecN/vecN(8); %the condition that the last element of vec should be 1.      

%     A = MatN(:,1:7);
%     b = -MatN(:,8);
% %     lb = [P2_w(3,5);0;0;0;-256;-256;-256];
% %     ub = [P2_w(3,5);256;256;256;256;256;256];
%     
%     lb = [0;0;0;0;-256;-256;-256];
%     ub = [256;256;256;256;256;256;256];
%     
%     sol = lsqlin(A,b,[],[],[],[],lb,ub);
%     
%     
% 
%     Rr = Rsa;
%     
%     YP1 = IA*[Rr sol(5:7)]*[P2_w(:,1);1];
%     YP1 = YP1/YP1(3);
%     
%     YP2 = IA*[Rr sol(5:7)]*[P2_w(:,2);1];
%     YP2 = YP2/YP2(3);
%     
%     YP3 = IA*[Rr sol(5:7)]*[P2_w(:,3);1];
%     YP3 = YP3/YP3(3);
%     
%     YP4 = IA*[Rr sol(5:7)]*[P2_w(:,4);1];
%     YP4 = YP4/YP4(3);
%      
%     
%     P2_w(3,5) = sol(1);
% 
%     YP5 = IA*[Rr sol(5:7)]*[P2_w(:,5);1];
%     YP5 = YP5/YP5(3);
%      
% 
%     P2_w(3,6) = sol(2);
% 
%     YP6 = IA*[Rr sol(5:7)]*[P2_w(:,6);1];
%     YP6 = YP6/YP6(3);
%     
%     P2_w(3,7) = sol(3);
% 
%     YP7 = IA*[Rr sol(5:7)]*[P2_w(:,7);1];
%     YP7 = YP7/YP7(3);
%     
%     P2_w(3,8) = sol(4);
% 
%     YP8 = IA*[Rr sol(5:7)]*[P2_w(:,8);1];
%     YP8 = YP8/YP8(3);
%     
% 
%     XP1 = IA*[Rr vecN(5:7)]*[P2_w(:,1);1];
%     XP1 = XP1/XP1(3);
%     
%     XP2 = IA*[Rr vecN(5:7)]*[P2_w(:,2);1];
%     XP2 = XP2/XP2(3);
%     
%     XP3 = IA*[Rr vecN(5:7)]*[P2_w(:,3);1];
%     XP3 = XP3/XP3(3);
%     
%     XP4 = IA*[Rr vecN(5:7)]*[P2_w(:,4);1];
%     XP4 = XP4/XP4(3);
%      
%     
%     P2_w(3,5) = vecN(1);
% 
%     XP5 = IA*[Rr vecN(5:7)]*[P2_w(:,5);1];
%     XP5 = XP5/XP5(3);
%      
% 
%     P2_w(3,6) = vecN(2);
% 
%     XP6 = IA*[Rr vecN(5:7)]*[P2_w(:,6);1];
%     XP6 = XP6/XP6(3);
%     
%     P2_w(3,7) = vecN(3);
% 
%     XP7 = IA*[Rr vecN(5:7)]*[P2_w(:,7);1];
%     XP7 = XP7/XP7(3);
%     
%     P2_w(3,8) = vecN(4);
% 
%     XP8 = IA*[Rr vecN(5:7)]*[P2_w(:,8);1];
%     XP8 = XP8/XP8(3);
%     
    
    
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
    

	
    
    
    