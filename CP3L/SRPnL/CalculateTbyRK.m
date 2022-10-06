function Rt = CalculateTbyRK(p1, p2, W1, W2, C_truth, Rr,unkown)
 
  
    pn1 = [p1 p2];
    pn2 = [p2 p2(:,2:3) p2(:,1)];
    n_nc = xcross_mat(pn1,pn2);
    nc_nbar = xnorm(n_nc);
    P1N_w = [W1 W2];

    MatN = zeros(4, 3);       
    Rw_c = Rr;
    for i = 1:3
        nxi = nc_nbar(1,i);  nyi = nc_nbar(2,i);  nzi = nc_nbar(3,i);
        MatN(i, 1) = nxi;
        MatN(i, 2) = nyi;
        MatN(i, 3) = nzi; 
        MatN(i, 4) = (P1N_w(1,i)*(nxi*Rw_c(1,1)+nyi*Rw_c(2,1)+nzi*Rw_c(3,1))) + (P1N_w(2,i)*(nxi*Rw_c(1,2)+nyi*Rw_c(2,2)+nzi*Rw_c(3,2))) + (P1N_w(3,i)*(nxi*Rw_c(1,3)+nyi*Rw_c(2,3)+nzi*Rw_c(3,3)));      
    end      

    invR = inv(Rw_c);
    
    MatN(4,1) = invR(2,1);
    MatN(4,2) = invR(2,2);
    MatN(4,3) = invR(2,3);
    MatN(4,4) = C_truth(2);
    
%     MatN(4,1) = invR(1,1);
%     MatN(4,2) = invR(1,2);
%     MatN(4,3) = invR(1,3);
%     MatN(4,4) = C_truth(1);
%     
%     if unkown == 1
%         MatN(4,1) = invR(1,1);
%         MatN(4,2) = invR(1,2);
%         MatN(4,3) = invR(1,3);
%         MatN(4,4) = C_truth(1);
%     elseif unkown == 2
%         MatN(4,1) = invR(2,1);
%         MatN(4,2) = invR(2,2);
%         MatN(4,3) = invR(2,3);
%         MatN(4,4) = C_truth(2);
%     elseif unkown == 3
%         MatN(4,1) = invR(3,1);
%         MatN(4,2) = invR(3,2);
%         MatN(4,3) = invR(3,3);
%         MatN(4,4) = C_truth(3);
%     end

%     MatN(4,1) = invR(2,1);
%     MatN(4,2) = invR(2,2);
%     MatN(4,3) = invR(2,3);
% %     MatN(4,4) = C_truth;
%      MatN(4,4) = C_truth(2);
     
%     MatN(4,1) = invR(3,1);
%     MatN(4,2) = invR(3,2);
%     MatN(4,3) = invR(3,3);
% %     MatN(4,4) = C_truth;
%     MatN(4,4) = C_truth(3);

%     MatN(4,1) = invR(1,1);
%     MatN(4,2) = invR(1,2);
%     MatN(4,3) = invR(1,3);
%     MatN(4,4) = C_truth(1);

%     MatN(5,1) = invR(1,1);
%     MatN(5,2) = invR(1,2);
%     MatN(5,3) = invR(1,3);
%     MatN(5,4) = C_truth(1);
    
    
    [UMatN, SMatN, VMatN] = svd(MatN);
    vecN = VMatN(:,4);% the last column of Vmat;
    vecN = vecN/vecN(4); %the condition that the last element of vec should be 1.     
    Rt = vecN(1:3);

end
