function [Rt,aPoints] = RefineEquACopy(p1, p2, W1, W2, Rr)

    pn1 = [p1 p2(:,5:8) p2(:,5:6) p1(:,1:4)];
    pn2 = [p2 p2(:,6:8) p2(:,5) p2(:,7:8) p2(:,6) p2(:,5) p2(:,8) p2(:,7)];
    nxs = pn1;
    nxe = pn2;
    n_nc = xcross_mat(nxs,nxe);
    nc_nbar = xnorm(n_nc);
    P1N_w = [W1 W2(:,5:8) W2(:,5:6) W1(:,1:4)];
    P2N_w = [W2 W2(:,6:8) W2(:,5) W2(:,7:8) W2(:,6) W2(:,5) W2(:,8) W2(:,7)];

    MatN = zeros(18, 8);       
    Rw_c = Rr;

    for i = 1:4
        nxi = nc_nbar(1,i);  nyi = nc_nbar(2,i);  nzi = nc_nbar(3,i);
        MatN(i, 5) = nxi;
        MatN(i, 6) = nyi;
        MatN(i, 7) = nzi;
        MatN(i, 8) = ((P1N_w(1,i)+P2N_w(1,i))*(nxi*Rw_c(1,1)+nyi*Rw_c(2,1)+nzi*Rw_c(3,1))/2) + ((P1N_w(2,i)+P2N_w(2,i))*(nxi*Rw_c(1,2)+nyi*Rw_c(2,2)+nzi*Rw_c(3,2))/2) + ((P1N_w(3,i)+P2N_w(3,i))*(nxi*Rw_c(1,3)+nyi*Rw_c(2,3)+nzi*Rw_c(3,3))/2);      
    end

    for i = 5:12                   
        % apply the constraint scaleproduct(Pi^c, ni^c) = 0
        nxi = nc_nbar(1,i);  nyi = nc_nbar(2,i);  nzi = nc_nbar(3,i); 
        if i < 9
           MatN(i, i-4) = (nxi*Rw_c(1,3)+nyi*Rw_c(2,3)+nzi*Rw_c(3,3))/2;
           MatN(i, 8) = ((P1N_w(1,i)+P2N_w(1,i))*(nxi*Rw_c(1,1)+nyi*Rw_c(2,1)+nzi*Rw_c(3,1))/2)+ ((P1N_w(2,i)+P2N_w(2,i))*(nxi*Rw_c(1,2)+nyi*Rw_c(2,2)+nzi*Rw_c(3,2))/2) + (P1N_w(3,i)*(nxi*Rw_c(1,3)+nyi*Rw_c(2,3)+nzi*Rw_c(3,3))/2); 
        else
           if i == 12
               MatN(i, i-8) = (nxi*Rw_c(1,3)+nyi*Rw_c(2,3)+nzi*Rw_c(3,3))/2;
               MatN(i, 1) = (nxi*Rw_c(1,3)+nyi*Rw_c(2,3)+nzi*Rw_c(3,3))/2;    
           else
               MatN(i, i-8) = (nxi*Rw_c(1,3)+nyi*Rw_c(2,3)+nzi*Rw_c(3,3))/2;
               MatN(i, i-7) = (nxi*Rw_c(1,3)+nyi*Rw_c(2,3)+nzi*Rw_c(3,3))/2;    
           end
           MatN(i, 8) = ((P1N_w(1,i)+P2N_w(1,i))*(nxi*Rw_c(1,1)+nyi*Rw_c(2,1)+nzi*Rw_c(3,1))/2)+ ((P1N_w(2,i)+P2N_w(2,i))*(nxi*Rw_c(1,2)+nyi*Rw_c(2,2)+nzi*Rw_c(3,2))/2);             
        end 
        MatN(i, 5) = nxi;
        MatN(i, 6) = nyi;
        MatN(i, 7) = nzi;        
    end
    
    for i = 13:14                   
        % apply the constraint scaleproduct(Pi^c, ni^c) = 0
        nxi = nc_nbar(1,i);  nyi = nc_nbar(2,i);  nzi = nc_nbar(3,i); 
        if i == 13
           MatN(i, 1) = (nxi*Rw_c(1,3)+nyi*Rw_c(2,3)+nzi*Rw_c(3,3))/2;
           MatN(i, 3) = (nxi*Rw_c(1,3)+nyi*Rw_c(2,3)+nzi*Rw_c(3,3))/2;    
        else
           MatN(i, 2) = (nxi*Rw_c(1,3)+nyi*Rw_c(2,3)+nzi*Rw_c(3,3))/2;
           MatN(i, 4) = (nxi*Rw_c(1,3)+nyi*Rw_c(2,3)+nzi*Rw_c(3,3))/2;    
        end
        MatN(i, 5) = nxi;
        MatN(i, 6) = nyi;
        MatN(i, 7) = nzi; 
        MatN(i, 8) = ((P1N_w(1,i)+P2N_w(1,i))*(nxi*Rw_c(1,1)+nyi*Rw_c(2,1)+nzi*Rw_c(3,1))/2)+ ((P1N_w(2,i)+P2N_w(2,i))*(nxi*Rw_c(1,2)+nyi*Rw_c(2,2)+nzi*Rw_c(3,2))/2);                    
    end
    
    for i = 15:18                   
        % apply the constraint scaleproduct(Pi^c, ni^c) = 0
        nxi = nc_nbar(1,i);  nyi = nc_nbar(2,i);  nzi = nc_nbar(3,i); 
        if i == 15
           MatN(i, 2) = (nxi*Rw_c(1,3)+nyi*Rw_c(2,3)+nzi*Rw_c(3,3))/2;
        elseif i == 16
           MatN(i, 1) = (nxi*Rw_c(1,3)+nyi*Rw_c(2,3)+nzi*Rw_c(3,3))/2;
        elseif i == 17
           MatN(i, 4) = (nxi*Rw_c(1,3)+nyi*Rw_c(2,3)+nzi*Rw_c(3,3))/2;
        else
           MatN(i, 3) = (nxi*Rw_c(1,3)+nyi*Rw_c(2,3)+nzi*Rw_c(3,3))/2; 
        end
        MatN(i, 5) = nxi;
        MatN(i, 6) = nyi;
        MatN(i, 7) = nzi; 
        MatN(i, 8) = ((P1N_w(1,i)+P2N_w(1,i))*(nxi*Rw_c(1,1)+nyi*Rw_c(2,1)+nzi*Rw_c(3,1))/2)+ ((P1N_w(2,i)+P2N_w(2,i))*(nxi*Rw_c(1,2)+nyi*Rw_c(2,2)+nzi*Rw_c(3,2))/2) + (P1N_w(3,i)*(nxi*Rw_c(1,3)+nyi*Rw_c(2,3)+nzi*Rw_c(3,3))/2); 
    end

    [UMatN, SMatN, VMatN] = svd(MatN);
    vecN = VMatN(:,8);% the last column of Vmat;
    vecN = vecN/vecN(8); %the condition that the last element of vec should be 1.     
    Rt = vecN(5:7);
    aPoints = vecN(1:4);
    
    
    
end
