function aPoints = CalculateAbyR2(p1, p2, W1, W2, Rr)

    pn1 = [p1(:,2:5) p2(:,2:3) p2(:,2:2:5) p2(:,2:3) p1(:,3:4) p1(:,3:4)];
    pn2 = [p2(:,2:5) p2(:,4:5) p2(:,3:2:5) p2(:,5) p2(:,4) p2(:,4) p2(:,2) p2(:,5) p2(:,3)];
    nxs = pn1;
    nxe = pn2;
    n_nc = xcross_mat(nxs,nxe);
    nc_nbar = xnorm(n_nc);
    P1N_w = [W1(:,2:5) W2(:,2) W2(:,3) W2(:,2:2:5) W2(:,2:3) W1(:,3:4) W1(:,3:4)];
    P2N_w = [W2(:,2:5) W2(:,4) W2(:,5) W2(:,3:2:5) W2(:,5) W2(:,4) W2(:,4) W2(:,2) W2(:,5) W2(:,3)];
 
    
    MatN = zeros(14, 5);       
    Rw_c = Rr;

     
    for i = 1:4
        nxi = nc_nbar(1,i);  nyi = nc_nbar(2,i);  nzi = nc_nbar(3,i);
        if rem(i,2) == 1
            MatN(i, i) = -(nxi*Rw_c(1,3)+nyi*Rw_c(2,3)+nzi*Rw_c(3,3));
            MatN(i, 5) = ((P1N_w(1,i)-P2N_w(1,i))*(nxi*Rw_c(1,1)+nyi*Rw_c(2,1)+nzi*Rw_c(3,1)))+ ((P1N_w(2,i)-P2N_w(2,i))*(nxi*Rw_c(1,2)+nyi*Rw_c(2,2)+nzi*Rw_c(3,2))) + (P1N_w(3,i)*(nxi*Rw_c(1,3)+nyi*Rw_c(2,3)+nzi*Rw_c(3,3))); 
        else
            MatN(i, i) = -(nxi*Rw_c(1,2)+nyi*Rw_c(2,2)+nzi*Rw_c(3,2));
            MatN(i, 5) = ((P1N_w(1,i)-P2N_w(1,i))*(nxi*Rw_c(1,1)+nyi*Rw_c(2,1)+nzi*Rw_c(3,1)))+ (P1N_w(2,i)*(nxi*Rw_c(1,2)+nyi*Rw_c(2,2)+nzi*Rw_c(3,2))) + ((P1N_w(3,i)-P2N_w(3,i))*(nxi*Rw_c(1,3)+nyi*Rw_c(2,3)+nzi*Rw_c(3,3))); 
        end     
    end
    
    
    for i = 5:14
        nxi = nc_nbar(1,i);  nyi = nc_nbar(2,i);  nzi = nc_nbar(3,i);
        switch i
            case 5
                MatN(i, 1) = (nxi*Rw_c(1,3)+nyi*Rw_c(2,3)+nzi*Rw_c(3,3));
                MatN(i, 3) = -(nxi*Rw_c(1,3)+nyi*Rw_c(2,3)+nzi*Rw_c(3,3));
                MatN(i, 5) = ((P1N_w(1,i)-P2N_w(1,i))*(nxi*Rw_c(1,1)+nyi*Rw_c(2,1)+nzi*Rw_c(3,1))) + ((P1N_w(2,i)-P2N_w(2,i))*(nxi*Rw_c(1,2)+nyi*Rw_c(2,2)+nzi*Rw_c(3,2))); 
            case 6
                MatN(i, 2) = (nxi*Rw_c(1,2)+nyi*Rw_c(2,2)+nzi*Rw_c(3,2));
                MatN(i, 4) = -(nxi*Rw_c(1,2)+nyi*Rw_c(2,2)+nzi*Rw_c(3,2));
                MatN(i, 5) = ((P1N_w(1,i)-P2N_w(1,i))*(nxi*Rw_c(1,1)+nyi*Rw_c(2,1)+nzi*Rw_c(3,1))) + ((P1N_w(3,i)-P2N_w(3,i))*(nxi*Rw_c(1,3)+nyi*Rw_c(2,3)+nzi*Rw_c(3,3))); 
            case 7
                MatN(i, 1) = (nxi*Rw_c(1,3)+nyi*Rw_c(2,3)+nzi*Rw_c(3,3));
                MatN(i, 2) = -(nxi*Rw_c(1,2)+nyi*Rw_c(2,2)+nzi*Rw_c(3,2));
                MatN(i, 5) = ((P1N_w(1,i)-P2N_w(1,i))*(nxi*Rw_c(1,1)+nyi*Rw_c(2,1)+nzi*Rw_c(3,1))) + (P1N_w(2,i)*(nxi*Rw_c(1,2)+nyi*Rw_c(2,2)+nzi*Rw_c(3,2))) + (-P2N_w(3,i)*(nxi*Rw_c(1,3)+nyi*Rw_c(2,3)+nzi*Rw_c(3,3))); 
            case 8
                MatN(i, 3) = (nxi*Rw_c(1,3)+nyi*Rw_c(2,3)+nzi*Rw_c(3,3));
                MatN(i, 4) = -(nxi*Rw_c(1,2)+nyi*Rw_c(2,2)+nzi*Rw_c(3,2));
                MatN(i, 5) = ((P1N_w(1,i)-P2N_w(1,i))*(nxi*Rw_c(1,1)+nyi*Rw_c(2,1)+nzi*Rw_c(3,1))) + (P1N_w(2,i)*(nxi*Rw_c(1,2)+nyi*Rw_c(2,2)+nzi*Rw_c(3,2))) + (-P2N_w(3,i)*(nxi*Rw_c(1,3)+nyi*Rw_c(2,3)+nzi*Rw_c(3,3))); 
            case 9
                MatN(i, 1) = (nxi*Rw_c(1,3)+nyi*Rw_c(2,3)+nzi*Rw_c(3,3));
                MatN(i, 4) = -(nxi*Rw_c(1,2)+nyi*Rw_c(2,2)+nzi*Rw_c(3,2));
                MatN(i, 5) = ((P1N_w(1,i)-P2N_w(1,i))*(nxi*Rw_c(1,1)+nyi*Rw_c(2,1)+nzi*Rw_c(3,1))) + (P1N_w(2,i)*(nxi*Rw_c(1,2)+nyi*Rw_c(2,2)+nzi*Rw_c(3,2))) + (-P2N_w(3,i)*(nxi*Rw_c(1,3)+nyi*Rw_c(2,3)+nzi*Rw_c(3,3))); 
            case 10
                MatN(i, 2) = (nxi*Rw_c(1,2)+nyi*Rw_c(2,2)+nzi*Rw_c(3,2));
                MatN(i, 3) = -(nxi*Rw_c(1,3)+nyi*Rw_c(2,3)+nzi*Rw_c(3,3));
                MatN(i, 5) = ((P1N_w(1,i)-P2N_w(1,i))*(nxi*Rw_c(1,1)+nyi*Rw_c(2,1)+nzi*Rw_c(3,1))) + (-P2N_w(2,i)*(nxi*Rw_c(1,2)+nyi*Rw_c(2,2)+nzi*Rw_c(3,2))) + (P1N_w(3,i)*(nxi*Rw_c(1,3)+nyi*Rw_c(2,3)+nzi*Rw_c(3,3))); 
            case 11
                MatN(i, 3) = -(nxi*Rw_c(1,3)+nyi*Rw_c(2,3)+nzi*Rw_c(3,3));
                MatN(i, 5) = ((P1N_w(1,i)-P2N_w(1,i))*(nxi*Rw_c(1,1)+nyi*Rw_c(2,1)+nzi*Rw_c(3,1)))+ ((P1N_w(2,i)-P2N_w(2,i))*(nxi*Rw_c(1,2)+nyi*Rw_c(2,2)+nzi*Rw_c(3,2))) + (P1N_w(3,i)*(nxi*Rw_c(1,3)+nyi*Rw_c(2,3)+nzi*Rw_c(3,3))); 
            case 12
                MatN(i, 1) = -(nxi*Rw_c(1,3)+nyi*Rw_c(2,3)+nzi*Rw_c(3,3));
                MatN(i, 5) = ((P1N_w(1,i)-P2N_w(1,i))*(nxi*Rw_c(1,1)+nyi*Rw_c(2,1)+nzi*Rw_c(3,1)))+ ((P1N_w(2,i)-P2N_w(2,i))*(nxi*Rw_c(1,2)+nyi*Rw_c(2,2)+nzi*Rw_c(3,2))) + (P1N_w(3,i)*(nxi*Rw_c(1,3)+nyi*Rw_c(2,3)+nzi*Rw_c(3,3))); 
            case 13 
                MatN(i, 4) = -(nxi*Rw_c(1,2)+nyi*Rw_c(2,2)+nzi*Rw_c(3,2));
                MatN(i, 5) = ((P1N_w(1,i)-P2N_w(1,i))*(nxi*Rw_c(1,1)+nyi*Rw_c(2,1)+nzi*Rw_c(3,1)))+ (P1N_w(2,i)*(nxi*Rw_c(1,2)+nyi*Rw_c(2,2)+nzi*Rw_c(3,2))) + ((P1N_w(3,i)-P2N_w(3,i))*(nxi*Rw_c(1,3)+nyi*Rw_c(2,3)+nzi*Rw_c(3,3))); 
            otherwise
                MatN(i, 2) = -(nxi*Rw_c(1,2)+nyi*Rw_c(2,2)+nzi*Rw_c(3,2));
                MatN(i, 5) = ((P1N_w(1,i)-P2N_w(1,i))*(nxi*Rw_c(1,1)+nyi*Rw_c(2,1)+nzi*Rw_c(3,1)))+ (P1N_w(2,i)*(nxi*Rw_c(1,2)+nyi*Rw_c(2,2)+nzi*Rw_c(3,2))) + ((P1N_w(3,i)-P2N_w(3,i))*(nxi*Rw_c(1,3)+nyi*Rw_c(2,3)+nzi*Rw_c(3,3))); 
        end 
    end
    
    [UMatN, SMatN, VMatN] = svd(MatN);
    vecN = VMatN(:,5);% the last column of Vmat;
    vecN = vecN/vecN(5); %the condition that the last element of vec should be 1.     
    aPoints = vecN(1:4);
    
    
    
end
