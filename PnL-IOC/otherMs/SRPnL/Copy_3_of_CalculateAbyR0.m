function aPoints = CalculateAbyR0(p1, p2, W1, W2, Rr)

    pn1 = [p1(:,5:8) p2(:,5:8) p2(:,5:6) p1(:,1:4)];
    pn2 = [p2(:,5:8) p2(:,6:8) p2(:,5) p2(:,7:8) p2(:,6) p2(:,5) p2(:,8) p2(:,7)];
    nxs = pn1;
    nxe = pn2;
    n_nc = xcross_mat(nxs,nxe);
    nc_nbar = xnorm(n_nc);
    P1N_w = [W1(:,5:8) W2(:,5:8) W2(:,5:6) W1(:,1:4)];
    P2N_w = [W2(:,5:8) W2(:,6:8) W2(:,5) W2(:,7:8) W2(:,6) W2(:,5) W2(:,8) W2(:,7)];

    MatN = zeros(28, 8);       
    Rw_c = Rr;

    for i = 1:4
        nxi = nc_nbar(1,i);  nyi = nc_nbar(2,i);  nzi = nc_nbar(3,i);
        MatN(i, 5) = nxi;
        MatN(i, 6) = nyi;
        MatN(i, 7) = nzi;
        MatN(i, 8) = P1N_w(1,i)*(nxi*Rw_c(1,1)+nyi*Rw_c(2,1)+nzi*Rw_c(3,1)) + P1N_w(2,i)*(nxi*Rw_c(1,2)+nyi*Rw_c(2,2)+nzi*Rw_c(3,2)) + P1N_w(3,i)*(nxi*Rw_c(1,3)+nyi*Rw_c(2,3)+nzi*Rw_c(3,3));      
    end
    
    for i = 5:10                  
        % apply the constraint scaleproduct(Pi^c, ni^c) = 0
        nxi = nc_nbar(1,i);  nyi = nc_nbar(2,i);  nzi = nc_nbar(3,i); 
        if i == 5 || i == 9
            MatN(i, 1) = (nxi*Rw_c(1,3)+nyi*Rw_c(2,3)+nzi*Rw_c(3,3));
        elseif i == 6 || i == 10
            MatN(i, 2) = (nxi*Rw_c(1,3)+nyi*Rw_c(2,3)+nzi*Rw_c(3,3));
        elseif i == 7
            MatN(i, 3) = (nxi*Rw_c(1,3)+nyi*Rw_c(2,3)+nzi*Rw_c(3,3));
        elseif i == 8
            MatN(i, 4) = (nxi*Rw_c(1,3)+nyi*Rw_c(2,3)+nzi*Rw_c(3,3));
        end
        MatN(i, 5) = nxi;
        MatN(i, 6) = nyi;
        MatN(i, 7) = nzi;
        MatN(i, 8) = P1N_w(1,i)*(nxi*Rw_c(1,1)+nyi*Rw_c(2,1)+nzi*Rw_c(3,1)) + P1N_w(2,i)*(nxi*Rw_c(1,2)+nyi*Rw_c(2,2)+nzi*Rw_c(3,2));      
    end
    
    for i = 11:14
        nxi = nc_nbar(1,i);  nyi = nc_nbar(2,i);  nzi = nc_nbar(3,i);
        MatN(i, 5) = nxi;
        MatN(i, 6) = nyi;
        MatN(i, 7) = nzi;
        MatN(i, 8) = P1N_w(1,i)*(nxi*Rw_c(1,1)+nyi*Rw_c(2,1)+nzi*Rw_c(3,1)) + P1N_w(2,i)*(nxi*Rw_c(1,2)+nyi*Rw_c(2,2)+nzi*Rw_c(3,2)) + P1N_w(3,i)*(nxi*Rw_c(1,3)+nyi*Rw_c(2,3)+nzi*Rw_c(3,3));      
    end


    for j = 1:14
        nxi = nc_nbar(1,j);  nyi = nc_nbar(2,j);  nzi = nc_nbar(3,j);
        Ln = j+14;
        if j == 1 || j == 8 || j ==12
            MatN(Ln, 1) = (nxi*Rw_c(1,3)+nyi*Rw_c(2,3)+nzi*Rw_c(3,3));
        elseif j == 2 || j == 5 || j ==11
            MatN(Ln, 2) = (nxi*Rw_c(1,3)+nyi*Rw_c(2,3)+nzi*Rw_c(3,3));
        elseif j == 3 || j == 6 || j == 9 || j == 14
            MatN(Ln, 3) = (nxi*Rw_c(1,3)+nyi*Rw_c(2,3)+nzi*Rw_c(3,3));
        elseif j == 4 || j == 7 || j == 10 || j == 13
            MatN(Ln, 4) = (nxi*Rw_c(1,3)+nyi*Rw_c(2,3)+nzi*Rw_c(3,3));
        end
        MatN(Ln, 5) = nxi;
        MatN(Ln, 6) = nyi;
        MatN(Ln, 7) = nzi;
        MatN(Ln, 8) = P2N_w(1,i)*(nxi*Rw_c(1,1)+nyi*Rw_c(2,1)+nzi*Rw_c(3,1)) + P2N_w(2,i)*(nxi*Rw_c(1,2)+nyi*Rw_c(2,2)+nzi*Rw_c(3,2));      
    end

    [UMatN, SMatN, VMatN] = svd(MatN);
    vecN = VMatN(:,8);% the last column of Vmat;
    vecN = vecN/vecN(8); %the condition that the last element of vec should be 1.     
    aPoints = vecN(1:4);    
end
