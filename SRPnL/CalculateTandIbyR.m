function [Rt,aPoints] = CalculateTandIbyR(p1, p2, W1, W2, cH, Rr,unkown,IA)
 
  
    pn1 = [p1; p1(1,:); p2(1,:); p1(1,:); p1(2,:)];
    pn2 = [p2; p1(2,:); p2(2,:); p2(2,:); p2(1,:)];
    pn1 = [pn1'; ones(1,6)];
    pn2 = [pn2'; ones(1,6)];
    nxs = pn1;
    nxe = pn2;
    n_nc = xcross_mat(nxs,nxe);
    nc_nbar = xnorm(n_nc);
    P1N_w = [W1 W1(:,1) W2(:,1) W1(:,1) W1(:,2)];
    P2N_w = [W2 W1(:,2) W2(:,2) W2(:,2) W2(:,1)];
    p2d = [p1(1,:);p2(1,:);p1(2,:);p2(2,:)];
    P3d = [W1(:,1) W2(:,1) W1(:,2) W2(:,2)];
    

    MatN = zeros(20, 8);       
    Rw_c = Rr;
    [~,n] = size(nc_nbar);
    
    if unkown == 1
       
        for i = 1:n
            nxi = nc_nbar(1,i);  nyi = nc_nbar(2,i);  nzi = nc_nbar(3,i);
            if i == 1 || i==3 || i==5
                MatN(i, 1) = nxi*Rw_c(1,1)+nyi*Rw_c(2,1)+nzi*Rw_c(3,1);
                MatN(i, 8) = P1N_w(2,i)*(nxi*Rw_c(1,2)+nyi*Rw_c(2,2)+nzi*Rw_c(3,2)) + P1N_w(3,i)*(nxi*Rw_c(1,3)+nyi*Rw_c(2,3)+nzi*Rw_c(3,3));
            elseif i == 2 || i == 6
                MatN(i, 3) = nxi*Rw_c(1,1)+nyi*Rw_c(2,1)+nzi*Rw_c(3,1);
                MatN(i, 8) = P1N_w(2,i)*(nxi*Rw_c(1,2)+nyi*Rw_c(2,2)+nzi*Rw_c(3,2)) + P1N_w(3,i)*(nxi*Rw_c(1,3)+nyi*Rw_c(2,3)+nzi*Rw_c(3,3));
            elseif i == 4
                MatN(i, 2) = nxi*Rw_c(1,1)+nyi*Rw_c(2,1)+nzi*Rw_c(3,1);
                MatN(i, 8) = P1N_w(2,i)*(nxi*Rw_c(1,2)+nyi*Rw_c(2,2)+nzi*Rw_c(3,2)) + P1N_w(3,i)*(nxi*Rw_c(1,3)+nyi*Rw_c(2,3)+nzi*Rw_c(3,3));
            end
            MatN(i, 5) = nxi;
            MatN(i, 6) = nyi;
            MatN(i, 7) = nzi; 
        end

        for j = 1:n                     
            % apply the constraint scaleproduct(Pi^c, ni^c) = 0
            nxi = nc_nbar(1,j);  nyi = nc_nbar(2,j);  nzi = nc_nbar(3,j); 
            if j == 1 || j ==6
                MatN(j+6, 2) = nxi*Rw_c(1,1)+nyi*Rw_c(2,1)+nzi*Rw_c(3,1);
                MatN(j+6, 8) = P2N_w(2,i)*(nxi*Rw_c(1,2)+nyi*Rw_c(2,2)+nzi*Rw_c(3,2)) + P2N_w(3,i)*(nxi*Rw_c(1,3)+nyi*Rw_c(2,3)+nzi*Rw_c(3,3));
            elseif j == 2 || j ==4 || j == 5
                MatN(j+6, 4) = nxi*Rw_c(1,1)+nyi*Rw_c(2,1)+nzi*Rw_c(3,1);
                MatN(j+6, 8) = P2N_w(2,i)*(nxi*Rw_c(1,2)+nyi*Rw_c(2,2)+nzi*Rw_c(3,2)) + P2N_w(3,i)*(nxi*Rw_c(1,3)+nyi*Rw_c(2,3)+nzi*Rw_c(3,3));
            elseif i == 3
                MatN(j+6, 3) = nxi*Rw_c(1,1)+nyi*Rw_c(2,1)+nzi*Rw_c(3,1);
                MatN(j+6, 8) = P2N_w(2,i)*(nxi*Rw_c(1,2)+nyi*Rw_c(2,2)+nzi*Rw_c(3,2)) + P2N_w(3,i)*(nxi*Rw_c(1,3)+nyi*Rw_c(2,3)+nzi*Rw_c(3,3));
           end
            MatN(j+6, 5) = nxi;
            MatN(j+6, 6) = nyi;
            MatN(j+6, 7) = nzi;
        end
    elseif unkown == 2
        for i = 1:n
            nxi = nc_nbar(1,i);  nyi = nc_nbar(2,i);  nzi = nc_nbar(3,i);
            if i == 1 || i==3 || i==5
                MatN(i, 1) = nxi*Rw_c(1,2)+nyi*Rw_c(2,2)+nzi*Rw_c(3,2);
                MatN(i, 8) = P1N_w(1,i)*(nxi*Rw_c(1,1)+nyi*Rw_c(2,1)+nzi*Rw_c(3,1)) + P1N_w(3,i)*(nxi*Rw_c(1,3)+nyi*Rw_c(2,3)+nzi*Rw_c(3,3));
            elseif i == 2 || i == 6
                MatN(i, 3) = nxi*Rw_c(1,2)+nyi*Rw_c(2,2)+nzi*Rw_c(3,2);
                MatN(i, 8) = P1N_w(1,i)*(nxi*Rw_c(1,1)+nyi*Rw_c(2,1)+nzi*Rw_c(3,1)) + P1N_w(3,i)*(nxi*Rw_c(1,3)+nyi*Rw_c(2,3)+nzi*Rw_c(3,3));
            elseif i == 4
                MatN(i, 2) = nxi*Rw_c(1,2)+nyi*Rw_c(2,2)+nzi*Rw_c(3,2);
                MatN(i, 8) = P1N_w(1,i)*(nxi*Rw_c(1,1)+nyi*Rw_c(2,1)+nzi*Rw_c(3,1)) + P1N_w(3,i)*(nxi*Rw_c(1,3)+nyi*Rw_c(2,3)+nzi*Rw_c(3,3));
            end
            MatN(i, 5) = nxi;
            MatN(i, 6) = nyi;
            MatN(i, 7) = nzi; 
        end

        for j = 1:n                     
            % apply the constraint scaleproduct(Pi^c, ni^c) = 0
            nxi = nc_nbar(1,j);  nyi = nc_nbar(2,j);  nzi = nc_nbar(3,j); 
            if j == 1 || j ==6
                MatN(j+6, 2) = nxi*Rw_c(1,2)+nyi*Rw_c(2,2)+nzi*Rw_c(3,2);
                MatN(j+6, 8) = P2N_w(1,i)*(nxi*Rw_c(1,1)+nyi*Rw_c(2,1)+nzi*Rw_c(3,1)) + P2N_w(3,i)*(nxi*Rw_c(1,3)+nyi*Rw_c(2,3)+nzi*Rw_c(3,3));
            elseif j == 2 || j ==4 || j == 5
                MatN(j+6, 4) = nxi*Rw_c(1,2)+nyi*Rw_c(2,2)+nzi*Rw_c(3,2);
                MatN(j+6, 8) = P2N_w(1,i)*(nxi*Rw_c(1,1)+nyi*Rw_c(2,1)+nzi*Rw_c(3,1)) + P2N_w(3,i)*(nxi*Rw_c(1,3)+nyi*Rw_c(2,3)+nzi*Rw_c(3,3));
            elseif i == 3
                MatN(j+6, 3) = nxi*Rw_c(1,2)+nyi*Rw_c(2,2)+nzi*Rw_c(3,2);
                MatN(j+6, 8) = P2N_w(1,i)*(nxi*Rw_c(1,1)+nyi*Rw_c(2,1)+nzi*Rw_c(3,1)) + P2N_w(3,i)*(nxi*Rw_c(1,3)+nyi*Rw_c(2,3)+nzi*Rw_c(3,3));
           end
            MatN(j+6, 5) = nxi;
            MatN(j+6, 6) = nyi;
            MatN(j+6, 7) = nzi;
        end
    elseif unkown == 3
        for i = 1:n
            nxi = nc_nbar(1,i);  nyi = nc_nbar(2,i);  nzi = nc_nbar(3,i);
            if i == 1 || i==3 || i==5
                MatN(i, 1) = nxi*Rw_c(1,3)+nyi*Rw_c(2,3)+nzi*Rw_c(3,3);
                MatN(i, 8) = P1N_w(1,i)*(nxi*Rw_c(1,1)+nyi*Rw_c(2,1)+nzi*Rw_c(3,1)) + P1N_w(2,i)*(nxi*Rw_c(1,2)+nyi*Rw_c(2,2)+nzi*Rw_c(3,2));
            elseif i == 2 || i == 6
                MatN(i, 3) = nxi*Rw_c(1,3)+nyi*Rw_c(2,3)+nzi*Rw_c(3,3);
                MatN(i, 8) = P1N_w(1,i)*(nxi*Rw_c(1,1)+nyi*Rw_c(2,1)+nzi*Rw_c(3,1)) + P1N_w(2,i)*(nxi*Rw_c(1,2)+nyi*Rw_c(2,2)+nzi*Rw_c(3,2));
            elseif i == 4
                MatN(i, 2) = nxi*Rw_c(1,3)+nyi*Rw_c(2,3)+nzi*Rw_c(3,3);
                MatN(i, 8) = P1N_w(1,i)*(nxi*Rw_c(1,1)+nyi*Rw_c(2,1)+nzi*Rw_c(3,1)) + P1N_w(2,i)*(nxi*Rw_c(1,2)+nyi*Rw_c(2,2)+nzi*Rw_c(3,2));
            end
            MatN(i, 5) = nxi;
            MatN(i, 6) = nyi;
            MatN(i, 7) = nzi; 
        end

        for j = 1:n                     
            % apply the constraint scaleproduct(Pi^c, ni^c) = 0
            nxi = nc_nbar(1,j);  nyi = nc_nbar(2,j);  nzi = nc_nbar(3,j); 
            if j == 1 || j ==6
                MatN(j+6, 2) = nxi*Rw_c(1,3)+nyi*Rw_c(2,3)+nzi*Rw_c(3,3);
                MatN(j+6, 8) = P2N_w(1,i)*(nxi*Rw_c(1,1)+nyi*Rw_c(2,1)+nzi*Rw_c(3,1)) + P2N_w(2,i)*(nxi*Rw_c(1,2)+nyi*Rw_c(2,2)+nzi*Rw_c(3,2));
            elseif j == 2 || j ==4 || j == 5
                MatN(j+6, 4) = nxi*Rw_c(1,3)+nyi*Rw_c(2,3)+nzi*Rw_c(3,3);
                MatN(j+6, 8) = P2N_w(1,i)*(nxi*Rw_c(1,1)+nyi*Rw_c(2,1)+nzi*Rw_c(3,1)) + P2N_w(2,i)*(nxi*Rw_c(1,2)+nyi*Rw_c(2,2)+nzi*Rw_c(3,2));
            elseif i == 3
                MatN(j+6, 3) = nxi*Rw_c(1,3)+nyi*Rw_c(2,3)+nzi*Rw_c(3,3);
                MatN(j+6, 8) = P2N_w(1,i)*(nxi*Rw_c(1,1)+nyi*Rw_c(2,1)+nzi*Rw_c(3,1)) + P2N_w(2,i)*(nxi*Rw_c(1,2)+nyi*Rw_c(2,2)+nzi*Rw_c(3,2));
           end
            MatN(j+6, 5) = nxi;
            MatN(j+6, 6) = nyi;
            MatN(j+6, 7) = nzi;
        end
    end
    
    numL = 12;
    
    f1 = IA(1,1);
    f2 = IA(2,2);
    u0 = IA(1,3);
    v0 = IA(2,3);
    for i = 1:4
        u = p2d(i,1);
        v = p2d(i,2);
        X = P3d(1,i);Y = P3d(2,i);Z = P3d(3,i);
        if unkown == 1
             MatN(numL+2*i-1,i) =  f1*Rw_c(1,1)+Rw_c(3,1)*u0-Rw_c(3,1)*u;
             MatN(numL+2*i-1,5) =  f1;
             MatN(numL+2*i-1,7) =  u0-u;
             MatN(numL+2*i-1,8) =  f1*Rw_c(1,2)*Y+Rw_c(3,2)*u0*Y+f1*Rw_c(1,3)*Z+Rw_c(3,3)*u0*Z-Rw_c(3,2)*u*Y-Rw_c(3,3)*u*Z;        
             MatN(numL+2*i,i) =  f2*Rw_c(2,1)+Rw_c(3,1)*v0-Rw_c(3,1)*v;
             MatN(numL+2*i,6) =  f2;
             MatN(numL+2*i,7) =  v0-v;
             MatN(numL+2*i,8) =  f2*Rw_c(2,2)*Y+Rw_c(3,2)*v0*Y+f2*Rw_c(2,3)*Z+Rw_c(3,3)*v0*Z-Rw_c(3,2)*v*Y-Rw_c(3,3)*v*Z;        
        elseif unkown == 2
             MatN(numL+2*i-1,i) =  f1*Rw_c(1,2)+Rw_c(3,2)*u0-Rw_c(3,2)*u;
             MatN(numL+2*i-1,5) =  f1;
             MatN(numL+2*i-1,7) =  u0-u;
             MatN(numL+2*i-1,8) =  f1*Rw_c(1,1)*X+Rw_c(3,1)*u0*X+f1*Rw_c(1,3)*Z+Rw_c(3,3)*u0*Z-Rw_c(3,1)*u*X-Rw_c(3,3)*u*Z;        
             MatN(numL+2*i,i) =  f2*Rw_c(2,2)+Rw_c(3,2)*v0-Rw_c(3,2)*v;
             MatN(numL+2*i,6) =  f2;
             MatN(numL+2*i,7) =  v0-v;
             MatN(numL+2*i,8) =  f2*Rw_c(2,1)*X+Rw_c(3,1)*v0*X+f2*Rw_c(2,3)*Z+Rw_c(3,3)*v0*Z-Rw_c(3,1)*v*X-Rw_c(3,3)*v*Z;        
        elseif unkown == 3
             MatN(numL+2*i-1,i) =  f1*Rw_c(1,3)+Rw_c(3,3)*u0-Rw_c(3,3)*u;
             MatN(numL+2*i-1,5) =  f1;
             MatN(numL+2*i-1,7) =  u0-u;
             MatN(numL+2*i-1,8) =  f1*Rw_c(1,1)*X+Rw_c(3,1)*u0*X+f1*Rw_c(1,2)*Y+Rw_c(3,2)*u0*Y-Rw_c(3,1)*u*X-Rw_c(3,2)*u*Y;        
             MatN(numL+2*i,i) =  f2*Rw_c(2,3)+Rw_c(3,3)*v0-Rw_c(3,3)*v;
             MatN(numL+2*i,6) =  f2;
             MatN(numL+2*i,7) =  v0-v;
             MatN(numL+2*i,8) =  f2*Rw_c(2,1)*X+Rw_c(3,1)*v0*X+f2*Rw_c(2,2)*Y+Rw_c(3,2)*v0*Y-Rw_c(3,1)*v*X-Rw_c(3,2)*v*Y;        
         end       
    end
    
    
    invR = inv(Rw_c);

    if unkown == 1
        MatN(21,5) = invR(1,1);
        MatN(21,6) = invR(1,2);
        MatN(21,7) = invR(1,3);
        MatN(21,8) = cH(1);
    elseif unknown == 2
        MatN(21,5) = invR(2,1);
        MatN(21,6) = invR(2,2);
        MatN(21,7) = invR(2,3);
        MatN(21,8) = cH(2);
    elseif unknown == 3
        MatN(21,5) = invR(3,1);
        MatN(21,6) = invR(3,2);
        MatN(21,7) = invR(3,3);
        MatN(21,8) = cH(3);
    end
        

    
    

    
    [UMatN, SMatN, VMatN] = svd(MatN);
    vecN = VMatN(:,8);% the last column of Vmat;
    vecN = vecN/vecN(8); %the condition that the last element of vec should be 1.     
    Rt = vecN(5:7);
    aPoints = vecN(1:4);
end
