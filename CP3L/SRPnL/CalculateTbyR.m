function [Rt,aPoints] = CalculateTbyR(p1, p2, W1, W2, cH, Rr)
 
  
    pn1 = [p1 p2];
    pn2 = [p2 p2(:,2:3) p2(:,1)];
    nxs = pn1;
    nxe = pn2;
    n_nc = xcross_mat(nxs,nxe);
    nc_nbar = xnorm(n_nc);
    P1N_w = [W1 W2];
    P2N_w = [W2 W2(:,2:3) W2(:,1)];
    
    W = [W1(:,1) W2];

    MatN = zeros(9, 7);       
    Rw_c = Rr;

    for i = 1:3
        nxi = nc_nbar(1,i);  nyi = nc_nbar(2,i);  nzi = nc_nbar(3,i);
        if i == 1
            MatN(i, i) = (nxi*Rw_c(1,1)+nyi*Rw_c(2,1)+nzi*Rw_c(3,1))/2;
            MatN(i, 7) = (P1N_w(1,i)*(nxi*Rw_c(1,1)+nyi*Rw_c(2,1)+nzi*Rw_c(3,1))/2)+ ((P1N_w(2,i)+P2N_w(2,i))*(nxi*Rw_c(1,2)+nyi*Rw_c(2,2)+nzi*Rw_c(3,2))/2) + ((P1N_w(3,i)+P2N_w(3,i))*(nxi*Rw_c(1,3)+nyi*Rw_c(2,3)+nzi*Rw_c(3,3))/2);
        elseif i == 2
            MatN(i, i) = (nxi*Rw_c(1,2)+nyi*Rw_c(2,2)+nzi*Rw_c(3,2))/2;
            MatN(i, 7) = ((P1N_w(1,i)+P2N_w(1,i))*(nxi*Rw_c(1,1)+nyi*Rw_c(2,1)+nzi*Rw_c(3,1))/2) + (P1N_w(2,i)*(nxi*Rw_c(1,2)+nyi*Rw_c(2,2)+nzi*Rw_c(3,2))/2) + ((P1N_w(3,i)+P2N_w(3,i))*(nxi*Rw_c(1,3)+nyi*Rw_c(2,3)+nzi*Rw_c(3,3))/2);      
        elseif i == 3
            MatN(i, i) = (nxi*Rw_c(1,3)+nyi*Rw_c(2,3)+nzi*Rw_c(3,3))/2;
            MatN(i, 7) = ((P1N_w(1,i)+P2N_w(1,i))*(nxi*Rw_c(1,1)+nyi*Rw_c(2,1)+nzi*Rw_c(3,1))/2) + ((P1N_w(2,i)+P2N_w(2,i))*(nxi*Rw_c(1,2)+nyi*Rw_c(2,2)+nzi*Rw_c(3,2))/2) + (P1N_w(3,i)*(nxi*Rw_c(1,3)+nyi*Rw_c(2,3)+nzi*Rw_c(3,3))/2);      
        end
        MatN(i, 4) = nxi;
        MatN(i, 5) = nyi;
        MatN(i, 6) = nzi; 
    end

    for i = 4:6                     
        % apply the constraint scaleproduct(Pi^c, ni^c) = 0
        nxi = nc_nbar(1,i);  nyi = nc_nbar(2,i);  nzi = nc_nbar(3,i); 
        if i == 4
            MatN(4, i-3) = (nxi*Rw_c(1,1)+nyi*Rw_c(2,1)+nzi*Rw_c(3,1))/2;
            MatN(4, i-2) = (nxi*Rw_c(1,2)+nyi*Rw_c(2,2)+nzi*Rw_c(3,2))/2;
            MatN(4, 7) = (P2N_w(1,i)*(nxi*Rw_c(1,1)+nyi*Rw_c(2,1)+nzi*Rw_c(3,1))/2)+ (P1N_w(2,i)*(nxi*Rw_c(1,2)+nyi*Rw_c(2,2)+nzi*Rw_c(3,2))/2) + ((P1N_w(3,i)+P2N_w(3,i))*(nxi*Rw_c(1,3)+nyi*Rw_c(2,3)+nzi*Rw_c(3,3))/2);    
        elseif i == 5
            MatN(5, i-3) = (nxi*Rw_c(1,2)+nyi*Rw_c(2,2)+nzi*Rw_c(3,2))/2;
            MatN(5, i-2) = (nxi*Rw_c(1,3)+nyi*Rw_c(2,3)+nzi*Rw_c(3,3))/2;
            MatN(5, 7) = ((P1N_w(1,i)+P2N_w(1,i))*(nxi*Rw_c(1,1)+nyi*Rw_c(2,1)+nzi*Rw_c(3,1))/2)+ (P2N_w(2,i)*(nxi*Rw_c(1,2)+nyi*Rw_c(2,2)+nzi*Rw_c(3,2))/2) + (P1N_w(3,i)*(nxi*Rw_c(1,3)+nyi*Rw_c(2,3)+nzi*Rw_c(3,3))/2);    
        elseif i == 6
            MatN(6, 1) = (nxi*Rw_c(1,1)+nyi*Rw_c(2,1)+nzi*Rw_c(3,1))/2;
            MatN(6, i-3) = (nxi*Rw_c(1,3)+nyi*Rw_c(2,3)+nzi*Rw_c(3,3))/2;
            MatN(6, 7) = (P1N_w(1,i)*(nxi*Rw_c(1,1)+nyi*Rw_c(2,1)+nzi*Rw_c(3,1))/2)+ ((P1N_w(2,i)+P2N_w(2,i))*(nxi*Rw_c(1,2)+nyi*Rw_c(2,2)+nzi*Rw_c(3,2))/2) + (P2N_w(3,i)*(nxi*Rw_c(1,3)+nyi*Rw_c(2,3)+nzi*Rw_c(3,3))/2);                   
        end
        MatN(i, 4) = nxi;
        MatN(i, 5) = nyi;
        MatN(i, 6) = nzi;
    end


    invR = inv(Rw_c);

%     MatN(7,4) = invR(2,1);
%     MatN(7,5) = invR(2,2);
%     MatN(7,6) = invR(2,3);
%     MatN(7,7) = cH;
    MatN(7,4) = invR(1,1);
    MatN(7,5) = invR(1,2);
    MatN(7,6) = invR(1,3);
    MatN(7,7) = cH;
    
%     numL = 7;
%     
%     f = IA(1,1);
%     u0 = IA(1,3);
%     v0 = IA(2,3);
%     for i = 1:4
%         if i == 1
%              u = p2d(i,1);v = p2d(i,2);
%              X = W(1,i);Y = W(2,i);Z = W(3,i);
%              MatN(numL+2*i-1,4) =  f;
%              MatN(numL+2*i-1,6) =  u0-u;
%              MatN(numL+2*i-1,7) =  f*Rw_c(1,1)*X+Rw_c(3,1)*u0*X+f*Rw_c(1,2)*Y+Rw_c(3,2)*u0*Y+f*Rw_c(1,3)*Z+Rw_c(3,3)*u0*Z-Rw_c(3,1)*u*X-Rw_c(3,2)*u*Y-Rw_c(3,3)*u*Z;        
%              MatN(numL+2*i,5) =  f;
%              MatN(numL+2*i,6) =  v0-v;
%              MatN(numL+2*i,7) =  f*Rw_c(2,1)*X+Rw_c(3,1)*v0*X+f*Rw_c(2,2)*Y+Rw_c(3,2)*v0*Y+f*Rw_c(2,3)*Z+Rw_c(3,3)*v0*Z-Rw_c(3,1)*v*X-Rw_c(3,2)*v*Y-Rw_c(3,3)*v*Z;        
%         elseif i == 2
%              u = p2d(i,1);v = p2d(i,2);
%              Y = W(2,i);Z = W(3,i);
%              MatN(numL+2*i-1,1) =  f*Rw_c(1,1)+Rw_c(3,1)*u0-Rw_c(3,1)*u;
%              MatN(numL+2*i-1,4) =  f;
%              MatN(numL+2*i-1,6) =  u0-u;
%              MatN(numL+2*i-1,7) =  f*Rw_c(1,2)*Y+Rw_c(3,2)*u0*Y+f*Rw_c(1,3)*Z+Rw_c(3,3)*u0*Z-Rw_c(3,2)*u*Y-Rw_c(3,3)*u*Z;        
%              MatN(numL+2*i,1) =  f*Rw_c(2,1)+Rw_c(3,1)*v0-Rw_c(3,1)*v;
%              MatN(numL+2*i,5) =  f;
%              MatN(numL+2*i,6) =  v0-v;
%              MatN(numL+2*i,7) =  f*Rw_c(2,2)*Y+Rw_c(3,2)*v0*Y+f*Rw_c(2,3)*Z+Rw_c(3,3)*v0*Z-Rw_c(3,2)*v*Y-Rw_c(3,3)*v*Z;        
%         elseif i == 3
%              u = p2d(i,1);v = p2d(i,2);
%              X = W(1,i);Z = W(3,i);
%              MatN(numL+2*i-1,2) =  f*Rw_c(1,2)+Rw_c(3,2)*u0-Rw_c(3,2)*u;
%              MatN(numL+2*i-1,4) =  f;
%              MatN(numL+2*i-1,6) =  u0-u;
%              MatN(numL+2*i-1,7) =  f*Rw_c(1,1)*X+Rw_c(3,1)*u0*X+f*Rw_c(1,3)*Z+Rw_c(3,3)*u0*Z-Rw_c(3,1)*u*X-Rw_c(3,3)*u*Z;        
%              MatN(numL+2*i,2) =  f*Rw_c(2,2)+Rw_c(3,2)*v0-Rw_c(3,2)*v;
%              MatN(numL+2*i,5) =  f;
%              MatN(numL+2*i,6) =  v0-v;
%              MatN(numL+2*i,7) =  f*Rw_c(2,1)*X+Rw_c(3,1)*v0*X+f*Rw_c(2,3)*Z+Rw_c(3,3)*v0*Z-Rw_c(3,1)*v*X-Rw_c(3,3)*v*Z;        
%         elseif i == 4
%              u = p2d(i,1);v = p2d(i,2);
%              X = W(1,i);Y = W(2,i);
%              MatN(numL+2*i-1,3) =  f*Rw_c(1,3)+Rw_c(3,3)*u0-Rw_c(3,3)*u;
%              MatN(numL+2*i-1,4) =  f;
%              MatN(numL+2*i-1,6) =  u0-u;
%              MatN(numL+2*i-1,7) =  f*Rw_c(1,1)*X+Rw_c(3,1)*u0*X+f*Rw_c(1,2)*Y+Rw_c(3,2)*u0*Y-Rw_c(3,1)*u*X-Rw_c(3,2)*u*Y;        
%              MatN(numL+2*i,3) =  f*Rw_c(2,3)+Rw_c(3,3)*v0-Rw_c(3,3)*v;
%              MatN(numL+2*i,5) =  f;
%              MatN(numL+2*i,6) =  v0-v;
%              MatN(numL+2*i,7) =  f*Rw_c(2,1)*X+Rw_c(3,1)*v0*X+f*Rw_c(2,2)*Y+Rw_c(3,2)*v0*Y-Rw_c(3,1)*v*X-Rw_c(3,2)*v*Y;        
%         end       
%     end
    
    [UMatN, SMatN, VMatN] = svd(MatN);
    vecN = VMatN(:,7);% the last column of Vmat;
    vecN = vecN/vecN(7); %the condition that the last element of vec should be 1.     
    Rt = vecN(4:6);
    aPoints = vecN(1:3);
end
