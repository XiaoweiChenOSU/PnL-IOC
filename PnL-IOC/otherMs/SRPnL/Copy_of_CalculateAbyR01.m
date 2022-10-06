function aPoints = Copy_of_CalculateAbyR01(p1, p2, W1, W2, Rr)

    pn1 = [p1 p2(:,5:8) p2(:,5:6) p1(:,1:4)];
    pn2 = [p2 p2(:,6:8) p2(:,5) p2(:,7:8) p2(:,6) p2(:,5) p2(:,8) p2(:,7)];
    nxs = pn1;
    nxe = pn2;
    n_nc = xcross_mat(nxs,nxe);
    nc_nbar = xnorm(n_nc);
    P1N_w = [W1 W2(:,5:8) W2(:,5:6) W1(:,1:4)];
    P2N_w = [W2 W2(:,6:8) W2(:,5) W2(:,7:8) W2(:,6) W2(:,5) W2(:,8) W2(:,7)];


    
%     pn1 = [p1 p2(:,5:8) p2(:,5:6)];
%     pn2 = [p2 p2(:,6:8) p2(:,5) p2(:,7:8)];
%     nxs = pn1;
%     nxe = pn2;
%     n_nc = xcross_mat(nxs,nxe);
%     nc_nbar = xnorm(n_nc);
%     P1N_w = [W1 W2(:,5:8) W2(:,5:6)];
%     P2N_w = [W2 W2(:,6:8) W2(:,5) W2(:,7:8)];
    
%     
    MatN = zeros(18, 8);       
    Rw_c = Rr;
% 

%     for i = 1:4
%         nxi = nc_nbar(1,i);  nyi = nc_nbar(2,i);  nzi = nc_nbar(3,i);
%         MatN(i, 5) = nxi;
%         MatN(i, 6) = nyi;
%         MatN(i, 7) = nzi; 
%         MatN(i, 8) = ((P1N_w(1,i)+P2N_w(1,i))*(nxi*Rw_c(1,1)+nyi*Rw_c(2,1)+nzi*Rw_c(3,1))/2) + ((P1N_w(2,i)+P2N_w(2,i))*(nxi*Rw_c(1,2)+nyi*Rw_c(2,2)+nzi*Rw_c(3,2))/2) + ((P1N_w(3,i)+P2N_w(3,i))*(nxi*Rw_c(1,3)+nyi*Rw_c(2,3)+nzi*Rw_c(3,3))/2);   
%     end
%     
%     for i = 5:8
%         nxi = nc_nbar(1,i);  nyi = nc_nbar(2,i);  nzi = nc_nbar(3,i);
%         MatN(i, i-4) = (nxi*Rw_c(1,3)+nyi*Rw_c(2,3)+nzi*Rw_c(3,3))/2;
%         MatN(i, 5) = nxi;
%         MatN(i, 6) = nyi;
%         MatN(i, 7) = nzi; 
%         MatN(i, 8) = ((P1N_w(1,i)+P2N_w(1,i))*(nxi*Rw_c(1,1)+nyi*Rw_c(2,1)+nzi*Rw_c(3,1))/2) + ((P1N_w(2,i)+P2N_w(2,i))*(nxi*Rw_c(1,2)+nyi*Rw_c(2,2)+nzi*Rw_c(3,2))/2) + (P1N_w(3,i)*(nxi*Rw_c(1,3)+nyi*Rw_c(2,3)+nzi*Rw_c(3,3))/2);      
%     end
%     
%     
% 
%     for i = 9:14                     
%         % apply the constraint scaleproduct(Pi^c, ni^c) = 0
%         nxi = nc_nbar(1,i);  nyi = nc_nbar(2,i);  nzi = nc_nbar(3,i); 
%         if i <= 11
%             MatN(i, i-8) = (nxi*Rw_c(1,3)+nyi*Rw_c(2,3)+nzi*Rw_c(3,3))/2;
%             MatN(i, i-7) = (nxi*Rw_c(1,3)+nyi*Rw_c(2,3)+nzi*Rw_c(3,3))/2;
%         elseif i == 12
%             MatN(i, 4) = (nxi*Rw_c(1,3)+nyi*Rw_c(2,3)+nzi*Rw_c(3,3))/2;
%             MatN(i, 1) = (nxi*Rw_c(1,3)+nyi*Rw_c(2,3)+nzi*Rw_c(3,3))/2;
%         elseif i == 13 || i == 14
%             MatN(i, i-12) = (nxi*Rw_c(1,3)+nyi*Rw_c(2,3)+nzi*Rw_c(3,3))/2;
%             MatN(i, i-10) = (nxi*Rw_c(1,3)+nyi*Rw_c(2,3)+nzi*Rw_c(3,3))/2;
%         end
%         MatN(i, 5) = nxi;
%         MatN(i, 6) = nyi;
%         MatN(i, 7) = nzi;
%         MatN(i, 8) = ((P1N_w(1,i)+P2N_w(1,i))*(nxi*Rw_c(1,1)+nyi*Rw_c(2,1)+nzi*Rw_c(3,1))/2) + ((P1N_w(2,i)+P2N_w(2,i))*(nxi*Rw_c(1,2)+nyi*Rw_c(2,2)+nzi*Rw_c(3,2))/2);  
%     end
%     

    for i = 1:4
        nxi = nc_nbar(1,i);  nyi = nc_nbar(2,i);  nzi = nc_nbar(3,i);
        MatN(i, 5) = nxi;
        MatN(i, 6) = nyi;
        MatN(i, 7) = nzi; 
        MatN(i, 8) = ((P1N_w(1,i)+P2N_w(1,i))*(nxi*Rw_c(1,1)+nyi*Rw_c(2,1)+nzi*Rw_c(3,1))/2) + ((P1N_w(2,i)+P2N_w(2,i))*(nxi*Rw_c(1,2)+nyi*Rw_c(2,2)+nzi*Rw_c(3,2))/2) + ((P1N_w(3,i)+P2N_w(3,i))*(nxi*Rw_c(1,3)+nyi*Rw_c(2,3)+nzi*Rw_c(3,3))/2);   
    end
    
    for i = 5:8
        nxi = nc_nbar(1,i);  nyi = nc_nbar(2,i);  nzi = nc_nbar(3,i);
        MatN(i, i-4) = (nxi*Rw_c(1,3)+nyi*Rw_c(2,3)+nzi*Rw_c(3,3))/2;
        MatN(i, 5) = nxi;
        MatN(i, 6) = nyi;
        MatN(i, 7) = nzi; 
        MatN(i, 8) = ((P1N_w(1,i)+P2N_w(1,i))*(nxi*Rw_c(1,1)+nyi*Rw_c(2,1)+nzi*Rw_c(3,1))/2) + ((P1N_w(2,i)+P2N_w(2,i))*(nxi*Rw_c(1,2)+nyi*Rw_c(2,2)+nzi*Rw_c(3,2))/2) + (P1N_w(3,i)*(nxi*Rw_c(1,3)+nyi*Rw_c(2,3)+nzi*Rw_c(3,3))/2);      
    end
    
    

    for i = 9:14                     
        % apply the constraint scaleproduct(Pi^c, ni^c) = 0
        nxi = nc_nbar(1,i);  nyi = nc_nbar(2,i);  nzi = nc_nbar(3,i); 
        if i <= 11
            MatN(i, i-8) = (nxi*Rw_c(1,3)+nyi*Rw_c(2,3)+nzi*Rw_c(3,3))/2;
            MatN(i, i-7) = (nxi*Rw_c(1,3)+nyi*Rw_c(2,3)+nzi*Rw_c(3,3))/2;
        elseif i == 12
            MatN(i, 4) = (nxi*Rw_c(1,3)+nyi*Rw_c(2,3)+nzi*Rw_c(3,3))/2;
            MatN(i, 1) = (nxi*Rw_c(1,3)+nyi*Rw_c(2,3)+nzi*Rw_c(3,3))/2;
        elseif i == 13 || i == 14
            MatN(i, i-12) = (nxi*Rw_c(1,3)+nyi*Rw_c(2,3)+nzi*Rw_c(3,3))/2;
            MatN(i, i-10) = (nxi*Rw_c(1,3)+nyi*Rw_c(2,3)+nzi*Rw_c(3,3))/2;
        end
        MatN(i, 5) = nxi;
        MatN(i, 6) = nyi;
        MatN(i, 7) = nzi;
        MatN(i, 8) = ((P1N_w(1,i)+P2N_w(1,i))*(nxi*Rw_c(1,1)+nyi*Rw_c(2,1)+nzi*Rw_c(3,1))/2) + ((P1N_w(2,i)+P2N_w(2,i))*(nxi*Rw_c(1,2)+nyi*Rw_c(2,2)+nzi*Rw_c(3,2))/2);  
    end
    
    for i = 15:18                    
        % apply the constraint scaleproduct(Pi^c, ni^c) = 0
        nxi = nc_nbar(1,i);  nyi = nc_nbar(2,i);  nzi = nc_nbar(3,i); 
        if i == 15 || i == 17
            MatN(i, i-13) = (nxi*Rw_c(1,3)+nyi*Rw_c(2,3)+nzi*Rw_c(3,3))/2;       
        elseif i == 16 || i == 18
            MatN(i, i-15) = (nxi*Rw_c(1,3)+nyi*Rw_c(2,3)+nzi*Rw_c(3,3))/2;
        end
        MatN(i, 5) = nxi;
        MatN(i, 6) = nyi;
        MatN(i, 7) = nzi;
        MatN(i, 8) = ((P1N_w(1,i)+P2N_w(1,i))*(nxi*Rw_c(1,1)+nyi*Rw_c(2,1)+nzi*Rw_c(3,1))/2) + ((P1N_w(2,i)+P2N_w(2,i))*(nxi*Rw_c(1,2)+nyi*Rw_c(2,2)+nzi*Rw_c(3,2))/2) + (P1N_w(3,i)*(nxi*Rw_c(1,3)+nyi*Rw_c(2,3)+nzi*Rw_c(3,3))/2);  
    end
    
    
%     MatNtemp = MatN;
    
%     
%     
    
%     for i = 1:4
%         nxi = nc_nbar(1,i);  nyi = nc_nbar(2,i);  nzi = nc_nbar(3,i);
%         MatN(i, i) = -(nxi*Rw_c(1,3)+nyi*Rw_c(2,3)+nzi*Rw_c(3,3));
%         MatN(i, 5) = ((P1N_w(1,i)-P2N_w(1,i))*(nxi*Rw_c(1,1)+nyi*Rw_c(2,1)+nzi*Rw_c(3,1))) + ((P1N_w(2,i)-P2N_w(2,i))*(nxi*Rw_c(1,2)+nyi*Rw_c(2,2)+nzi*Rw_c(3,2))) + (P1N_w(3,i)*(nxi*Rw_c(1,3)+nyi*Rw_c(2,3)+nzi*Rw_c(3,3)));      
%     end
% 
%     for i = 5:8                   
%         % apply the constraint scaleproduct(Pi^c, ni^c) = 0
%         nxi = nc_nbar(1,i);  nyi = nc_nbar(2,i);  nzi = nc_nbar(3,i); 
%         if i == 8
%             MatN(i, i-4) = (nxi*Rw_c(1,3)+nyi*Rw_c(2,3)+nzi*Rw_c(3,3));
%             MatN(i, 1) = -(nxi*Rw_c(1,3)+nyi*Rw_c(2,3)+nzi*Rw_c(3,3));    
%         else
%             MatN(i, i-4) = (nxi*Rw_c(1,3)+nyi*Rw_c(2,3)+nzi*Rw_c(3,3));
%             MatN(i, i-3) = -(nxi*Rw_c(1,3)+nyi*Rw_c(2,3)+nzi*Rw_c(3,3));    
%         end
%         MatN(i, 5) = ((P1N_w(1,i)-P2N_w(1,i))*(nxi*Rw_c(1,1)+nyi*Rw_c(2,1)+nzi*Rw_c(3,1)))+ ((P1N_w(2,i)-P2N_w(2,i))*(nxi*Rw_c(1,2)+nyi*Rw_c(2,2)+nzi*Rw_c(3,2)));                    
%     end
%     
%     for i = 9:10                   
%         % apply the constraint scaleproduct(Pi^c, ni^c) = 0
%         nxi = nc_nbar(1,i);  nyi = nc_nbar(2,i);  nzi = nc_nbar(3,i); 
%         if i == 9
%            MatN(i, 1) = (nxi*Rw_c(1,3)+nyi*Rw_c(2,3)+nzi*Rw_c(3,3));
%            MatN(i, 3) = -(nxi*Rw_c(1,3)+nyi*Rw_c(2,3)+nzi*Rw_c(3,3));    
%         else
%            MatN(i, 2) = (nxi*Rw_c(1,3)+nyi*Rw_c(2,3)+nzi*Rw_c(3,3));
%            MatN(i, 4) = -(nxi*Rw_c(1,3)+nyi*Rw_c(2,3)+nzi*Rw_c(3,3));    
%         end
%         MatN(i, 5) = ((P1N_w(1,i)-P2N_w(1,i))*(nxi*Rw_c(1,1)+nyi*Rw_c(2,1)+nzi*Rw_c(3,1)))+ ((P1N_w(2,i)-P2N_w(2,i))*(nxi*Rw_c(1,2)+nyi*Rw_c(2,2)+nzi*Rw_c(3,2)));                    
%     end
%     
%     for i = 11:14                  
%         % apply the constraint scaleproduct(Pi^c, ni^c) = 0
%         nxi = nc_nbar(1,i);  nyi = nc_nbar(2,i);  nzi = nc_nbar(3,i); 
%         if i == 11
%            MatN(i, 2) = -(nxi*Rw_c(1,3)+nyi*Rw_c(2,3)+nzi*Rw_c(3,3));
%         elseif i == 12
%            MatN(i, 1) = -(nxi*Rw_c(1,3)+nyi*Rw_c(2,3)+nzi*Rw_c(3,3));
%         elseif i == 13
%            MatN(i, 4) = -(nxi*Rw_c(1,3)+nyi*Rw_c(2,3)+nzi*Rw_c(3,3));
%         else
%            MatN(i, 3) = -(nxi*Rw_c(1,3)+nyi*Rw_c(2,3)+nzi*Rw_c(3,3)); 
%         end
%         MatN(i, 5) = ((P1N_w(1,i)-P2N_w(1,i))*(nxi*Rw_c(1,1)+nyi*Rw_c(2,1)+nzi*Rw_c(3,1)))+ ((P1N_w(2,i)-P2N_w(2,i))*(nxi*Rw_c(1,2)+nyi*Rw_c(2,2)+nzi*Rw_c(3,2))) + (P1N_w(3,i)*(nxi*Rw_c(1,3)+nyi*Rw_c(2,3)+nzi*Rw_c(3,3))); 
%     end 

    
    
%     numL = 18;
%     W = W2;
%     
%     f = IA(1,1);
%     u0 = IA(1,3);
%     v0 = IA(2,3);
%     for i = 1:8
%         if i >= 1 && i <= 4
%              u = p2d(i,1);v = p2d(i,2);
%              X = W(1,i);Y = W(2,i);Z = W(3,i);
%              MatN(numL+2*i-1,5) =  f;
%              MatN(numL+2*i-1,7) =  u0-u;
%              MatN(numL+2*i-1,8) =  f*Rw_c(1,1)*X+Rw_c(3,1)*u0*X+f*Rw_c(1,2)*Y+Rw_c(3,2)*u0*Y+f*Rw_c(1,3)*Z+Rw_c(3,3)*u0*Z-Rw_c(3,1)*u*X-Rw_c(3,2)*u*Y-Rw_c(3,3)*u*Z;        
%              MatN(numL+2*i,6) =  f;
%              MatN(numL+2*i,7) =  v0-v;
%              MatN(numL+2*i,8) =  f*Rw_c(2,1)*X+Rw_c(3,1)*v0*X+f*Rw_c(2,2)*Y+Rw_c(3,2)*v0*Y+f*Rw_c(2,3)*Z+Rw_c(3,3)*v0*Z-Rw_c(3,1)*v*X-Rw_c(3,2)*v*Y-Rw_c(3,3)*v*Z;        
%         elseif i == 5
%              u = p2d(i,1);v = p2d(i,2);
%              X = W(1,i);Y = W(2,i);
%              MatN(numL+2*i-1,1) =  f*Rw_c(1,3)+Rw_c(3,3)*u0-Rw_c(3,3)*u;
%              MatN(numL+2*i-1,5) =  f;
%              MatN(numL+2*i-1,7) =  u0-u;
%              MatN(numL+2*i-1,8) =  f*Rw_c(1,1)*X+Rw_c(3,1)*u0*X+f*Rw_c(1,2)*Y+Rw_c(3,2)*u0*Y-Rw_c(3,1)*u*X-Rw_c(3,2)*u*Y;        
%              MatN(numL+2*i,1) =  f*Rw_c(2,3)+Rw_c(3,3)*v0-Rw_c(3,3)*v;
%              MatN(numL+2*i,6) =  f;
%              MatN(numL+2*i,7) =  v0-v;
%              MatN(numL+2*i,8) =  f*Rw_c(2,1)*X+Rw_c(3,1)*v0*X+f*Rw_c(2,2)*Y+Rw_c(3,2)*v0*Y-Rw_c(3,1)*v*X-Rw_c(3,2)*v*Y;        
%         elseif i == 6
%              u = p2d(i,1);v = p2d(i,2);
%              X = W(1,i);Y = W(2,i);
%              MatN(numL+2*i-1,2) =  f*Rw_c(1,3)+Rw_c(3,3)*u0-Rw_c(3,3)*u;
%              MatN(numL+2*i-1,5) =  f;
%              MatN(numL+2*i-1,7) =  u0-u;
%              MatN(numL+2*i-1,8) =  f*Rw_c(1,1)*X+Rw_c(3,1)*u0*X+f*Rw_c(1,2)*Y+Rw_c(3,2)*u0*Y-Rw_c(3,1)*u*X-Rw_c(3,2)*u*Y;        
%              MatN(numL+2*i,2) =  f*Rw_c(2,3)+Rw_c(3,3)*v0-Rw_c(3,3)*v;
%              MatN(numL+2*i,6) =  f;
%              MatN(numL+2*i,7) =  v0-v;
%              MatN(numL+2*i,8) =  f*Rw_c(2,1)*X+Rw_c(3,1)*v0*X+f*Rw_c(2,2)*Y+Rw_c(3,2)*v0*Y-Rw_c(3,1)*v*X-Rw_c(3,2)*v*Y;        
%         elseif i == 7
%              u = p2d(i,1);v = p2d(i,2);
%              X = W(1,i);Y = W(2,i);
%              MatN(numL+2*i-1,3) =  f*Rw_c(1,3)+Rw_c(3,3)*u0-Rw_c(3,3)*u;
%              MatN(numL+2*i-1,5) =  f;
%              MatN(numL+2*i-1,7) =  u0-u;
%              MatN(numL+2*i-1,8) =  f*Rw_c(1,1)*X+Rw_c(3,1)*u0*X+f*Rw_c(1,2)*Y+Rw_c(3,2)*u0*Y-Rw_c(3,1)*u*X-Rw_c(3,2)*u*Y;        
%              MatN(numL+2*i,3) =  f*Rw_c(2,3)+Rw_c(3,3)*v0-Rw_c(3,3)*v;
%              MatN(numL+2*i,6) =  f;
%              MatN(numL+2*i,7) =  v0-v;
%              MatN(numL+2*i,8) =  f*Rw_c(2,1)*X+Rw_c(3,1)*v0*X+f*Rw_c(2,2)*Y+Rw_c(3,2)*v0*Y-Rw_c(3,1)*v*X-Rw_c(3,2)*v*Y;        
%         elseif i == 8
%              u = p2d(i,1);v = p2d(i,2);
%              X = W(1,i);Y = W(2,i);
%              MatN(numL+2*i-1,4) =  f*Rw_c(1,3)+Rw_c(3,3)*u0-Rw_c(3,3)*u;
%              MatN(numL+2*i-1,5) =  f;
%              MatN(numL+2*i-1,7) =  u0-u;
%              MatN(numL+2*i-1,8) =  f*Rw_c(1,1)*X+Rw_c(3,1)*u0*X+f*Rw_c(1,2)*Y+Rw_c(3,2)*u0*Y-Rw_c(3,1)*u*X-Rw_c(3,2)*u*Y;        
%              MatN(numL+2*i,4) =  f*Rw_c(2,3)+Rw_c(3,3)*v0-Rw_c(3,3)*v;
%              MatN(numL+2*i,6) =  f;
%              MatN(numL+2*i,7) =  v0-v;
%              MatN(numL+2*i,8) =  f*Rw_c(2,1)*X+Rw_c(3,1)*v0*X+f*Rw_c(2,2)*Y+Rw_c(3,2)*v0*Y-Rw_c(3,1)*v*X-Rw_c(3,2)*v*Y;        
%         end       
%     end
%     
    
    [~, ~, VMatN] = svd(MatN);
    vecN = VMatN(:,8);% the last column of Vmat;
    vecN = vecN/vecN(8); %the condition that the last element of vec should be 1.     
    aPoints = vecN(1:4);
    
    
end
