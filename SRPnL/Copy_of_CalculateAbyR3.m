function aPoints = Copy_of_CalculateAbyR3(p1, p2, W1, W2, cH, Rr)

    pn1 = [p1 p2];
    pn2 = [p2 p2(:,2:3) p2(:,1)];
    n_nc = xcross_mat(pn1,pn2);
    nc_nbar = xnorm(n_nc);
    P1N_w = [W1 W2];
    P2N_w = [W2 W2(:,2:3) W2(:,1)];
    
    MatN = zeros(6, 4);    
    Rw_c = Rr;

    
    for i = 1:3
        nxi = nc_nbar(1,i);  nyi = nc_nbar(2,i);  nzi = nc_nbar(3,i);
        if i == 1
            MatN(i, i) = -(nxi*Rw_c(1,1)+nyi*Rw_c(2,1)+nzi*Rw_c(3,1));
            MatN(i, 4) = (P1N_w(1,i)*(nxi*Rw_c(1,1)+nyi*Rw_c(2,1)+nzi*Rw_c(3,1)))+ ((P1N_w(2,i)-P2N_w(2,i))*(nxi*Rw_c(1,2)+nyi*Rw_c(2,2)+nzi*Rw_c(3,2))) + ((P1N_w(3,i)-P2N_w(3,i))*(nxi*Rw_c(1,3)+nyi*Rw_c(2,3)+nzi*Rw_c(3,3)));
        elseif i == 2
            MatN(i, i) = -(nxi*Rw_c(1,2)+nyi*Rw_c(2,2)+nzi*Rw_c(3,2));
            MatN(i, 4) = ((P1N_w(1,i)-P2N_w(1,i))*(nxi*Rw_c(1,1)+nyi*Rw_c(2,1)+nzi*Rw_c(3,1))) + (P1N_w(2,i)*(nxi*Rw_c(1,2)+nyi*Rw_c(2,2)+nzi*Rw_c(3,2))) + ((P1N_w(3,i)-P2N_w(3,i))*(nxi*Rw_c(1,3)+nyi*Rw_c(2,3)+nzi*Rw_c(3,3)));      
        elseif i == 3
            MatN(i, i) = -(nxi*Rw_c(1,3)+nyi*Rw_c(2,3)+nzi*Rw_c(3,3));
            MatN(i, 4) = ((P1N_w(1,i)-P2N_w(1,i))*(nxi*Rw_c(1,1)+nyi*Rw_c(2,1)+nzi*Rw_c(3,1))) + ((P1N_w(2,i)-P2N_w(2,i))*(nxi*Rw_c(1,2)+nyi*Rw_c(2,2)+nzi*Rw_c(3,2))) + (P1N_w(3,i)*(nxi*Rw_c(1,3)+nyi*Rw_c(2,3)+nzi*Rw_c(3,3)));      
        end
    end

    for i = 4:6                     
        % apply the constraint scaleproduct(Pi^c, ni^c) = 0
        nxi = nc_nbar(1,i);  nyi = nc_nbar(2,i);  nzi = nc_nbar(3,i); 
        if i == 4
            MatN(4, i-3) = (nxi*Rw_c(1,1)+nyi*Rw_c(2,1)+nzi*Rw_c(3,1));
            MatN(4, i-2) = -(nxi*Rw_c(1,2)+nyi*Rw_c(2,2)+nzi*Rw_c(3,2));
            MatN(4, 4) = (-P2N_w(1,i)*(nxi*Rw_c(1,1)+nyi*Rw_c(2,1)+nzi*Rw_c(3,1)))+ (P1N_w(2,i)*(nxi*Rw_c(1,2)+nyi*Rw_c(2,2)+nzi*Rw_c(3,2))) + ((P1N_w(3,i)-P2N_w(3,i))*(nxi*Rw_c(1,3)+nyi*Rw_c(2,3)+nzi*Rw_c(3,3)));    
        elseif i == 5
            MatN(5, i-3) = (nxi*Rw_c(1,2)+nyi*Rw_c(2,2)+nzi*Rw_c(3,2));
            MatN(5, i-2) = -(nxi*Rw_c(1,3)+nyi*Rw_c(2,3)+nzi*Rw_c(3,3));
            MatN(5, 4) = ((P1N_w(1,i)-P2N_w(1,i))*(nxi*Rw_c(1,1)+nyi*Rw_c(2,1)+nzi*Rw_c(3,1)))+ (-P2N_w(2,i)*(nxi*Rw_c(1,2)+nyi*Rw_c(2,2)+nzi*Rw_c(3,2))) + (P1N_w(3,i)*(nxi*Rw_c(1,3)+nyi*Rw_c(2,3)+nzi*Rw_c(3,3)));    
        elseif i == 6
            MatN(6, 1) = (nxi*Rw_c(1,1)+nyi*Rw_c(2,1)+nzi*Rw_c(3,1));
            MatN(6, i-3) = -(nxi*Rw_c(1,3)+nyi*Rw_c(2,3)+nzi*Rw_c(3,3));
            MatN(6, 4) = (P1N_w(1,i)*(nxi*Rw_c(1,1)+nyi*Rw_c(2,1)+nzi*Rw_c(3,1)))+ ((P1N_w(2,i)-P2N_w(2,i))*(nxi*Rw_c(1,2)+nyi*Rw_c(2,2)+nzi*Rw_c(3,2))) + (-P2N_w(3,i)*(nxi*Rw_c(1,3)+nyi*Rw_c(2,3)+nzi*Rw_c(3,3)));                   
        end
    end

%     for i = 1:3
%         nxi = nc_nbar(1,i);  nyi = nc_nbar(2,i);  nzi = nc_nbar(3,i);
%         if i == 1
%             MatN(i, i) = (nxi*Rw_c(1,1)+nyi*Rw_c(2,1)+nzi*Rw_c(3,1))/2;
%             MatN(i, 7) = (P1N_w(1,i)*(nxi*Rw_c(1,1)+nyi*Rw_c(2,1)+nzi*Rw_c(3,1))/2)+ ((P1N_w(2,i)+P2N_w(2,i))*(nxi*Rw_c(1,2)+nyi*Rw_c(2,2)+nzi*Rw_c(3,2))/2) + ((P1N_w(3,i)+P2N_w(3,i))*(nxi*Rw_c(1,3)+nyi*Rw_c(2,3)+nzi*Rw_c(3,3))/2);
%         elseif i == 2
%             MatN(i, i) = (nxi*Rw_c(1,2)+nyi*Rw_c(2,2)+nzi*Rw_c(3,2))/2;
%             MatN(i, 7) = ((P1N_w(1,i)+P2N_w(1,i))*(nxi*Rw_c(1,1)+nyi*Rw_c(2,1)+nzi*Rw_c(3,1))/2) + (P1N_w(2,i)*(nxi*Rw_c(1,2)+nyi*Rw_c(2,2)+nzi*Rw_c(3,2))/2) + ((P1N_w(3,i)+P2N_w(3,i))*(nxi*Rw_c(1,3)+nyi*Rw_c(2,3)+nzi*Rw_c(3,3))/2);      
%         elseif i == 3
%             MatN(i, i) = (nxi*Rw_c(1,3)+nyi*Rw_c(2,3)+nzi*Rw_c(3,3))/2;
%             MatN(i, 7) = ((P1N_w(1,i)+P2N_w(1,i))*(nxi*Rw_c(1,1)+nyi*Rw_c(2,1)+nzi*Rw_c(3,1))/2) + ((P1N_w(2,i)+P2N_w(2,i))*(nxi*Rw_c(1,2)+nyi*Rw_c(2,2)+nzi*Rw_c(3,2))/2) + (P1N_w(3,i)*(nxi*Rw_c(1,3)+nyi*Rw_c(2,3)+nzi*Rw_c(3,3))/2);      
%         end
%         MatN(i, 4) = nxi;
%         MatN(i, 5) = nyi;
%         MatN(i, 6) = nzi; 
%     end
% 
%     for i = 4:6                     
%         % apply the constraint scaleproduct(Pi^c, ni^c) = 0
%         nxi = nc_nbar(1,i);  nyi = nc_nbar(2,i);  nzi = nc_nbar(3,i); 
%         if i == 4
%             MatN(4, i-3) = (nxi*Rw_c(1,1)+nyi*Rw_c(2,1)+nzi*Rw_c(3,1))/2;
%             MatN(4, i-2) = (nxi*Rw_c(1,2)+nyi*Rw_c(2,2)+nzi*Rw_c(3,2))/2;
%             MatN(4, 7) = (P2N_w(1,i)*(nxi*Rw_c(1,1)+nyi*Rw_c(2,1)+nzi*Rw_c(3,1))/2)+ (P1N_w(2,i)*(nxi*Rw_c(1,2)+nyi*Rw_c(2,2)+nzi*Rw_c(3,2))/2) + ((P1N_w(3,i)+P2N_w(3,i))*(nxi*Rw_c(1,3)+nyi*Rw_c(2,3)+nzi*Rw_c(3,3))/2);    
%         elseif i == 5
%             MatN(5, i-3) = (nxi*Rw_c(1,2)+nyi*Rw_c(2,2)+nzi*Rw_c(3,2))/2;
%             MatN(5, i-2) = (nxi*Rw_c(1,3)+nyi*Rw_c(2,3)+nzi*Rw_c(3,3))/2;
%             MatN(5, 7) = ((P1N_w(1,i)+P2N_w(1,i))*(nxi*Rw_c(1,1)+nyi*Rw_c(2,1)+nzi*Rw_c(3,1))/2)+ (P2N_w(2,i)*(nxi*Rw_c(1,2)+nyi*Rw_c(2,2)+nzi*Rw_c(3,2))/2) + (P1N_w(3,i)*(nxi*Rw_c(1,3)+nyi*Rw_c(2,3)+nzi*Rw_c(3,3))/2);    
%         elseif i == 6
%             MatN(6, 1) = (nxi*Rw_c(1,1)+nyi*Rw_c(2,1)+nzi*Rw_c(3,1))/2;
%             MatN(6, i-3) = (nxi*Rw_c(1,3)+nyi*Rw_c(2,3)+nzi*Rw_c(3,3))/2;
%             MatN(6, 7) = (P1N_w(1,i)*(nxi*Rw_c(1,1)+nyi*Rw_c(2,1)+nzi*Rw_c(3,1))/2)+ ((P1N_w(2,i)+P2N_w(2,i))*(nxi*Rw_c(1,2)+nyi*Rw_c(2,2)+nzi*Rw_c(3,2))/2) + (P2N_w(3,i)*(nxi*Rw_c(1,3)+nyi*Rw_c(2,3)+nzi*Rw_c(3,3))/2);                   
%         end
%         MatN(i, 4) = nxi;
%         MatN(i, 5) = nyi;
%         MatN(i, 6) = nzi;
%     end


%     invR = inv(Rw_c);
% 
%     MatN(7,4) = invR(2,1);
%     MatN(7,5) = invR(2,2);
%     MatN(7,6) = invR(2,3);
%     MatN(7,7) = cH;
    
    [UMatN, SMatN, VMatN] = svd(MatN);
    vecN = VMatN(:,4);% the last column of Vmat;
    vecN = vecN/vecN(4); %the condition that the last element of vec should be 1.     
    aPoints = vecN(1:3);
    
    
end
