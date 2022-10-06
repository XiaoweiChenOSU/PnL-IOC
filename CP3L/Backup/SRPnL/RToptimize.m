function [RR,tt,curErr]=RToptimize(ps, pe, Ps, Pe, R0, T0,T1A,R_truth)

    
    T1p1 = ps;
    T1p2 = pe;

    for i = 1:length(T1p1)
        temp1 = inv(T1A)*[T1p1(i,:) 1].';
        temp1 = (temp1/temp1(3)).';
        iT1p1(i,:) = temp1(:,1:2);
        temp2 = inv(T1A)*[T1p2(i,:) 1].';
        temp2 = (temp2/temp2(3)).';
        iT1p2(i,:) = temp2(:,1:2);
    end

    nl = getProjNorm(iT1p1',iT1p2');
    
%     [d, ~] = getVP(Ps(1:3,:), Pe(1:3,:));
    [d, ~] = getVP(Ps, Pe);
    
    In = 50;
    
    [RR,K] = Roptimzation(ps, pe,T1A,R0,d,In);
    
    tt = T0;
    

    Xw = Ps';
    U = ps;
    [curErr,~] = reprojection_error_usingRT(Xw,U,RR,tt,T1A);

    
    
    
%     initInfo = [352.0494
%       139.2621
%       347.7099
%              0
%       347.4974
%       395.3755]; 
%     initInfo =  InitialRT(initR,nl,T1A); 
%     
%     initT = initInfo(4:6);
%      
%     k = 1; 
%     R{k} = initR;
%     T(:,k) = initT;
%     xP{k} = initInfo(1:3);
%     Pc{k} = xP{k};
%     
%     T1P1_w = [256 256 256;256 256 256;256 256 256;256 256 xP{k}(3);256 xP{k}(2) 256;xP{k}(1) 256 256];
%     T1P2_w = [256 256 xP{k}(3);256 xP{k}(2) 256;xP{k}(1) 256 256;256 xP{k}(2) 256;xP{k}(1) 256 256;256 256 xP{k}(3)];
%     
%     nLine = length(iT1p1');
% 	p1 = [iT1p1'; ones(1,nLine)];
% 	p2 = [iT1p2'; ones(1,nLine)];
%     
%     [rot_wc, pos_wc, err] =  GN(p1, p2, T1P1_w', T1P2_w', initR);		
%     
%     errormin = inf;
%     count = 1;
%     while k <= 50 && count < 500
%         T1P1_w = [256 256 256;256 256 256;256 256 256;256 256 xP{k}(3);256 xP{k}(2) 256;xP{k}(1) 256 256];
%         T1P2_w = [256 256 xP{k}(3);256 xP{k}(2) 256;xP{k}(1) 256 256;256 xP{k}(2) 256;xP{k}(1) 256 256;256 256 xP{k}(3)];
%         d = xnorm((T1P1_w-T1P2_w)');
%         [tempR,tempK] = Roptimzation(T1p1, T1p2,T1A,R{k},d,In);
%         tempInfo = InitialT(tempR,nl,T1A);
%         xP{k} = tempInfo(1:3);
%         p = [256 256 256;256 256 xP{k}(3);256 xP{k}(2) 256;xP{k}(1) 256 256;];
%         Pc{k} = xP{k};       
%         %         z1 = (P_L(1,1)*tempT(3)-f*tempT(1)-(f*tempR(1,1)-P_L(1,1)*tempR(1,1))* 256)/(f*tempR(1,3)-P_L(1,1)*tempR(3,3));
% %         z2 = (P_L(1,2)*tempT(3)-f*tempT(2)-(f*tempR(2,1)-P_L(1,2)*tempR(3,1))* 256)/(f*tempR(2,3)-P_L(1,2)*tempR(3,3));
% %         z = mean([z1,z2]);
% 
% 
%         
%         for i = 1:length(T1P1_w) 
%             xtemp1(i,:) = [R{k} tempInfo(4:6)]*[T1P1_w(i,:) 1].';
%             xtemp2(i,:) = [R{k} tempInfo(4:6)]*[T1P2_w(i,:) 1].';
%         end
%         
%         
%         for i = 1:length(T1p1)
%             temp1 = inv(T1A)*[T1p1(i,:) 1].';
%             temp1 = (temp1/temp1(3)).';
%             iT1p1(i,:) = temp1(:,1:2);
%             temp2 = inv(T1A)*[T1p2(i,:) 1].';
%             temp2 = (temp2/temp2(3)).';
%             iT1p2(i,:) = temp2(:,1:2);
%         end
%         
%         [tempNR, tempNT] = ASPnL(xtemp1', xtemp2', T1P1_w', T1P2_w');
%         
% %         cp = [iT1p1(3:6,:) ones(4,1)];
%         cp = xtemp1(3:6,:);
%         mp = sum(p)/length(p);
%         mcp = sum(cp)/length(cp);
%         p_ = p - mp;
%         cp_ = cp - mcp;
%         
%         M = zeros(3,3);
%         for i = 1:length(p)
%             M = M + p_(i,:)'*cp_(i,:);
%         end
%         
%         [u, m, v] = svd(M);
%         TR = v*u';
%         
%         tempInfo = InitialT(TR,nl,T1A);
%         tempNT = tempInfo(4:6);
%         
%         xP{k} = tempInfo(1:3);
%         T1P1_w = [256 256 256;256 256 256;256 256 256;256 256 xP{k}(3);256 xP{k}(2) 256;xP{k}(1) 256 256];
%         T1P2_w = [256 256 xP{k}(3);256 xP{k}(2) 256;xP{k}(1) 256 256;256 xP{k}(2) 256;xP{k}(1) 256 256;256 256 xP{k}(3)];
%         p = [256 256 256;256 256 xP{k}(3);256 xP{k}(2) 256;xP{k}(1) 256 256;];
%       
% 
% % 
% % %         [tempNR, tempNT] = ASPnL(iT1p1(4:6,:)', iT1p2(4:6,:)', T1P1_w(4:6,:)', T1P2_w(4:6,:)');
% %         [tempNR1, tempNT1] =  efficient_pnp_gauss(p,tP,T1A);
% %              
% %         p = [T1P1_w(1,:);T1P1_w(4:6,:)]
%         temperror =  reprojection_error_usingRT(p,tP,TR,tempNT,T1A);
%         if temperror < errormin
%            errorT{k} = temperror;
%            k = k +1;
%            R{k} = TR;
%            T(:,k) = tempNT;
%            errormin = temperror;
%         end
%         count = count +1;
%     end
    toc
end