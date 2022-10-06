function [RR,tt,err]=TestIterative(p0,p_l,P0, P_L,f)

    P0 = [337 355;337 355;337 355];
    P_L  = [19 472;340 1;620 565];
  
%     p1 = [337 355;337 355;337 355;19 472;340 1;620 565];
%     p2 = [19 472;340 1;620 565;340 1;620 565;19 472];
%     
%     P1_w = [256 0 296;256 85 256;256 50 512;256 256 291;269 0 256;271 256 256;281 256 512;291 0 512];
%     P2_w = [256 0 381;256 110 256;256 160 512;256 256 366;286 0 256;286 256 256;291 256 512;301 0 512];



    T1A = [512 0 0;0 512 0;0   0   1];
    
    Rt = [0.515129347795564,-0.0363540396566050,-0.856341134618887;0.0306234324096387,-0.997681568573212,0.0607757609310401;-0.856565210843383,-0.0575314829457806,-0.512821770250533];

    d = [0 0 1; 0 1 0; 1 0 0];
    
    In = 80;
    initR = Roptimzation(P0, P_L,T1A,Rt,d,In);
    initInfo = EstimateT(initR); 
    
    initT = initInfo(4:6);
    

    f = 512;
    k = 1;
    R{k} = initR;
    T{k} = initT
    xP{k} = initInfo(1:3);
    while k <= 100
        tempR = Roptimzation(P0, P_L,T1A,R{k},d,20);
        tempInfo = EstimateT(tempR);
        tempT = tempInfo(4:6);
        xP{k} = tempInfo(1:3);
        p = [256 0 256;256 0 xP{k}(3);256 xP{k}(2) 256;xP{k}(1) 0 256;];
        
        
        z1 = (P_L(1,1)*tempT(3)-f*tempT(1)-(f*tempR(1,1)-P_L(1,1)*tempR(1,1))* 256)/(f*tempR(1,3)-P_L(1,1)*tempR(3,3));
        z2 = (P_L(1,2)*tempT(3)-f*tempT(2)-(f*tempR(2,1)-P_L(1,2)*tempR(3,1))* 256)/(f*tempR(2,3)-P_L(1,2)*tempR(3,3));
        z = mean([z1,z2]);
        q = (tempR*p'+ tempT)';
        pm = mean(p);
        qm = mean(q);
        p_ = p - pm;
        q_ = q - qm;
        
        [n,m] = size(p);
        M = zeros(m,m);
        for i = 1:n
            M = M + q_(i,:)'*p_(i,:);
        end
        
        [U, S, V] = svd(M);
        
        k = k +1;
        
        R{k} = V*U';
        tempfInfo = EstimateT(R{k});
        T{k} = tempfInfo(4:6);
    end
end