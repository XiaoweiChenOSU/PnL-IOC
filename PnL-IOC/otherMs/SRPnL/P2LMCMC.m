function [R,T,Ierr] = P2LMCMC(p1,p2,W1,W2,cH)
    
    Ym = 128;
    
    y1 = rand(1,1)*Ym;                            % an initial position
    y2 = rand(1,1)*Ym;
    y3 = Ym + rand(1,1)*Ym;
    y4 = Ym + rand(1,1)*Ym;
    %p1 is the initial points 
    
    W1(1,2) = y1;
    W1(2,2) = y2;
    W1(3,2) = y1;
    W1(4,2) = y3;
    
    W2(1,2) = y3;
    W2(2,2) = y4;
    W2(3,2) = y2;
    W2(4,2) = y4;
    

   

    Kmax=1000;                               % the number of random walks
%     [R0,T0, Ierr] = APSRPnLMCMC(ip1', ip2', W1, W2, cH, A); % the initial error
    [R0, T0, Ierr1] = SRPnL6(p1', p2', W1', W2'); 
    tempCT = -inv(R0)*T0;
    Ierr2 = (tempCT(2) - cH)^2;
    Ierr = Ierr1 + Ierr2;
    for j=1:Kmax
%         step = 0.005*(rand(1,2)-0.5);  %quite good
        step = 5*(rand(1,1)-0.5);
        y1 = y1 + step;                            % an initial position
        step = 5*(rand(1,1)-0.5);
        y2 = y2 + step;
        step = 5*(rand(1,1)-0.5);
        y3 = y3 + step;
        step = 5*(rand(1,1)-0.5);
        y4 = y4 + step;
        NW1 = W1;
        NW2 = W2;
        NW1(1,2) = y1;
        NW1(2,2) = y2;
        NW1(3,2) = y1;
        NW1(4,2) = y3;

        NW2(1,2) = y3;
        NW2(2,2) = y4;
        NW2(3,2) = y2;
        NW2(4,2) = y4;
        [NR, NT, Ierr1] = SRPnL6(p1', p2', NW1', NW2'); 
        tempCT = -inv(NR)*NT;
        Ierr2 = (tempCT(2) - cH)^2;
        Ierrcur = Ierr1 + Ierr2;

        v=min(1,Ierrcur/Ierr);                     % compute the acceptance ratio
        u=rand;
        if u < v
          W1 = NW1;
          W2 = NW2;
        end
    end
    [R, T, Ierr] = SRPnL6(p1', p2', W1', W2'); 
 end
