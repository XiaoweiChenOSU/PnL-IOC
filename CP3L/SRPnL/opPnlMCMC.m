function [Rf,ip,Ierr] = opPnlMCMC(p1,p2,W1,W2,A,cH)
    
    %p1 is the initial points 
    
    for i = 1:length(p1)
        temp1 = inv(A)*[p1(i,:) 1].';
        temp1 = (temp1/temp1(3)).';
        ip1(i,:) = temp1(:,1:2);
        temp2 = inv(A)*[p2(i,:) 1].';
        temp2 = (temp2/temp2(3)).';
        ip2(i,:) = temp2(:,1:2);
    end 
   

    Kmax=1000;                               % the number of random walks
%     [R0,T0, Ierr] = APSRPnLMCMC(ip1', ip2', W1, W2, cH, A); % the initial error
    [R0, Ierr] = APSRPnLMCMC1(ip1', ip2', W1, W2, cH);    
    for j=1:Kmax
%         step = 0.005*(rand(1,2)-0.5);  %quite good
        step = 0.015*(rand(1,2)-0.5);
        tp1=p1+[1;1;1]*step;       % random walk
        for i = 1:length(p1)
            temp1 = inv(A)*[tp1(i,:) 1].';
            temp1 = (temp1/temp1(3)).';
            ip1(i,:) = temp1(:,1:2);
            temp2 = inv(A)*[p2(i,:) 1].';
            temp2 = (temp2/temp2(3)).';
            ip2(i,:) = temp2(:,1:2);
        end 
%        [R{j},T{j}, err{j}] = APSRPnLMCMC(ip1', ip2', W1, W2, cH, A);           % evaluate the likelihood
        [R{j}, err{j}] = APSRPnLMCMC1(ip1', ip2', W1, W2, cH);
%         v=min(1,err{j}/Ierr);                     % compute the acceptance ratio
%         u=rand;
        if err{j}<Ierr
%         if u < v
            p1 = tp1;
            ip = ip1;
            Rf = R{j};
%             Tf = T{j};
            Ierr = err{j};
        end
    end
 end
