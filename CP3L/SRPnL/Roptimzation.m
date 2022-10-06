function [R,Kp] = Roptimzation(p1, p2,IA,Rpnl,d,In)
 
  
%     T1p1 = P0;
%     T1p2 = P_L;

    T1A = IA;

%     for i = 1:length(T1p1)
%         temp1 = inv(T1A)*[T1p1(i,:) 1].';
%         temp1 = (temp1/temp1(3)).';
%         iT1p1(i,:) = temp1(:,1:2);
%         temp2 = inv(T1A)*[T1p2(i,:) 1].';
%         temp2 = (temp2/temp2(3)).';
%         iT1p2(i,:) = temp2(:,1:2);
%     end
% 
    nl = getProjNorm(p1,p2);
    % nl = getProjNorm(iT1p2',iT1p1');

    Rt{1} = Rpnl;
    
    for i = 1:length(p1)
        Kp{i} = eye(3) - nl(:,i)*nl(:,i)';
    end



    B = d;
    

    k = 1;
    while k <= In

        for i = 1:length(p1)
            A(:,i) = Kp{i}*Rt{k}*d(:,i);
        end
       
        M  = A*B';

        [U ,D ,V] = svd(M);

        if rank(M) > 2
            if det(M) >= 0
                S = eye(3);
            else
                S = [1 0 0; 0 1 0; 0 0 -1];
            end
        else
            if det(U)*det(V) == 1
                S = eye(3)
            elseif det(U)*det(V) == -1
                S = [1 0 0; 0 1 0; 0 0 -1];
            end
        end

        k = k +1;
        Rt{k} = U*S*V'; 
        
%         [RAt,aPointsCh] = CalculateTbyR(p1, p2, W1, W2, cH, RrA);
        
    end
    R = Rt{k};    
end