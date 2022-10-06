function [R,T] = RToptimzation(pw1, pw2, PW1, PW2, Rs, In)
 
  
    nl = getProjNorm(pw1,pw2);
    % nl = getProjNorm(iT1p2',iT1p1');

    Rt{1} = Rs;


    np = length(pw1);
    for i = 1:np
        Kp{i} = eye(3) - nl(:,i)*nl(:,i)';
    end

    d=PW1-PW2;
    d = xnorm(d);

    B = d;
    

    k = 1;
    while k <= In

        for i = 1:np
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
                S = eye(3);
            elseif det(U)*det(V) == -1
                S = [1 0 0; 0 1 0; 0 0 -1];
            end
        end

        k = k +1;
        Rtemp = U*S*V'; 
        TK1 = zeros(3,3);
        TK2 = zeros(3,1);
        PA = (PW1+PW2)/2;
        for i = 1:np
            TK1 = TK1 + (eye(3)-Kp{i});
            if i < 4
                TK2 = TK2 + (Kp{i} - eye(3))* Rtemp * PA(:,i);
                piw(:,i) = PA(:,i);
            else
                TK2 = TK2 + (Kp{i} - eye(3))* Rtemp * PW1(:,i);
                piw(:,i) = PW1(:,i);
            end
        end
        Ttemp = TK1\TK2;
        for i = 1:np
            qic(:,i) = Rtemp*piw(:,i)+ Ttemp;
        end
        
        pa = mean(piw,2);
        qa = mean(qic,2);
        
        piw = piw - pa;
        qic = qic - qa;
        
        M = zeros(3,3);
        for i = 1:np
            M = M + piw(:,i)*qic(:,i)';
        end
        
        [UM, SM, VM] = svd(M);
        
        Rt{k} = VM*UM';
        
        TK1 = zeros(3,3);
        TK2 = zeros(3,1);
        for i = 1:np
            TK1 = TK1 + (eye(3)-Kp{i});
            if i < 4
                TK2 = TK2 + (Kp{i} - eye(3))* Rt{k} * PA(:,i);
            else
                TK2 = TK2 + (Kp{i} - eye(3))* Rt{k} * PW1(:,i);
            end
        end
        T{k} = TK1\TK2; 
    end
    R = Rt{k}; 
    T = T{k};
end