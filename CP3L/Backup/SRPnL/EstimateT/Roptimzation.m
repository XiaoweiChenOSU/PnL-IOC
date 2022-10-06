function R = Roptimzation(P0, P_L,A,Rpnl,d,In)
 
  
    T1p1 = P0;
    T1p2 = P_L;

    T1A = A;

    for i = 1:length(T1p1)
        temp1 = inv(T1A)*[T1p1(i,:) 1].';
        temp1 = (temp1/temp1(3)).';
        iT1p1(i,:) = temp1(:,1:2);
        temp2 = inv(T1A)*[T1p2(i,:) 1].';
        temp2 = (temp2/temp2(3)).';
        iT1p2(i,:) = temp2(:,1:2);
    end

    nl = getProjNorm(iT1p1',iT1p2');
    % nl = getProjNorm(iT1p2',iT1p1');

    Rt{1} = Rpnl;

    K1 = eye(3) - nl(:,1)*nl(:,1)';
    K2 = eye(3) - nl(:,2)*nl(:,2)';
    K3 = eye(3) - nl(:,3)*nl(:,3)';
    
  

    B = [d(1,:); d(2,:); d(3,:)];

    k = 1;
    while k <= In

        A = [K1*Rt{k}*d(1,:)' K2*Rt{k}*d(2,:)' K3*Rt{k}*d(3,:)'];

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
        
    end
    R = Rt{k};
end