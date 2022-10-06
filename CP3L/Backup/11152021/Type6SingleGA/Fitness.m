% Fitness (objective function)
function f = Fitness(s, T1p1, T1p2, Pn1_w, Pn2_w, cH)
    [n,~] = size(s);
    for j = 1:n
        tempS = s(j,:);
        Pn1_w(1,2) =  tempS(1);
        Pn1_w(2,2) =  tempS(3);
        Pn1_w(3,2) =  tempS(1);
        Pn1_w(4,2) =  tempS(2);
        Pn2_w(1,2) =  tempS(2);
        Pn2_w(2,2) =  tempS(4);
        Pn2_w(3,2) =  tempS(3);
        Pn2_w(4,2) =  tempS(4);

        T1A = [180 0 160;0 180 320;0 0 1];

        for i = 1:length(T1p1)
            temp1 = inv(T1A)*[T1p1(i,:) 1].';
            temp1 = (temp1/temp1(3)).';
            iT1p1(i,:) = temp1(:,1:2);
            temp2 = inv(T1A)*[T1p2(i,:) 1].';
            temp2 = (temp2/temp2(3)).';
            iT1p2(i,:) = temp2(:,1:2);
        end


        [R, T, err] = SRPnL6(iT1p1', iT1p2', Pn1_w', Pn2_w', cH);  

        f(j) = err;
    end
end