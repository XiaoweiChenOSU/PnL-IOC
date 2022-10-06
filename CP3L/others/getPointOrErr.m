function err = getPointOrErr(p1,p2,W1,W2,R,T)
%GETERR Summary of this function goes here
%   Detailed explanation goes here
    n_c=xnorm(cross(p1,p2));
    [~,nLine] = size(W1);
    err = 0;
    for i = 1:nLine
        err = err + abs(n_c(:,i)'*(R*W1(:,i)+T));
        err = err + abs(n_c(:,i+1)'*(R*W2(:,i)+T));
    end
    err = err/nLine/2;
end