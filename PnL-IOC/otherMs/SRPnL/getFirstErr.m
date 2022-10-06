function err = getFirstErr(p1,p2,W1,W2,R)
%GETERR Summary of this function goes here
%   Detailed explanation goes here
    [Vw, Pw] = getVP(W1, W2);
    n_c=xnorm(cross(p1,p2));
    nLine = length(p1);
    err = 0;
    for i = 1:nLine
        err = err + abs(n_c(:,i)'*R*Vw(:,i));
    end
    err = err/nLine;
end

