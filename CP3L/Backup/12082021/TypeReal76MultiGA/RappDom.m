%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Dominance ratio
%
% Check if an individual dominates a
% other
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function isDom = RappDom(ind1, ind2)

    if isstruct(ind1)
        ind1 = ind1.ValObjective;
    end

    if isstruct(ind2)
        ind2 = ind2.ValObjective;
    end

    isDom = all(ind1<=ind2) && any(ind1<ind2);