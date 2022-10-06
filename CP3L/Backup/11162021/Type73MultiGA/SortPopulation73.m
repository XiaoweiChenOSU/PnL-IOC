%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%   Sorting by row then by distance
%      
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [I, F] = SortPopulation73(I)

    % Sorted by Crowding Distance
    [~, CDSO] = sort([I.CrowdingDistance],'descend');
    I = I(CDSO);
    
    % Sorted by Crowding Distance
    [~, RSO] = sort([I.Rank]);
    I = I(RSO);
    
    % Updates the clues for each front
    Ranks = [I.Rank];
    MaxRank = max(Ranks);
    F = cell(MaxRank,1);

    for r = 1:MaxRank
        F{r} = find(Ranks==r);
    end
end