%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%   Non Dominated Sorting
%      
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [P, F] = NonDominatedSorting77(P)

%-----------------------------------------
%	Init variables
%-----------------------------------------
N = size(P,1);
F{1}=[];

%-----------------------------------------
% Comparison loop between
%   the individuals
%-----------------------------------------
for i = 1:N
    %Selection of an individual
    individu = P(i);
    individu.DominationSet=[];
    individu.DominatedCount=0;

    for j = 1:N
        %Avoid comparing an individual to himself
        if j==i
            continue;
        end

        % Dominance of the individual => we add the individual compared in "DominationSet"
        individuCompar = P(j);
        
        %Dominance ratio
        if RappDom(individu,individuCompar)
            % Dominance of the individual => we add the individual compared in
            % "DominationSet"
            individu.DominationSet = [individu.DominationSet j];
        elseif RappDom(individuCompar,individu)
            % Dominance of the compared individual => the counter is increased dominated
            % of individual
            individu.DominatedCount = individu.DominatedCount+1;
        end
    end
    
    % Rank 1 for the individual who dominates all the others
    if individu.DominatedCount == 0
        individu.Rank = 1;
        F{1}=[F{1} i];
    end
    
    % We reinsert the changes of an individual in the table
    P(i) = individu;
end

f = 1; % row number

%-----------------------------------------
% Ranking loop by rank
% Population
%-----------------------------------------
while true
    Q = [];

    for i = 1:size(F{f},2)
        individu = P(F{f}(i));

        for j = 1:size(individu.DominationSet,2)
            indivDom = P(individu.DominationSet(j));          
            indivDom.DominatedCount = indivDom.DominatedCount - 1;

            if(indivDom.DominatedCount == 0)
                indivDom.Rank = f + 1; % next row
                Q = [Q individu.DominationSet(j)];
            end
            
            P(individu.DominationSet(j)) = indivDom; %major of a dominated individual
        end
    end
    
    %Infinite loop exit condition
    if isempty(Q)
        break;
    end

    F{f+1} = Q;
    f = f + 1;
end