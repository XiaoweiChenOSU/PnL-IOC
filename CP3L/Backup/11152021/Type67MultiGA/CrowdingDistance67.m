%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%   Crowding Distance
%      
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function P = CrowdingDistance67(P,F)

m = 2; %number of objective values
nbFront = size(F,2); %number of values in edge F

%go through each front one by one
for i = 1:nbFront
    Fi = F{i};
    nbSolFront = size(Fi,2);
    
    valObj = [P(Fi).ValObjective];
    dist = zeros(nbSolFront, m);
    
    %Calculation of distance crowding for each objective
    for j = 1:m        
        [valTri, indTri] = sort(valObj(j,:));
        
        dist(indTri(1),j) = inf;
        for k = 2:nbSolFront-1
            dist(indTri(k),j) = abs(valTri(k+1)-valTri(k-1))/abs(valTri(1)-valTri(end));
        end
        dist(indTri(end),j) = inf;
    end
    
    for j = 1:nbSolFront
        P(Fi(j)).CrowdingDistance = sum(dist(j,:));
    end

end