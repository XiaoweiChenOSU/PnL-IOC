%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%   Simulated Binary Crossover
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function Enfants = SimulatedBinaryCrossover74(individuVide, GAParameters, ProblemParameters, MP)

%-----------------------------------------
%	Init variables
%-----------------------------------------
n = 3;
pc = GAParameters.Pc;
s1U = ProblemParameters.s1UpperLimit;
s1L = ProblemParameters.s1LowerLimit;
s2U = ProblemParameters.s2UpperLimit;
s2L = ProblemParameters.s2LowerLimit;
s3U = ProblemParameters.s3UpperLimit;
s3L = ProblemParameters.s3LowerLimit; 


   

N = size(MP,1);
Enfants = repmat(individuVide, N, 1);

%-----------------------------------------
%	Parents route 2 by 2
%-----------------------------------------
j = 1;
for i = 1:2:N
    
    %Selection of parent pairs 
    P1 = MP(i);
    if(i < N)
        P2 = MP(i+1);
    else
        P2 = P1;
    end
    
    %Beta i
    ui = rand;
    if(ui <= 0.5)
        Bi = power(2*ui,1/(1+n));
    elseif(ui > 0.5)
        Bi = power(2*(1-ui),-(1+1/n));
    end
       
    %Crossover with probability of crossing pc
    if(rand < pc)
        alpha = P2.Rank/(P2.Rank + P1.Rank);
        for k = 1:2
            Enfants(j,k).Val(1:3) = alpha*(P1.Val(1:3)+P2.Val(1:3)) + alpha*Bi*(P1.Val(1:3)-P2.Val(1:3));
        end
        j = j + 1;
    else
        Enfants(j,1).Val = P1.Val;
        j = j + 1;

        Enfants(j,1).Val = P2.Val;    
        j = j + 1;
    end
end

%Restructuring of Children + erasing empty clues
Enfants = Enfants(:);
Enfants = Enfants(~cellfun(@isempty,{Enfants.Val}));

% %Feasibility operation
for i = 1:size(Enfants)
    if  Enfants(i).Val(1) < s1L
        Enfants(i).Val(1) = s1L;
    elseif Enfants(i).Val(1) > s1U
        Enfants(i).Val(1) = s1U;
    end 
    if  Enfants(i).Val(2) < s2L
        Enfants(i).Val(2) = s2L;
    elseif Enfants(i).Val(2) > s2U
        Enfants(i).Val(2) = s2U;                    
    end
    if  Enfants(i).Val(3) < s3L
        Enfants(i).Val(3) = s3L;
    elseif Enfants(i).Val(3) > s3U
        Enfants(i).Val(3) = s3U;
    end
  
end

