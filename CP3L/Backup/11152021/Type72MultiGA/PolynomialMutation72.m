%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%   Polynomial Mutation
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function Mutants = PolynomialMutation72(GAParameters, ProblemParameters, Enfants)

%-----------------------------------------
%	Init variables
%-----------------------------------------
    s1U = ProblemParameters.s1UpperLimit;
    s1L = ProblemParameters.s1LowerLimit;
    s2U = ProblemParameters.s2UpperLimit;
    s2L = ProblemParameters.s2LowerLimit;
    s3U = ProblemParameters.s3UpperLimit;
    s3L = ProblemParameters.s3LowerLimit; 


    pm = GAParameters.Pm;
    n = 5;
    Mutants = Enfants;
    N = size(Enfants,1);

    %-----------------------------------------
    %	Children's route
    %-----------------------------------------
    for i = 1:N

        if(rand < pm)
            u = rand;
            if(u < 0.5)
                  deltaBar = power((2*u),(1/(n+1)))-1;
            elseif(u >= 0.5)
                  deltaBar = 1-power(2*(1-u),(1/n+1));
            end
            Mutants(i).Val(1) = abs(Enfants(i).Val(1) + (s1U-s1L)*deltaBar);
            Mutants(i).Val(2) = abs(Enfants(i).Val(2) + (s2U-s2L)*deltaBar);
            Mutants(i).Val(3) = abs(Enfants(i).Val(3) + (s3U-s3L)*deltaBar);
        end
    end

    %Feasibility operation
    for i = 1:size(Mutants)
        if  Mutants(i).Val(1) < s1L
            Mutants(i).Val(1) = s1L;
        elseif Mutants(i).Val(1) > s1U
            Mutants(i).Val(1) = s1U;
        end 
        if  Mutants(i).Val(2) < s2L
            Mutants(i).Val(2) = s2L;
        elseif Mutants(i).Val(2) > s2U
            Mutants(i).Val(2) = s2U;                    
        end
        if  Mutants(i).Val(3) < s3L
            Mutants(i).Val(3) = s3L;
        elseif Mutants(i).Val(3) > s3U
            Mutants(i).Val(3) = s3U;
        end

    end
end
