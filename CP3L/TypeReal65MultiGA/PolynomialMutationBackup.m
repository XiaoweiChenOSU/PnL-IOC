%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%   Polynomial Mutation
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function Mutants = PolynomialMutationBackup(GAParameters, ProblemParameters, Enfants)

%-----------------------------------------
%	Init variables
%-----------------------------------------
if (ProblemParameters.MultiLimit)
    US = ProblemParameters.StartUpperLimit;
    LS = ProblemParameters.StartLowerLimit;
    L = ProblemParameters.LowerLimit;
    U = ProblemParameters.UpperLimit;
    pm = GAParameters.Pm;
    n = 5;
    Mutants = Enfants;
    N = size(Enfants,1);
    NbDecVar = ProblemParameters.NbVariablesDecision;

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
            Mutants(i).Val(1) = abs(Enfants(i).Val(1) + (US-LS)*deltaBar);
            Mutants(i).Val(2:end) = abs(Enfants(i).Val(2:end) + (U-L)*deltaBar);
        end

    end

    %Feasibility operation
    for i = 1:size(Mutants)
        for j = 1:NbDecVar
            if j == 1
                if  Mutants(i).Val(j) < LS
                    Mutants(i).Val(j) = LS;
                elseif Mutants(i).Val(j) > US
                    Mutants(i).Val(j) = US;                    
                end 
            else
                if  Mutants(i).Val(j) < L
                    Mutants(i).Val(j) = L;
                elseif Mutants(i).Val(j) > U
                    Mutants(i).Val(j) = U;                    
                end                  
            end         
        end
    end
else
    L = ProblemParameters.LowerLimit;
    U = ProblemParameters.UpperLimit;
    pm = GAParameters.Pm;
    n = 5;
    Mutants = Enfants;
    N = size(Enfants,1);
    NbDecVar = ProblemParameters.NbVariablesDecision;

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

            Mutants(i).Val = abs(Enfants(i).Val + (U-L)*deltaBar);
        end

    end

    %Feasibility operation
    for i = 1:size(Mutants)
        for j = 1:NbDecVar
            if  Mutants(i).Val(j) < L
                Mutants(i).Val(j) = L;
            elseif Mutants(i).Val(j) > U
                Mutants(i).Val(j) = U;                    
            end
        end
    end 
end
