%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%   Simulated Binary Crossover
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function Enfants = SimulatedBinaryCrossoverBackup(individuVide, GAParameters, ProblemParameters, MP)

%-----------------------------------------
%	Init variables
%-----------------------------------------
n = 3;
pc = GAParameters.Pc;
if (ProblemParameters.MultiLimit)
    US = ProblemParameters.StartUpperLimit;
    LS = ProblemParameters.StartLowerLimit;
else
    US = ProblemParameters.UpperLimit;
    LS = ProblemParameters.LowerLimit;
end
L = ProblemParameters.LowerLimit;
U = ProblemParameters.UpperLimit;
NbDecVar = ProblemParameters.NbVariablesDecision;

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
        Bi = power(2*ui,1/1+n);
    elseif(ui > 0.5)
        Bi = power(2*(1-ui),-(1+1/n));
    end
       
    %Crossover with probability of crossing pc
    if(rand < pc)
        for k = 1:2
            Enfants(j,k).Val = 0.5*(P1.Val+P2.Val) + 0.5*Bi*(P1.Val-P2.Val);
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
    for j = 1:NbDecVar
        if j == 1
            if  Enfants(i).Val(j) < LS
                Enfants(i).Val(j) = LS;
            elseif Enfants(i).Val(j) > US
                Enfants(i).Val(j) = US;                    
            end 
        else
            if  Enfants(i).Val(j) < L
                Enfants(i).Val(j) = L;
            elseif Enfants(i).Val(j) > U
                Enfants(i).Val(j) = U;                    
            end                  
        end         
    end
end

