%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%   Genetic algorithm NSGA2
%      
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%-----------------------------------------
%   Init individual empty
%-----------------------------------------
individuVide.Val = [];
individuVide.ValObjective = [];
individuVide.Rank = [];
individuVide.CrowdingDistance = [];
individuVide.DominationSet = [];
individuVide.DominatedCount = [];

%-----------------------------------------
%   Init GA paprameters
%-----------------------------------------
GAParameters.PopSize = 100; %temp
GAParameters.ArchiveSize = 50; %temp
GAParameters.Gmax = 50000; %temp
GAParameters.Pc = 0.5; %temp
GAParameters.Pm = 0.5; %temp
% GAParameters.Pc = 0.95; %temp
% GAParameters.Pm = 0.05; %temp
% GAParameters.Pc = 0.2; %temp
% GAParameters.Pm = 0.4; %temp

ProblemId = 7;

ProblemParameters = SetProblemParameters(ProblemId);

%-----------------------------------------
%   Initialization of the first population Po
%-----------------------------------------
Po = InitialisationNSGA2(individuVide, ProblemParameters,GAParameters,ProblemId);
G = 1;

%-----------------------------------------
%   Po (P) evaluation
%-----------------------------------------
P = EvaluationNSGA2(individuVide,ProblemId,Po);

%-----------------------------------------
%   Non Dominated Sorting
%-----------------------------------------
[P, F] = NonDominatedSorting(P);

%-----------------------------------------
%   Crowding Distance
%-----------------------------------------
P = CrowdingDistance(P,F);

%-----------------------------------------
%  Sorting by row then by distance
%-----------------------------------------
[P, ~] = SortPopulation(P);

%-----------------------------------------
%  Iteration for Gmax generations
%-----------------------------------------
tic
while (G < GAParameters.Gmax)
    
    %-----------------------------------------
    %   MP selection by binary tournament
    %-----------------------------------------
    MP = SelectionTournoi(individuVide, P);

    %-----------------------------------------
    %   Crossover (simulated binary crossover)
    %-----------------------------------------
    if ProblemId >= 7
        Enfants = SimulatedBinaryCrossover(individuVide, GAParameters, ProblemParameters, MP);

        %-----------------------------------------
        %   Mutation (polynomial mutation)
        %-----------------------------------------
        Mutants = PolynomialMutation(GAParameters, ProblemParameters, Enfants);
    else
        Enfants = SimulatedBinaryCrossoverBackup(individuVide, GAParameters, ProblemParameters, MP);

        %-----------------------------------------
        %   Mutation (polynomial mutation)
        %-----------------------------------------
        Mutants = PolynomialMutationBackup(GAParameters, ProblemParameters, Enfants);
  
    end
    Mutants = EvaluationNSGA2(individuVide,ProblemId, Mutants);

    %-----------------------------------------
    %   Parents + Children transferred
    %-----------------------------------------
    R = [P ; Mutants];
    
    
    
    %-----------------------------------------
    %   Non Dominated Sorting & Crowding Distance
    %-----------------------------------------
    [R, F] = NonDominatedSorting(R);
    R = CrowdingDistance(R,F);
    
    %-----------------------------------------
    %   Sorting by row then by distance
    %-----------------------------------------
    [Q, ~] = SortPopulation(R);

    %-----------------------------------------
    %  We keep the first N (pop size) elements
    %-----------------------------------------
    [nQ,~] = size(Q);
    if nQ > GAParameters.PopSize
        P = Q(1:GAParameters.PopSize);
    end
    
    
    %-----------------------------------------
    %   Non Dominated Sorting & Crowding Distance
    %-----------------------------------------
    [P, F] = NonDominatedSorting(P);
    P = CrowdingDistance(P,F);
    
    %-----------------------------------------
    %   Sorting by row then by distance
    %-----------------------------------------
    [P, F] = SortPopulation(P);
    
    % Store F1
    F1 = P(F{1});

    %-----------------------------------------
    %   Final results display
    %-----------------------------------------
    ResultatsDisplay(F1, G, ProblemId);
    
    G = G+1;
end
toc
%-----------------------------------------
%  Final results display
%-----------------------------------------
ResultatsDisplay(F1, G, ProblemId);
