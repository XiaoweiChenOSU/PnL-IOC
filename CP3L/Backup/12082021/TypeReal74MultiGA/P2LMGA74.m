function [F1, G] = P2LMGA74(xsA, xeA, Pn1_wA, Pn2_wA, xsB, xeB, Pn1_wB, Pn2_wB, cH, A, Lwidth, Lheight)
%P3LGA Summary of this function goes here
%   Detailed explanation goes here

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
individuVide.RA = [];
individuVide.TA = [];
individuVide.RB = [];
individuVide.TB = [];

%-----------------------------------------
%   Init GA paprameters
%-----------------------------------------
GAParameters.PopSize = 100; %temp
GAParameters.ArchiveSize = 50; %temp
GAParameters.Gmax = 200; %temp
GAParameters.Pc = 0.8; %temp
GAParameters.Pm = 0.1; %temp
GAParameters.pSwap=0.2;                              %?????????
GAParameters.pReversion=0.5;                         %?????????
GAParameters.pInsertion=1-GAParameters.pSwap-GAParameters.pReversion;          %?????????

%% ????????
GAParameters.MaxOutIter=20;                         %??????????
GAParameters.MaxInIter=5;                           %??????????
GAParameters.T0=0.025;                               %????
GAParameters.alpha=0.99;                             %????



GAParameters.p1A = xsA; %temp
GAParameters.p2A = xeA; %temp
GAParameters.W1A = Pn1_wA; %temp
GAParameters.W2A = Pn2_wA; %temp
GAParameters.p1B = xsB; %temp
GAParameters.p2B = xeB; %temp
GAParameters.W1B = Pn1_wB; %temp
GAParameters.W2B = Pn2_wB; %temp
GAParameters.cH = cH; %temp
GAParameters.A = A; %temp
GAParameters.Lwidth = Lwidth; %temp
GAParameters.Lheight = Lheight; %temp

ProblemParameters = SetProblemParameters74(GAParameters);

%-----------------------------------------
%   Initialization of the first population Po
%-----------------------------------------
Po = InitialisationNSGA274(individuVide, ProblemParameters,GAParameters);
G = 1;

%-----------------------------------------
%   Po (P) evaluation
%-----------------------------------------
P = EvaluationNSGA274(Po,GAParameters);

% if isValid == 0
%     F1 = [];
%     G = [];
%     return
% end


%-----------------------------------------
%   Non Dominated Sorting
%-----------------------------------------
[P, F] = NonDominatedSorting74(P);

%-----------------------------------------
%   Crowding Distance
%-----------------------------------------
P = CrowdingDistance74(P,F);

%-----------------------------------------
%  Sorting by row then by distance
%-----------------------------------------
[P, ~] = SortPopulation74(P);


Pini = P;

%-----------------------------------------
%  Iteration for Gmax generations
%-----------------------------------------
while (G < GAParameters.Gmax)
    
    %-----------------------------------------
    %   MP selection by binary tournament
    %-----------------------------------------
    MP = SelectionTournoi74(individuVide, P);

    %-----------------------------------------
    %   Crossover (simulated binary crossover)
    %-----------------------------------------
    Enfants = SimulatedBinaryCrossover74(individuVide, GAParameters, ProblemParameters, MP);

    %-----------------------------------------
    %   Mutation (polynomial mutation)
    %-----------------------------------------
    Mutants = PolynomialMutation74(GAParameters, ProblemParameters, Enfants);
  
%     SelCh=SA_Chrom(Mutants,GAParameters,ProblemParameters);
%     
%     Mutants = [Mutants;SelCh];
    
    Mutants = EvaluationNSGA274(Mutants,GAParameters);

    %-----------------------------------------
    %   Parents + Children transferred
    %-----------------------------------------
    R = [P ; Mutants];
    
    
    
    %-----------------------------------------
    %   Non Dominated Sorting & Crowding Distance
    %-----------------------------------------
    [R, F] = NonDominatedSorting74(R);
    R = CrowdingDistance74(R,F);
    
    %-----------------------------------------
    %   Sorting by row then by distance
    %-----------------------------------------
    [Q, ~] = SortPopulation74(R);

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
    [P, F] = NonDominatedSorting74(P);
    P = CrowdingDistance74(P,F);
    
    %-----------------------------------------
    %   Sorting by row then by distance
    %-----------------------------------------
    [P, F] = SortPopulation74(P);
    
    % Store F1
    F1 = P(F{1});

    %-----------------------------------------
    %   Final results display
    %-----------------------------------------
%     ResultatsDisplay(F1, G);
    
    G = G+1;
end

%-----------------------------------------
%  Final results display
%-----------------------------------------
% ResultatsDisplay(F1, G);

end
