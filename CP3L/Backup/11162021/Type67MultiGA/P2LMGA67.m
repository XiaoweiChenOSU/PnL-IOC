function [F1, G] = P2LMGA67(xsA, xeA, Pn1_wA, Pn2_wA, xsB, xeB, Pn1_wB, Pn2_wB, xsC, xeC, Pn1_wC, Pn2_wC, xsD, xeD, Pn1_wD, Pn2_wD, cH, A, Lheight)
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
individuVide.RC = [];
individuVide.TC = [];
individuVide.RD = [];
individuVide.TD = [];



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





GAParameters.p1A = xsA; %temp
GAParameters.p2A = xeA; %temp
GAParameters.W1A = Pn1_wA; %temp
GAParameters.W2A = Pn2_wA; %temp
GAParameters.p1B = xsB; %temp
GAParameters.p2B = xeB; %temp
GAParameters.W1B = Pn1_wB; %temp
GAParameters.W2B = Pn2_wB; %temp
GAParameters.p1C = xsC; %temp
GAParameters.p2C = xeC; %temp
GAParameters.W1C = Pn1_wC; %temp
GAParameters.W2C = Pn2_wC; %temp
GAParameters.p1D = xsD; %temp
GAParameters.p2D = xeD; %temp
GAParameters.W1D = Pn1_wD; %temp
GAParameters.W2D = Pn2_wD; %temp
GAParameters.cH = cH; %temp
GAParameters.A = A; %temp
GAParameters.Lheight = Lheight; %temp


ProblemParameters = SetProblemParameters67(GAParameters);

%-----------------------------------------
%   Initialization of the first population Po
%-----------------------------------------
Po = InitialisationNSGA267(individuVide, ProblemParameters,GAParameters);
G = 1;

%-----------------------------------------
%   Po (P) evaluation
%-----------------------------------------
P = EvaluationNSGA267(Po,GAParameters);

% if isValid == 0
%     F1 = [];
%     G = [];
%     return
% end


%-----------------------------------------
%   Non Dominated Sorting
%-----------------------------------------
[P, F] = NonDominatedSorting67(P);

%-----------------------------------------
%   Crowding Distance
%-----------------------------------------
P = CrowdingDistance67(P,F);

%-----------------------------------------
%  Sorting by row then by distance
%-----------------------------------------
[P, ~] = SortPopulation67(P);



%-----------------------------------------
%  Iteration for Gmax generations
%-----------------------------------------
while (G < GAParameters.Gmax)
    
    %-----------------------------------------
    %   MP selection by binary tournament
    %-----------------------------------------
    MP = SelectionTournoi67(individuVide, P);

    %-----------------------------------------
    %   Crossover (simulated binary crossover)
    %-----------------------------------------
    Enfants = SimulatedBinaryCrossover67(individuVide, GAParameters, ProblemParameters, MP);

    %-----------------------------------------
    %   Mutation (polynomial mutation)
    %-----------------------------------------
    Mutants = PolynomialMutation67(GAParameters, ProblemParameters, Enfants);
  
%     SelCh=SA_Chrom(Mutants,GAParameters,ProblemParameters);
%     
%     Mutants = [Mutants;SelCh];
    
    Mutants = EvaluationNSGA267(Mutants,GAParameters);

    %-----------------------------------------
    %   Parents + Children transferred
    %-----------------------------------------
    R = [P ; Mutants];
    
    
    
    %-----------------------------------------
    %   Non Dominated Sorting & Crowding Distance
    %-----------------------------------------
    [R, F] = NonDominatedSorting67(R);
    R = CrowdingDistance67(R,F);
    
    %-----------------------------------------
    %   Sorting by row then by distance
    %-----------------------------------------
    [Q, ~] = SortPopulation67(R);

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
    [P, F] = NonDominatedSorting67(P);
    P = CrowdingDistance67(P,F);
    
    %-----------------------------------------
    %   Sorting by row then by distance
    %-----------------------------------------
    [P, F] = SortPopulation67(P);
    
    % Store F1
    F1 = P(F{1});

    %-----------------------------------------
    %   Final results display
    %-----------------------------------------
%     ResultatsDisplay67(F1, G);
    
    G = G+1;
end

%-----------------------------------------
%  Final results display
%-----------------------------------------
% ResultatsDisplay(F1, G);

end
