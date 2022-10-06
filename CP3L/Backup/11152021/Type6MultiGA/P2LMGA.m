function [F1, G] = P2LMGA(xs, xe, Pn1_w, Pn2_w, cH, ProblemParameters)
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
individuVide.R = [];
individuVide.T = [];

%-----------------------------------------
%   Init GA paprameters
%-----------------------------------------
GAParameters.PopSize = 100; %temp
GAParameters.ArchiveSize = 50; %temp
GAParameters.Gmax = 100; %temp
GAParameters.Pc = 0.9; %temp
GAParameters.Pm = 0.1; %temp
GAParameters.pSwap=0.2;                              %?????????
GAParameters.pReversion=0.5;                         %?????????
GAParameters.pInsertion=1-GAParameters.pSwap-GAParameters.pReversion;          %?????????

%% ????????
GAParameters.MaxOutIter=20;                         %??????????
GAParameters.MaxInIter=5;                           %??????????
GAParameters.T0=0.025;                               %????
GAParameters.alpha=0.99;                             %????



GAParameters.p1 = xs; %temp
GAParameters.p2 = xe; %temp
GAParameters.W1 = Pn1_w; %temp
GAParameters.W2 = Pn2_w; %temp
GAParameters.cH = cH; %temp



%-----------------------------------------
%   Initialization of the first population Po
%-----------------------------------------
Po = InitialisationNSGA2(individuVide, ProblemParameters,GAParameters);
G = 1;

%-----------------------------------------
%   Po (P) evaluation
%-----------------------------------------
P = EvaluationNSGA2(Po,GAParameters);

% if isValid == 0
%     F1 = [];
%     G = [];
%     return
% end


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


Pini = P;

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
    Enfants = SimulatedBinaryCrossover(individuVide, GAParameters, ProblemParameters, MP);

    %-----------------------------------------
    %   Mutation (polynomial mutation)
    %-----------------------------------------
    Mutants = PolynomialMutation(GAParameters, ProblemParameters, Enfants);
  
    SelCh=SA_Chrom(Mutants,GAParameters,ProblemParameters);
    
    Mutants = [Mutants;SelCh];
    
    Mutants = EvaluationNSGA2(Mutants,GAParameters);

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
    ResultatsDisplay(F1, G);
    
    G = G+1;
end
toc

%-----------------------------------------
%  Final results display
%-----------------------------------------
ResultatsDisplay(F1, G);

end


function newChrom=SA_Chrom(Chrom,GAParameters,ProblemParameters)
    NIND=size(Chrom,1);         %????
    Obj=EvaluationNSGA2(Chrom,GAParameters); %??????????
    [R, F] = NonDominatedSorting(Obj);
    R = CrowdingDistance(R,F);
    [~, index] = SortPopulation(R); %?????????????????????
    index = index{1};
%     [~,index]=sort(Obj);       
    SA_num=ceil(NIND*0.3);      %???30%???????????
    SA_Chrom=Chrom(index(1:SA_num),:);  %???????????
    newChrom=Chrom;             %???????????
    for i=1:SA_num
        Individual=SA_Chrom(i,:);  %SA_Chrom??i?????Chrom???index(i)???
        %?SA_Chrom??i??????????????????
        newIndividual=SA_Individual(Individual,GAParameters,ProblemParameters);
        newChrom(index(i),:)=newIndividual;    %???????index(i)???
    end
end


function newIndividual=SA_Individual(Individual,GAParameters,ProblemParameters)
    %% ???
    currIndividual=Individual;               %?????????
    currL=EvaluationNSGA2(currIndividual,GAParameters);  %??????
    bestIndividual=currIndividual;                %??????????????
    bestL=currL;                        %????????????????????
    T=GAParameters.T0;                               %?????
    %% ????
    for outIter=1:GAParameters.MaxOutIter
        for inIter=1:GAParameters.MaxInIter
            newIndividual=Neighbor(currIndividual,ProblemParameters);       %??????????????
            newL=EvaluationNSGA2(newIndividual,GAParameters);                             %???????
            %??????????????????????????????
            if newL.ValObjective(1)<=currL.ValObjective(1) 
                currIndividual=newIndividual; 
                currL=newL;
            else 
                %???????????????????????????????
                delta=(newL.ValObjective(1)-currL.ValObjective(1) )/currL.ValObjective(1);           %???????????????????
                P=exp(-delta/T);                    %??????????
                %??0~1??????P?????????????????????????
                if rand<=P
                    currIndividual=newIndividual; 
                    currL=newL;
                end
            end
            %???????????????????????????????????????????????
            if currL.ValObjective(1)<=bestL.ValObjective(1)
                bestIndividual=currIndividual;
                bestL=currL;
            end
        end
        %??????
        T=GAParameters.alpha*T;
    end
    newIndividual=bestIndividual;                %?bestRoute???newIndividual?????Individual
end




