function s = P2LSGA(xs, xe, Pn1_w, Pn2_w, cH, Height, K)
%P3LGA Summary of this function goes here
%   Detailed explanation goes here

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%   Genetic algorithm NSGA2
%      
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


Pc = 0.6;      % Probability of CrossOver
Pm = 0.1;      % Probability of Mutation
PopSize = 1000;  % Population Size
Gen = 5000;      % Generation
Len1 = 24;      % Bits of x
Len2 = 24;     % Bits of y
Len3 = 24;      % Bits of z
Len4 = 24;      % Bits of t1
lens = [Len1,Len2,Len3,Len4];
totalLen = Len1+Len2+Len3+Len4;
tic

% Binary encoding initialization
Genes = logical(randi([0 1], PopSize, totalLen));
BestGenes = ones(Gen, totalLen);
BestGenesEver = ones(Gen, totalLen);
BestGene = Genes(1,:);
BestGeneEver = Genes(1,:);
Fits = zeros(PopSize, 1) + Inf;
BestFits = zeros(Gen, 1) + Inf;
BestFitsEver = zeros(Gen, 1) + Inf;
BestFit = 0;
BestFitEver = Inf;
GenEver = 0;
point1 = zeros(Gen, 1)+Inf;
point2 = zeros(Gen, 1)+Inf;
point3 = zeros(Gen, 1)+Inf;
point4 = zeros(Gen, 1)+Inf;
range{1} = [floor(Pn1_w(1,2)/10)*10, floor(Pn1_w(1,2)/10)*10+10];
range{2}= [floor(Pn2_w(1,2)/10)*10, floor(Pn2_w(1,2)/10)*10+10];
range{3} = [floor(Pn1_w(2,2)/10)*10, floor(Pn1_w(2,2)/10)*10+10];
range{4} = [floor(Pn2_w(2,2)/10)*10, floor(Pn2_w(2,2)/10)*10+10];

% GeneticAlg
for i=1:Gen
    % compute fitness
    [y1, y2, y3, y4] = Decoding(Genes,lens,range);
    s = [y1, y2, y3, y4];
    Fits = Fitness(s, xs, xe, Pn1_w, Pn2_w, cH);
    
    
    % save the best gene
    BestFit = min(Fits);
    GeneRow = find(Fits==BestFit,1); % save only the first index
    BestGene = Genes(GeneRow,:);
    BestFits(i) = BestFit;
    BestGenes(i,:) = BestGene(1,:);
    if BestFit < BestFitEver
        BestFitEver = BestFit;
        BestFitsEver(i) = BestFit;
        BestGeneEver(1,:) = BestGene(1,:);
        BestGenesEver(i,:) = BestGene(1,:);
        GenEver = i;
        point1(i) = y1(GeneRow);
        point2(i) = y2(GeneRow);
        point3(i) = y3(GeneRow);
        point4(i) = y4(GeneRow);
    else
        BestFitsEver(i) = BestFitEver;
        BestGenesEver(i,:) = BestGeneEver(1,:);
    end
    
    % Selection Operator
    Genes = TournamentSelection(Genes, Fits, PopSize);
%     Genes = RouletteSelection(Genes, Fits, PopSize);

     % CrossOver Operator
    Genes = CrossOver(Genes, Pc);
    
    % Mutation Operator
%     Genes = BitMutation(Genes, Pm);
    Genes = IndividualMutation(Genes, Pm);
end
toc

% plot preparation
dense = 100;
x1 = linspace(range(1), range(2), dense);
x2 = linspace(range(1), range(2), dense);
F = zeros(40);
for i=1:dense
    for j=1:dense
        F(i,j) = Fitness(x1(i),x2(j));
    end
end
num = find(point1~=Inf);
p1 = point1(num);
p2 = point2(num);
p3 = point3(num);
p4 = point4(num);
FitEver = Fitness([p1,p2,p3,p4], xs, xe, Pn1_w, Pn2_w, cH);
t = length(num);

s = [p1(t),p2(t),p3(t),p4(t),p5(t),p6(t)];

plot
figure(1)
% surf(x1,x2,F)
hold on 
plot3(p1,p2,FitEver,'r.-')
FB = Fitness([p1(t),p2(t),p3(t),p4(t),p5(t),p6(t)],xs, xe, Pn1_w, Pn2_w);
plot3(p1(t),p2(t),FB,'y*')
xlabel('x1')
ylabel('x2')
zlabel('F(x)')
title(['where x1 = ',num2str(p1(t)),'   x2 = ',num2str(p2(t)),'  minF(x) = ',num2str(FB)])
hold off

figure(2)
plot(BestFits)
hold on 
grid on
plot(BestFitsEver,'r')
plot(GenEver,BestFitsEver(GenEver),'r*')
xlabel('Generation')
ylabel('F(x)')
% ylim([-1 6])
title(['Generation = ',num2str(GenEver),'   F(x) = ',num2str(BestFitEver)])
hold off
