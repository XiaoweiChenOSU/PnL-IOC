% GA for unconstrained single-objective optimization
%   max F(x1,x2) = 21.5 + x1*sin(4*pi*x1) + x2*sin(20*pi*x2)
%   S.t.  -3.0 <= x1 <= 12.1
%          4.1 <= x2 <= 5.8
%   The precision required is 0.0001
%   
%   March 30, 2017, by HanzheTeng

% Parameters
clear variables
Pc = 0.5;      % Probability of CrossOver
Pm = 0.2;      % Probability of Mutation
PopSize = 10000;  % Population Size
Gen = 1000;      % Generation
Len1 = 22;      % Bits of x
Len2 = 22;     % Bits of y
Len3 = 22;      % Bits of z
Len4 = 24;      % Bits of t1
Len5 = 24;     % Bits of t2
Len6 = 24;      % Bits of t3
lens = [Len1,Len2,Len3,Len4,Len5,Len6];
range{1} = [257, 512];
range{2}= [1, 256];
range{3} = [257, 512];
range{4} = [0, 512];
range{5}= [0, 512];
range{6} = [0, 512];
totalLen = Len1+Len2+Len3+Len4+Len5+Len6;
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
point5 = zeros(Gen, 1)+Inf;
point6 = zeros(Gen, 1)+Inf;

% GeneticAlg
for i=1:Gen
    % compute fitness
    [x,y,z,t1,t2,t3] = Decoding(Genes,lens,range);
    Fits = Fitness(x,y,z,t1,t2,t3);
    
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
        point1(i) = x(GeneRow);
        point2(i) = y(GeneRow);
        point3(i) = z(GeneRow);
        point4(i) = t1(GeneRow);
        point5(i) = t2(GeneRow);
        point6(i) = t3(GeneRow);
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
% dense = 100;
% x1 = linspace(range(1), range(2), dense);
% x2 = linspace(range(1), range(2), dense);
% F = zeros(40);
% for i=1:dense
%     for j=1:dense
%         F(i,j) = Fitness(x1(i),x2(j));
%     end
% end
num = find(point1~=Inf);
p1 = point1(num);
p2 = point2(num);
p3 = point3(num);
p4 = point4(num);
p5 = point5(num);
p6 = point6(num);
FitEver = Fitness(p1,p2,p3,p4,p5,p6);
t = length(num);

% plot
figure(1)
% surf(x1,x2,F)
hold on 
plot3(p1,p2,FitEver,'r.-')
FB = Fitness(p1(t),p2(t),p3(t),p4(t),p5(t),p6(t));
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