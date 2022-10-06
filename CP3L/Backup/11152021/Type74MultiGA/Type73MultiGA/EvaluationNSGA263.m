%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Assessment step
%
% Assignment of a 'quality' to each
% individual, a fitness value
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [P,isValid] = EvaluationNSGA2(Po,GAParameters)

%-----------------------------------------
%	Init variables
%-----------------------------------------
N = size(Po,1);
P = Po;

% P = repmat(individuVide, 2, 1);

% T1A = GAParameters.A; %temp
p1A = GAParameters.p1A; %temp
p2A = GAParameters.p2A; %temp
W1A = GAParameters.W1A; %temp
W2A = GAParameters.W2A; %temp
p1B = GAParameters.p1B; %temp
p2B = GAParameters.p2B; %temp
W1B = GAParameters.W1B; %temp
W2B = GAParameters.W2B; %temp
cH = GAParameters.cH; %temp

isValid = 0;
%-----------------------------------------
%	Assignment of fitness values
%-----------------------------------------

for i = 1:N
   
    [P(i).ValObjective,P(i).RA,P(i).RB,P(i).TA,P(i).TB] = CameraPose63(Po(i).Val,p1A,p2A,W1A,W2A,p1B,p2B,W1B,W2B,cH);
%     [ValObjective,R,T] = CameraPose(Po(i).Val,p1,p2,W1,W2,cH);
%     if ~isinf(ValObjective(1)) 
%         P(count).Val = Po(i).Val;
%         P(count).ValObjective = ValObjective;
%         P(count).R = R;
%         P(count).T = T;
%         count = count + 1;
%     end
    if ~isinf(P(i).ValObjective(1))
        isValid = 1;
    end
    
end




