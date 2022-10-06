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
p1 = GAParameters.p1; %temp
p2 = GAParameters.p2; %temp
W1 = GAParameters.W1; %temp
W2 = GAParameters.W2; %temp
cH = GAParameters.cH; %temp

isValid = 0;
%-----------------------------------------
%	Assignment of fitness values
%-----------------------------------------

for i = 1:N
   
    [P(i).ValObjective,P(i).R,P(i).T] = CameraPose(Po(i).Val,p1,p2,W1,W2,cH);
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




