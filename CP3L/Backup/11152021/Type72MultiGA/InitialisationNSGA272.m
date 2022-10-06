%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Initialization step
%
% Generates random individuals within the limits of the problem
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function Pop = InitialisationNSGA272(individuVide, ProblemParameters,GAParameters)

%-----------------------------------------
%	Init variables
%-----------------------------------------
    N = GAParameters.PopSize*4;
%     N = 5;
    s1U = ProblemParameters.s1UpperLimit;
    s1L = ProblemParameters.s1LowerLimit;
    s2U = ProblemParameters.s2UpperLimit;
    s2L = ProblemParameters.s2LowerLimit;
    s3U = ProblemParameters.s3UpperLimit;
    s3L = ProblemParameters.s3LowerLimit; 

   
    
    Pop = repmat(individuVide,N,1);
    for i = 1:N
        Pop(i).Val(1) = s1L+(s1U-s1L)*rand(1,1);
        Pop(i).Val(2) = Pop(i).Val(1) + 60*(rand(1,1)-0.5);
        if  Pop(i).Val(2) < s2L
            Pop(i).Val(2) = s2L;
        elseif Pop(i).Val(2) > s2U
            Pop(i).Val(2) = s2U;
        end
        Pop(i).Val(3) = s3L+(s3U-s3L)*rand(1,1);

%         Pop(i).Val(1) = s1L+(s1U-s1L)*rand(1,1);
%         Pop(i).Val(2) = s2L+(s2U-s2L)*rand(1,1);
%         Pop(i).Val(3) = s3L+(s3U-s3L)*rand(1,1);
%         Pop(i).Val(4) = s4L+(s4U-s4L)*rand(1,1);
    end
%     count = 1;
%     for i = 1:N
%         tempVal1 = s1L+(s1U-s1L)*rand(1,1);
%         for j = 1:N
%             tempVal2 = s2L+(s2U-s2L)*rand(1,1);
%             for m  = 1:N
%                 tempVal3 = s3L+(s3U-s3L)*rand(1,1);
%                 for n = 1:N
%                     Pop(count).Val(1) = tempVal1;
%                     Pop(count).Val(2) = tempVal2;
%                     Pop(count).Val(3) = tempVal3;
%                     Pop(count).Val(4) = s4L+(s4U-s4L)*rand(1,1);
%                     count = count + 1;
%                 end
%             end
%         end
%     end     
end  