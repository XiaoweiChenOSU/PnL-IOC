%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   
%  Set the problem parameters according to the chosen problem
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function ProblemParameters = SetProblemParameters66(GAParameters)
%     ProblemParameters.s1UpperLimit = ProblemParametersOption(K,2);
%     ProblemParameters.s1LowerLimit = ProblemParametersOption(K,1);
%     ProblemParameters.s2UpperLimit = ProblemParametersOption(K,4);
%     ProblemParameters.s2LowerLimit = ProblemParametersOption(K,3);
%     ProblemParameters.s3UpperLimit = ProblemParametersOption(K,6);
%     ProblemParameters.s3LowerLimit = ProblemParametersOption(K,5);
%     ProblemParameters.s4UpperLimit = ProblemParametersOption(K,8);
%     ProblemParameters.s4LowerLimit = ProblemParametersOption(K,7);
%     ProblemParameters.s1UpperLimit = 66;
%     ProblemParameters.s1LowerLimit = 16;
%     ProblemParameters.s2UpperLimit = 256;
%     ProblemParameters.s2LowerLimit = 180;
%     ProblemParameters.s3UpperLimit = 256;
%     ProblemParameters.s3LowerLimit = 180;
%     ProblemParameters.s4UpperLimit = 66;
%     ProblemParameters.s4LowerLimit = 16;

    Oheiht = GAParameters.Lheight;
    Height = GAParameters.Lheight;
    W1 =  GAParameters.W1A;
    W2 =  GAParameters.W2A;
    [~,n] = max(abs(W2(1,:) - W1(1,:)));
    ProblemParameters.unknown = n;
%     ProblemParameters.s1UpperLimit = Height/5;
%     ProblemParameters.s1LowerLimit = 0;
%     ProblemParameters.s2UpperLimit = Height/5;
%     ProblemParameters.s2LowerLimit = 0;
%     ProblemParameters.s3UpperLimit = Height;
%     ProblemParameters.s3LowerLimit = 4*Height/5;
%     ProblemParameters.s4UpperLimit = Height;
%     ProblemParameters.s4LowerLimit = 4*Height/5;

    scale = 10;
    ProblemParameters.s1UpperLimit = W1(1,n)+Height/scale;
    ProblemParameters.s1LowerLimit = W1(1,n)-Height/scale;
    ProblemParameters.s2UpperLimit = W1(2,n)+Height/scale;
    ProblemParameters.s2LowerLimit = W1(2,n)-Height/scale;
    ProblemParameters.s3UpperLimit = W2(1,n)+Height/scale;
    ProblemParameters.s3LowerLimit = W2(1,n)-Height/scale;
    ProblemParameters.s4UpperLimit = W2(2,n)+Height/scale;
    ProblemParameters.s4LowerLimit = W2(2,n)-Height/scale;
%     
%     if ProblemParameters.s1LowerLimit < 0
%         ProblemParameters.s1LowerLimit = 0;
%     end
%     if ProblemParameters.s1UpperLimit > Oheiht/2
%         ProblemParameters.s1UpperLimit = Oheiht/2;
%     end
%     
%     if ProblemParameters.s2LowerLimit < 0
%         ProblemParameters.s2LowerLimit = 0;
%     end
%     if ProblemParameters.s2UpperLimit > Oheiht/2
%         ProblemParameters.s2UpperLimit = Oheiht/2;
%     end
%     
%     if ProblemParameters.s3LowerLimit < Oheiht/2
%         ProblemParameters.s3LowerLimit = Oheiht/2;
%     end
%     if ProblemParameters.s3UpperLimit > Oheiht
%         ProblemParameters.s3UpperLimit = Oheiht;
%     end
%     
%     if ProblemParameters.s4LowerLimit < Oheiht/2
%         ProblemParameters.s4LowerLimit = Oheiht/2;
%     end
%     if ProblemParameters.s4UpperLimit > Oheiht
%         ProblemParameters.s4UpperLimit = Oheiht;
%     end
    
%     ProblemParameters.s1UpperLimit = 256;
%     ProblemParameters.s1LowerLimit = 0;
%     ProblemParameters.s2UpperLimit = 256;
%     ProblemParameters.s2LowerLimit = 0;
%     ProblemParameters.s3UpperLimit = 256;
%     ProblemParameters.s3LowerLimit = 0;
%     ProblemParameters.s4UpperLimit = 256;
%     ProblemParameters.s4LowerLimit = 0;



