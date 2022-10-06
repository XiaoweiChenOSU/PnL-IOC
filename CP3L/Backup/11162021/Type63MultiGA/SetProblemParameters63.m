%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   
%  Set the problem parameters according to the chosen problem
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function ProblemParameters = SetProblemParameters63(GAParameters)
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

    Height = GAParameters.Lheight;

    ProblemParameters.s1UpperLimit = Height/2;
    ProblemParameters.s1LowerLimit = 0;
    ProblemParameters.s2UpperLimit = Height;
    ProblemParameters.s2LowerLimit = Height/2;
    ProblemParameters.s3UpperLimit = Height;
    ProblemParameters.s3LowerLimit = Height/2;

%     ProblemParameters.s1UpperLimit = 256;
%     ProblemParameters.s1LowerLimit = 0;
%     ProblemParameters.s2UpperLimit = 256;
%     ProblemParameters.s2LowerLimit = 0;
%     ProblemParameters.s3UpperLimit = 256;
%     ProblemParameters.s3LowerLimit = 0;
%     ProblemParameters.s4UpperLimit = 256;
%     ProblemParameters.s4LowerLimit = 0;



