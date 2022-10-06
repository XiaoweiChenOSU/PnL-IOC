%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   
%  Set the problem parameters according to the chosen problem
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function ProblemParameters = SetProblemParameters(ProblemParametersOption,K)
%     ProblemParameters.s1UpperLimit = ProblemParametersOption(K,2);
%     ProblemParameters.s1LowerLimit = ProblemParametersOption(K,1);
%     ProblemParameters.s2UpperLimit = ProblemParametersOption(K,4);
%     ProblemParameters.s2LowerLimit = ProblemParametersOption(K,3);
%     ProblemParameters.s3UpperLimit = ProblemParametersOption(K,6);
%     ProblemParameters.s3LowerLimit = ProblemParametersOption(K,5);
%     ProblemParameters.s4UpperLimit = ProblemParametersOption(K,8);
%     ProblemParameters.s4LowerLimit = ProblemParametersOption(K,7);
    ProblemParameters.s1UpperLimit = 56;
    ProblemParameters.s1LowerLimit = 16;
    ProblemParameters.s2UpperLimit = 256;
    ProblemParameters.s2LowerLimit = 180;
    ProblemParameters.s3UpperLimit = 56;
    ProblemParameters.s3LowerLimit = 16;
    ProblemParameters.s4UpperLimit = 256;
    ProblemParameters.s4LowerLimit = 180;



