%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   
%  Set the problem parameters according to the chosen problem
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function ProblemParametersOption = ProblemParametersOptions71()
    Low = [16,26,36,46,56];
    Upper = [206,216,226,236,246];
    count = 1;
    for a = 1:5
        s1LowerLimit = Low(a);
        s1UpperLimit = s1LowerLimit + 10;
        for b = 1:5
            s2LowerLimit = Upper(b);
            s2UpperLimit = s2LowerLimit + 10;
            for c = 1:5
                s3LowerLimit = Low(c);
                s3UpperLimit = s3LowerLimit + 10; 
                for d = 1:5
                    s4LowerLimit = Upper(d);
                    s4UpperLimit = s4LowerLimit + 10;
                    ProblemParametersOption(count,:) = [s1LowerLimit,s1UpperLimit,s2LowerLimit,s2UpperLimit,s3LowerLimit,s3UpperLimit,s4LowerLimit,s4UpperLimit];
                    count = count + 1;
                end
            end
        end
    end
   