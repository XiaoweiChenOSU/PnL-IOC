% Author :  Ping Wang                                                        
% Contact:  pingwangsky@gmail.com 
% This programe is implemented in matlab 2018a
% License:  Copyright (c) 2019 Ping Wang, All rights reserved       
% Address:  College of Electrical and Information Engineering, Lanzhou University of Technology              
% My site:  https://sites.google.com/view/ping-wang-homepage    


function [solution,obj_pre] = RefineForApoints(solution,E,G1)
%refine the solution by using one step Gauss Newton method
    s1=solution(1); s2=solution(2); s3=solution(3);
    w=[1,s1,s2,s3].';
    obj_pre = w.'*G1*w;
    solution_temp = solution;
    for in = 1:50
        s1=solution(1); s2=solution(2); s3=solution(3);
        w=[1,s1,s2,s3].';
        %Jacobian matrix
        Jac=[ 0,        0,     0;
              1,        0,     0;
              0,        1,     0;
              0,        0,     1;
            ];
        Fk=E*w;  
        Jk=E*Jac;

        %increment
        if det(Jk.'*Jk) < 2e-10
            dk=-pinv(Jk.'*Jk)*(Jk.'*Fk);
        else
            dk=-(Jk.'*Jk)\(Jk.'*Fk);
        end
%         dk=-pinv(Jk.'*Jk)*(Jk.'*Fk);
    %     dk=-(Jk.'*Jk)\(Jk.'*Fk);
        %update parameter
        solution_temp = solution_temp + dk; 
        s1=solution_temp(1); s2=solution_temp(2); s3=solution_temp(3); 
        w=[1,s1,s2,s3].';
        %evaluate the error of objection; 
        obj_cur = w.'*G1*w;
        if abs(obj_cur)<abs(obj_pre) 
            solution=solution_temp;
            obj_pre = obj_cur;
        end
    end  
end


