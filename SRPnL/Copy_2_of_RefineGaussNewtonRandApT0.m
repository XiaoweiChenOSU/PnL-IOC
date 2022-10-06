% Author :  Ping Wang                                                        
% Contact:  pingwangsky@gmail.com 
% This programe is implemented in matlab 2018a
% License:  Copyright (c) 2019 Ping Wang, All rights reserved       
% Address:  College of Electrical and Information Engineering, Lanzhou University of Technology              
% My site:  https://sites.google.com/view/ping-wang-homepage    

function [solution, obj_cur] = RefineGaussNewtonRandApT0(solution,E,G1)
%refine the solution by using one step Gauss Newton method
    s1=solution(1); s2=solution(2); s3=solution(3); 
    s4=s1^2; s5=s1*s2; s6=s1*s3;
    s7=s2^2; s8=s2*s3; s9=s3^2;
    s10 = solution(4);
    w=[1,s1,s2,s3,s4,s5,s6,s7,s8,s9,...
       s10,s10*s1,s10*s2,s10*s4,s10*s6,s10*s7,s10*s8,s10*s9,...
       s1^2,s1*s2,s1*s3,s2^2,s2*s3,s3^2]';
    obj_pre = w.'*G1*w;
    %Jacobian matrix
    for i = 1:100
        Jac=[ 0,     0,     0,     0,     0,     0,     0,     0,     0,     0;
              1,     0,     0,     0,     0,     0,     0,     0,     0,     0;
              0,     1,     0,     0,     0,     0,     0,     0,     0,     0;
              0,     0,     1,     0,     0,     0,     0,     0,     0,     0;
              0,     0,     0,     1,     0,     0,     0,     0,     0,     0;
              0,     0,     0,     0,     1,     0,     0,     0,     0,     0;
              0,     0,     0,     0,     0,     1,     0,     0,     0,     0;
              0,     0,     0,     0,     0,     0,     1,     0,     0,     0;
              0,     0,     0,     0,     0,     0,     0,     1,     0,     0;
              0,     0,     0,     0,     0,     0,     0,     0,     1,     0;              
              0,     0,     0,     0,     0,     0,     0,     0,     0,     1;
            s10,     0,     0,     0,     0,     0,     0,     0,     0,    s1;
              0,   s10,     0,     0,     0,     0,     0,     0,     0,    s2;
              0,     0,     0,   s10,     0,     0,     0,     0,     0,    s4;
              0,     0,     0,     0,     0,   s10,     0,     0,     0,    s6;
              0,     0,     0,     0,     0,     0,   s10,     0,     0,    s7;
              0,     0,     0,     0,     0,     0,     0,   s10,     0,    s8;
              0,     0,     0,     0,     0,     0,     0,     0,   s10,    s9;      
           2*s1,     0,     0,     0,     0,     0,     0,     0,     0,     0;
             s2,    s1,     0,     0,     0,     0,     0,     0,     0,     0;
             s3,     0,    s1,     0,     0,     0,     0,     0,     0,     0;
              0,  2*s2,     0,     0,     0,     0,     0,     0,     0,     0;
              0,    s3,    s2,     0,     0,     0,     0,     0,     0,     0;
              0,     0,  2*s3,     0,     0,     0,     0,     0,     0,     0;
           ];
        Fk=E*w;  
        Jk=E*Jac;

        temp_sol = [s1,s2,s3,s4,s5,s6,s7,s8,s9,s10]';

        solution_temp = temp_sol;
        %increment
        dk=-(Jk.'*Jk)\(Jk.'*Fk);
        %update parameter
        solution_temp = solution_temp + dk; 
        s1=solution_temp(1); s2=solution_temp(2); s3=solution_temp(3);
        s4=solution_temp(4); s5=solution_temp(5); s6=solution_temp(6);
        s7=solution_temp(7); s8=solution_temp(8); s9=solution_temp(9);
        s10=solution_temp(10);
        w=[1,s1,s2,s3,s4,s5,s6,s7,s8,s9,...
           s10,s10*s1,s10*s2,s10*s4,s10*s6,s10*s7,s10*s8,s10*s9,...
           s1^2,s1*s2,s1*s3,s2^2,s2*s3,s3^2]';
        %evaluate the error of objection; 
        obj_cur = w.'*G1*w;
        if abs(obj_cur)<obj_pre 
            solution=solution_temp;
            obj_pre = abs(obj_cur);
        end
    end
  
end

