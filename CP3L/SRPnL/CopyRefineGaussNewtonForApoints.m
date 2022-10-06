% Author :  Ping Wang                                                        
% Contact:  pingwangsky@gmail.com 
% This programe is implemented in matlab 2018a
% License:  Copyright (c) 2019 Ping Wang, All rights reserved       
% Address:  College of Electrical and Information Engineering, Lanzhou University of Technology              
% My site:  https://sites.google.com/view/ping-wang-homepage    

function [solution,obj_pre] = CopyRefineGaussNewtonForApoints(solution,E,G1)
%refine the solution by using one step Gauss Newton method
    s1=solution(1); s2=solution(2); s3=solution(3);s4 =solution(4);s5=solution(5);s6=solution(6);
    s7=s1^2; s8=s1*s2; s9=s1*s3;
    s10=s2^2; s11=s2*s3; s12=s3^2;
    w=[1,s1,s2,s3,s7,s8,s9,s10,s11,s12,...
        s4,s4*s2,s4*s3,s4*s7,s4*s8,s4*s9,s4*s10,s4*s12,...
        s5,s5*s1,s5*s3,s5*s7,s5*s8,s5*s10,s5*s11,s5*s12,...
        s6,s6*s1,s6*s2,s6*s7,s6*s9,s6*s10,s6*s11,s6*s12,...
        s1^2, s1*s2, s1*s3, s2^2, s2*s3, s3^2].';
    obj_pre = w.'*G1*w;
    solution = [s1,s2,s3,s4,s5,s6,s7,s8,s9,s10,s11,s12]';
    for in = 1:5
        %Jacobian matrix
        Jac=[ 0,        0,     0,    0,     0,     0,     0,        0,     0,    0,     0,     0;
              1,        0,     0,    0,     0,     0,     0,        0,     0,    0,     0,     0;
              0,        1,     0,    0,     0,     0,     0,        0,     0,    0,     0,     0;
              0,        0,     1,    0,     0,     0,     0,        0,     0,    0,     0,     0;
              0,        0,     0,    0,     0,     0,     1,        0,     0,    0,     0,     0;
              0,        0,     0,    0,     0,     0,     0,        1,     0,    0,     0,     0;
              0,        0,     0,    0,     0,     0,     0,        0,     1,    0,     0,     0;
              0,        0,     0,    0,     0,     0,     0,        0,     0,    1,     0,     0;
              0,        0,     0,    0,     0,     0,     0,        0,     0,    0,     1,     0;
              0,        0,     0,    0,     0,     0,     0,        0,     0,    0,     0,     1;   
              0,        0,     0,    1,     0,     0,     0,        0,     0,    0,     0,     0;
              0,       s4,     0,   s2,     0,     0,     0,        0,     0,    0,     0,     0;
              0,        0,    s4,   s3,     0,     0,     0,        0,     0,    0,     0,     0;
              0,        0,     0,   s7,     0,     0,    s4,        0,     0,    0,     0,     0;
              0,        0,     0,   s8,     0,     0,     0,       s4,     0,    0,     0,     0;
              0,        0,     0,   s9,     0,     0,     0,        0,    s4,    0,     0,     0;
              0,        0,     0,  s10,     0,     0,     0,        0,     0,   s4,     0,     0;
              0,        0,     0,  s12,     0,     0,     0,        0,     0,    0,     0,    s4;  
              0,        0,     0,    0,     1,     0,     0,        0,     0,    0,     0,     0;
             s5,        0,     0,    0,    s1,     0,     0,        0,     0,    0,     0,     0;
              0,        0,    s5,    0,    s3,     0,     0,        0,     0,    0,     0,     0;
              0,        0,     0,    0,    s7,     0,    s5,        0,     0,    0,     0,     0;
              0,        0,     0,    0,    s8,     0,     0,       s5,     0,    0,     0,     0;
              0,        0,     0,    0,   s10,     0,     0,        0,     0,   s5,     0,     0;
              0,        0,     0,    0,   s11,     0,     0,        0,     0,    0,    s5,     0;
              0,        0,     0,    0,   s12,     0,     0,        0,     0,    0,     0,    s5;  
              0,        0,     0,    0,     0,     1,     0,        0,     0,    0,     0,     0;
             s6,        0,     0,    0,     0,    s1,     0,        0,     0,    0,     0,     0;
              0,       s6,     0,    0,     0,    s2,     0,        0,     0,    0,     0,     0;
              0,        0,     0,    0,     0,    s7,    s6,        0,     0,    0,     0,     0;
              0,        0,     0,    0,     0,    s9,     0,        0,    s6,    0,     0,     0;
              0,        0,     0,    0,     0,   s10,     0,        0,     0,   s6,     0,     0;
              0,        0,     0,    0,     0,   s11,     0,        0,     0,    0,    s6,     0;
              0,        0,     0,    0,     0,   s12,     0,        0,     0,    0,     0,    s6;
           2*s1,        0,     0,    0,     0,     0,     0,        0,     0,    0,     0,     0;
             s2,       s1,     0,    0,     0,     0,     0,        0,     0,    0,     0,     0;
             s3,        0,    s1,    0,     0,     0,     0,        0,     0,    0,     0,     0;
              0,     2*s2,     0,    0,     0,     0,     0,        0,     0,    0,     0,     0;
              0,       s3,    s2,    0,     0,     0,     0,        0,     0,    0,     0,     0;
              0,        0,  2*s3,    0,     0,     0,     0,        0,     0,    0,     0,     0;
              
              ];
        Fk=E*w;  
        Jk=E*Jac;
        
        temp_sol = [s1,s2,s3,s4,s5,s6,s7,s8,s9,s10,s11,s12]';
        
        solution_temp = temp_sol;


        dk=-(Jk.'*Jk)\(Jk.'*Fk);
        %update parameter
        
        solution_temp = solution_temp + dk; 
        s1=solution_temp(1); s2=solution_temp(2); s3=solution_temp(3);
        s4 =solution_temp(4);s5=solution_temp(5); s6=solution_temp(6); 
        s7=solution_temp(7); s8=solution_temp(8); s9=solution_temp(9);
        s10=solution_temp(10); s11=solution_temp(11); s12=solution_temp(12);
        w=[1,s1,s2,s3,s7,s8,s9,s10,s11,s12,...
            s4,s4*s2,s4*s3,s4*s7,s4*s8,s4*s9,s4*s10,s4*s12,...
            s5,s5*s1,s5*s3,s5*s7,s5*s8,s5*s10,s5*s11,s5*s12,...
            s6,s6*s1,s6*s2,s6*s7,s6*s9,s6*s10,s6*s11,s6*s12,...
            s1^2, s1*s2, s1*s3, s2^2, s2*s3, s3^2].';
        %evaluate the error of objection; 
        obj_cur = w.'*G1*w;
        if abs(obj_cur) < abs(obj_pre)
            solution=solution_temp;
            obj_pre = obj_cur;
        end
    end  
end

