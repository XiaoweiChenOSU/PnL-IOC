% Author :  Ping Wang                                                        
% Contact:  pingwangsky@gmail.com 
% This programe is implemented in matlab 2018a
% License:  Copyright (c) 2019 Ping Wang, All rights reserved       
% Address:  College of Electrical and Information Engineering, Lanzhou University of Technology              
% My site:  https://sites.google.com/view/ping-wang-homepage    

function [solution, obj_cur] = Copy_T_of_RefineGaussNewtonRandApT0(solution,E,G1)
%refine the solution by using one step Gauss Newton method
    s1=solution(1); s2=solution(2); s3=solution(3); 
    s4=solution(4); s5=solution(5); s6=solution(6); 
    s7=solution(7); s8=solution(8); s9=solution(9); 
    s10 = solution(10); s11=solution(11); s12=solution(12); s13=solution(13);  
    w=[s1,s2,s3,s4,s5,s6,s7,s8,s9,...
       s10*s3,s10*s6,s10*s9,...
       s11*s3,s11*s6,s11*s9,...
       s12*s3,s12*s6,s12*s9,...
       s13*s3,s13*s6,s13*s9]';
%     factor=1/(1+s1^2+s2^2+s3^2);
%     E = factor*E;
%     G1=E.'*E;
    obj_pre = w.'*G1*w;

    %Jacobian matrix
    for i = 1:5
        Jac=[ 1,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0;
              0,     1,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0;
              0,     0,     1,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0;
              0,     0,     0,     1,     0,     0,     0,     0,     0,     0,     0,     0,     0;
              0,     0,     0,     0,     1,     0,     0,     0,     0,     0,     0,     0,     0;
              0,     0,     0,     0,     0,     1,     0,     0,     0,     0,     0,     0,     0;
              0,     0,     0,     0,     0,     0,     1,     0,     0,     0,     0,     0,     0;
              0,     0,     0,     0,     0,     0,     0,     1,     0,     0,     0,     0,     0;
              0,     0,     0,     0,     0,     0,     0,     0,     1,     0,     0,     0,     0;              
              0,     0,   s10,     0,     0,     0,     0,     0,     0,    s3,     0,     0,     0;
              0,     0,     0,     0,     0,   s10,     0,     0,     0,    s6,     0,     0,     0;
              0,     0,     0,     0,     0,     0,     0,     0,   s10,    s9,     0,     0,     0;      
              0,     0,   s11,     0,     0,     0,     0,     0,     0,     0,    s3,     0,     0;
              0,     0,     0,     0,     0,   s11,     0,     0,     0,     0,    s6,     0,     0;
              0,     0,     0,     0,     0,     0,     0,     0,   s11,     0,    s9,     0,     0;            
              0,     0,   s12,     0,     0,     0,     0,     0,     0,     0,     0,    s3,     0;
              0,     0,     0,     0,     0,   s12,     0,     0,     0,     0,     0,    s6,     0;
              0,     0,     0,     0,     0,     0,     0,     0,   s12,     0,     0,    s9,     0;              
              0,     0,   s13,     0,     0,     0,     0,     0,     0,     0,     0,     0,    s3;
              0,     0,     0,     0,     0,   s13,     0,     0,     0,     0,     0,     0,    s6;
              0,     0,     0,     0,     0,     0,     0,     0,   s13,     0,     0,     0,    s9; 
           ];
        Fk=E*w;  
        Jk=E*Jac;

        temp_sol = [s1,s2,s3,s4,s5,s6,s7,s8,s9,s10,s11,s12,s13]';

        solution_temp = temp_sol;
        %increment
        dk=-(Jk.'*Jk)\(Jk.'*Fk);
        %update parameter
        solution_temp = solution_temp + dk; 
        s1=solution_temp(1); s2=solution_temp(2); s3=solution_temp(3);
        s4=solution_temp(4); s5=solution_temp(5); s6=solution_temp(6);
        s7=solution_temp(7); s8=solution_temp(8); s9=solution_temp(9);
        s10=solution_temp(10); s11=solution_temp(11); s12=solution_temp(12);s13=solution_temp(13);
       w=[s1,s2,s3,s4,s5,s6,s7,s8,s9,...
           s10*s3,s10*s6,s10*s9,...
           s11*s3,s11*s6,s11*s9,...
           s12*s3,s12*s6,s12*s9,...
           s13*s3,s13*s6,s13*s9]';
        %evaluate the error of objection; 
        obj_cur = w.'*G1*w;
        if abs(obj_cur)<obj_pre 
            solution=solution_temp;
            obj_pre = abs(obj_cur);
        end
    end
  
end

