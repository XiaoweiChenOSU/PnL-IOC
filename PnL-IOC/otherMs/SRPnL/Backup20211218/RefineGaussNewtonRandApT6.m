% Author :  Ping Wang                                                        
% Contact:  pingwangsky@gmail.com 
% This programe is implemented in matlab 2018a
% License:  Copyright (c) 2019 Ping Wang, All rights reserved       
% Address:  College of Electrical and Information Engineering, Lanzhou University of Technology              
% My site:  https://sites.google.com/view/ping-wang-homepage    

function [solution, obj_cur] = RefineGaussNewtonRandApT6(solution,E,G1,CC)
%refine the solution by using one step Gauss Newton method
    s1=solution(1); s2=solution(2); s3=solution(3); 
    s4=s1^2; s5=s1*s2; s6=s1*s3;
    s7=s2^2; s8=s2*s3; s9=s3^2;
    s10 = solution(4); s11=solution(5); s12=solution(6); s13=solution(7);  
%     s5,s5*s1,s5*s3,s5*(s1^2),s5*s1*s2,s5*(s2^2),s5*s2*s3,s5*(s3^2),...
    w=[1,s1,s2,s3,s4,s5,s6,s7,s8,s9,...
       s10,s10*s1,s10*s3,s10*s4,s10*s5,s10*s7,s10*s8,s10*s9,...
       s11,s11*s1,s11*s3,s11*s4,s11*s5,s11*s7,s11*s8,s11*s9,...
       s12,s12*s1,s12*s3,s12*s4,s12*s5,s12*s7,s12*s8,s12*s9,...
       s13,s13*s1,s13*s3,s13*s4,s13*s5,s13*s7,s13*s8,s13*s9,...
       s1^2,s1*s2,s1*s3,s2^2,s2*s3,s3^2]';
    obj_pre = w.'*G1*w;
%     factor=1/(1+s1^2+s2^2+s3^2); 
%     Rr=factor*[1+s1^2-s2^2-s3^2,2*s1*s2-2*s3,2*s2+2*s1*s3;
%                    2*s3+2*s1*s2,1-s1^2+s2^2-s3^2,2*s2*s3-2*s1;
%                    2*s1*s3-2*s2,2*s1+2*s2*s3,1-s1^2-s2^2+s3^2];
%     Tr =  factor*CC*w; 
%     nLine = length(p1);
%     p1 = [p1; ones(1,nLine)];
%     p2 = [p2; ones(1,nLine)];
%     P1_w(2,1:3) = s10;
%     P1_w(2,4:5) = s11;
%     P1_w(2,6) = s12;
%     P2_w(2,1) = s11;
%     P2_w(2,2:2:4) = s12;
%     P2_w(2,3) = s13;
%     P2_w(2,5:6) = s13;
%     errOrt = getErr(p1, p2, P1_w, P2_w,Rr,Tr);
    %Jacobian matrix
    for i = 1:30
        Jac=[ 0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0;
              1,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0;
              0,     1,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0;
              0,     0,     1,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0;
              0,     0,     0,     1,     0,     0,     0,     0,     0,     0,     0,     0,     0;
              0,     0,     0,     0,     1,     0,     0,     0,     0,     0,     0,     0,     0;
              0,     0,     0,     0,     0,     1,     0,     0,     0,     0,     0,     0,     0;
              0,     0,     0,     0,     0,     0,     1,     0,     0,     0,     0,     0,     0;
              0,     0,     0,     0,     0,     0,     0,     1,     0,     0,     0,     0,     0;
              0,     0,     0,     0,     0,     0,     0,     0,     1,     0,     0,     0,     0;              
              0,     0,     0,     0,     0,     0,     0,     0,     0,     1,     0,     0,     0;
            s10,     0,     0,     0,     0,     0,     0,     0,     0,    s1,     0,     0,     0;
              0,     0,   s10,     0,     0,     0,     0,     0,     0,    s3,     0,     0,     0;
              0,     0,     0,   s10,     0,     0,     0,     0,     0,    s4,     0,     0,     0;
              0,     0,     0,     0,   s10,     0,     0,     0,     0,    s5,     0,     0,     0;
              0,     0,     0,     0,     0,     0,   s10,     0,     0,    s7,     0,     0,     0;
              0,     0,     0,     0,     0,     0,     0,   s10,     0,    s8,     0,     0,     0;
              0,     0,     0,     0,     0,     0,     0,     0,   s10,    s9,     0,     0,     0;      
              0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     1,     0,     0;
            s11,     0,     0,     0,     0,     0,     0,     0,     0,     0,    s1,     0,     0;
              0,     0,   s11,     0,     0,     0,     0,     0,     0,     0,    s3,     0,     0;
              0,     0,     0,   s11,     0,     0,     0,     0,     0,     0,    s4,     0,     0;
              0,     0,     0,     0,   s11,     0,     0,     0,     0,     0,    s5,     0,     0;
              0,     0,     0,     0,     0,     0,   s11,     0,     0,     0,    s7,     0,     0;
              0,     0,     0,     0,     0,     0,     0,   s11,     0,     0,    s8,     0,     0;
              0,     0,     0,     0,     0,     0,     0,     0,   s11,     0,    s9,     0,     0;            
              0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     1,     0;
            s12,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,    s1,     0;
              0,     0,   s12,     0,     0,     0,     0,     0,     0,     0,     0,    s3,     0;
              0,     0,     0,   s12,     0,     0,     0,     0,     0,     0,     0,    s4,     0;
              0,     0,     0,     0,   s12,     0,     0,     0,     0,     0,     0,    s5,     0;
              0,     0,     0,     0,     0,     0,   s12,     0,     0,     0,     0,    s7,     0;
              0,     0,     0,     0,     0,     0,     0,   s12,     0,     0,     0,    s8,     0;
              0,     0,     0,     0,     0,     0,     0,     0,   s12,     0,     0,    s9,     0;              
              0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     1;
            s13,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,    s1;
              0,     0,   s13,     0,     0,     0,     0,     0,     0,     0,     0,     0,    s3;
              0,     0,     0,   s13,     0,     0,     0,     0,     0,     0,     0,     0,    s4;
              0,     0,     0,     0,   s13,     0,     0,     0,     0,     0,     0,     0,    s5;
              0,     0,     0,     0,     0,     0,   s13,     0,     0,     0,     0,     0,    s7;
              0,     0,     0,     0,     0,     0,     0,   s13,     0,     0,     0,     0,    s8;
              0,     0,     0,     0,     0,     0,     0,     0,   s13,     0,     0,     0,    s9; 
           2*s1,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0;
             s2,    s1,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0;
             s3,     0,    s1,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0;
              0,  2*s2,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0;
              0,    s3,    s2,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0;
              0,     0,  2*s3,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0;
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
        w=[1,s1,s2,s3,s4,s5,s6,s7,s8,s9,...
           s10,s10*s1,s10*s3,s10*s4,s10*s5,s10*s7,s10*s8,s10*s9,...
           s11,s11*s1,s11*s3,s11*s4,s11*s5,s11*s7,s11*s8,s11*s9,...
           s12,s12*s1,s12*s3,s12*s4,s12*s5,s12*s7,s12*s8,s12*s9,...
           s13,s13*s1,s13*s3,s13*s4,s13*s5,s13*s7,s13*s8,s13*s9,...
           s1^2,s1*s2,s1*s3,s2^2,s2*s3,s3^2]';
        %evaluate the error of objection; 
%         factor=1/(1+s1^2+s2^2+s3^2); 
%         Rr=factor*[1+s1^2-s2^2-s3^2,2*s1*s2-2*s3,2*s2+2*s1*s3;
%                        2*s3+2*s1*s2,1-s1^2+s2^2-s3^2,2*s2*s3-2*s1;
%                        2*s1*s3-2*s2,2*s1+2*s2*s3,1-s1^2-s2^2+s3^2];
%         Tr =  factor*CC*w; 
%         P1_w(2,1:3) = s10;
%         P1_w(2,4:5) = s11;
%         P1_w(2,6) = s12;
%         P2_w(2,1) = s11;
%         P2_w(2,2:2:4) = s12;
%         P2_w(2,3) = s13;
%         P2_w(2,5:6) = s13;
%         errOrtcur = getErr(p1, p2, P1_w, P2_w,Rr,Tr); 
        obj_cur = w.'*G1*w;
        if obj_cur<obj_pre 
%             && errOrtcur < errOrt && s10 > 0 && s10 < 256 && s11 > 0 && s11 < 256 && s12 > 0 && s12 < 256 && s13 > 0 && s13 < 256
            solution=solution_temp;
            obj_pre = obj_cur;
        end
    end
end

