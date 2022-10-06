%******************************************************************************
% This programe is implemented via MATLAB 2018.                              *
% Author :  Ping Wang                                                        *
% Contact:  pingwangsky@gmail.com; pingwangsky@163.com                       *
% License:  Copyright (c) 2020 Ping Wang, All rights reserved.               *
% Address:  College of Electrical and Information Engineering,               *
%           Lanzhou University of Technology                                 *
% My site:  https://sites.google.com/view/ping-wang-homepage                 *
%*****************************************************************************/

function [R,t] = Hidden_PnL_main(p1,p2,W1,W2 ) 
nLine = length(p1);
p1 = [p1; ones(1,nLine)];
p2 = [p2; ones(1,nLine)];

n_c=xnorm(cross(p1,p2));

Q=zeros(2*nLine,10);
N=zeros(2*nLine,3);
nx=n_c(1,:)'; ny=n_c(2,:)'; nz=n_c(3,:)';
Px=W1(1,:)'; Py=W1(2,:)'; Pz=W1(3,:)';
Q(1:2:end,:)=[Px.*nx+Py.*ny+Pz.*nz,...
          2*Py.*nz-2*Pz.*ny,...
          2*Pz.*nx-2*Px.*nz,...
          2*Px.*ny-2*Py.*nx,...
          Px.*nx-Py.*ny-Pz.*nz,...
          2.*Px.*ny+2*Py.*nx,...
          2.*Px.*nz+2*Pz.*nx,...
          Py.*ny-Px.*nx-Pz.*nz,...
          2*Py.*nz+2*Pz.*ny,...
          Pz.*nz-Px.*nx-Py.*ny];
Px=W2(1,:)'; Py=W2(2,:)'; Pz=W2(3,:)';
Q(2:2:end,:)=[Px.*nx+Py.*ny+Pz.*nz,...
          2*Py.*nz-2*Pz.*ny,...
          2*Pz.*nx-2*Px.*nz,...
          2*Px.*ny-2*Py.*nx,...
          Px.*nx-Py.*ny-Pz.*nz,...
          2.*Px.*ny+2*Py.*nx,...
          2.*Px.*nz+2*Pz.*nx,...
          Py.*ny-Px.*nx-Pz.*nz,...
          2*Py.*nz+2*Pz.*ny,...
          Pz.*nz-Px.*nx-Py.*ny];
N(1:2:end,:)=-n_c.';
N(2:2:end,:)=-n_c.';

C=((N.'*N)\N.')*Q;
E=Q-N*C;
G=E.'*E;    %the size of G is 10*10;
     
M=-[E(:,8),E(:,10),E(:,9)]; 
M1=M.'*M;

% If no ssolution, return!
if rank(M1)<3
    %fprintf('no solution!\n');
    R=[];    t=[];
    return;
end

N=M1\M.';
c11=N(1,:)*E(:,6); c12=N(1,:)*E(:,3); c13=N(1,:)*E(:,7);
c14=N(1,:)*E(:,4); c15=N(1,:)*E(:,5); c16=N(1,:)*E(:,2); c17=N(1,:)*E(:,1);

c21=N(2,:)*E(:,6); c22=N(2,:)*E(:,3); c23=N(2,:)*E(:,7);
c24=N(2,:)*E(:,4); c25=N(2,:)*E(:,5); c26=N(2,:)*E(:,2); c27=N(2,:)*E(:,1);

c31=N(3,:)*E(:,6); c32=N(3,:)*E(:,3); c33=N(3,:)*E(:,7);
c34=N(3,:)*E(:,4); c35=N(3,:)*E(:,5); c36=N(3,:)*E(:,2); c37=N(3,:)*E(:,1);

d11=c12 - c34; d12=c11-c33;
e11=c13*c21 - c35 - c11*c31 + c31*d12;
e12=c13*c22 - c36 + c14*c21 - c11*c32 - c12*c31 + c31*d11 + c32*d12;
e13=- c37 + c14*c22 - c12*c32 + c32*d11;
e14=c15 + c13*c23 - c13*c31 + c33*d12;
e15=c16 + c13*c24 + c14*c23 - c13*c32 - c14*c31 + c33*d11 + c34*d12;
e16=c17 + c14*c24 - c14*c32 + c34*d11;
e17=c13*c25 - c15*c31 + c35*d12;
e18=c13*c26 + c14*c25 - c15*c32 - c16*c31 + c35*d11 + c36*d12;
e19=c13*c27 + c14*c26 - c16*c32 - c17*c31 + c36*d11 + c37*d12;
e110=c14*c27 - c17*c32 + c37*d11;


d21=c32 - c24; d22=c31-c23;
e21=c21*c33 - c11*c21 - c25 + c31*d22;
e22=c21*c34 - c11*c22 - c12*c21 - c26 + c22*c33 + c31*d21 + c32*d22;
e23=- c27 - c12*c22 + c22*c34 + c32*d21;
e24=c35 - c13*c21 + c23*c33 + c33*d22;
e25=c36 - c13*c22 - c14*c21 + c23*c34 + c24*c33 + c33*d21 + c34*d22;
e26=c37 - c14*c22 + c24*c34 + c34*d21;
e27=c25*c33 - c15*c21 + c35*d22;
e28=c25*c34 - c16*c21 - c15*c22 + c26*c33 + c35*d21 + c36*d22;
e29=c26*c34 - c17*c21 - c16*c22 + c27*c33 + c36*d21 + c37*d22;
e210=- c17*c22 + c27*c34 + c37*d21;


d31=c31^2 - c11*c21;
d32=2*c31*c32 - c12*c21 - c11*c22;
d33=c32^2 - c12*c22;
d34=2*c31*c33 - c13*c21 - c11*c23;
d35=2*c31*c34 - c12*c23 - c13*c22 - c14*c21 - c11*c24 + 2*c32*c33;
d36=-c12*c24 - c14*c22 + 2*c32*c34;
d37=2*c31*c35 - c15*c21 - c11*c25;
d38=2*c31*c36 - c12*c25 - c15*c22 - c16*c21 - c11*c26 + 2*c32*c35;
d39=2*c31*c37 - c12*c26 - c16*c22 - c17*c21 - c11*c27 + 2*c32*c36;
d310=-c12*c27 - c17*c22 + 2*c32*c37;

d41=c33^2 - c13*c23;
d42=2*c33*c34 - c14*c23 - c13*c24;
d43=c34^2 - c14*c24;
d44=2*c33*c35 - c15*c23 - c13*c25;
d45=2*c33*c36 - c14*c25 - c15*c24 - c16*c23 - c13*c26 + 2*c34*c35;
d46=2*c33*c37 - c14*c26 - c16*c24 - c17*c23 - c13*c27 + 2*c34*c36;
d47=- c14*c27 - c17*c24 + 2*c34*c37;

d51=c35^2 - c15*c25;
d52=2*c35*c36 - c16*c25 - c15*c26;
d53=c36^2 - c15*c27 - c16*c26 - c17*c25 + 2*c35*c37;
d54=2*c36*c37 - c17*c26 - c16*c27;
d55=c37^2 - c17*c27;

e31=d37 + c11*d31 + c21*d41 + c31*d34;
e32=d38 + c11*d32 + c12*d31 + c21*d42 + c22*d41 + c31*d35 + c32*d34;
e33=d39 + c11*d33 + c12*d32 + c21*d43 + c22*d42 + c31*d36 + c32*d35;
e34=d310 + c12*d33 + c22*d43 + c32*d36;
e35=d44 + c13*d31 + c23*d41 + c33*d34;
e36=d45 + c13*d32 + c14*d31 + c23*d42 + c24*d41 + c33*d35 + c34*d34;
e37=d46 + c13*d33 + c14*d32 + c23*d43 + c24*d42 + c33*d36 + c34*d35;
e38=d47 + c14*d33 + c24*d43 + c34*d36;
e41=d51 + c15*d31 + c25*d41 + c35*d34;
e42=d52 + c15*d32 + c16*d31 + c25*d42 + c26*d41 + c35*d35 + c36*d34;
e43=d53 + c15*d33 + c16*d32 + c17*d31 + c25*d43 + c26*d42 + c27*d41 + c35*d36 + c36*d35 + c37*d34;
e44=d54 + c16*d33 + c17*d32 + c26*d43 + c27*d42 + c36*d36 + c37*d35;
e45=d55 + c17*d33 + c27*d43 + c37*d36;

u8=e14*e27*e31 - e17*e24*e31 - e11*e27*e35 + e17*e21*e35 + e11*e24*e41 - e14*e21*e41;
u7=e14*e27*e32 + e14*e28*e31 + e15*e27*e31 - e17*e24*e32 - e17*e25*e31 - e18*e24*e31 - e11*e27*e36 - e11*e28*e35 - e12*e27*e35 + e17*e21*e36 + e17*e22*e35 + e18*e21*e35 + e11*e24*e42 + e11*e25*e41 + e12*e24*e41 - e14*e21*e42 - e14*e22*e41 - e15*e21*e41;
u6=e14*e27*e33 + e14*e28*e32 + e14*e29*e31 + e15*e27*e32 + e15*e28*e31 + e16*e27*e31 - e17*e24*e33 - e17*e25*e32 - e17*e26*e31 - e18*e24*e32 - e18*e25*e31 - e19*e24*e31 - e11*e27*e37 - e11*e28*e36 - e11*e29*e35 - e12*e27*e36 - e12*e28*e35 - e13*e27*e35 + e17*e21*e37 + e17*e22*e36 + e17*e23*e35 + e18*e21*e36 + e18*e22*e35 + e19*e21*e35 + e11*e24*e43 + e11*e25*e42 + e11*e26*e41 + e12*e24*e42 + e12*e25*e41 + e13*e24*e41 - e14*e21*e43 - e14*e22*e42 - e14*e23*e41 - e15*e21*e42 - e15*e22*e41 - e16*e21*e41;
u5=e14*e27*e34 + e14*e28*e33 + e14*e29*e32 + e15*e27*e33 + e15*e28*e32 + e15*e29*e31 + e16*e27*e32 + e16*e28*e31 - e17*e24*e34 - e17*e25*e33 - e17*e26*e32 - e18*e24*e33 - e18*e25*e32 - e18*e26*e31 - e19*e24*e32 - e19*e25*e31 - e11*e27*e38 - e11*e28*e37 - e11*e29*e36 - e12*e27*e37 - e12*e28*e36 - e12*e29*e35 - e13*e27*e36 - e13*e28*e35 + e17*e21*e38 + e17*e22*e37 + e17*e23*e36 + e18*e21*e37 + e18*e22*e36 + e18*e23*e35 + e19*e21*e36 + e19*e22*e35 + e11*e24*e44 + e11*e25*e43 + e11*e26*e42 + e12*e24*e43 + e12*e25*e42 + e12*e26*e41 + e13*e24*e42 + e13*e25*e41 - e14*e21*e44 - e14*e22*e43 - e14*e23*e42 - e15*e21*e43 - e15*e22*e42 - e15*e23*e41 - e16*e21*e42 - e16*e22*e41 - e24*e31*e110 + e21*e35*e110 + e14*e31*e210 - e11*e35*e210;
u4=e14*e28*e34 + e14*e29*e33 + e15*e27*e34 + e15*e28*e33 + e15*e29*e32 + e16*e27*e33 + e16*e28*e32 + e16*e29*e31 - e17*e25*e34 - e17*e26*e33 - e18*e24*e34 - e18*e25*e33 - e18*e26*e32 - e19*e24*e33 - e19*e25*e32 - e19*e26*e31 - e11*e28*e38 - e11*e29*e37 - e12*e27*e38 - e12*e28*e37 - e12*e29*e36 - e13*e27*e37 - e13*e28*e36 - e13*e29*e35 + e17*e22*e38 + e17*e23*e37 + e18*e21*e38 + e18*e22*e37 + e18*e23*e36 + e19*e21*e37 + e19*e22*e36 + e19*e23*e35 + e11*e24*e45 + e11*e25*e44 + e11*e26*e43 + e12*e24*e44 + e12*e25*e43 + e12*e26*e42 + e13*e24*e43 + e13*e25*e42 + e13*e26*e41 - e14*e21*e45 - e14*e22*e44 - e14*e23*e43 - e15*e21*e44 - e15*e22*e43 - e15*e23*e42 - e16*e21*e43 - e16*e22*e42 - e16*e23*e41 - e24*e32*e110 - e25*e31*e110 + e21*e36*e110 + e22*e35*e110 + e14*e32*e210 + e15*e31*e210 - e11*e36*e210 - e12*e35*e210;
u3=e14*e29*e34 + e15*e28*e34 + e15*e29*e33 + e16*e27*e34 + e16*e28*e33 + e16*e29*e32 - e17*e26*e34 - e18*e25*e34 - e18*e26*e33 - e19*e24*e34 - e19*e25*e33 - e19*e26*e32 - e11*e29*e38 - e12*e28*e38 - e12*e29*e37 - e13*e27*e38 - e13*e28*e37 - e13*e29*e36 + e17*e23*e38 + e18*e22*e38 + e18*e23*e37 + e19*e21*e38 + e19*e22*e37 + e19*e23*e36 + e11*e25*e45 + e11*e26*e44 + e12*e24*e45 + e12*e25*e44 + e12*e26*e43 + e13*e24*e44 + e13*e25*e43 + e13*e26*e42 - e14*e22*e45 - e14*e23*e44 - e15*e21*e45 - e15*e22*e44 - e15*e23*e43 - e16*e21*e44 - e16*e22*e43 - e16*e23*e42 - e24*e33*e110 - e25*e32*e110 - e26*e31*e110 + e21*e37*e110 + e22*e36*e110 + e23*e35*e110 + e14*e33*e210 + e15*e32*e210 + e16*e31*e210 - e11*e37*e210 - e12*e36*e210 - e13*e35*e210;
u2=e15*e29*e34 + e16*e28*e34 + e16*e29*e33 - e18*e26*e34 - e19*e25*e34 - e19*e26*e33 - e12*e29*e38 - e13*e28*e38 - e13*e29*e37 + e18*e23*e38 + e19*e22*e38 + e19*e23*e37 + e11*e26*e45 + e12*e25*e45 + e12*e26*e44 + e13*e24*e45 + e13*e25*e44 + e13*e26*e43 - e14*e23*e45 - e15*e22*e45 - e15*e23*e44 - e16*e21*e45 - e16*e22*e44 - e16*e23*e43 - e24*e34*e110 - e25*e33*e110 - e26*e32*e110 + e21*e38*e110 + e22*e37*e110 + e23*e36*e110 + e14*e34*e210 + e15*e33*e210 + e16*e32*e210 - e11*e38*e210 - e12*e37*e210 - e13*e36*e210;
u1=e16*e29*e34 - e19*e26*e34 - e13*e29*e38 + e19*e23*e38 + e12*e26*e45 + e13*e25*e45 + e13*e26*e44 - e15*e23*e45 - e16*e22*e45 - e16*e23*e44 - e25*e34*e110 - e26*e33*e110 + e22*e38*e110 + e23*e37*e110 + e15*e34*e210 + e16*e33*e210 - e12*e38*e210 - e13*e37*e210;
u0=e13*e26*e45 - e16*e23*e45 - e26*e34*e110 + e23*e38*e110 + e16*e34*e210 - e13*e38*e210;
UF=[u8,u7,u6,u5,u4,u3,u2,u1,u0];
rs=roots(UF);
solution=real(rs);
% maxreal= max(abs(real(rs)));
% rs(abs(imag(rs))/maxreal > 0.01)= [];
% solution = real(rs);

for i=1:length(solution)
    b=solution(i);
    A1=e11*b^2+e12*b+e13;
    A2=e14*b^2+e15*b+e16;
    A3=e17*b^3+e18*b^2+e19*b+e110;
    B1=e21*b^2+e22*b+e23;
    B2=e24*b^2+e25*b+e26;
    B3=e27*b^3+e28*b^2+e29*b+e210;
    C1=e31*b^3+e32*b^2+e33*b+e34;
    C2=e35*b^3+e36*b^2+e37*b+e38;
    C3=e41*b^4+e42*b^3+e43*b^2+e44*b+e45;
    U1=[A1,A2;
        B1,B2;
        C1,C2];
    U2=[-A3;-B3;-C3];
    v=((U1.'*U1)\U1.')*U2; 
    %利用Gauss-Newton精定位;
    solution2 = RefineGaussNewton([b;v],E,G);
    b=solution2(1);
    c=solution2(2);
    d=solution2(3);
    Rs=[1+b^2-c^2-d^2,2*b*c-2*d,2*b*d+2*c;
        2*b*c+2*d,1-b^2+c^2-d^2,2*c*d-2*b;
        2*b*d-2*c,2*c*d+2*b,1-b^2-c^2+d^2];
    factor=1/(1+b^2+c^2+d^2);
    w=[1,b,c,d,b^2,b*c,b*d,c^2,c*d,d^2].';
    R(:,:,i)=Rs*factor;
    t(:,i)=C*w*factor;
end
end

function solution= RefineGaussNewton(solution,E,G)
    %refine the solution by using Gauss Newton method
    maxItr=1;  %maximum allowed iterations;
    lambda=1e-8;    %damped factor;
    maxLambda=1e2;  %max lambda;
    minLambda=1e-8; %min lambda;
    s1=solution(1); s2=solution(2); s3=solution(3);
    w=[1,s1,s2,s3,s1^2,s1*s2,s1*s3,s2^2,s2*s3,s3^2].';
    obj_pre = w.'*G*w;     %objective function;
    itr=1;
    %iteration
    while itr<=maxItr
        s1=solution(1); s2=solution(2); s3=solution(3);
        w=[1,s1,s2,s3,s1^2,s1*s2,s1*s3,s2^2,s2*s3,s3^2].';
        %Jacobian matrixa
         Jac=[ 0,     0,     0;
               1,     0,     0;
               0,     1,     0;
               0,     0,     1;
            2*s1,     0,     0;
              s2,    s1,     0;
              s3,     0,    s1;
               0,  2*s2,     0;
               0,    s3,    s2;
               0,     0,   2*s3 ];
        Fk=E*w;
        Jk=E*Jac;
        solution_temp = solution;
        while lambda<maxLambda
            %increment
            dk=-((Jk.'*Jk)+lambda*eye(3))\(Jk.'*Fk);
            %update parameter
            solution = solution_temp + dk;
            s1=solution(1); s2=solution(2); s3=solution(3);
            w=[1,s1,s2,s3,s1^2,s1*s2,s1*s3,s2^2,s2*s3,s3^2].';
            %evaluate thee rror;
            obj_cur = w.'*G*w;
            %check convergence
            if obj_cur >= obj_pre
                lambda = 10*lambda;
                continue;
            else
                obj_pre = obj_cur; 
                lambda = 0.1*lambda;
                break;
            end 
        end
        if lambda >= maxLambda
            solution = solution_temp;
            break;
        end
        
        if lambda <= minLambda
            lambda = minLambda;
        end
        itr = itr + 1;
    end
    
end


