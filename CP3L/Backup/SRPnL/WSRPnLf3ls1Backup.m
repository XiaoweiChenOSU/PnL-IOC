% Author :  Ping Wang                                                        
% Contact:  pingwangsky@gmail.com 
% This programe is implemented in matlab 2018a
% License:  Copyright (c) 2019 Ping Wang, All rights reserved       
% Address:  College of Electrical and Information Engineering, Lanzhou University of Technology              
% My site:  https://sites.google.com/view/ping-wang-homepage  

function [ R_wc,t_wc] = WSRPnLf3ls(p1,p2,W1,W2,IA,po1,po2)
% ,minErr 
% P1N_w = [W1 W2];
% P2N_w = [W2 W2(:,2:3) W2(:,1)];
% Vw=P2N_w-P1N_w;
% Vw = xnorm(Vw);


nLine = length(p1);
p1 = [p1; ones(1,nLine)];
p2 = [p2; ones(1,nLine)];

% pn1 = [p1 p2];
% pn2 = [p2 p2(:,2:3) p2(:,1)];
% nxs = pn1;
% nxe = pn2;
% n_nc = xcross_mat(nxs,nxe);
% nc_nbar = xnorm(n_nc);

n_c=xnorm(cross(p1,p2));
%用来计算旋转矩阵和平移矩阵关系的系数;
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
%计算用于Gauss-Newton优化的参数;

if det(N.'*N) < 2e-10
    CC=(pinv(N.'*N)*N.')*Q;
else
    CC=((N.'*N)\N.')*Q;
end

EE=Q-N*CC;
GG=EE.'*EE;

%计算直线方向和直线上任意一点;
Vw=W2-W1;
Vw = xnorm(Vw);
Pw = (W1 + W2) * 0.5;

xs=p1;
xe=p2;

n=nLine;
%choose the line with longest length in the image plane;
lineLenVec = sqrt(sum((xs-xe).^2));
[longestLineLen, LineID] = max(lineLenVec);
%将投影最大的直线存在第一条直线位置，即最前头;
% temp = xs(:,1);  xs(:,1) = xs(:, LineID); xs(:,LineID) = temp;
% temp = xe(:,1);  xe(:,1) = xe(:, LineID); xe(:,LineID) = temp;
temp=n_c(:,1);   n_c(:,1)=n_c(:,LineID);  n_c(:,LineID)=temp;
temp = Vw(:,1);  Vw(:,1) = Vw(:, LineID); Vw(:,LineID) = temp;
temp = Pw(:,1);  Pw(:,1) = Pw(:, LineID); Pw(:,LineID) = temp;
lineLenVec(1) = 0;
l1 = xs(:,1) - xe(:,1); 
l1 = l1/norm(l1);

%first line is fixed. Find the second line
%在余下的直线里面找投影最大的直线,并将其存在第二位;
for i=2:n
    lineLenVec(LineID) = 0;
    [longestLineLen, LineID] = max(lineLenVec);%the current lonest line
    l2 = xs(:,LineID) - xe(:,LineID);
    l2 = l2/norm(l2);
    cosAngle = abs(l1'*l2);
    if cosAngle <  1.1 % 0<angle<180, 15<angle<165,or 30<angle<150
        break;
    end
end
% temp = xs(:,2);  xs(:,2) = xs(:, LineID); xs(:,LineID) = temp;
% temp = xe(:,2);  xe(:,2) = xe(:, LineID); xe(:,LineID) = temp;
temp=n_c(:,2);   n_c(:,2)=n_c(:,LineID);  n_c(:,LineID)=temp;
temp = Vw(:,2);  Vw(:,2) = Vw(:, LineID); Vw(:,LineID) = temp;
temp = Pw(:,2);  Pw(:,2) = Pw(:, LineID); Pw(:,LineID) = temp;
 
% Get the canonical configuration
% nc1 = cross(xs(:,1),xe(:,1));  nc1 = nc1/norm(nc1);
nc1=n_c(:,1);
Vw1 = Vw(:,1);    

    
Xm = cross(nc1,Vw1); Xm = Xm/norm(Xm); %the X axis of Model frame
Ym = nc1; %the Y axis of Model frame
Zm = cross(Xm,Ym);  Zm= Zm/norm(Zm);%the Z axis of Model frame;


Rot = [Xm, Ym, Zm]'; % Rot * [Xm, Ym, Zm] = I.

%rotate all the vector by Rot.    
% n_c = cross(xs,xe);
% n_c = xnorm(n_c);

nc_bar = Rot * n_c;
Vw_bar = Rot * Vw;
% Pw_bar = Rot * Pw;
    
% determine the angle psi, it is the angle between z axis and Vw_bar(:,1).
% the rotation matrix Rx(alpha) rotates Vw_bar(:,1) to z axis
cos_alpha= [0,0,1]*Vw_bar(:,1); %the angle between z axis and Vw_bar(:,1).
%上面公式的得来依据通过向量和模值求夹角;

sin_alpha= sqrt(1 - cos_alpha*cos_alpha);
Rx= [1 0 0; 0 cos_alpha -sin_alpha; 0 sin_alpha cos_alpha];
Zaxis = Rx * Vw_bar(:,1); % should be the Z axis, i.e. [0, 0, 1]';
if 1 - abs(Zaxis(3)) > 1e-5
   Rx = Rx';
end
    
%fourth step, estimate the rotation angle phi by least square residual.
%i.e the rotation matrix Rz(phi)
Vm2 = Rx * Vw_bar(:,2);
A2= Vm2(1);      B2= Vm2(2);      C2= Vm2(3);
x2= nc_bar(1,2); y2= nc_bar(2,2); z2= nc_bar(3,2);

coef   = zeros(9,1); %coefficients of equation (7)
polyDF = zeros(16,1); %dF = ployDF(1) * t^15 + ployDF(2) * t^14 + ... + ployDF(15) * t + ployDF(16);
% construct the  polynomial F'
for i=3:n %The first two lines are included in every triplet.
    Vm3 = Rx*Vw_bar(:,i);
    A3= Vm3(1);      B3= Vm3(2);      C3= Vm3(3);
    x3= nc_bar(1,i); y3= nc_bar(2,i); z3= nc_bar(3,i);
    u11 = -z2*A2*y3*B3 + y2*B2*z3*A3;
    u12 = -y2*A2*z3*B3 + z2*B2*y3*A3;
    u13 = -y2*B2*z3*B3 + z2*B2*y3*B3 + y2*A2*z3*A3 - z2*A2*y3*A3;
    u14 = -y2*B2*x3*C3 + x2*C2*y3*B3;
    u15 =  x2*C2*y3*A3 - y2*A2*x3*C3;
    u21 = -x2*A2*y3*B3 + y2*B2*x3*A3;
    u22 = -y2*A2*x3*B3 + x2*B2*y3*A3;
    u23 =  x2*B2*y3*B3 - y2*B2*x3*B3 - x2*A2*y3*A3 + y2*A2*x3*A3;
    u24 =  y2*B2*z3*C3 - z2*C2*y3*B3;
    u25 =  y2*A2*z3*C3 - z2*C2*y3*A3;
    u31 = -x2*A2*z3*A3 + z2*A2*x3*A3;
    u32 = -x2*B2*z3*B3 + z2*B2*x3*B3;
    u33 =  x2*A2*z3*B3 - z2*A2*x3*B3 + x2*B2*z3*A3 - z2*B2*x3*A3;
    u34 =  z2*A2*z3*C3 + x2*A2*x3*C3 - z2*C2*z3*A3 - x2*C2*x3*A3;
    u35 = -z2*B2*z3*C3 - x2*B2*x3*C3 + z2*C2*z3*B3 + x2*C2*x3*B3;
    u36 = -x2*C2*z3*C3 + z2*C2*x3*C3;
        
    a4 =   u11*u11 + u12*u12 - u13*u13 - 2*u11*u12 +   u21*u21 + u22*u22 - u23*u23...
            -2*u21*u22 - u31*u31 - u32*u32 +   u33*u33 + 2*u31*u32;
    a3 =2*(u11*u14 - u13*u15 - u12*u14 +   u21*u24 -   u23*u25...
            - u22*u24 - u31*u34 + u33*u35 +   u32*u34);
    a2 =-2*u12*u12 + u13*u13 + u14*u14 -   u15*u15 + 2*u11*u12 - 2*u22*u22 + u23*u23...
            + u24*u24 - u25*u25 +2*u21*u22+ 2*u32*u32 -   u33*u33...
            - u34*u34 + u35*u35 -2*u31*u32- 2*u31*u36 + 2*u32*u36;
    a1 =2*(u12*u14 + u13*u15 +  u22*u24 +  u23*u25 -   u32*u34 - u33*u35 - u34*u36);
    a0 =   u12*u12 + u15*u15+   u22*u22 +  u25*u25 -   u32*u32 - u35*u35 - u36*u36 - 2*u32*u36;
    b3 =2*(u11*u13 - u12*u13 +  u21*u23 -  u22*u23 -   u31*u33 + u32*u33);
    b2 =2*(u11*u15 - u12*u15 +  u13*u14 +  u21*u25 -   u22*u25 + u23*u24 - u31*u35 + u32*u35 - u33*u34);
    b1 =2*(u12*u13 + u14*u15 +  u22*u23 +  u24*u25 -   u32*u33 - u34*u35 - u33*u36);
    b0 =2*(u12*u15 + u22*u25 -  u32*u35 -  u35*u36);
        
    d0 =    a0*a0 -   b0*b0;
    d1 = 2*(a0*a1 -   b0*b1);
    d2 =    a1*a1 + 2*a0*a2 +   b0*b0 - b1*b1 - 2*b0*b2;
    d3 = 2*(a0*a3 +   a1*a2 +   b0*b1 - b1*b2 -   b0*b3);
    d4 =    a2*a2 + 2*a0*a4 + 2*a1*a3 + b1*b1 + 2*b0*b2 - b2*b2 - 2*b1*b3;
    d5 = 2*(a1*a4 +   a2*a3 +   b1*b2 + b0*b3 -   b2*b3);
    d6 =    a3*a3 + 2*a2*a4 +   b2*b2 - b3*b3 + 2*b1*b3;
    d7 = 2*(a3*a4 +   b2*b3);
    d8 =    a4*a4 +   b3*b3;
        
    coef = coef + [a4, a3, a2, a1, a0, b3, b2, b1, b0]';
        
    polyDF(1) = polyDF(1) +                                8*d8*d8;
    polyDF(2) = polyDF(2) + 15* d7*d8;
    polyDF(3) = polyDF(3) + 14* d6*d8 +                    7*d7*d7;
    polyDF(4) = polyDF(4) + 13*(d5*d8 +  d6*d7);
    polyDF(5) = polyDF(5) + 12*(d4*d8 +  d5*d7)+           6*d6*d6;
    polyDF(6) = polyDF(6) + 11*(d3*d8 +  d4*d7 +  d5*d6);
    polyDF(7) = polyDF(7) + 10*(d2*d8 +  d3*d7 +  d4*d6) + 5*d5*d5;
    polyDF(8) = polyDF(8) + 9 *(d1*d8 +  d2*d7 +  d3*d6  +   d4*d5);
    polyDF(9) = polyDF(9) + 8 *(d1*d7 +  d2*d6 +  d3*d5) + 4*d4*d4 + 8*d0*d8;
    polyDF(10)= polyDF(10)+ 7 *(d1*d6 +  d2*d5 +  d3*d4) +           7*d0*d7;
    polyDF(11)= polyDF(11)+ 6 *(d1*d5 +  d2*d4)+           3*d3*d3 + 6*d0*d6;
    polyDF(12)= polyDF(12)+ 5 *(d1*d4 +  d2*d3)+                     5*d0*d5;
    polyDF(13)= polyDF(13)+ 4 * d1*d3 +                    2*d2*d2 + 4*d0*d4;
    polyDF(14)= polyDF(14)+ 3 * d1*d2 +                              3*d0*d3;
    polyDF(15)= polyDF(15)+                                  d1*d1 + 2*d0*d2;
    polyDF(16)= polyDF(16)+                                            d0*d1;
end

%solve polyDF
rs= roots(polyDF);
% retriving the local minima of the cost function.
maxreal= max(abs(real(rs)));
rs(abs(imag(rs))/maxreal > 0.01)= [];
minRoots = real(rs);

% poly    = (15:-1:1).*polyDF(1:15)';
% PolyVal = polyval(poly, minRoots);
% minRoots(PolyVal <= 0)= [];

count=1;
minErr=inf;
numOfRoots = length(minRoots);
for rootId = 1 : numOfRoots
    cosbeta= minRoots(rootId);
    sign1 = sign(coef(1) * cosbeta^4 + coef(2) * cosbeta^3 + coef(3) * cosbeta^2 + coef(4) * cosbeta + coef(5));
    sign2 = sign(coef(6) * cosbeta^3 + coef(7) * cosbeta^2 + coef(8) * cosbeta   + coef(9));
    sinbeta= -sign1*sign2*sqrt(abs(1-cosbeta*cosbeta));
    Rz = [cosbeta -sinbeta 0; sinbeta cosbeta 0; 0 0 1];
    RzRxRot = Rz*Rx*Rot;
    
    nxi=nc_bar(1,:)'; nyi=nc_bar(2,:)'; nzi=nc_bar(3,:)';
    Vm=RzRxRot*Vw;
    Vxi=Vm(1,:)'; Vyi=Vm(2,:)'; Vzi=Vm(3,:)';
    A=[nxi .* Vxi + nzi .* Vzi,nxi .* Vzi - nzi .* Vxi,nyi .* Vyi];
    
    G=A.'*A;
    g11=G(1,1);
    g12=G(1,2);
    g13=G(1,3);
    g22=G(2,2);
    g23=G(2,3);
    F4=4*g12^2+g22^2+g11^2-2*g11*g22;
    F3=4*g12*g23+2*g11*g13-2*g13*g22;
    F2=g23^2+2*g11*g22+g13^2-4*g12^2-g11^2-g22^2;
    F1=2*g13*g22-2*g11*g13-2*g12*g23;
    F0=g12^2-g13^2;
    
    c= roots([F4,F3,F2,F1,F0]);
    maxreal1= max(abs(real(c)));
    c(abs(imag(c))/maxreal1 > 0.01)= [];
    
    c= real(c);
    c=c.';
    s=(2*g12*c.^2+g23*c-g12)./((g11-g22)*c+g13);
    
%     pn1 = [p1 p2];
%     pn2 = [p2 p2(:,2:3) p2(:,1)];
%     nxs = pn1;
%     nxe = pn2;
%     n_nc = xcross_mat(nxs,nxe);
%     nc_nbar = xnorm(n_nc);
%     
%     P1N_w = [W1 W2];
%     P2N_w = [W2 W2(:,2:3) W2(:,1)];
%     
%     p2d = [ip1 ip2];
    
    
    for j=1:length(c)
        s1=c(j);
        s2=s(j);
        Ry = [s1, 0, s2; 0, 1, 0; -s2, 0, s1];
        R1 = (Rot') * (Ry * Rz * Rx) * Rot;
        %t1=Rot.'*V*[s1,s2,1].';
        
        solution = RefineGaussNewton(Cayley(R1),EE,GG);
        s1=solution(1);
        s2=solution(2);
        s3=solution(3);
        sr=[1,s1,s2,s3,s1^2,s1*s2,s1*s3,s2^2,s2*s3,s3^2].';
        factor=1/(1+s1^2+s2^2+s3^2); 
        Rr=factor*[1+s1^2-s2^2-s3^2,2*s1*s2-2*s3,2*s2+2*s1*s3;
                   2*s3+2*s1*s2,1-s1^2+s2^2-s3^2,2*s2*s3-2*s1;
                   2*s1*s3-2*s2,2*s1+2*s2*s3,1-s1^2-s2^2+s3^2];
                  
        Rt=factor*CC*sr;       
               
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%        
        %According to the fact that n_i^C should be orthogonal to Pi^c and Vi^c, we 
        %have: scaleproduct(Vi^c, ni^c) = 0  and scaleproduct(Pi^c, ni^c) = 0.
        %where Vi^c = Rwc * Vi^w,  Pi^c = Rwc *(Pi^w - pos_cw) = Rwc * Pi^w - pos;
        %Using the above two constraints to construct linear equation system Mat about 
        %For type0,there will be four points which only have one unknown
        %axis, choose z as unknown axises, then the linear equation system Mat abou
        %[x, y, z, tx, ty, tz, 1]. 
        
        pn1 = [p1 p2];
        pn2 = [p2 p2(:,2:3) p2(:,1)];
        nxs = pn1;
        nxe = pn2;
        n_nc = xcross_mat(nxs,nxe);
        nc_nbar = xnorm(n_nc);
        P1N_w = [W1 W2];
        P2N_w = [W2 W2(:,2:3) W2(:,1)];
        
        MatN = zeros(11, 7);       
        Rw_c = Rr;
        for i = 1:3
            nxi = nc_nbar(1,i);  nyi = nc_nbar(2,i);  nzi = nc_nbar(3,i);
            if i == 1
                MatN(i, i) = nxi*Rw_c(1,1)+nyi*Rw_c(2,1)+nzi*Rw_c(3,1);
                MatN(i, 7) = P2N_w(2,i)*(nxi*Rw_c(1,2)+nyi*Rw_c(2,2)+nzi*Rw_c(3,2)) + P2N_w(3,i)*(nxi*Rw_c(1,3)+nyi*Rw_c(2,3)+nzi*Rw_c(3,3));
            elseif i == 2
                MatN(i, i) = nxi*Rw_c(1,2)+nyi*Rw_c(2,2)+nzi*Rw_c(3,2);
                MatN(i, 7) = P2N_w(1,i)*(nxi*Rw_c(1,1)+nyi*Rw_c(2,1)+nzi*Rw_c(3,1)) + P2N_w(3,i)*(nxi*Rw_c(1,3)+nyi*Rw_c(2,3)+nzi*Rw_c(3,3));      
            elseif i == 3
                MatN(i, i) = nxi*Rw_c(1,3)+nyi*Rw_c(2,3)+nzi*Rw_c(3,3);
                MatN(i, 7) = P2N_w(1,i)*(nxi*Rw_c(1,1)+nyi*Rw_c(2,1)+nzi*Rw_c(3,1)) + P2N_w(2,i)*(nxi*Rw_c(1,2)+nyi*Rw_c(2,2)+nzi*Rw_c(3,2));      
            end
            MatN(i, 4) = nxi;
            MatN(i, 5) = nyi;
            MatN(i, 6) = nzi; 
        end
        
%         for i = 4:7          
%             % apply the constraint scaleproduct(Vi^c, ni^c) = 0
%             if i>4 %because that if i=1, scaleproduct(Vi^c, ni^c) always be 0
%                 j = i-1;
%                 nxi = nc_nbar(1,j);  nyi = nc_nbar(2,j);  nzi = nc_nbar(3,j);
%                 if i == 5
%                     MatN(2*i-5, i-4) = nxi*Rw_c(1,1)+nyi*Rw_c(2,1)+nzi*Rw_c(3,1);
%                     MatN(2*i-5, i-3) = -(nxi*Rw_c(1,2)+nyi*Rw_c(2,2)+nzi*Rw_c(3,2));
%                     MatN(2*i-5, 7) = (-P2N_w(1,j)*(nxi*Rw_c(1,1)+nyi*Rw_c(2,1)+nzi*Rw_c(3,1)))+ (P1N_w(2,j)*(nxi*Rw_c(1,2)+nyi*Rw_c(2,2)+nzi*Rw_c(3,2))) + ((P1N_w(3,j)-P2N_w(3,j))*(nxi*Rw_c(1,3)+nyi*Rw_c(2,3)+nzi*Rw_c(3,3)));    
%                 elseif i == 6
%                     MatN(2*i-5, i-4) = (nxi*Rw_c(1,2)+nyi*Rw_c(2,2)+nzi*Rw_c(3,2));
%                     MatN(2*i-5, i-3) = -(nxi*Rw_c(1,3)+nyi*Rw_c(2,3)+nzi*Rw_c(3,3));
%                     MatN(2*i-5, 7) = ((P1N_w(1,j)-P2N_w(1,j))*(nxi*Rw_c(1,1)+nyi*Rw_c(2,1)+nzi*Rw_c(3,1)))-(P2N_w(2,j)*(nxi*Rw_c(1,2)+nyi*Rw_c(2,2)+nzi*Rw_c(3,2))) + (P1N_w(3,j)*(nxi*Rw_c(1,3)+nyi*Rw_c(2,3)+nzi*Rw_c(3,3)));    
%                 elseif i == 7
%                     MatN(2*i-5, 1) = -(nxi*Rw_c(1,1)+nyi*Rw_c(2,1)+nzi*Rw_c(3,1));
%                     MatN(2*i-5, i-4) = (nxi*Rw_c(1,3)+nyi*Rw_c(2,3)+nzi*Rw_c(3,3));
%                     MatN(2*i-5, 7) = (P1N_w(1,j)*(nxi*Rw_c(1,1)+nyi*Rw_c(2,1)+nzi*Rw_c(3,1)))+ ((P1N_w(2,j)-P2N_w(2,j))*(nxi*Rw_c(1,2)+nyi*Rw_c(2,2)+nzi*Rw_c(3,2))) - (P2N_w(3,j)*(nxi*Rw_c(1,3)+nyi*Rw_c(2,3)+nzi*Rw_c(3,3)));                   
%                 end 
%             end
%             % apply the constraint scaleproduct(Pi^c, ni^c) = 0
%             if i < 7
%                 nxi = nc_nbar(1,i);  nyi = nc_nbar(2,i);  nzi = nc_nbar(3,i); 
%                 if i == 4
%                     MatN(2*i-4, i-3) = (nxi*Rw_c(1,1)+nyi*Rw_c(2,1)+nzi*Rw_c(3,1))/2;
%                     MatN(2*i-4, i-2) = (nxi*Rw_c(1,2)+nyi*Rw_c(2,2)+nzi*Rw_c(3,2))/2;
%                     MatN(2*i-4, 7) = (P2N_w(1,i)*(nxi*Rw_c(1,1)+nyi*Rw_c(2,1)+nzi*Rw_c(3,1))/2)+ (P1N_w(2,i)*(nxi*Rw_c(1,2)+nyi*Rw_c(2,2)+nzi*Rw_c(3,2))/2) + ((P1N_w(3,i)+P2N_w(3,i))*(nxi*Rw_c(1,3)+nyi*Rw_c(2,3)+nzi*Rw_c(3,3))/2);    
%                 elseif i == 5
%                     MatN(2*i-4, i-3) = (nxi*Rw_c(1,2)+nyi*Rw_c(2,2)+nzi*Rw_c(3,2))/2;
%                     MatN(2*i-4, i-2) = (nxi*Rw_c(1,3)+nyi*Rw_c(2,3)+nzi*Rw_c(3,3))/2;
%                     MatN(2*i-4, 7) = ((P1N_w(1,i)+P2N_w(1,i))*(nxi*Rw_c(1,1)+nyi*Rw_c(2,1)+nzi*Rw_c(3,1))/2)+ (P2N_w(2,i)*(nxi*Rw_c(1,2)+nyi*Rw_c(2,2)+nzi*Rw_c(3,2))/2) + (P1N_w(3,i)*(nxi*Rw_c(1,3)+nyi*Rw_c(2,3)+nzi*Rw_c(3,3))/2);    
%                 elseif i == 6
%                     MatN(2*i-4, 1) = (nxi*Rw_c(1,1)+nyi*Rw_c(2,1)+nzi*Rw_c(3,1))/2;
%                     MatN(2*i-4, i-3) = (nxi*Rw_c(1,3)+nyi*Rw_c(2,3)+nzi*Rw_c(3,3))/2;
%                     MatN(2*i-4, 7) = (P1N_w(1,i)*(nxi*Rw_c(1,1)+nyi*Rw_c(2,1)+nzi*Rw_c(3,1))/2)+ ((P1N_w(2,i)+P2N_w(2,i))*(nxi*Rw_c(1,2)+nyi*Rw_c(2,2)+nzi*Rw_c(3,2))/2) + (P2N_w(3,i)*(nxi*Rw_c(1,3)+nyi*Rw_c(2,3)+nzi*Rw_c(3,3))/2);                   
%                 end
%                 MatN(2*i-4, 4) = nxi;
%                 MatN(2*i-4, 5) = nyi;
%                 MatN(2*i-4, 6) = nzi;
%             end
%         end
        for i = 4:6          
            % apply the constraint scaleproduct(Pi^c, ni^c) = 0
            nxi = nc_nbar(1,i);  nyi = nc_nbar(2,i);  nzi = nc_nbar(3,i); 
            if i == 4
                MatN(i, i-3) = (nxi*Rw_c(1,1)+nyi*Rw_c(2,1)+nzi*Rw_c(3,1))/2;
                MatN(i, i-2) = (nxi*Rw_c(1,2)+nyi*Rw_c(2,2)+nzi*Rw_c(3,2))/2;
                MatN(i, 7) = (P2N_w(1,i)*(nxi*Rw_c(1,1)+nyi*Rw_c(2,1)+nzi*Rw_c(3,1))/2)+ (P1N_w(2,i)*(nxi*Rw_c(1,2)+nyi*Rw_c(2,2)+nzi*Rw_c(3,2))/2) + ((P1N_w(3,i)+P2N_w(3,i))*(nxi*Rw_c(1,3)+nyi*Rw_c(2,3)+nzi*Rw_c(3,3))/2);    
            elseif i == 5
                MatN(i, i-3) = (nxi*Rw_c(1,2)+nyi*Rw_c(2,2)+nzi*Rw_c(3,2))/2;
                MatN(i, i-2) = (nxi*Rw_c(1,3)+nyi*Rw_c(2,3)+nzi*Rw_c(3,3))/2;
                MatN(i, 7) = ((P1N_w(1,i)+P2N_w(1,i))*(nxi*Rw_c(1,1)+nyi*Rw_c(2,1)+nzi*Rw_c(3,1))/2)+ (P2N_w(2,i)*(nxi*Rw_c(1,2)+nyi*Rw_c(2,2)+nzi*Rw_c(3,2))/2) + (P1N_w(3,i)*(nxi*Rw_c(1,3)+nyi*Rw_c(2,3)+nzi*Rw_c(3,3))/2);    
            elseif i == 6
                MatN(i, 1) = (nxi*Rw_c(1,1)+nyi*Rw_c(2,1)+nzi*Rw_c(3,1))/2;
                MatN(i, i-3) = (nxi*Rw_c(1,3)+nyi*Rw_c(2,3)+nzi*Rw_c(3,3))/2;
                MatN(i, 7) = (P1N_w(1,i)*(nxi*Rw_c(1,1)+nyi*Rw_c(2,1)+nzi*Rw_c(3,1))/2)+ ((P1N_w(2,i)+P2N_w(2,i))*(nxi*Rw_c(1,2)+nyi*Rw_c(2,2)+nzi*Rw_c(3,2))/2) + (P2N_w(3,i)*(nxi*Rw_c(1,3)+nyi*Rw_c(2,3)+nzi*Rw_c(3,3))/2);                   
            end
            MatN(i, 4) = nxi;
            MatN(i, 5) = nyi;
            MatN(i, 6) = nzi;
        end
        
        f = IA(1,1);
        u0 = IA(1,3);
        v0 = IA(2,3);
        MatN(7,4) = f;
        MatN(7,6) = u0-po1(1,1);
        MatN(7,7) = (f*Rw_c(1,1) + u0*Rw_c(3,1) - po1(1,1)*Rw_c(3,1))*W1(1,1)+(f*Rw_c(1,2) + u0*Rw_c(3,2) - po1(1,1)*Rw_c(3,2))*W1(2,1) + (f*Rw_c(1,3) + u0*Rw_c(3,3) - po1(1,1)*Rw_c(3,3))*W1(3,1);
        MatN(8,5) = f;
        MatN(8,6) = v0-po1(1,2);
        MatN(8,7) = (f*Rw_c(2,1) + v0*Rw_c(3,1) - po1(1,2)*Rw_c(3,1))*W1(1,1)+(f*Rw_c(2,2) + v0*Rw_c(3,2) - po1(1,2)*Rw_c(3,2))*W1(2,1) + (f*Rw_c(2,3) + v0*Rw_c(3,3) - po1(1,2)*Rw_c(3,3))*W1(3,1);
        for i = 1:3
            nxi = nc_nbar(1,i);  nyi = nc_nbar(2,i);  nzi = nc_nbar(3,i);   
            MatN(8+i, 4) = nxi;
            MatN(8+i, 5) = nyi;
            MatN(8+i, 6) = nzi; 
            MatN(8+i, 7) = P1N_w(1,i)*(nxi*Rw_c(1,1)+nyi*Rw_c(2,1)+nzi*Rw_c(3,1)) + P1N_w(2,i)*(nxi*Rw_c(1,2)+nyi*Rw_c(2,2)+nzi*Rw_c(3,2)) + P1N_w(3,i)*(nxi*Rw_c(1,3)+nyi*Rw_c(2,3)+nzi*Rw_c(3,3));
        end
        
        for x = 1:256
            MatN(12,1)  = 1;
            MatN(12,7)  = x;
            %solve the linear system MatN * [costheta, sintheta, tx, ty, tz, 1]' = 0  using SVD,
            [UMatN, SMatN, VMatN] = svd(MatN);
            vecN = VMatN(:,7);% the last column of Vmat;
            vecN = vecN/vecN(7); %the condition that the last element of vec should be 1.     
            pos1 = vecN(4:6);
            resM(x,:) = pos1;
            estPM(x,:)  = vecN(1:3);
            W2(1,4) = vecN(1);
            W2(2,5) = vecN(2);
            W2(3,6) = vecN(3);
            P1N_w = [W1 W2];
            P2N_w = [W2 W2(:,2:3) W2(:,1)];
            [rot_wc, pos_wc, err] =  GN(pn1, pn2, P1N_w, P2N_w, Rr);
            estR{x} = rot_wc;
            estT{x} = pos_wc;
            estErr(x) = err;
        end
        
        
        
     
        pos1 = vecN(4:6);
        
%         A = MatN(:,1:6);
%         b = -MatN(:,7);
%         
%         lb = [100;50;100;-256;0;0];
%         ub = [256;256;256;256;256;256];
% 
%         sol = lsqlin(A,b,[],[],[],[],lb,ub);
%         
%         
%         XP1 = IA*[Rr pos1]*[P1N_w(:,1);1];
%         XP1 = XP1/XP1(3);
%         
% 
% %         
%         P1N_w(1,4) = vecN(1);
%         
%         XP2 = IA*[Rr pos1]*[P1N_w(:,4);1];
%         XP2 = XP2/XP2(3);
%         
%         vecN = vecN/vecN(7);
%         
%         count = 1;
%         for x = 1:256
%             lb = [x;0;0;-256;-256;-256];
%             ub = [x;256;256;256;256;256];
%             sol = lsqlin(A,b,[],[],[],[],lb,ub); 
% 
%             YP1 = IA*[Rr sol(4:6)]*[P1N_w(:,1);1];
%             YP1 = YP1/YP1(3);
% 
%             P1N_w(1,4) = sol(1);
% 
%             YP2 = IA*[Rr sol(4:6)]*[P1N_w(:,4);1];
%             YP2 = YP2/YP2(3);
% 
%             P1N_w(2,5) = sol(2);
% 
%             YP3 = IA*[Rr sol(4:6)]*[P1N_w(:,5);1];
%             YP3 = YP3/YP3(3);
% 
%             P1N_w(3,6) = sol(3);
% 
%             YP4 = IA*[Rr sol(4:6)]*[P1N_w(:,6);1];
%             YP4 = YP4/YP4(3);
% 
%             res{count} = sol;
%             estP{count}  = [YP1,YP2,YP3,YP4];
%             count =  count + 1;
%         end
        
        
%         

%         P1N_w(2,5) = vecN(2);
%         
%         XP3 = IA*[Rr pos1]*[P1N_w(:,5);1];
%         XP3 = XP3/XP3(3);
%         
%         P1N_w(3,6) = vecN(3);
%         
%         XP4 = IA*[Rr pos1]*[P1N_w(:,6);1];
%         XP4 = XP4/XP4(3);
%         
        
%         testMat(1:3,1:3) = MatN(5:2:9,1:3);
%         testMat(1:3,4) = MatN(5:2:9,7);
%         testMat(4,1) = 1;
%         testMat(4,4) = W2(1,1);
%         [tUMatN, tSMatN, tVMatN] = svd(testMat);
        
        


        
%         count = 1;
%         for x = 1:10:256
%             for y = 1:10:256
%                 for z = 1:10:256
%                     lb = [x;y;z;-256;-256;-256];
%                     ub = [256;150;255;256;256;256];
%                     sol = lsqlin(A,b,[],[],[],[],lb,ub); 
%                     
%                     YP1 = IA*[Rr sol(4:6)]*[P1N_w(:,1);1];
%                     YP1 = YP1/YP1(3);
% 
%                     P1N_w(1,4) = sol(1);
% 
%                     YP2 = IA*[Rr sol(4:6)]*[P1N_w(:,4);1];
%                     YP2 = YP2/YP2(3);
% 
%                     P1N_w(2,5) = sol(2);
% 
%                     YP3 = IA*[Rr sol(4:6)]*[P1N_w(:,5);1];
%                     YP3 = YP3/YP3(3);
% 
%                     P1N_w(3,6) = sol(3);
% 
%                     YP4 = IA*[Rr sol(4:6)]*[P1N_w(:,6);1];
%                     YP4 = YP4/YP4(3);
%                     
%                     res{count} = sol;
%                     estP{count}  = [YP1,YP2,YP3,YP4];
%                     count =  count + 1;
%                 end
%             end
%         end
%         
%    
%         MatNP(1:9,:)  = MatN;
%         
% %         MatN(10, 1) = 1;
% %         MatN(10, 7) = W2(1,1);
%         f = IA(1,1);
%         u0 = IA(1,3);
%         v0 = IA(2,3);
%         
%         for i = 10:13
%             if i > 10
%                 if i == 11
%                     MatNP(2*i-11,1) = f*Rw_c(2,1) + v0*Rw_c(3,1) - po2(1,2)*Rw_c(3,1);
%                     MatNP(2*i-11,5) = f;
%                     MatNP(2*i-11,6) = v0-po2(1,2);
%                     MatNP(2*i-11,7) = (f*Rw_c(2,2) + v0*Rw_c(3,2) - po2(1,2)*Rw_c(3,2))*W2(2,1) + (f*Rw_c(2,3) + v0*Rw_c(3,3) - po2(1,2)*Rw_c(3,3))*W2(3,1);
%                 end
%                 if i == 12
%                     MatNP(2*i-11,2) = f*Rw_c(2,2) + v0*Rw_c(3,2) - po2(2,2)*Rw_c(3,2);
%                     MatNP(2*i-11,5) = f;
%                     MatNP(2*i-11,6) = v0-po2(2,2);
%                     MatNP(2*i-11,7) = (f*Rw_c(2,1) + v0*Rw_c(3,1) - po2(2,2)*Rw_c(3,1))*W2(1,2) + (f*Rw_c(2,3) + v0*Rw_c(3,3) - po2(2,2)*Rw_c(3,3))*W2(3,2);
%                 end
%                 if i == 13
%                     MatNP(2*i-11,3) = f*Rw_c(2,3) + v0*Rw_c(3,3) - po2(3,2)*Rw_c(3,3);
%                     MatNP(2*i-11,5) = f;
%                     MatNP(2*i-11,6) = v0-po2(3,2);
%                     MatNP(2*i-11,7) = (f*Rw_c(1,1) + v0*Rw_c(3,1) - po2(3,2)*Rw_c(3,1))*W2(1,3) + (f*Rw_c(2,2) + v0*Rw_c(3,2) - po2(3,2)*Rw_c(3,2))*W2(2,3);
%                 end
%             end
%             if i < 13
%                 if i == 10
%                     MatNP(2*i-10,1) = f*Rw_c(1,1) + u0*Rw_c(3,1) - po2(1,1)*Rw_c(3,1);
%                     MatNP(2*i-10,4) = f;
%                     MatNP(2*i-10,6) = u0-po2(1,1);
%                     MatNP(2*i-10,7) = (f*Rw_c(1,2) + u0*Rw_c(3,2) - po2(1,1)*Rw_c(3,2))*W2(2,1) + (f*Rw_c(1,3) + u0*Rw_c(3,3) - po2(1,1)*Rw_c(3,3))*W2(3,1);
%                 end
%                 if i == 11
%                     MatNP(2*i-10,2) = f*Rw_c(1,2) + u0*Rw_c(3,2) - po2(2,1)*Rw_c(3,2);
%                     MatNP(2*i-10,4) = f;
%                     MatNP(2*i-10,6) = u0-po2(2,1);
%                     MatNP(2*i-10,7) = (f*Rw_c(1,1) + u0*Rw_c(3,1) - po2(2,1)*Rw_c(3,1))*W2(1,2) + (f*Rw_c(1,3) + u0*Rw_c(3,3) - po2(2,1)*Rw_c(3,3))*W2(3,2);
%                 end
%                 if i == 12
%                     MatNP(2*i-10,3) = f*Rw_c(1,3) + u0*Rw_c(3,3) - po2(3,1)*Rw_c(3,3);
%                     MatNP(2*i-10,4) = f;
%                     MatNP(2*i-10,6) = u0-po2(3,1);
%                     MatNP(2*i-10,7) = (f*Rw_c(1,1) + u0*Rw_c(3,1) - po2(3,1)*Rw_c(3,1))*W2(1,3) + (f*Rw_c(1,2) + u0*Rw_c(3,2) - po2(3,1)*Rw_c(3,2))*W2(2,3);
%                 end
%             end
%         end
%         
%         MatNP(16,4) = f;
%         MatNP(16,6) = u0-po1(1,1);
%         MatNP(16,7) = (f*Rw_c(1,1) + u0*Rw_c(3,1) - po1(1,1)*Rw_c(3,1))*W1(1,1)+(f*Rw_c(1,2) + u0*Rw_c(3,2) - po1(1,1)*Rw_c(3,2))*W1(2,1) + (f*Rw_c(1,3) + u0*Rw_c(3,3) - po1(1,1)*Rw_c(3,3))*W1(3,1);
%         MatNP(17,5) = f;
%         MatNP(17,6) = v0-po1(1,2);
%         MatNP(17,7) = (f*Rw_c(2,1) + v0*Rw_c(3,1) - po1(1,2)*Rw_c(3,1))*W1(1,1)+(f*Rw_c(2,2) + v0*Rw_c(3,2) - po1(1,2)*Rw_c(3,2))*W1(2,1) + (f*Rw_c(2,3) + v0*Rw_c(3,3) - po1(1,2)*Rw_c(3,3))*W1(3,1);
%      
        
        
%          for i = 1:4
%             if i > 1
%                 if i == 2
%                     MatNP(2*i-2,1) = f*Rw_c(2,1) + v0*Rw_c(3,1) - po2(1,2)*Rw_c(3,1);
%                     MatNP(2*i-2,5) = f;
%                     MatNP(2*i-2,6) = v0-po2(1,2);
%                     MatNP(2*i-2,7) = (f*Rw_c(2,2) + v0*Rw_c(3,2) - po2(1,2)*Rw_c(3,2))*W2(2,1) + (f*Rw_c(2,3) + v0*Rw_c(3,3) - po2(1,2)*Rw_c(3,3))*W2(3,1);
%                 end
%                 if i == 3
%                     MatNP(2*i-2,2) = f*Rw_c(2,2) + v0*Rw_c(3,2) - po2(2,2)*Rw_c(3,2);
%                     MatNP(2*i-2,5) = f;
%                     MatNP(2*i-2,6) = v0-po2(2,2);
%                     MatNP(2*i-2,7) = (f*Rw_c(2,1) + v0*Rw_c(3,1) - po2(2,2)*Rw_c(3,1))*W2(1,2) + (f*Rw_c(2,3) + v0*Rw_c(3,3) - po2(2,2)*Rw_c(3,3))*W2(3,2);
%                 end
%                 if i == 4
%                     MatNP(2*i-2,3) = f*Rw_c(2,3) + v0*Rw_c(3,3) - po2(3,2)*Rw_c(3,3);
%                     MatNP(2*i-2,5) = f;
%                     MatNP(2*i-2,6) = v0-po2(3,2);
%                     MatNP(2*i-2,7) = (f*Rw_c(1,1) + v0*Rw_c(3,1) - po2(3,2)*Rw_c(3,1))*W2(1,3) + (f*Rw_c(2,2) + v0*Rw_c(3,2) - po2(3,2)*Rw_c(3,2))*W2(2,3);
%                 end
%             end
%             if i < 4
%                 if i == 1
%                     MatNP(2*i-10,1) = f*Rw_c(1,1) + u0*Rw_c(3,1) - po2(1,1)*Rw_c(3,1);
%                     MatNP(2*i-10,4) = f;
%                     MatNP(2*i-10,6) = u0-po2(1,1);
%                     MatNP(2*i-10,7) = (f*Rw_c(1,2) + u0*Rw_c(3,2) - po2(1,1)*Rw_c(3,2))*W2(2,1) + (f*Rw_c(1,3) + u0*Rw_c(3,3) - po2(1,1)*Rw_c(3,3))*W2(3,1);
%                 end
%                 if i == 2
%                     MatNP(2*i-10,2) = f*Rw_c(1,2) + u0*Rw_c(3,2) - po2(2,1)*Rw_c(3,2);
%                     MatNP(2*i-10,4) = f;
%                     MatNP(2*i-10,6) = u0-po2(2,1);
%                     MatNP(2*i-10,7) = (f*Rw_c(1,1) + u0*Rw_c(3,1) - po2(2,1)*Rw_c(3,1))*W2(1,2) + (f*Rw_c(1,3) + u0*Rw_c(3,3) - po2(2,1)*Rw_c(3,3))*W2(3,2);
%                 end
%                 if i == 3
%                     MatNP(2*i-10,3) = f*Rw_c(1,3) + u0*Rw_c(3,3) - po2(3,1)*Rw_c(3,3);
%                     MatNP(2*i-10,4) = f;
%                     MatNP(2*i-10,6) = u0-po2(3,1);
%                     MatNP(2*i-10,7) = (f*Rw_c(1,1) + u0*Rw_c(3,1) - po2(3,1)*Rw_c(3,1))*W2(1,3) + (f*Rw_c(1,2) + u0*Rw_c(3,2) - po2(3,1)*Rw_c(3,2))*W2(2,3);
%                 end
%             end
%         end
%         
%         MatNP(16,4) = f;
%         MatNP(16,6) = u0-po1(1,1);
%         MatNP(16,7) = (f*Rw_c(1,1) + u0*Rw_c(3,1) - po1(1,1)*Rw_c(3,1))*W1(1,1)+(f*Rw_c(1,2) + u0*Rw_c(3,2) - po1(1,1)*Rw_c(3,2))*W1(2,1) + (f*Rw_c(1,3) + u0*Rw_c(3,3) - po1(1,1)*Rw_c(3,3))*W1(3,1);
%         MatNP(17,5) = f;
%         MatNP(17,6) = v0-po1(1,2);
%         MatNP(17,7) = (f*Rw_c(2,1) + v0*Rw_c(3,1) - po1(1,2)*Rw_c(3,1))*W1(1,1)+(f*Rw_c(2,2) + v0*Rw_c(3,2) - po1(1,2)*Rw_c(3,2))*W1(2,1) + (f*Rw_c(2,3) + v0*Rw_c(3,3) - po1(1,2)*Rw_c(3,3))*W1(3,1);
%      
            
        
%         
%         %solve the linear system MatN * [costheta, sintheta, tx, ty, tz, 1]' = 0  using SVD,
%         [UMatNP, SMatNP, VMatNP] = svd(MatNP);
%         vecNP = VMatNP(:,7);% the last column of Vmat;
%         vecNP = vecNP/vecNP(7); %the condition that the last element of vec should be 1.      
%         pos2    = vecNP(4:6);            
%                
%         Rt=factor*CC*sr;
        
        
        
%         curErr=sr.'*GG*sr;
%         if curErr<minErr
%             optR=Rr;
%             optT=Rt;
%             minErr=curErr;
%         end        
        R_wc(:,:,count)=Rr;
        t_wc(:,count)=Rt;
        
        
        count=count+1;
    end
    
end
% R_wc=optR;
% t_wc=optT;

end

