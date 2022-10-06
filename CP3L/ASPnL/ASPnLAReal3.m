function [R_wc,t_wc, aPointsCh, Oaerrs] = ASPnLAReal3(p1,p2,W1,W2,C_truth)
% ,minErr 
% P1N_w = [W1 W2];
% P2N_w = [W2 W2(:,2:3) W2(:,1)];
% Vw=P2N_w-P1N_w;
% Vw = xnorm(Vw);

% ,aFp,errp


optR = [1 0 0; 0 1 0; 0 0 1];
optT = [0;0;0];
Oaerrs = 1;
aPointsCh = [0,0,0];
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
[~, LineID] = max(lineLenVec);
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

d=W2-W1;
d = xnorm(d);

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
    
    
    count = 1;
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
%                   
%         Rt=factor*CC*sr; 
%         errR1 = cal_rotation_err(Rr,R_truth);
%         Rr = Roptimzation(p1(1:2,:), p2(1:2,:), IA, Rr, d, 30);
%         errR2 = cal_rotation_err(Rr,R_truth);
%         
               
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%        
        %According to the fact that n_i^C should be orthogonal to Pi^c and Vi^c, we 
        %have: scaleproduct(Vi^c, ni^c) = 0  and scaleproduct(Pi^c, ni^c) = 0.
        %where Vi^c = Rwc * Vi^w,  Pi^c = Rwc *(Pi^w - pos_cw) = Rwc * Pi^w - pos;
        %Using the above two constraints to construct linear equation system Mat about 
        %For type0,there will be four points which only have one unknown
        %axis, choose z as unknown axises, then the linear equation system Mat abou
        %[x, y, z, tx, ty, tz, 1]. 
        
        cH = C_truth(2);
        [Rt,aPoints] = CalculateTbyR(p1, p2, W1, W2, cH, Rr);
        
        

        cT = -inv(Rr)*Rt;
        if cT(1) < C_truth(1)-2 || cT(2) < C_truth(2)-2 || cT(3) < C_truth(3)-2 || cT(1) > C_truth(1)+2 || cT(2) > C_truth(2)+2 || cT(3) > C_truth(3)+2 
            continue;
        end

        
        pn1 = [p1 p2];
        pn2 = [p2 p2(:,2:3) p2(:,1)];
        tempW2 = W2;
        tempW2(1,1) = aPoints(1);
        tempW2(2,2) = aPoints(2);
        tempW2(3,3) = aPoints(3);
        Pn1_w1 = [W1 tempW2];
        Pn2_w1 = [tempW2 tempW2(:,2:3) tempW2(:,1)];
        
        
       
%         RrA = Rr;
        
        
%         for i = 1:20
%             [EEA,GGA,CCA] = refineEquation(p1,pn1,pn2,Pn1_w1,Pn2_w1,RrA,cH);
%             s = [Cayley(RrA);aPoints;Rt];
%             [solutionA, errA] = RefineGaussNewtonForApoints(s,EEA,GGA);
% 
%             s1=solutionA(1);s2=solutionA(2);s3=solutionA(3);
%             s4 =solutionA(4);s5=solutionA(5); s6=solutionA(6); 
%             s7=solutionA(7); s8=solutionA(8); s9=solutionA(9);
%             s10=solutionA(10); s11=solutionA(11); s12=solutionA(12);
%             t1=solutionA(13); t2=solutionA(14); t3=solutionA(15);
%     %         sr=[1,s1,s2,s3,s1^2,s1*s2,s1*s3,s2^2,s2*s3,s3^2].';
%             sr= [1,s1,s2,s3,s7,s8,s9,s10,s11,s12,...
%                 s4,s4*s2,s4*s3,s4*s7,s4*s8,s4*s9,s4*s10,s4*s12,...
%                 s5,s5*s1,s5*s3,s5*s7,s5*s8,s5*s10,s5*s11,s5*s12,...
%                 s6,s6*s1,s6*s2,s6*s7,s6*s9,s6*s10,s6*s11,s6*s12,...
%                 s1^2, s1*s2, s1*s3, s2^2, s2*s3, s3^2,...
%                 t1,t2,t3,t1*s3,t1*s7,t1*s8,t1*s10,t1*s12,t2*s7,t2*s10,t2*s12,t3*s1,t3*s7,t3*s10,t3*s11,t3*s12].';
%             factor=1/(1+s1^2+s2^2+s3^2); 
%             RrA=factor*[1+s1^2-s2^2-s3^2,2*s1*s2-2*s3,2*s2+2*s1*s3;
%                        2*s3+2*s1*s2,1-s1^2+s2^2-s3^2,2*s2*s3-2*s1;
%                        2*s1*s3-2*s2,2*s1+2*s2*s3,1-s1^2-s2^2+s3^2];
%                    
%         end

        RrA = Rr;
        for i = 1:1
        
            [EEA,GGA,CCA] = CopyrefineEquation(p1,pn1,pn2,Pn1_w1,Pn2_w1,RrA,cH);
%             s = [Cayley(RrA);WT2(1,1);WT2(2,2);WT2(3,3)];
            s = [Cayley(RrA);aPoints];
            [solutionA, errA] = CopyRefineGaussNewtonForApoints(s,EEA,GGA);


            s1=solutionA(1);s2=solutionA(2);s3=solutionA(3);
            s4 =solutionA(4);s5=solutionA(5); s6=solutionA(6); 
            s7=solutionA(7); s8=solutionA(8); s9=solutionA(9);
            s10=solutionA(10); s11=solutionA(11); s12=solutionA(12);
%             t1=solutionA(13); t2=solutionA(14); t3=solutionA(15);
    %         sr=[1,s1,s2,s3,s1^2,s1*s2,s1*s3,s2^2,s2*s3,s3^2].';
            sr= [1,s1,s2,s3,s7,s8,s9,s10,s11,s12,...
                s4,s4*s2,s4*s3,s4*s7,s4*s8,s4*s9,s4*s10,s4*s12,...
                s5,s5*s1,s5*s3,s5*s7,s5*s8,s5*s10,s5*s11,s5*s12,...
                s6,s6*s1,s6*s2,s6*s7,s6*s9,s6*s10,s6*s11,s6*s12,...
                s1^2, s1*s2, s1*s3, s2^2, s2*s3, s3^2].';
            factor=1/(1+s1^2+s2^2+s3^2); 
            RrA=factor*[1+s1^2-s2^2-s3^2,2*s1*s2-2*s3,2*s2+2*s1*s3;
                       2*s3+2*s1*s2,1-s1^2+s2^2-s3^2,2*s2*s3-2*s1;
                       2*s1*s3-2*s2,2*s1+2*s2*s3,1-s1^2-s2^2+s3^2];
            RAt = factor*CCA*sr;        
            tempW2(1,1) = solutionA(4);
            tempW2(2,2) = solutionA(5);
            tempW2(3,3) = solutionA(6);
            Pn1_w1 = [W1 tempW2];
            Pn2_w1 = [tempW2 tempW2(:,2:3) tempW2(:,1)];
                   
     
        end
 
        aPointsCh = solutionA(4:6);
%         
        

%        
%         RrA = Rr;
% %         curErr0 = cal_rotation_err(Rr,R_truth);
% %         aPointsCh = aPoints;
%         for i = 1:3
%             
% %             curErr1 = cal_rotation_err(RrA,R_truth);
%         
%             [EEcH,GGcH,CCcH] = refineEquationForCH(pn1, pn2,Pn1_w1,Pn2_w1,cH);
% 
%             [solutioncH, errcH] = RefineGaussNewtonCH(Cayley(RrA),EEcH,GGcH,CCcH,pn1, pn2,Pn1_w1,Pn2_w1);
% 
%             s1=solutioncH(1);
%             s2=solutioncH(2);
%             s3=solutioncH(3);
%             sr=[1,s1,s2,s3,s1^2,s1*s2,s1*s3,s2^2,s2*s3,s3^2].';
%             factor=1/(1+s1^2+s2^2+s3^2); 
%             RrA=factor*[1+s1^2-s2^2-s3^2,2*s1*s2-2*s3,2*s2+2*s1*s3;
%                        2*s3+2*s1*s2,1-s1^2+s2^2-s3^2,2*s2*s3-2*s1;
%                        2*s1*s3-2*s2,2*s1+2*s2*s3,1-s1^2-s2^2+s3^2];
%                    
%             RAt = factor*CCcH*sr;    
% 
% 
% %             curErr2 = cal_rotation_err(RrA,R_truth);
% 
%             
%             
%             [~,aPointsCh] = CalculateTbyR(p1, p2, W1, W2, cH, RrA);
%    
% %             [EEA,GGA,CCA] = refineEquationForPoints(pn1,pn2,Pn1_w1,Pn2_w1,solutioncH);
% %             [aPointsCh,~] = RefineForApoints(aPointsCh,EEA,GGA);
%             
%             tempW2 = W2;
%             tempW2(1,1) = aPointsCh(1);
%             tempW2(2,2) = aPointsCh(2);
%             tempW2(3,3) = aPointsCh(3);
%             Pn1_w1 = [W1 tempW2];
%             Pn2_w1 = [tempW2 tempW2(:,2:3) tempW2(:,1)];        
%         end
%         
        
        
%         [RrA, RAt, curErr] = SRPnL3IPB(pn1(1:2,:), pn2(1:2,:), Pn1_w1, Pn2_w1);
%         [RrA, RAt, curErr] = SRPnL(pn1(1:2,:), pn2(1:2,:), Pn1_w1, Pn2_w1);

%         
%         
%        [Rr, Rt, curErr] =  GN(pn1, pn2, Pn1_w1, Pn2_w1, Rr);
%         
%         Xw = [W1(:,1) Pn2_w1(:,1:3)]';
%         U = [po1(1,:);po2];
%         
%         [Rerr,Urep] = reprojection_error_usingRT(Xw,U,RrA,RAt,IA);
%         
%         [Rerr1,Urep1] = reprojection_error_usingRT(Xw,U,R_truth,T_truth,IA);
%         
% %         [Rerr,Urep] = reprojection_error_usingRT(Xw,U,RrcH,Rt,IA);
%         
%         Gw = [W1(:,1)'; WT2];
%         Gp = [po1(1,:);po2];
%         
%         [GRerr,Grep] = reprojection_error_usingRT(Gw,Gp,R_truth,T_truth,IA);
%         
% %         [Rerr,Urep] = reprojection_error_usingRT(Xw,U,Rr,Rt,IA);
        curErr = getErr(pn1,pn2,Pn1_w1,Pn2_w1,RrA,RAt);
        if abs(curErr)<minErr
            optR=RrA;
            optT=RAt;
            minErr=abs(curErr);
            Oaerrs = abs(curErr);
        end         
        count=count+1;
    end
    
end

R_wc=optR;
t_wc=optT;

end