function [EE,GG,CC] = RefineEquA3Original(p, W,R,cH)


    pn1 = [p(:,1) p(:,1) p(:,1) p(:,2) p(:,2) p(:,3)];
    pn2 = [p(:,2) p(:,3) p(:,4) p(:,3) p(:,4) p(:,4)];
    n_nc = xcross_mat(pn1,pn2);
    ne_c = xnorm(n_nc);
    Pn1_w = [W(:,1) W(:,1) W(:,1) W(:,2) W(:,2) W(:,3)];
    Pn2_w = [W(:,2) W(:,3) W(:,4) W(:,3) W(:,4) W(:,4)];
 
    neLine = length(pn1);


    Q=zeros(2*neLine,40);
    N=zeros(2*neLine,3);
    for i = 1:3
        nx=ne_c(1,i); ny=ne_c(2,i); nz=ne_c(3,i);
        Px=Pn1_w(1,i); Py=Pn1_w(2,i); Pz=Pn1_w(3,i);
        Q(i,1:10)=[Px.*nx+Py.*ny+Pz.*nz,...    %1
              2*Py.*nz-2*Pz.*ny,...       %s1
              2*Pz.*nx-2*Px.*nz,...       %s2
              2*Px.*ny-2*Py.*nx,...       %s3
              Px.*nx-Py.*ny-Pz.*nz,...     %s1^2
              2.*Px.*ny+2*Py.*nx,...      %s1*s2
              2.*Px.*nz+2*Pz.*nx,...       %s1*s3
              Py.*ny-Px.*nx-Pz.*nz,...     %s2^2
              2*Py.*nz+2*Pz.*ny,...       %s2*s3
              Pz.*nz-Px.*nx-Py.*ny,...     %s3^2
             ];
    end
    
    for i = 4:5
        nx=ne_c(1,i); ny=ne_c(2,i); nz=ne_c(3,i);
        Py=Pn1_w(2,i);Pz=Pn1_w(3,i);
        Q(i,1:18)=[Py.*ny+Pz.*nz,...
              2*Py.*nz-2*Pz.*ny,...
              2*Pz.*nx,...
              -2*Py.*nx,...
              -Py.*ny-Pz.*nz,...
              2*Py.*nx,...
              2*Pz.*nx,...
              Py.*ny-Pz.*nz,...
              2*Py.*nz+2*Pz.*ny,...
              Pz.*nz-Py.*ny,...
              nx,-2*nz,2*ny,nx,2*ny,2*nz,-nx,-nx];
    end
    for i = 6:6
        nx=ne_c(1,i); ny=ne_c(2,i); nz=ne_c(3,i);
        Px=Pn1_w(1,i); Pz=Pn1_w(3,i);
        Q(i,1:26)=[Px.*nx+Pz.*nz,...
              -2*Pz.*ny,...
              2*Pz.*nx-2*Px.*nz,...
              2*Px.*ny,...
              Px.*nx-Pz.*nz,...
              2.*Px.*ny,...
              2.*Px.*nz+2*Pz.*nx,...
              -Px.*nx-Pz.*nz,...
              2*Pz.*ny,...
              Pz.*nz-Px.*nx,...
              0,0,0,0,0,0,0,0,ny,2*nz,-2*nx,-ny,2*nx,ny,2*nz,-ny]; 
    end

    for j = 1:6
        nx=ne_c(1,j); ny=ne_c(2,j); nz=ne_c(3,j);
        if j==1
             Py=Pn2_w(2,j); Pz=Pn2_w(3,j);
             Q(j+6,1:18)=[Py.*ny+Pz.*nz,...
                      2*Py.*nz-2*Pz.*ny,...
                      2*Pz.*nx,...
                      -2*Py.*nx,...
                      -Py.*ny-Pz.*nz,...
                      2*Py.*nx,...
                      2*Pz.*nx,...
                      Py.*ny-Pz.*nz,...
                      2*Py.*nz+2*Pz.*ny,...
                      Pz.*nz-Py.*ny,...
                      nx,-2*nz,2*ny,nx,2*ny,2*nz,-nx,-nx];
        end
        if j==2 || j==4
             Px=Pn2_w(1,j); Pz=Pn2_w(3,j);
             Q(j+6,1:26)=[Px.*nx+Pz.*nz,...
                  -2*Pz.*ny,...
                  2*Pz.*nx-2*Px.*nz,...
                  2*Px.*ny,...
                  Px.*nx-Pz.*nz,...
                  2.*Px.*ny,...
                  2.*Px.*nz+2*Pz.*nx,...
                  -Px.*nx-Pz.*nz,...
                  2*Pz.*ny,...
                  Pz.*nz-Px.*nx,...
                  0,0,0,0,0,0,0,0,ny,2*nz,-2*nx,-ny,2*nx,ny,2*nz,-ny];
        end
        if j==3 || j==5 || j==6
             Px=Pn2_w(1,j); Py=Pn2_w(2,j);
             Q(j+6,1:34)=[Px.*nx+Py.*ny,...
              2*Py.*nz,...
              -2*Px.*nz,...
              2*Px.*ny-2*Py.*nx,...
              Px.*nx-Py.*ny,...
              2.*Px.*ny+2*Py.*nx,...
              2.*Px.*nz,...
              Py.*ny-Px.*nx,...
              2*Py.*nz,...
              -Px.*nx-Py.*ny,...
              0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,nz,-2*ny,2*nx,-nz,2*nx,-nz,2*ny,nz];
        end
    end
    
    N(1:neLine,:)=-ne_c.';
    N(neLine+1:2*neLine,:)=-ne_c.';

    if abs(det(N.'*N)) < 2e-10
        CC=(pinv(N.'*N)*N.')*Q;
    else
        CC=((N.'*N)\N.')*Q;
    end

    EE=Q-N*CC;


    EE(2*neLine+1,:) = [0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,-1,0,0,0,0,0];
    EE(2*neLine+2,:) = [0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,-1,0,0,0,0];
    EE(2*neLine+3,:) = [0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,-1,0,0,0];
    EE(2*neLine+4,:) = [0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,-1,0,0];
    EE(2*neLine+5,:) = [0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,-1,0];
    EE(2*neLine+6,:) = [0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,-1];
  
    
    
    invR = inv(R);
    
    tempL = zeros(1,40);
    tempL(1) = cH;tempL(5) = cH;tempL(8) = cH;tempL(10) = cH;
    
    EE(2*neLine+7,:) = invR(2,:)*CC + tempL;
    
    numL = 2*neLine+7;
    
    
%     solution = Cayley(R);
%     
%     s1=solution(1); s2=solution(2); s3=solution(3); 
%     s4=s1^2; s5=s1*s2; s6=s1*s3;
%     s7=s2^2; s8=s2*s3; s9=s3^2;
%     s10 = W(1,2); s11=W(2,3); s12=W(3,4);  
%     w=[1,s1,s2,s3,s4,s5,s6,s7,s8,s9,...
%        s10,s10*s2,s10*s3,s10*s4,s10*s5,s10*s6,s10*s7,s10*s9,...
%        s11,s11*s1,s11*s3,s11*s4,s11*s5,s11*s7,s11*s8,s11*s9,...
%        s12,s12*s1,s12*s2,s12*s4,s12*s6,s12*s7,s12*s8,s12*s9,...
%        s1^2,s1*s2,s1*s3,s2^2,s2*s3,s3^2]';
%     fac = 1/(1+s1^2+s2^2+s3^2); 
%     T = fac*CC*w;
    
    for i = 1:6
        nx=ne_c(1,i); ny=ne_c(2,i); nz=ne_c(3,i);
        if i == 1
            Px1=Pn1_w(1,i); Py1=Pn1_w(2,i); Pz1=Pn1_w(3,i);
            Py2=Pn2_w(2,i); Pz2=Pn2_w(3,i);
            EE(numL+i,1:18)=[(Px1).*nx+(Py1 - Py2).*ny+(Pz1 - Pz2).*nz,...    %1
                  2*(Py1 - Py2).*nz-2*(Pz1 - Pz2).*ny,...       %s1
                  2*(Pz1 - Pz2).*nx-2*(Px1).*nz,...       %s2
                  2*(Px1).*ny-2*(Py1 - Py2).*nx,...       %s3
                  (Px1).*nx-(Py1 - Py2).*ny-(Pz1 - Pz2).*nz,...     %s1^2
                  2.*(Px1).*ny+2*(Py1 - Py2).*nx,...      %s1*s2
                  2.*(Px1).*nz+2*(Pz1 - Pz2).*nx,...       %s1*s3
                  (Py1 - Py2).*ny-(Px1).*nx-(Pz1 - Pz2).*nz,...     %s2^2
                  2*(Py1 - Py2).*nz+2*(Pz1 - Pz2).*ny,...       %s2*s3
                  (Pz1 - Pz2).*nz-(Px1).*nx-(Py1 - Py2).*ny,...     %s3^2  
                  -nx,2*nz,-2*ny,-nx,-2*ny,-2*nz,nx,nx];
        elseif i == 2
            Px1=Pn1_w(1,i); Py1=Pn1_w(2,i); Pz1=Pn1_w(3,i);
            Px2=Pn2_w(1,i); Pz2=Pn2_w(3,i);
            EE(numL+i,1:26)=[(Px1 - Px2).*nx+(Py1).*ny+(Pz1 - Pz2).*nz,...    %1
                  2*(Py1).*nz-2*(Pz1 - Pz2).*ny,...       %s1
                  2*(Pz1 - Pz2).*nx-2*(Px1 - Px2).*nz,...       %s2
                  2*(Px1 - Px2).*ny-2*(Py1).*nx,...       %s3
                  (Px1 - Px2).*nx-(Py1).*ny-(Pz1 - Pz2).*nz,...     %s1^2
                  2.*(Px1 - Px2).*ny+2*(Py1).*nx,...      %s1*s2
                  2.*(Px1 - Px2).*nz+2*(Pz1 - Pz2).*nx,...       %s1*s3
                  (Py1).*ny-(Px1 - Px2).*nx-(Pz1 - Pz2).*nz,...     %s2^2
                  2*(Py1).*nz+2*(Pz1 - Pz2).*ny,...       %s2*s3
                  (Pz1 - Pz2).*nz-(Px1 - Px2).*nx-(Py1).*ny,...     %s3^2  
                  0,0,0,0,0,0,0,0,-ny,-2*nz,2*nx,ny,-2*nx,-ny,-2*nz,ny
                 ];
        elseif i == 3
            Px1=Pn1_w(1,i); Py1=Pn1_w(2,i); Pz1=Pn1_w(3,i);
            Px2=Pn2_w(1,i); Py2=Pn2_w(2,i);
            EE(numL+i,1:34)=[(Px1 - Px2).*nx+(Py1 - Py2).*ny+(Pz1).*nz,...    %1
                  2*(Py1 - Py2).*nz-2*(Pz1).*ny,...       %s1
                  2*(Pz1).*nx-2*(Px1 - Px2).*nz,...       %s2
                  2*(Px1 - Px2).*ny-2*(Py1 - Py2).*nx,...       %s3
                  (Px1 - Px2).*nx-(Py1 - Py2).*ny-(Pz1).*nz,...     %s1^2
                  2.*(Px1 - Px2).*ny+2*(Py1 - Py2).*nx,...      %s1*s2
                  2.*(Px1 - Px2).*nz+2*(Pz1).*nx,...       %s1*s3
                  (Py1 - Py2).*ny-(Px1 - Px2).*nx-(Pz1).*nz,...     %s2^2
                  2*(Py1 - Py2).*nz+2*(Pz1).*ny,...       %s2*s3
                  (Pz1).*nz-(Px1 - Px2).*nx-(Py1 - Py2).*ny,...     %s3^2  
                  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,-nz,2*ny,-2*nx,nz,-2*nx,nz,-2*ny,-nz
                 ];
        elseif i == 4
            Py1=Pn1_w(2,i); Pz1=Pn1_w(3,i);
            Px2=Pn2_w(1,i); Pz2=Pn2_w(3,i);
            EE(numL+i,1:26)=[(-Px2).*nx+(Py1).*ny+(Pz1 - Pz2).*nz,...    %1
                  2*(Py1).*nz-2*(Pz1 - Pz2).*ny,...       %s1
                  2*(Pz1 - Pz2).*nx-2*(-Px2).*nz,...       %s2
                  2*(-Px2).*ny-2*(Py1).*nx,...       %s3
                  (-Px2).*nx-(Py1).*ny-(Pz1 - Pz2).*nz,...     %s1^2
                  2.*(-Px2).*ny+2*(Py1).*nx,...      %s1*s2
                  2.*(-Px2).*nz+2*(Pz1 - Pz2).*nx,...       %s1*s3
                  (Py1).*ny-(-Px2).*nx-(Pz1 - Pz2).*nz,...     %s2^2
                  2*(Py1).*nz+2*(Pz1 - Pz2).*ny,...       %s2*s3
                  (Pz1 - Pz2).*nz-(-Px2).*nx-(Py1).*ny,...     %s3^2  
                  nx,-2*nz,2*ny,nx,2*ny,2*nz,-nx,-nx,-ny,-2*nz,2*nx,ny,-2*nx,-ny,-2*nz,ny
                 ];
        elseif i == 5
            Py1=Pn1_w(2,i); Pz1=Pn1_w(3,i);
            Px2=Pn2_w(1,i); Py2=Pn2_w(2,i);
            EE(numL+i,1:34)=[(-Px2).*nx+(Py1 - Py2).*ny+(Pz1).*nz,...    %1
                  2*(Py1 - Py2).*nz-2*(Pz1).*ny,...       %s1
                  2*(Pz1).*nx-2*(-Px2).*nz,...       %s2
                  2*(-Px2).*ny-2*(Py1 - Py2).*nx,...       %s3
                  (-Px2).*nx-(Py1 - Py2).*ny-(Pz1).*nz,...     %s1^2
                  2.*(-Px2).*ny+2*(Py1 - Py2).*nx,...      %s1*s2
                  2.*(-Px2).*nz+2*(Pz1).*nx,...       %s1*s3
                  (Py1 - Py2).*ny-(-Px2).*nx-(Pz1).*nz,...     %s2^2
                  2*(Py1 - Py2).*nz+2*(Pz1).*ny,...       %s2*s3
                  (Pz1).*nz-(-Px2).*nx-(Py1 - Py2).*ny,...     %s3^2  
                  nx,-2*nz,2*ny,nx,2*ny,2*nz,-nx,-nx,0,0,0,0,0,0,0,0,-nz,2*ny,-2*nx,nz,-2*nx,nz,-2*ny,-nz
                 ];
        elseif i == 6
            Px1=Pn1_w(1,i); Pz1=Pn1_w(3,i);
            Px2=Pn2_w(1,i); Py2=Pn2_w(2,i);
            EE(numL+i,1:34)=[(Px1 - Px2).*nx+(-Py2).*ny+(Pz1).*nz,...    %1
                  2*(-Py2).*nz-2*(Pz1).*ny,...       %s1
                  2*(Pz1).*nx-2*(Px1 - Px2).*nz,...       %s2
                  2*(Px1 - Px2).*ny-2*(-Py2).*nx,...       %s3
                  (Px1 - Px2).*nx-(-Py2).*ny-(Pz1).*nz,...     %s1^2
                  2.*(Px1 - Px2).*ny+2*(-Py2).*nx,...      %s1*s2
                  2.*(Px1 - Px2).*nz+2*(Pz1).*nx,...       %s1*s3
                  (-Py2).*ny-(Px1 - Px2).*nx-(Pz1).*nz,...     %s2^2
                  2*(-Py2).*nz+2*(Pz1).*ny,...       %s2*s3
                  (Pz1).*nz-(Px1 - Px2).*nx-(-Py2).*ny,...     %s3^2  
                  0,0,0,0,0,0,0,0,ny,2*nz,-2*nx,-ny,2*nx,ny,2*nz,-ny,-nz,2*ny,-2*nx,nz,-2*nx,nz,-2*ny,-nz
                 ];
        end
    end
    

    GG=EE.'*EE;
    
end
