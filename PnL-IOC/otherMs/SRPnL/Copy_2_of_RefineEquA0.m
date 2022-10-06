function [EE,GG,CC] = Copy_2_of_RefineEquA0(p1, p2, P1_w, W2,solution)


%     pn1 = [p(:,1) p(:,1) p(:,1) p(:,1) p(:,1) p(:,1) p(:,1) p(:,2) p(:,2) p(:,2) p(:,2) p(:,2) p(:,2) p(:,3) p(:,3) p(:,3) p(:,3) p(:,3) p(:,4) p(:,4) p(:,4) p(:,4) p(:,5) p(:,5) p(:,5) p(:,6) p(:,6) p(:,7)];
%     pn2 = [p(:,2) p(:,3) p(:,4) p(:,5) p(:,6) p(:,7) p(:,8) p(:,3) p(:,4) p(:,5) p(:,6) p(:,7) p(:,8) p(:,4) p(:,5) p(:,6) p(:,7) p(:,8) p(:,5) p(:,6) p(:,7) p(:,8) p(:,6) p(:,7) p(:,8) p(:,7) p(:,8) p(:,8)];
    pn1 = [p1 p2(:,5:8)];
    pn2 = [p2 p2(:,6:8) p2(:,5)];
    nxs = pn1;
    nxe = pn2;
    n_nc = xcross_mat(nxs,nxe);
    ne_c = xnorm(n_nc);
%     Pn1_w = [W(:,1) W(:,1) W(:,1) W(:,1) W(:,1) W(:,1) W(:,1) W(:,2) W(:,2) W(:,2) W(:,2) W(:,2) W(:,2) W(:,3) W(:,3) W(:,3) W(:,3) W(:,3) W(:,4) W(:,4) W(:,4) W(:,4) W(:,5) W(:,5) W(:,5) W(:,6) W(:,6) W(:,7)];
%     Pn2_w = [W(:,2) W(:,3) W(:,4) W(:,5) W(:,6) W(:,7) W(:,8) W(:,3) W(:,4) W(:,5) W(:,6) W(:,7) W(:,8) W(:,4) W(:,5) W(:,6) W(:,7) W(:,8) W(:,5) W(:,6) W(:,7) W(:,8) W(:,6) W(:,7) W(:,8) W(:,7) W(:,8) W(:,8)];
%  
    Pn1_w = [P1_w W2(:,5:8)];
    Pn2_w = [W2 W2(:,6:8) W2(:,5)];
    neLine = length(pn1);
%     
    s1=solution(1); s2=solution(2); s3=solution(3); 
    s4=s1^2; s5=s1*s2; s6=s1*s3;
    s7=s2^2; s8=s2*s3; s9=s3^2;
    s10 = solution(4); s11=solution(5); s12=solution(6); s13=solution(7);  
    w=[1,s1,s2,s3,s4,s5,s6,s7,s8,s9,...
       s10,s10*s1,s10*s2,s10*s4,s10*s6,s10*s7,s10*s8,s10*s9,...
       s11,s11*s1,s11*s2,s11*s4,s11*s6,s11*s7,s11*s8,s11*s9,...
       s12,s12*s1,s12*s2,s12*s4,s12*s6,s12*s7,s12*s8,s12*s9,...
       s13,s13*s1,s13*s2,s13*s4,s13*s6,s13*s7,s13*s8,s13*s9,...
       s1^2,s1*s2,s1*s3,s2^2,s2*s3,s3^2]';


    Q=zeros(neLine,48);
%     N=zeros(neLine,3);

    for i = 1:4
        nx=ne_c(1,i); ny=ne_c(2,i); nz=ne_c(3,i);
        Px=Pn1_w(1,i)-Pn2_w(1,i); Py=Pn1_w(2,i)-Pn2_w(2,i); Pz=Pn1_w(3,i)-Pn2_w(3,i);
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
    
    for i = 5:8
        nx=ne_c(1,i); ny=ne_c(2,i); nz=ne_c(3,i);
        if i == 5
            Px=Pn1_w(1,i)-Pn2_w(1,i); Py=Pn1_w(2,i)-Pn2_w(2,i);Pz=Pn1_w(3,i);
            Q(i,1:18)=[Px.*nx+Py.*ny+Pz.*nz,...    %1
              2*Py.*nz-2*Pz.*ny,...       %s1
              2*Pz.*nx-2*Px.*nz,...       %s2
              2*Px.*ny-2*Py.*nx,...       %s3
              Px.*nx-Py.*ny-Pz.*nz,...     %s1^2
              2.*Px.*ny+2*Py.*nx,...      %s1*s2
              2.*Px.*nz+2*Pz.*nx,...       %s1*s3
              Py.*ny-Px.*nx-Pz.*nz,...     %s2^2
              2*Py.*nz+2*Pz.*ny,...       %s2*s3
              Pz.*nz-Px.*nx-Py.*ny,...     %s3^2
              -nz,2*ny,-2*nx,nz,-2*nx,nz,-2*ny,-nz
             ];
        elseif i == 6
            Px=Pn1_w(1,i)-Pn2_w(1,i); Py=Pn1_w(2,i)-Pn2_w(2,i);Pz=Pn1_w(3,i);
            Q(i,1:26)=[Px.*nx+Py.*ny+Pz.*nz,...    %1
              2*Py.*nz-2*Pz.*ny,...       %s1
              2*Pz.*nx-2*Px.*nz,...       %s2
              2*Px.*ny-2*Py.*nx,...       %s3
              Px.*nx-Py.*ny-Pz.*nz,...     %s1^2
              2.*Px.*ny+2*Py.*nx,...      %s1*s2
              2.*Px.*nz+2*Pz.*nx,...       %s1*s3
              Py.*ny-Px.*nx-Pz.*nz,...     %s2^2
              2*Py.*nz+2*Pz.*ny,...       %s2*s3
              Pz.*nz-Px.*nx-Py.*ny,...     %s3^2
              0,0,0,0,0,0,0,0,-nz,2*ny,-2*nx,nz,-2*nx,nz,-2*ny,-nz
             ];
        elseif i == 7
            Px=Pn1_w(1,i)-Pn2_w(1,i); Py=Pn1_w(2,i)-Pn2_w(2,i);Pz=Pn1_w(3,i);
            Q(i,1:34)=[Px.*nx+Py.*ny+Pz.*nz,...    %1
              2*Py.*nz-2*Pz.*ny,...       %s1
              2*Pz.*nx-2*Px.*nz,...       %s2
              2*Px.*ny-2*Py.*nx,...       %s3
              Px.*nx-Py.*ny-Pz.*nz,...     %s1^2
              2.*Px.*ny+2*Py.*nx,...      %s1*s2
              2.*Px.*nz+2*Pz.*nx,...       %s1*s3
              Py.*ny-Px.*nx-Pz.*nz,...     %s2^2
              2*Py.*nz+2*Pz.*ny,...       %s2*s3
              Pz.*nz-Px.*nx-Py.*ny,...     %s3^2
              0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,-nz,2*ny,-2*nx,nz,-2*nx,nz,-2*ny,-nz
             ];
        else
            Px=Pn1_w(1,i)-Pn2_w(1,i); Py=Pn1_w(2,i)-Pn2_w(2,i);Pz=Pn1_w(3,i);
            Q(i,1:42)=[Px.*nx+Py.*ny+Pz.*nz,...    %1
              2*Py.*nz-2*Pz.*ny,...       %s1
              2*Pz.*nx-2*Px.*nz,...       %s2
              2*Px.*ny-2*Py.*nx,...       %s3
              Px.*nx-Py.*ny-Pz.*nz,...     %s1^2
              2.*Px.*ny+2*Py.*nx,...      %s1*s2
              2.*Px.*nz+2*Pz.*nx,...       %s1*s3
              Py.*ny-Px.*nx-Pz.*nz,...     %s2^2
              2*Py.*nz+2*Pz.*ny,...       %s2*s3
              Pz.*nz-Px.*nx-Py.*ny,...     %s3^2
              0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,-nz,2*ny,-2*nx,nz,-2*nx,nz,-2*ny,-nz
             ];
        end
    end   
    for i = 9:12
        nx=ne_c(1,i); ny=ne_c(2,i); nz=ne_c(3,i);
        if i == 9
            Px=Pn1_w(1,i)-Pn2_w(1,i); Py=Pn1_w(2,i)-Pn2_w(2,i);
            Q(i,1:26)=[Px.*nx+Py.*ny,...
              2*Py.*nz,...
              -2*Px.*nz,...
              2*Px.*ny-2*Py.*nx,...
              Px.*nx-Py.*ny,...
              2.*Px.*ny+2*Py.*nx,...
              2.*Px.*nz,...
              Py.*ny-Px.*nx,...
              2*Py.*nz,...
              -Px.*nx-Py.*ny,...
              nz,-2*ny,2*nx,-nz,2*nx,-nz,2*ny,nz,...
              -nz,2*ny,-2*nx,nz,-2*nx,nz,-2*ny,-nz
             ];
        elseif i == 10
            Px=Pn1_w(1,i)-Pn2_w(1,i); Py=Pn1_w(2,i)-Pn2_w(2,i);
            Q(i,1:34)=[Px.*nx+Py.*ny,...
              2*Py.*nz,...
              -2*Px.*nz,...
              2*Px.*ny-2*Py.*nx,...
              Px.*nx-Py.*ny,...
              2.*Px.*ny+2*Py.*nx,...
              2.*Px.*nz,...
              Py.*ny-Px.*nx,...
              2*Py.*nz,...
              -Px.*nx-Py.*ny,...
              0,0,0,0,0,0,0,0,nz,-2*ny,2*nx,-nz,2*nx,-nz,2*ny,nz,-nz,2*ny,-2*nx,nz,-2*nx,nz,-2*ny,-nz
             ];
        elseif i == 11
            Px=Pn1_w(1,i)-Pn2_w(1,i); Py=Pn1_w(2,i)-Pn2_w(2,i);
            Q(i,1:42)=[Px.*nx+Py.*ny,...
              2*Py.*nz,...
              -2*Px.*nz,...
              2*Px.*ny-2*Py.*nx,...
              Px.*nx-Py.*ny,...
              2.*Px.*ny+2*Py.*nx,...
              2.*Px.*nz,...
              Py.*ny-Px.*nx,...
              2*Py.*nz,...
              -Px.*nx-Py.*ny,...
              0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,nz,-2*ny,2*nx,-nz,2*nx,-nz,2*ny,nz,-nz,2*ny,-2*nx,nz,-2*nx,nz,-2*ny,-nz
             ];
        else
            Px=Pn1_w(1,i)-Pn2_w(1,i); Py=Pn1_w(2,i)-Pn2_w(2,i);
            Q(i,1:42)=[Px.*nx+Py.*ny,...
              2*Py.*nz,...
              -2*Px.*nz,...
              2*Px.*ny-2*Py.*nx,...
              Px.*nx-Py.*ny,...
              2.*Px.*ny+2*Py.*nx,...
              2.*Px.*nz,...
              Py.*ny-Px.*nx,...
              2*Py.*nz,...
              -Px.*nx-Py.*ny,...
              -nz,2*ny,-2*nx,nz,-2*nx,nz,-2*ny,-nz,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,nz,-2*ny,2*nx,-nz,2*nx,-nz,2*ny,nz
             ];
        end
    end
   
%     N(1:neLine,:)=-ne_c.';
%     N(neLine+1:2*neLine,:)=-ne_c.';
% 
% 
% 
%     if abs(det(N.'*N)) < 2e-10
%         CC=(pinv(N.'*N)*N.')*Q;
%     else
%         CC=((N.'*N)\N.')*Q;
%     end
% 
%     EE=Q-N*CC;


    EE=Q;
    EE(neLine+1,:) = [0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,-1,0,0,0,0,0];
    EE(neLine+2,:) = [0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,-1,0,0,0,0];
    EE(neLine+3,:) = [0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,-1,0,0,0];
    EE(neLine+4,:) = [0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,-1,0,0];
    EE(neLine+5,:) = [0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,-1,0];
    EE(neLine+6,:) = [0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,-1];
  

    GG=EE.'*EE;
    
end
