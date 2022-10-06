function [EE,GG,CC] = refineEquationForTest(xs, xe,Pn1_w,Pn2_w,cH)
%REFINEEQUATION Summary of this function goes here
%   Detailed explanation goes here
    neLine = length(xs);
    n_c=xnorm(cross(xs,xe));

    
    nLe = 2*neLine;
    Q=zeros(nLe,10);
    N=zeros(nLe,3);
    nx=n_c(1,:)'; ny=n_c(2,:)'; nz=n_c(3,:)';
    Px=Pn1_w(1,:)'; Py=Pn1_w(2,:)'; Pz=Pn1_w(3,:)';
    Q(1:2:end,:)=[Px.*nx+Py.*ny+Pz.*nz,...
              2*Py.*nz-2*Pz.*ny,...
              2*Pz.*nx-2*Px.*nz,...
              2*Px.*ny-2*Py.*nx,...
              Px.*nx-Py.*ny-Pz.*nz,...
              2.*Px.*ny+2*Py.*nx,...
              2.*Px.*nz+2*Pz.*nx,...
              Py.*ny-Px.*nx-Pz.*nz,...
              2*Py.*nz+2*Pz.*ny,...
              Pz.*nz-Px.*nx-Py.*ny,...
              ];
    Px=Pn2_w(1,:)'; Py=Pn2_w(2,:)'; Pz=Pn2_w(3,:)';
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

    if abs(det(N.'*N)) < 2e-10
        CC=(pinv(N.'*N)*N.')*Q;
    else
        CC=((N.'*N)\N.')*Q;
    end

    EE=zeros(nLe+1,41);
    EE(1:nLe,1:10)=Q-N*CC;
    c = CC;
    
    
    EE(nLe+1,:) = [...
        c(2) + cH;
        2*c(3) + c(5); 
        c(8); 
        - 2*c(1) + c(11);
        - c(2) + 2*c(6) + c(14) + 2*cH;
        2*c(1) + 2*c(9) + c(17);
        - 2*c(4) + 2*c(12) + c(20);
        c(2) + c(23) + 2*cH;
        2*c(3) - 2*c(7) + c(26);
        - c(2) - 2*c(10) + c(29) + 2*cH;
        - c(5) + 2*c(15);
        c(8);
        - c(11) - 2*c(28);
        - c(14) + cH;
        c(23) + cH;
        - c(29) + cH;
        2*c(4)  - c(8) + 2*c(18); 
        - c(11) - 2*c(13) + 2*c(21); 
        c(5) + 2*c(7) + 2*c(24);
        2*c(9)  + c(11) - 2*c(22);
        - c(5) - 2*c(19) + 2*c(30);
        - c(8) + 2*c(12) - 2*c(25);
        2*c(13) - c(17);
        - c(20); 
        c(17) + 2*c(22);
        2*c(24) + c(26);
        - c(20); 
        - c(26) + 2*c(30);
        c(14) + 2*c(16) - c(23) + 2*cH;
        - c(14) - c(29) + 2*cH;
        - c(23) + 2*c(27) + c(29) + 2*cH;
        2*c(6) + 2*c(10) - 2*c(16) + 2*c(27); 
        2*c(15) + 2*c(19) - c(26);
        2*c(18) + c(20)+ 2*c(25);
        - c(17) + 2*c(21) + 2*c(28);
        0;0;0;0;0;0;
   ]';

   EE(nLe+2,:) = [0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,-1,0,0,0,0,0];
   EE(nLe+3,:) = [0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,-1,0,0,0,0];
   EE(nLe+4,:) = [0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,-1,0,0,0];
   EE(nLe+5,:) = [0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,-1,0,0];
   EE(nLe+6,:) = [0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,-1,0];
   EE(nLe+7,:) = [0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,-1];
  
   numL = nLe+7;
   for i = 1:6
        nx=n_c(1,i); ny=n_c(2,i); nz=n_c(3,i);
        Px1=Pn1_w(1,i); Py1=Pn1_w(2,i); Pz1=Pn1_w(3,i);
        Px2=Pn2_w(1,i); Py2=Pn2_w(2,i); Pz2=Pn2_w(3,i);
        EE(numL+i,1:10)=[(Px1-Px2).*nx+(Py1 - Py2).*ny+(Pz1 - Pz2).*nz,...    %1
                  2*(Py1 - Py2).*nz-2*(Pz1 - Pz2).*ny,...       %s1
                  2*(Pz1 - Pz2).*nx-2*(Px1-Px2).*nz,...       %s2
                  2*(Px1-Px2).*ny-2*(Py1 - Py2).*nx,...       %s3
                  (Px1-Px2).*nx-(Py1 - Py2).*ny-(Pz1 - Pz2).*nz,...     %s1^2
                  2.*(Px1-Px2).*ny+2*(Py1 - Py2).*nx,...      %s1*s2
                  2.*(Px1-Px2).*nz+2*(Pz1 - Pz2).*nx,...       %s1*s3
                  (Py1 - Py2).*ny-(Px1-Px2).*nx-(Pz1 - Pz2).*nz,...     %s2^2
                  2*(Py1 - Py2).*nz+2*(Pz1 - Pz2).*ny,...       %s2*s3
                  (Pz1 - Pz2).*nz-(Px1-Px2).*nx-(Py1 - Py2).*ny     %s3^2  
                  ];
   end

   GG=EE.'*EE;
     
end

