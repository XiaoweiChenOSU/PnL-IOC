function [EE,GG,CC] = RefineEquA(p1, p2, P1_w, P2_w)


    pn1 = p1;
    pn2 = [p2(:,1:4) kron(p2(:,5),ones(1,4))];
    n_nc = xcross_mat(pn1,pn2);
    ne_c = xnorm(n_nc);
    Pn1_w = P1_w;
    Pn2_w = [P2_w(:,1:4) kron(P2_w(:,5),ones(1,4))];
 
    neLine = length(pn1);

    Q=zeros(2*neLine,24);
    N=zeros(2*neLine,3);
    
    nx=ne_c(1,1:8)'; ny=ne_c(2,1:8)'; nz=ne_c(3,1:8)';
    Px=Pn1_w(1,1:8)'; Py=Pn1_w(2,1:8)'; Pz=Pn1_w(3,1:8)';
    Q(1:2:16,1:10)=[Px.*nx+Py.*ny+Pz.*nz,...    %1
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
         
    clearvars nx ny nz Px Py Pz    
    nx=ne_c(1,1:4)'; ny=ne_c(2,1:4)'; nz=ne_c(3,1:4)';           
    Px=Pn2_w(1,1:4)'; Py=Pn2_w(2,1:4)'; Pz=Pn2_w(3,1:4)';
    Q(2:2:8,1:10)=[Px.*nx+Py.*ny+Pz.*nz,...
              2*Py.*nz-2*Pz.*ny,...
              2*Pz.*nx-2*Px.*nz,...
              2*Px.*ny-2*Py.*nx,...
              Px.*nx-Py.*ny-Pz.*nz,...
              2.*Px.*ny+2*Py.*nx,...
              2.*Px.*nz+2*Pz.*nx,...
              Py.*ny-Px.*nx-Pz.*nz,...
              2*Py.*nz+2*Pz.*ny,...
              Pz.*nz-Px.*nx-Py.*ny];  
    clearvars nx ny nz Px Py Pz       
    nx=ne_c(1,5:8)'; ny=ne_c(2,5:8)'; nz=ne_c(3,5:8)';
    Px=Pn2_w(1,5:8)'; Py=Pn2_w(2,5:8)';
    Q(10:2:16,1:18)=[Px.*nx+Py.*ny,...
          2*Py.*nz,...
          -2*Px.*nz,...
          2*Px.*ny-2*Py.*nx,...
          Px.*nx-Py.*ny,...
          2.*Px.*ny+2*Py.*nx,...
          2.*Px.*nz,...
          Py.*ny-Px.*nx,...
          2*Py.*nz,...
          -Px.*nx-Py.*ny,...
          nz,-2*ny,2*nx,-nz,2*nx,-nz,2*ny,nz];          
    
    N(1:2:end,:)=-ne_c.';
    N(2:2:end,:)=-ne_c.';

    if abs(det(N.'*N)) < 2e-10
        CC=(pinv(N.'*N)*N.')*Q;
    else
        CC=((N.'*N)\N.')*Q;
    end

    EE=Q-N*CC;
    


    EE(2*neLine+1,:) = [0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,-1,0,0,0,0,0];
    EE(2*neLine+2,:) = [0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,-1,0,0,0,0];
    EE(2*neLine+3,:) = [0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,-1,0,0,0];
    EE(2*neLine+4,:) = [0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,-1,0,0];
    EE(2*neLine+5,:) = [0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,-1,0];
    EE(2*neLine+6,:) = [0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,-1];
  

    GG=EE.'*EE;
    
end
