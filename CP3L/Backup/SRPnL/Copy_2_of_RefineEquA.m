function [EE,GG,CC] = Copy_2_of_RefineEquA(p1, p2, P1_w, P2_w)


    pn1 = [p1 p2(:,5:8)];
    pn2 = [p2 p2(:,6:8) p2(:,5)];
    n_nc = xcross_mat(pn1,pn2);
    ne_c = xnorm(n_nc);
    Pn1_w = [P1_w P2_w(:,5:8)];
    Pn2_w = [P2_w P2_w(:,6:8) P2_w(:,5)];
 
    neLine = length(pn1);


    Q=zeros(2*neLine,48);
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
    nx=ne_c(1,5); ny=ne_c(2,5); nz=ne_c(3,5);
    Px=Pn2_w(1,5); Py=Pn2_w(2,5);
    Q(10,1:18)=[Px.*nx+Py.*ny,...
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
      
    clearvars nx ny nz Px Py Pz   
    nx=ne_c(1,6); ny=ne_c(2,6); nz=ne_c(3,6);
    Px=Pn2_w(1,6); Py=Pn2_w(2,6);
    Q(12,1:26)=[Px.*nx+Py.*ny,...
          2*Py.*nz,...
          -2*Px.*nz,...
          2*Px.*ny-2*Py.*nx,...
          Px.*nx-Py.*ny,...
          2.*Px.*ny+2*Py.*nx,...
          2.*Px.*nz,...
          Py.*ny-Px.*nx,...
          2*Py.*nz,...
          -Px.*nx-Py.*ny,...
          0,0,0,0,0,0,0,0,nz,-2*ny,2*nx,-nz,2*nx,-nz,2*ny,nz];
      
    clearvars nx ny nz Px Py Pz   
    nx=ne_c(1,7); ny=ne_c(2,7); nz=ne_c(3,7);
    Px=Pn2_w(1,7); Py=Pn2_w(2,7);
    Q(14,1:34)=[Px.*nx+Py.*ny,...
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
   
    clearvars nx ny nz Px Py Pz   
    nx=ne_c(1,8); ny=ne_c(2,8); nz=ne_c(3,8);
    Px=Pn2_w(1,8); Py=Pn2_w(2,8);
    Q(16,1:42)=[Px.*nx+Py.*ny,...
          2*Py.*nz,...
          -2*Px.*nz,...
          2*Px.*ny-2*Py.*nx,...
          Px.*nx-Py.*ny,...
          2.*Px.*ny+2*Py.*nx,...
          2.*Px.*nz,...
          Py.*ny-Px.*nx,...
          2*Py.*nz,...
          -Px.*nx-Py.*ny,...
          0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,nz,-2*ny,2*nx,-nz,2*nx,-nz,2*ny,nz];
    
    clearvars nx ny nz Px Py Pz   
    nx=ne_c(1,9); ny=ne_c(2,9); nz=ne_c(3,9);
    Px=Pn1_w(1,9); Py=Pn1_w(2,9);
    Q(17,1:18)=[Px.*nx+Py.*ny,...
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
    
    clearvars Px Py Pz   
    Px=Pn2_w(1,9); Py=Pn2_w(2,9);
    Q(18,1:26)=[Px.*nx+Py.*ny,...
          2*Py.*nz,...
          -2*Px.*nz,...
          2*Px.*ny-2*Py.*nx,...
          Px.*nx-Py.*ny,...
          2.*Px.*ny+2*Py.*nx,...
          2.*Px.*nz,...
          Py.*ny-Px.*nx,...
          2*Py.*nz,...
          -Px.*nx-Py.*ny,...
          0,0,0,0,0,0,0,0,nz,-2*ny,2*nx,-nz,2*nx,-nz,2*ny,nz];
      
    
    clearvars nx ny nz Px Py Pz   
    nx=ne_c(1,10); ny=ne_c(2,10); nz=ne_c(3,10);
    Px=Pn1_w(1,10); Py=Pn1_w(2,10);
    Q(19,1:26)=[Px.*nx+Py.*ny,...
          2*Py.*nz,...
          -2*Px.*nz,...
          2*Px.*ny-2*Py.*nx,...
          Px.*nx-Py.*ny,...
          2.*Px.*ny+2*Py.*nx,...
          2.*Px.*nz,...
          Py.*ny-Px.*nx,...
          2*Py.*nz,...
          -Px.*nx-Py.*ny,...
          0,0,0,0,0,0,0,0,nz,-2*ny,2*nx,-nz,2*nx,-nz,2*ny,nz];
    
    clearvars Px Py Pz   
    Px=Pn2_w(1,10); Py=Pn2_w(2,10);
    Q(20,1:34)=[Px.*nx+Py.*ny,...
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

    clearvars nx ny nz Px Py Pz   
    nx=ne_c(1,11); ny=ne_c(2,11); nz=ne_c(3,11);
    Px=Pn1_w(1,11); Py=Pn1_w(2,11);
    Q(21,1:34)=[Px.*nx+Py.*ny,...
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
    
    clearvars Px Py Pz
    Px=Pn2_w(1,11); Py=Pn2_w(2,11);
    Q(22,1:42)=[Px.*nx+Py.*ny,...
          2*Py.*nz,...
          -2*Px.*nz,...
          2*Px.*ny-2*Py.*nx,...
          Px.*nx-Py.*ny,...
          2.*Px.*ny+2*Py.*nx,...
          2.*Px.*nz,...
          Py.*ny-Px.*nx,...
          2*Py.*nz,...
          -Px.*nx-Py.*ny,...
          0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,nz,-2*ny,2*nx,-nz,2*nx,-nz,2*ny,nz];

    clearvars nx ny nz Px Py Pz     
    nx=ne_c(1,12); ny=ne_c(2,12); nz=ne_c(3,12);
    Px=Pn1_w(1,12); Py=Pn1_w(2,12);
    Q(23,1:42)=[Px.*nx+Py.*ny,...
          2*Py.*nz,...
          -2*Px.*nz,...
          2*Px.*ny-2*Py.*nx,...
          Px.*nx-Py.*ny,...
          2.*Px.*ny+2*Py.*nx,...
          2.*Px.*nz,...
          Py.*ny-Px.*nx,...
          2*Py.*nz,...
          -Px.*nx-Py.*ny,...
          0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,nz,-2*ny,2*nx,-nz,2*nx,-nz,2*ny,nz];
    clearvars Px Py Pz
    Px=Pn2_w(1,12); Py=Pn2_w(2,12);
    Q(24,1:18)=[Px.*nx+Py.*ny,...
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
    


    EE(2*neLine+1,:) = [0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,-1,0,0,0,0,0];
    EE(2*neLine+2,:) = [0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,-1,0,0,0,0];
    EE(2*neLine+3,:) = [0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,-1,0,0,0];
    EE(2*neLine+4,:) = [0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,-1,0,0];
    EE(2*neLine+5,:) = [0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,-1,0];
    EE(2*neLine+6,:) = [0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,-1];
  

    GG=EE.'*EE;
    
end
