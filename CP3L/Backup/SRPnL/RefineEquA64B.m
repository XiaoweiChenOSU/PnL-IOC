function [EE, GG, CC] = RefineEquA64B(p, W)

    pn1 = [p(:,1) p(:,1) p(:,1) p(:,2) p(:,2)];
    pn2 = [p(:,3) p(:,4) p(:,2) p(:,3) p(:,4)];
    nxs = pn1;
    nxe = pn2;
    n_nc = xcross_mat(nxs,nxe);
    ne_c = xnorm(n_nc);
    Pn1_w = [W(:,1) W(:,1) W(:,1)];
    Pn2_w = [W(:,2) W(:,2) W(:,2)];
 
    neLine = length(Pn1_w);


    Q=zeros(2*neLine,32);
    N=zeros(2*neLine,3);
    
    for i = 1:3
        nx=ne_c(1,i); ny=ne_c(2,i); nz=ne_c(3,i);
        Px=Pn1_w(1,i); Pz=Pn1_w(3,i);
        Q(i,1:18)=[Px.*nx+Pz.*nz,...
              -2*Pz.*ny,...
              2*Pz.*nx-2*Px.*nz,...
              2*Px.*ny,...
              Px.*nx-Pz.*nz,...
              2.*Px.*ny,...
              2.*Px.*nz+2*Pz.*nx,...
              -Px.*nx-Pz.*nz,...
              2*Pz.*ny,...
              Pz.*nz-Px.*nx,...
              ny,2*nz,-2*nx,-ny,2*nx,ny,2*nz,-ny]; 
    end
    for j = 1:3
        nx=ne_c(1,j+2); ny=ne_c(2,j+2); nz=ne_c(3,j+2);
        Px=Pn2_w(1,j); Pz=Pn2_w(3,j);
        Q(j+3,1:26)=[Px.*nx+Pz.*nz,...
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
    
   
    N(1:neLine,:)=-ne_c(:,1:3).';
    N(neLine+1:2*neLine,:)=-ne_c(:,3:5).';

    if abs(det(N.'*N)) < 2e-10
        CC=(pinv(N.'*N)*N.')*Q;
    else
        CC=((N.'*N)\N.')*Q;
    end

    EE=Q-N*CC;


    EE(2*neLine+1,:) = [0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,-1,0,0,0,0,0];
    EE(2*neLine+2,:) = [0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,-1,0,0,0,0];
    EE(2*neLine+3,:) = [0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,-1,0,0,0];
    EE(2*neLine+4,:) = [0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,-1,0,0];
    EE(2*neLine+5,:) = [0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,-1,0];
    EE(2*neLine+6,:) = [0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,-1];
  

    GG=EE.'*EE;
    
end
