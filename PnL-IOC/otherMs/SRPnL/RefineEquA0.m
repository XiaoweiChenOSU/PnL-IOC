function [EE,GG,CC] = RefineEquA0(p1, p2, P1_w, W2)


%     pn1 = [p(:,1) p(:,1) p(:,1) p(:,1) p(:,1) p(:,1) p(:,1) p(:,2) p(:,2) p(:,2) p(:,2) p(:,2) p(:,2) p(:,3) p(:,3) p(:,3) p(:,3) p(:,3) p(:,4) p(:,4) p(:,4) p(:,4) p(:,5) p(:,5) p(:,5) p(:,6) p(:,6) p(:,7)];
%     pn2 = [p(:,2) p(:,3) p(:,4) p(:,5) p(:,6) p(:,7) p(:,8) p(:,3) p(:,4) p(:,5) p(:,6) p(:,7) p(:,8) p(:,4) p(:,5) p(:,6) p(:,7) p(:,8) p(:,5) p(:,6) p(:,7) p(:,8) p(:,6) p(:,7) p(:,8) p(:,7) p(:,8) p(:,8)];
%     pn1 = [p1 p2(:,5:8)];
%     pn2 = [p2 p2(:,6:8) p2(:,5)];
%     nxs = pn1;
%     nxe = pn2;
%     n_nc = xcross_mat(nxs,nxe);
%     ne_c = xnorm(n_nc);
%     Pn1_w = [W(:,1) W(:,1) W(:,1) W(:,1) W(:,1) W(:,1) W(:,1) W(:,2) W(:,2) W(:,2) W(:,2) W(:,2) W(:,2) W(:,3) W(:,3) W(:,3) W(:,3) W(:,3) W(:,4) W(:,4) W(:,4) W(:,4) W(:,5) W(:,5) W(:,5) W(:,6) W(:,6) W(:,7)];
%     Pn2_w = [W(:,2) W(:,3) W(:,4) W(:,5) W(:,6) W(:,7) W(:,8) W(:,3) W(:,4) W(:,5) W(:,6) W(:,7) W(:,8) W(:,4) W(:,5) W(:,6) W(:,7) W(:,8) W(:,5) W(:,6) W(:,7) W(:,8) W(:,6) W(:,7) W(:,8) W(:,7) W(:,8) W(:,8)];
%  
%     Pn1_w = [P1_w W2(:,5:8)];
%     Pn2_w = [W2 W2(:,6:8) W2(:,5)];
    
    W1 = P1_w;
    
    pn1 = [p1(:,5:8) p2(:,5:8) p2(:,5:6) p1(:,1:4)];
    pn2 = [p2(:,5:8) p2(:,6:8) p2(:,5) p2(:,7:8) p2(:,6) p2(:,5) p2(:,8) p2(:,7)];
    nxs = pn1;
    nxe = pn2;
    n_nc = xcross_mat(nxs,nxe);
    ne_c = xnorm(n_nc);
    Pn1_w = [W1(:,5:8) W2(:,5:8) W2(:,5:6) W1(:,1:4)];
    Pn2_w = [W2(:,5:8) W2(:,6:8) W2(:,5) W2(:,7:8) W2(:,6) W2(:,5) W2(:,8) W2(:,7)];


    neLine = length(pn1);


    Q=zeros(2*neLine,10);
    N=zeros(2*neLine,3);
    
    nx=ne_c(1,:)'; ny=ne_c(2,:)'; nz=ne_c(3,:)';
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
              Pz.*nz-Px.*nx-Py.*ny];
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
    N(1:2:end,:)=-ne_c.';
    N(2:2:end,:)=-ne_c.';


    if abs(det(N.'*N)) < 2e-10
        CC=(pinv(N.'*N)*N.')*Q;
    else
        CC=((N.'*N)\N.')*Q;
    end

    EE=Q-N*CC;
    GG=EE.'*EE;
    
end
