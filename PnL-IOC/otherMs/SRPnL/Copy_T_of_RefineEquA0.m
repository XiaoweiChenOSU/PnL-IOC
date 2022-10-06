function [EE,GG,CC] = Copy_T_of_RefineEquA0(p1, p2, P1_w, W2,solution,tR,tT)


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
% %  
%     Pn1_w = [P1_w W2(:,5:8)];
%     Pn2_w = [W2 W2(:,6:8) W2(:,5)];
%     neLine = length(pn1);
%     



    pn1 = [p1 p2(:,5:8) p2(:,5:6)];
    pn2 = [p2 p2(:,6:8) p2(:,5) p2(:,7:8)];
    nxs = pn1;
    nxe = pn2;
    n_nc = xcross_mat(nxs,nxe);
    ne_c = xnorm(n_nc);
    Pn1_w = [P1_w W2(:,5:8) W2(:,5:6)];
    Pn2_w = [W2 W2(:,6:8) W2(:,5) W2(:,7:8)];


    
    neLine = length(pn1);
    

    Q=zeros(2*neLine,21);
    N=zeros(2*neLine,3);
    
    
    for i = 1:8
        nx=ne_c(1,i); ny=ne_c(2,i); nz=ne_c(3,i);
        Px=Pn1_w(1,i); Py=Pn1_w(2,i); Pz=Pn1_w(3,i);
        Q(i,1:9)=[nx*Px,...       %R11
              nx*Py,...       %R12
              nx*Pz,...       %R13
              ny*Px,...     %R21
              ny*Py,...      %R22
              ny*Pz,...       %R23
              nz*Px,...     %R31
              nz*Py,...       %R32
              nz*Pz,...     %R33
             ];
    end
    
    for i = 9:14
        nx=ne_c(1,i); ny=ne_c(2,i); nz=ne_c(3,i);
        if i == 9 || i == 13
            Px=Pn1_w(1,i); Py=Pn1_w(2,i);
            Q(i,1:12)=[nx*Px,...       %R11
                  nx*Py,...       %R12
                  0,...       %R13
                  ny*Px,...     %R21
                  ny*Py,...      %R22
                  0,...       %R23
                  nz*Px,...     %R31
                  nz*Py,...       %R32
                  0,...     %R33
                  nx,ny,nz];
        elseif i == 10 || i == 14
            Px=Pn1_w(1,i); Py=Pn1_w(2,i);
            Q(i,1:15)=[nx*Px,...       %R11
                  nx*Py,...       %R12
                  0,...       %R13
                  ny*Px,...     %R21
                  ny*Py,...      %R22
                  0,...       %R23
                  nz*Px,...     %R31
                  nz*Py,...       %R32
                  0,...     %R33
                  0,0,0,nx,ny,nz]; 
        elseif i == 11
            Px=Pn1_w(1,i); Py=Pn1_w(2,i);
            Q(i,1:18)=[nx*Px,...       %R11
                  nx*Py,...       %R12
                  0,...       %R13
                  ny*Px,...     %R21
                  ny*Py,...      %R22
                  0,...       %R23
                  nz*Px,...     %R31
                  nz*Py,...       %R32
                  0,...     %R33
                  0,0,0,0,0,0,nx,ny,nz];
        else
            Px=Pn1_w(1,i); Py=Pn1_w(2,i);
            Q(i,1:21)=[nx*Px,...       %R11
                  nx*Py,...       %R12
                  0,...       %R13
                  ny*Px,...     %R21
                  ny*Py,...      %R22
                  0,...       %R23
                  nz*Px,...     %R31
                  nz*Py,...       %R32
                  0,...     %R33
                  0,0,0,0,0,0,0,0,0,nx,ny,nz];
       end
    end      
    for j = 1:14
        nx=ne_c(1,j); ny=ne_c(2,j); nz=ne_c(3,j);
        if j>=1 && j<=4
             Px=Pn2_w(1,j); Py=Pn2_w(2,j); Pz=Pn2_w(3,j);
             Q(j+12,1:9)=[nx*Px,...       %R11
              nx*Py,...       %R12
              nx*Pz,...       %R13
              ny*Px,...     %R21
              ny*Py,...      %R22
              ny*Pz,...       %R23
              nz*Px,...     %R31
              nz*Py,...       %R32
              nz*Pz,...     %R33
             ];
        end
        if j==5 || j==12 || j==16
             Px=Pn2_w(1,j); Py=Pn2_w(2,j);
              Q(j+12,1:12)=[nx*Px,...       %R11
                  nx*Py,...       %R12
                  0,...       %R13
                  ny*Px,...     %R21
                  ny*Py,...      %R22
                  0,...       %R23
                  nz*Px,...     %R31
                  nz*Py,...       %R32
                  0,...     %R33
                  nx,ny,nz];
        end
        if j==6 || j==9 || j==15
             Px=Pn2_w(1,j); Py=Pn2_w(2,j);
              Q(j+12,1:15)=[nx*Px,...       %R11
                  nx*Py,...       %R12
                  0,...       %R13
                  ny*Px,...     %R21
                  ny*Py,...      %R22
                  0,...       %R23
                  nz*Px,...     %R31
                  nz*Py,...       %R32
                  0,...     %R33
                  0,0,0,nx,ny,nz]; 
        end
        if j==7 || j==10 || j==13 || j==18
             Px=Pn2_w(1,j); Py=Pn2_w(2,j);
             Q(j+12,1:18)=[nx*Px,...       %R11
                  nx*Py,...       %R12
                  0,...       %R13
                  ny*Px,...     %R21
                  ny*Py,...      %R22
                  0,...       %R23
                  nz*Px,...     %R31
                  nz*Py,...       %R32
                  0,...     %R33
                  0,0,0,0,0,0,nx,ny,nz];
        end
        if j==8 || j==11 || j==14 || j==17
             Px=Pn2_w(1,j); Py=Pn2_w(2,j);
             Q(j+12,1:21)=[nx*Px,...       %R11
                  nx*Py,...       %R12
                  0,...       %R13
                  ny*Px,...     %R21
                  ny*Py,...      %R22
                  0,...       %R23
                  nz*Px,...     %R31
                  nz*Py,...       %R32
                  0,...     %R33
                  0,0,0,0,0,0,0,0,0,nx,ny,nz];
        end
    end
    
    
    N(1:neLine,:)=-ne_c.';
    N(neLine+1:2*neLine,:)=-ne_c.';
%     N(1:8,:)=-ne_c(:,1:8).';


    if abs(det(N.'*N)) < 2e-10
        CC=(pinv(N.'*N)*N.')*Q;
    else
        CC=((N.'*N)\N.')*Q;
    end

    EE=Q-N*CC;

    GG=EE.'*EE;
    
end
