function [EE,GG] = RefineEquA3R(p, W)


    pn1 = [p(:,1) p(:,1) p(:,1) p(:,2) p(:,2) p(:,3)];
    pn2 = [p(:,2) p(:,3) p(:,4) p(:,3) p(:,4) p(:,4)];
    n_nc = xcross_mat(pn1,pn2);
    ne_c = xnorm(n_nc);
    Pn1_w = [W(:,1) W(:,1) W(:,1) W(:,2) W(:,2) W(:,3)];
    Pn2_w = [W(:,2) W(:,3) W(:,4) W(:,3) W(:,4) W(:,4)];
 
    neLine = length(pn1);
    
    
    EE=zeros(neLine,40);
    
    for i = 1:6
        nx=ne_c(1,i); ny=ne_c(2,i); nz=ne_c(3,i);
        if i == 1
            Px1=Pn1_w(1,i); Py1=Pn1_w(2,i); Pz1=Pn1_w(3,i);
            Py2=Pn2_w(2,i); Pz2=Pn2_w(3,i);
            EE(i,1:18)=[(Px1).*nx+(Py1 - Py2).*ny+(Pz1 - Pz2).*nz,...    %1
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
            EE(i,1:26)=[(Px1 - Px2).*nx+(Py1).*ny+(Pz1 - Pz2).*nz,...    %1
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
            EE(i,1:34)=[(Px1 - Px2).*nx+(Py1 - Py2).*ny+(Pz1).*nz,...    %1
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
            EE(i,1:26)=[(-Px2).*nx+(Py1).*ny+(Pz1 - Pz2).*nz,...    %1
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
            EE(i,1:34)=[(-Px2).*nx+(Py1 - Py2).*ny+(Pz1).*nz,...    %1
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
            EE(i,1:34)=[(Px1 - Px2).*nx+(-Py2).*ny+(Pz1).*nz,...    %1
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
    
    



    EE(neLine+1,:) = [0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,-1,0,0,0,0,0];
    EE(neLine+2,:) = [0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,-1,0,0,0,0];
    EE(neLine+3,:) = [0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,-1,0,0,0];
    EE(neLine+4,:) = [0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,-1,0,0];
    EE(neLine+5,:) = [0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,-1,0];
    EE(neLine+6,:) = [0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,-1];
  
        

    GG=EE.'*EE;
    
end
