function [RR,tt,err]=TestIterative(p0,p_l,P0, P_L,f)
    n = 3;

    P0 = [279 297;279 297;279 297];
    P_L  = [1 470;278 2;635 490];
    
    GR = [[ 0.51378347 -0.03592724  -0.8571673 ]
 [0.03497143  -0.99741529  0.06276735]
 [-0.85720683 -0.0622252   -0.51119906]];
    % P0 = [316, 385; 316, 385; 316, 385;];
    % P_L = [233, 436; 314, 153; 563, 541];
    % 
    % P0 = [316, 385];
    % P_L = [233, 436];
    % 
    % c = [124, 503, 1; 316, 385, 1; 233, 436, 1];
    % [V,D] = eig(c)
    % 
    % [A1,B1,C1] = solve('316*A1 + 385*B1 + C1=0','233*A1 + 436*B1 + C1=0');

    T1p1 = P0;
    T1p2 = P_L;

    T1P1_w = [256 0 256;256 30 256;281 0 256];
    T1P2_w = [256 0 371;256 100 256;336 0 256];


    T1A = [512 0 0;0 512 0;0   0   1];

    for i = 1:length(T1p1)
        temp1 = inv(T1A)*[T1p1(i,:) 1].';
        temp1 = (temp1/temp1(3)).';
        iT1p1(i,:) = temp1(:,1:2);
        temp2 = inv(T1A)*[T1p2(i,:) 1].';
        temp2 = (temp2/temp2(3)).';
        iT1p2(i,:) = temp2(:,1:2);
    end

    nl = getProjNorm(iT1p1',iT1p2');
    % nl = getProjNorm(iT1p2',iT1p1');

    Rt{1} = [0.515129347795564,-0.0363540396566050,-0.856341134618887;0.0306234324096387,-0.997681568573212,0.0607757609310401;-0.856565210843383,-0.0575314829457806,-0.512821770250533];


    d = [0 0 1; 0 1 0; 1 0 0];

    K1 = eye(3) - nl(:,1)*nl(:,1)';
    K2 = eye(3) - nl(:,2)*nl(:,2)';
    K3 = eye(3) - nl(:,3)*nl(:,3)';
    
    I = eye(3);


    
    B = [d(1,:); d(2,:); d(3,:)];

    k = 1;
    while k <= 200

        A = [K1*Rt{k}*d(1,:)' K2*Rt{k}*d(2,:)' K3*Rt{k}*d(3,:)'];

        M  = A*B';

        [U ,D ,V] = svd(M);

        if rank(M) > 2
            if det(M) >= 0
                S = eye(3);
            else
                S = [1 0 0; 0 1 0; 0 0 -1];
            end
        else
            if det(U)*det(V) == 1
                S = eye(3)
            elseif det(U)*det(V) == -1
                S = [1 0 0; 0 1 0; 0 0 -1];
            end
        end

        k = k +1;
        Rt{k} = U*S*V'; 
        
    end

    R =  Rt{k};
    


    syms X Y Z 

    f = 512;
%     T = [186.9010081 80.40337436 531.77185951]';

    PW0 = [256, 0, 256];
%     p0 = [279.0507 297.7435 1];
    p0 = [279 297 1];



    PW1 = [256, 0, Z];
%     p1 =[1.3811 470.9088 1];
    p1 =[1 470 1];



    PW2 = [256, Y, 256];
    % p2 = [278.1006 16.9544 1];
%     p2 = [278.0513 2.3916 1];
    p2 = [278 2 1];

%     testp0 = T1A*[R T]*[256, 105, 256,1]';
%     testp0 = testp0/testp0(3);

    PW3 = [X, 0, 256];
%     p3 = [634.9015 491.0976 1];
    p3 = [635 490 1];




    % A = [
    %      0 0 0 f 0 0 -p0(1) 0 0 0;...
    %      0 0 0 0 f 0 -p0(2) 0 0 0;...
    %      0 0 0 0 0 1 -1 0 0 0;...
    %      0 0 f*R(1,3) f 0 0 0 -p1(1) 0 0;...
    %      0 0 f*R(2,3) 0 f 0 0 -p1(2) 0 0;...
    %      0 0 R(3,3) 0 0 1 0 -1 0 0;...
    %      0 f*R(1,2) 0 f 0 0 0 0 -p2(1) 0;...
    %      0 f*R(2,2) 0 0 f 0 0 0 -p2(2) 0;...
    %      0 R(3,2) 0 0 0 1 0 0 -1 0;...
    %      f*R(1,1) 0 0 f 0 0 0 0 0 -p3(1);...
    %      f*R(2,1) 0 0 0 f 0 0 0 0 -p3(2);...
    %      R(3,1) 0 0 0 0 1 0 0 0 -1;...
    %     ]
    % 
    % b = [
    %       f*R(1,1)*PW0(1)+f*R(1,2)*PW0(2)+f*R(1,3)*PW0(3);...
    %       f*R(2,1)*PW0(1)+f*R(2,2)*PW0(2)+f*R(2,3)*PW0(3);...
    %       R(3,1)*PW0(1)+R(3,2)*PW0(2)+R(3,3)*PW0(3);...
    %       f*R(1,1)*PW1(1)+f*R(1,2)*PW1(2);...
    %       f*R(2,1)*PW1(1)+f*R(2,2)*PW1(2);...
    %       R(3,1)*PW1(1)+R(3,2)*PW1(2);...
    %       f*R(1,1)*PW2(1)+f*R(1,3)*PW2(3);...
    %       f*R(2,1)*PW2(1)+f*R(2,3)*PW2(3);...
    %       R(3,1)*PW2(1)+R(3,3)*PW2(3);...
    %       f*R(1,2)*PW3(2)+f*R(1,3)*PW3(3);...
    %       f*R(2,2)*PW3(2)+f*R(2,3)*PW3(3);...
    %       R(3,2)*PW3(2)+R(3,3)*PW3(3);...
    %     ]

    A = [0 0 0 f 0 -p0(1);...
         0 0 0 0 f -p0(2);...
         0 0 f*R(1,3)-p1(1)*R(3,3) f 0 -p1(1);...
         0 0 f*R(2,3)-p1(2)*R(3,3) 0 f -p1(2);...
         0 f*R(1,2)-p2(1)*R(3,2) 0 f 0 -p2(1);...
         0 f*R(2,2)-p2(2)*R(3,2) 0 0 f -p2(2);...
         f*R(1,1)-p3(1)*R(3,1) 0 0 f 0 -p3(1);...
         f*R(2,1)-p3(2)*R(3,1) 0 0 0 f -p3(2);...
         ];

    b = -[ f*R(1,1)*PW0(1)+f*R(1,2)*PW0(2)+f*R(1,3)*PW0(3)-p0(1)*R(3,1)*PW0(1)-p0(1)*R(3,2)*PW0(2)-p0(1)*R(3,3)*PW0(3);...
          f*R(2,1)*PW0(1)+f*R(2,2)*PW0(2)+f*R(2,3)*PW0(3)-p0(2)*R(3,1)*PW0(1)-p0(2)*R(3,2)*PW0(2)-p0(2)*R(3,3)*PW0(3);...
          f*R(1,1)*PW1(1)+f*R(1,2)*PW1(2)-p1(1)*R(3,1)*PW1(1)-p1(1)*R(3,2)*PW1(2);...
          f*R(2,1)*PW1(1)+f*R(2,2)*PW1(2)-p1(2)*R(3,1)*PW1(1)-p1(2)*R(3,2)*PW1(2);...
          f*R(1,1)*PW2(1)+f*R(1,3)*PW2(3)-p2(1)*R(3,1)*PW2(1)-p2(1)*R(3,3)*PW2(3);...
          f*R(2,1)*PW2(1)+f*R(2,3)*PW2(3)-p2(2)*R(3,1)*PW2(1)-p2(2)*R(3,3)*PW2(3);...
          f*R(1,2)*PW3(2)+f*R(1,3)*PW3(3)-p3(1)*R(3,2)*PW3(2)-p3(1)*R(3,3)*PW3(3);...
          f*R(2,2)*PW3(2)+f*R(2,3)*PW3(3)-p3(2)*R(3,2)*PW3(2)-p3(2)*R(3,3)*PW3(3);...
          ]  
      
      
      
      b = double(b);
      
%       testx = [336 100 371 186.9010081 80.40337436 531.77185951]';
%     % testx = [336 100 371 189.426121320767 80.7895147131118 531.310178777817]';
% 
%       re = A*testx - b
%       
%       x =double(A\b)
%       x1 = double(linsolve(A,b))
%       s1 = lsqminnorm(A,double(b),1e-2)


      %constaints based on the layout 
      lb = [257;1;257;0;0;0];
      ub = [512;256;512;512;512;512];
      
      x = lsqlin(A,b,[],[],[],[],lb,ub)
      
      re = double(A*x - b)
      
%      cb = double(sum(b));
%      x1 = optimvar('x1','LowerBound',257,'UpperBound',512);
%      x2 = optimvar('x2','LowerBound',1,'UpperBound',256);
%      x3 = optimvar('x3','LowerBound',257,'UpperBound',512);
%      x4 = optimvar('x4','LowerBound',0,'UpperBound',512);
%      x5 = optimvar('x5','LowerBound',0,'UpperBound',512);
%      x6 = optimvar('x6','LowerBound',0,'UpperBound',512);
%      x7 = optimvar('x7','LowerBound',1,'UpperBound',1);
%      
%      prob = optimproblem('Objective',(sum(A(:,1))*x1+sum(A(:,2))*x2+sum(A(:,3))*x3+sum(A(:,4))*x4+sum(A(:,5))*x5+sum(A(:,6))*x6+cb*x7).^2,'ObjectiveSense','min');
% 
%      prob.Constraints.c1 = x7 == 1;
% 
% %      prob = optimproblem('Objective',sum(A(:,1))*x1+sum(A(:,2))*x2+sum(A(:,3))*x3+sum(A(:,4))*x4+sum(A(:,5))*x5+sum(A(:,6))*x6+sum(b)*x7,'ObjectiveSense','min');
%      problem = prob2struct(prob);
%      [sol,fval,exitflag,output] = linprog(problem)
     %       s = 
%     x =double(A\-b)
%     x1 = double(linsolve(A,-b))
%     s1 = lsqminnorm(A,double(-b),1e-2)


    testx = [336 100 371 186.9010081 80.40337436 531.77185951]';
    % testx = [336 100 371 189.426121320767 80.7895147131118 531.310178777817]';

    re = double(A*testx + b)


    M = [A b];
    [U, S, V] =  svd(M);
    v1 = V(:,7)/V(7,7)
    v2 = V(:,6)/V(7,6)
    v3 = V(:,5)/V(7,5)
    v4 = V(:,4)/V(7,4)
    v5 = V(:,3)/V(7,3)
    v6 = V(:,2)/V(7,2)
    v7 = V(:,1)/V(7,1)
    

%     testK  = (I - K1)+(I - K2)+(I - K3)
%     testL = (K1 - I)*R*[256, 0, 256]'+(K2 - I)*R*[256, 0, 256]'+(K3 - I)*R*[256, 0, 256]'
%     testK  = (I - K3)+(I - K2)+(I - K1)
%     testL = (K3 - I)*R*[256, 0, 256]'+(K1 - I)*R*[256, 0, 256]'+(K2 - I)*R*[256, 0, 256]'
    testK  = (I - K2)+(I - K1)+(I - K3)
    testL = (K2 - I)*R*[256, 0, 256]'+(K1 - I)*R*[256, 0, 256]'+(K3 - I)*R*[256, 0, 256]'

    t = inv(testK)*testL
    

end