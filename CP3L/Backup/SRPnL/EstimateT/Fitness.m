% Fitness (objective function)
function f = Fitness(x, y, z, t1, t2, t3)

    pwd

    
    T1A = [512 0 0;0 512 0;0 0 1];
    
    
    R = [0.513555289318380,-0.0371730087097575,-0.857250915564737;
         0.0343603280499138,-0.997368879268819,0.0638332712804395;
         -0.857368259658251,-0.0622373367804580,-0.510926786576176];
    
    p  = [279 297;1 470;278 2;635 490];
    
    PW0 = [256, 0, 256, 1];
%     p0 = [279.0507 297.7435 1];
    % p0 = [279 297 1];



    PW1 = [256, 0, z, 1];
%     p1 =[1.3811 470.9088 1];
    % p1 =[1 470 1];



    PW2 = [256, y, 256, 1];
    % p2 = [278.1006 16.9544 1];
%     p2 = [278.0513 2.3916 1];
    % p2 = [278 2 1];


    PW3 = [x, 0, 256, 1];
%     p3 = [634.9015 491.0976 1];

    T = [t1 t2 t3]';
    
    ep1 = T1A*[R T]*PW0;
    ep1 = ep1/ep1(3);
    
    d1 = norm(p(1,:)-ep1);
    
    ep2 = T1A*[R T]*PW1;
    ep2 = ep2/ep2(3);
    
    d2 = norm(p(2,:)-ep2);
    
    ep3 = T1A*[R T]*PW2;
    ep3 = ep3/ep3(3);
    
    d3 = norm(p(3,:)-ep3);
    
    ep4 = T1A*[R T]*PW3;
    ep4 = ep4/ep4(3);
    
    d4 = norm(p(3,:)-ep4);

  
    f = d1+d2+d3+d4;
    
end