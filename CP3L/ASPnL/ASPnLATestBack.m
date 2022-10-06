function [R, T, err] = ASPnLATestBack(p1, p2, P1_w, P2_w,cH,IA,p2d,R)
% the line is expressed by the start and end points
% inputs:
%	 p1: 2d projection of the start point
%	 p2: 2d projection of the end point
%	 P1_w: 3d coordinates of the start point in world frame
%	 P2_w: 3d coordinates of the end point in world frame
% outputs:
%	 R: estimated rotation
%	 T: estimated translation


% 	nLine = length(p1);
% 	p1 = [p1; ones(1,nLine)];
% 	p2 = [p2; ones(1,nLine)];
    
    [tR, tT, errt] = SRPnL3(p1, p2, P1_w, P2_w,cH);  

	
%     [tR, tT, errt] = SRPnL1(p1(1:2,:), p2(1:2,:), P1_w, P2_w);  
 
   

    d=P2_w-P1_w;
    d = xnorm(d);
    tR = Roptimzation(p1, p2, IA, tR, d, 10);
    
    
    nLine = length(p1);
    p1 = [p1; ones(1,nLine)];
	p2 = [p2; ones(1,nLine)];

    [Rt,aPoints] = CalculateAbyR3(p1, p2, P1_w, P2_w, cH, tR);
    
    
%     W2 = P2_w;
%     W2(1,1) = aPoints(1);
%     W2(2,2) = aPoints(2);
%     W2(3,3) = aPoints(3);
  

    p = [p1(:,1) p2];
    W = [P1_w(:,1) P2_w];
%     [EE,GG] = RefineEquA3R(p, W);
%     [EE,GG,CC] = RefineEquA3(p, W,tR,cH,IA,p2d,aPoints);
%     [EE,GG,CC] = RefineEquA3Original(p, W,R,cH);

%     [solution, obj_cur] = RefineGaussNewtonRandApT3(solutionO,EE,GG);
    solutionO = [Cayley(tR);aPoints]; 
    [solution, obj_cur] = RefineGaussNewtonRandApT3(solutionO,p, W,R,cH);
    if size(solution) ~= 12
        aPointsN = solution(4:6);
    else
        aPointsN = solution(10:12);
    end
    

    pn1 = [p1 p2];
    pn2 = [p2 p2(:,2:3) p2(:,1)];
    
    W2 = P2_w;
    W2(1,1) = aPointsN(1);
    W2(2,2) = aPointsN(2);
    W2(3,3) = aPointsN(3);
    
    P1N_w = [P1_w W2];
    P2N_w = [W2 W2(:,2:3) W2(:,1)];
    
    
    
    
    [R, T, err] = SRPnLA3(pn1(1:2,:), pn2(1:2,:), P1N_w, P2N_w);
%     [R, T, err] = LPnL_Bar_ENull(pn1(1:2,:), pn2(1:2,:), P1N_w, P2N_w);
%       [R, T, err] = ASPnL2(pn1(1:2,:), pn2(1:2,:), P1N_w, P2N_w);
%     [R0, T0, err] = RSPnL_(pn1, pn2, P1N_w, P2N_w,IA); 
%     [R,T] = invertRT(R0,T0);
%     if err > errt
%         R = tR;
%         T = Rt;
%     end
%     [R0, T0, err] = RSPnL_(pn1, pn2, P1N_w, P2N_w); 
%     [R,T] = invertRT(R0,T0);
    
    
    
   