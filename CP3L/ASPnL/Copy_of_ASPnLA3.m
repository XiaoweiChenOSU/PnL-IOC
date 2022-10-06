function [R, T, err] = Copy_of_ASPnLA3(p1, p2, P1_w, P2_w,cH,IA,Rtruth)
% the line is expressed by the start and end points
% inputs:
%	 p1: 2d projection of the start point
%	 p2: 2d projection of the end point
%	 P1_w: 3d coordinates of the start point in world frame
%	 P2_w: 3d coordinates of the end point in world frame
% outputs:
%	 R: estimated rotation
%	 T: estimated translation

    Rth = Rtruth;

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
    
    
    pn1 = [p1 p2];
    pn2 = [p2 p2(:,2:3) p2(:,1)];
    tempW2 = P2_w;
    tempW2(1,1) = aPoints(1);
    tempW2(2,2) = aPoints(2);
    tempW2(3,3) = aPoints(3);
    Pn1_w1 = [P1_w tempW2];
    Pn2_w1 = [tempW2 tempW2(:,2:3) tempW2(:,1)];

    RrA = tR;


%     for i = 1:20
%         [EEA,GGA,CCA] = refineEquation(p1,pn1,pn2,Pn1_w1,Pn2_w1,RrA,cH);
%         s = [Cayley(RrA);aPoints;Rt];
%         [solutionA, errA] = RefineGaussNewtonForApoints(s,EEA,GGA);
% 
%         s1=solutionA(1);s2=solutionA(2);s3=solutionA(3);
%         s4 =solutionA(4);s5=solutionA(5); s6=solutionA(6); 
%         s7=solutionA(7); s8=solutionA(8); s9=solutionA(9);
%         s10=solutionA(10); s11=solutionA(11); s12=solutionA(12);
%         t1=solutionA(13); t2=solutionA(14); t3=solutionA(15);
%         sr= [1,s1,s2,s3,s7,s8,s9,s10,s11,s12,...
%             s4,s4*s2,s4*s3,s4*s7,s4*s8,s4*s9,s4*s10,s4*s12,...
%             s5,s5*s1,s5*s3,s5*s7,s5*s8,s5*s10,s5*s11,s5*s12,...
%             s6,s6*s1,s6*s2,s6*s7,s6*s9,s6*s10,s6*s11,s6*s12,...
%             s1^2, s1*s2, s1*s3, s2^2, s2*s3, s3^2,...
%             t1,t2,t3,t1*s3,t1*s7,t1*s8,t1*s10,t1*s12,t2*s7,t2*s10,t2*s12,t3*s1,t3*s7,t3*s10,t3*s11,t3*s12].';
%         factor=1/(1+s1^2+s2^2+s3^2); 
%         RrA=factor*[1+s1^2-s2^2-s3^2,2*s1*s2-2*s3,2*s2+2*s1*s3;
%                    2*s3+2*s1*s2,1-s1^2+s2^2-s3^2,2*s2*s3-2*s1;
%                    2*s1*s3-2*s2,2*s1+2*s2*s3,1-s1^2-s2^2+s3^2];
% 
%     end

    for i = 1:20

        [EEA,GGA,CCA] = CopyrefineEquation(p1,pn1,pn2,Pn1_w1,Pn2_w1,RrA,cH);
        s = [Cayley(RrA);tempW2(1,1);tempW2(2,2);tempW2(3,3)];
        [solutionA, err] = CopyRefineGaussNewtonForApoints(s,EEA,GGA);


        s1=solutionA(1);s2=solutionA(2);s3=solutionA(3);
        s4 =solutionA(4);s5=solutionA(5); s6=solutionA(6); 
        s7=solutionA(7); s8=solutionA(8); s9=solutionA(9);
        s10=solutionA(10); s11=solutionA(11); s12=solutionA(12);
%             t1=solutionA(13); t2=solutionA(14); t3=solutionA(15);
%         sr=[1,s1,s2,s3,s1^2,s1*s2,s1*s3,s2^2,s2*s3,s3^2].';
        sr= [1,s1,s2,s3,s7,s8,s9,s10,s11,s12,...
            s4,s4*s2,s4*s3,s4*s7,s4*s8,s4*s9,s4*s10,s4*s12,...
            s5,s5*s1,s5*s3,s5*s7,s5*s8,s5*s10,s5*s11,s5*s12,...
            s6,s6*s1,s6*s2,s6*s7,s6*s9,s6*s10,s6*s11,s6*s12,...
            s1^2, s1*s2, s1*s3, s2^2, s2*s3, s3^2].';
        factor=1/(1+s1^2+s2^2+s3^2); 
        RrA=factor*[1+s1^2-s2^2-s3^2,2*s1*s2-2*s3,2*s2+2*s1*s3;
                   2*s3+2*s1*s2,1-s1^2+s2^2-s3^2,2*s2*s3-2*s1;
                   2*s1*s3-2*s2,2*s1+2*s2*s3,1-s1^2-s2^2+s3^2];

%         curErr2 = cal_rotation_err(RrA,R_truth);
    end

    RAt = factor*CCA*sr;    

    R = RrA;
    T = RAt;
    

    aPointsN = solutionA(4:6);
    
%     RAt = solutionA(13:15);
    
%     err = getErr(pn1,pn2,Pn1_w1,Pn2_w1,tR,Rt);
%     errA = getErr(pn1,pn2,Pn1_w1,Pn2_w1,RrA,RAt);
%     
%     if err < errA
%         R = tR;
%         T = Rt;
%     else
%         R = RrA;
%         T = RAt;
%     end
% 
    pn1 = [p1 p2];
    pn2 = [p2 p2(:,2:3) p2(:,1)];
    
    W2 = P2_w;
    W2(1,1) = aPointsN(1);
    W2(2,2) = aPointsN(2);
    W2(3,3) = aPointsN(3);
    
    P1N_w = [P1_w W2];
    P2N_w = [W2 W2(:,2:3) W2(:,1)];
    
    
    
%     [R, T, err] = SRPnLA3(pn1(1:2,:), pn2(1:2,:), P1N_w, P2N_w);
%     [R, T, err] = LPnL_Bar_ENull(pn1(1:2,:), pn2(1:2,:), P1N_w, P2N_w);
%     [R0, T0, err] = RSPnL_(pn1, pn2, P1N_w, P2N_w,IA); 
%     [R,T] = invertRT(R0,T0);
%     if err > errA
%         R = tR;
%         T = Rt;
%     end
%     [R0, T0, err] = RSPnL_(pn1, pn2, P1N_w, P2N_w); 
%     [R,T] = invertRT(R0,T0);
    
    
    
   