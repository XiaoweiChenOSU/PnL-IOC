% Author :  Ping Wang                                                        
% Contact:  pingwangsky@gmail.com 
% This programe is implemented in matlab 2018a
% License:  Copyright (c) 2019 Ping Wang, All rights reserved       
% Address:  College of Electrical and Information Engineering, Lanzhou University of Technology              
% My site:  https://sites.google.com/view/ping-wang-homepage  

function [ R,T,minErr] = TSRPnLMCMC(p1,p2,W1,W2,cH,Rr)

        minErr = Inf;
        nLine = length(p1);
        p1 = [p1; ones(1,nLine)];
        p2 = [p2; ones(1,nLine)];
        [Rt,aPoints] = CalculateTbyR(p1, p2, W1, W2, cH, Rr);

        pn1 = [p1 p2];
        pn2 = [p2 p2(:,2:3) p2(:,1)];
        tempW2 = W2;
        tempW2(1,1) = aPoints(1);
        tempW2(2,2) = aPoints(2);
        tempW2(3,3) = aPoints(3);
        Pn1_w1 = [W1 tempW2];
        Pn2_w1 = [tempW2 tempW2(:,2:3) tempW2(:,1)];
        
        
        RrA = Rr;
%         for i = 1:10
%             
%         
%             [EEcH,GGcH,CCcH] = refineEquationForCH(pn1, pn2,Pn1_w1,Pn2_w1,cH);
% 
%             [solutioncH, errcH] = RefineGaussNewtonCH(Cayley(RrA),EEcH,GGcH);
% 
%             s1=solutioncH(1);
%             s2=solutioncH(2);
%             s3=solutioncH(3);
%             sr=[1,s1,s2,s3,s1^2,s1*s2,s1*s3,s2^2,s2*s3,s3^2].';
%             factor=1/(1+s1^2+s2^2+s3^2); 
%             RrA=factor*[1+s1^2-s2^2-s3^2,2*s1*s2-2*s3,2*s2+2*s1*s3;
%                        2*s3+2*s1*s2,1-s1^2+s2^2-s3^2,2*s2*s3-2*s1;
%                        2*s1*s3-2*s2,2*s1+2*s2*s3,1-s1^2-s2^2+s3^2];
%                    
% %             RAt = factor*CCcH*sr;    
% 
% 
%             
%             [RAt,aPointsCh] = CalculateTbyR(p1, p2, W1, W2, cH, RrA);
%    
% %             [EEA,GGA,CCA] = refineEquationForPoints(pn1,pn2,Pn1_w1,Pn2_w1,solutioncH);
% %             [aPointsCh,~] = RefineForApoints(aPointsCh,EEA,GGA);
%             
%             tempW2 = W2;
%             tempW2(1,1) = aPointsCh(1);
%             tempW2(2,2) = aPointsCh(2);
%             tempW2(3,3) = aPointsCh(3);
%             Pn1_w1 = [W1 tempW2];
%             Pn2_w1 = [tempW2 tempW2(:,2:3) tempW2(:,1)]; 
%             [RrA, RAt, curErr] = SRPnL(pn1(1:2,:), pn2(1:2,:), Pn1_w1, Pn2_w1);
%             
%             if curErr < minErr
%                 R = RrA;
%                 T = RAt;
%                 minErr = curErr
%             end
%         end
        
        for i = 1:20
        
            [EEA,GGA,CCA] = CopyrefineEquation(p1,pn1,pn2,Pn1_w1,Pn2_w1,RrA,cH);
%             s = [Cayley(RrA);WT2(1,1);WT2(2,2);WT2(3,3)];
            s = [Cayley(RrA);aPoints];
            [solutionA, errA] = CopyRefineGaussNewtonForApoints(s,EEA,GGA);


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
                   
            RAt = factor*CCA*sr;   
            
            tempW2 = W2;
            tempW2(1,1) = solutionA(4);
            tempW2(2,2) = solutionA(5);
            tempW2(3,3) = solutionA(6);
            Pn1_w1 = [W1 tempW2];
            Pn2_w1 = [tempW2 tempW2(:,2:3) tempW2(:,1)];
            Pn1_w1 = [W1 tempW2];
            Pn2_w1 = [tempW2 tempW2(:,2:3) tempW2(:,1)]; 
            [RrA, RAt, curErr] = SRPnL(pn1(1:2,:), pn2(1:2,:), Pn1_w1, Pn2_w1);
            
            if curErr < minErr
                R = RrA;
                T = RAt;
                minErr = curErr
            end
                   
%             curErr2 = cal_rotation_err(RrA,R_truth);
        end

%         RAt = factor*CCA*sr;    

%         curErr2 = cal_rotation_err(RrA,R_truth);
        
%         tempW2(1,1) = solutionA(4);
%         tempW2(2,2) = solutionA(5);
%         tempW2(3,3) = solutionA(6);
%         Pn1_w1 = [W1 tempW2];
%         Pn2_w1 = [tempW2 tempW2(:,2:3) tempW2(:,1)];

                      

end

