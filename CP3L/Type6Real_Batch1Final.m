close all;
clc;
clear;


addpath('SRPnL');
addpath('others');
addpath('TypeReal61MultiGA');
addpath('TypeReal62MultiGA');
addpath('TypeReal63MultiGA');
addpath('TypeReal64MultiGA');
addpath('TypeReal65MultiGA');
addpath('TypeReal66MultiGA');
addpath('TypeReal67MultiGA');

load('type7GroundTruth1.mat');
[~,numImage] = size(groundTruth);

for j = 1:numImage

    %Type6 Room Camera Pose Estimation
    
    Rtruth = groundTruth(j).Rotation;
    Ttruth = groundTruth(j).Translation;
    p = groundTruth(j).point;
    P = groundTruth(j).wpoint;
    C_truth = -inv(Rtruth)*Ttruth;
    A = groundTruth(j).intrinsics_matrix;
    cH = C_truth(2);
    p1 = [p(1,:);p(2,:)];
    p2 = [p(3,:);p(4,:)];
    
    
    Lheight = groundTruth(j).Lheight;

%     Lheight =groundTruth(j).Lwidth;
    Lwidth = groundTruth(j).Lwidth;
    
%     Lheight = groundTruth(j).Lheight;

%     Lheight = 4;
    P1_w = [P(1,:);P(2,:)];
    P2_w = [P(3,:);P(4,:)];
    P_W1 = [P(1,:);P(4,:)];
    p_w1 = [p(1,:);p(4,:)];
    P_W2 = [P(3,:);P(2,:)];
    p_w2 = [p(3,:);p(2,:)];
    P_W3 = [P(1,:);P(3,:)];
    p_w3 = [p(1,:);p(3,:)];
    P_W4 = [P(2,:);P(4,:)];
    p_w4 = [p(2,:);p(4,:)];








    %     
    T1p1 = [p1(1,:);p2(1,:);p1(1,:);p1(2,:);p1(1,:);p2(1,:)];
    T1p2 = [p1(2,:);p2(2,:);p2(1,:);p2(2,:);p2(2,:);p1(2,:)];

    T1p1A = [p1(1,:);p2(1,:);p1(1,:)];
    T1p2A = [p1(2,:);p2(2,:);p2(2,:)];

    T1p1B = [p1(1,:);p2(1,:);p1(2,:)];
    T1p2B = [p1(2,:);p2(2,:);p2(1,:)];

    T1p1C = [p1(1,:);p2(1,:);p1(1,:)];
    T1p2C = [p1(2,:);p2(2,:);p2(1,:)];

    T1p1D = [p1(1,:);p2(1,:);p1(2,:)];
    T1p2D = [p1(2,:);p2(2,:);p2(2,:)];


    for i = 1:length(T1p1)
        temp1 = inv(A)*[T1p1(i,:) 1].';
        temp1 = (temp1/temp1(3)).';
        ip1(i,:) = temp1(:,1:2);
        temp2 = inv(A)*[T1p2(i,:) 1].';
        temp2 = (temp2/temp2(3)).';
        ip2(i,:) = temp2(:,1:2);
    end

    for i = 1:length(T1p1A)
        temp1 = inv(A)*[T1p1A(i,:) 1].';
        temp1 = (temp1/temp1(3)).';
        ip1A(i,:) = temp1(:,1:2);
        temp2 = inv(A)*[T1p2A(i,:) 1].';
        temp2 = (temp2/temp2(3)).';
        ip2A(i,:) = temp2(:,1:2);
    end

    for i = 1:length(T1p1B)
        temp1 = inv(A)*[T1p1B(i,:) 1].';
        temp1 = (temp1/temp1(3)).';
        ip1B(i,:) = temp1(:,1:2);
        temp2 = inv(A)*[T1p2B(i,:) 1].';
        temp2 = (temp2/temp2(3)).';
        ip2B(i,:) = temp2(:,1:2);
    end

    for i = 1:length(T1p1C)
        temp1 = inv(A)*[T1p1C(i,:) 1].';
        temp1 = (temp1/temp1(3)).';
        ip1C(i,:) = temp1(:,1:2);
        temp2 = inv(A)*[T1p2C(i,:) 1].';
        temp2 = (temp2/temp2(3)).';
        ip2C(i,:) = temp2(:,1:2);
    end

    for i = 1:length(T1p1D)
        temp1 = inv(A)*[T1p1D(i,:) 1].';
        temp1 = (temp1/temp1(3)).';
        ip1D(i,:) = temp1(:,1:2);
        temp2 = inv(A)*[T1p2D(i,:) 1].';
        temp2 = (temp2/temp2(3)).';
        ip2D(i,:) = temp2(:,1:2);
    end


    pn1 = ip1A(1:2,:)';
    pn2 = ip2A(1:2,:)';




    T1P1_w = [P1_w(1,:);P2_w(1,:);P1_w(1,:);P1_w(2,:);P1_w(1,:);P2_w(1,:)];
    T1P2_w = [P1_w(2,:);P2_w(2,:);P2_w(1,:);P2_w(2,:);P2_w(2,:);P1_w(2,:)];
    T1P1_wA = [P1_w(1,:);P2_w(1,:);P1_w(1,:)];
    T1P2_wA = [P1_w(2,:);P2_w(2,:);P2_w(2,:)];
%     T1P1_wB = [P1_w(1,:);P2_w(1,:);P1_w(2,:)];
%     T1P2_wB = [P1_w(2,:);P2_w(2,:);P2_w(1,:)];
%     T1P1_wC = [P1_w(1,:);P2_w(1,:);P1_w(1,:)];
%     T1P2_wC = [P1_w(2,:);P2_w(2,:);P2_w(1,:)];
%     T1P1_wD = [P1_w(1,:);P2_w(1,:);P1_w(2,:)];
%     T1P2_wD = [P1_w(2,:);P2_w(2,:);P2_w(2,:)];
%     

%     L1 = (P1_w(2,:)- P1_w(1,:))/norm(P1_w(2,:)- P1_w(1,:));
%     L2 = (P2_w(2,:)- P2_w(1,:))/norm(P2_w(2,:)- P2_w(1,:));
%     T1P1_wA = [P1_w(1,:);P2_w(1,:);P1_w(1,:)];
%     T1P2_wA = [P1_w(1,:)+L1;P2_w(1,:)+L2;P2_w(2,:)];
% % 
%     [RA, TA, errA] = SRPnLReal61(ip1A', ip2A', T1P1_wA', T1P2_wA', C_truth);
%     [RB, TB, errB] = SRPnLReal61(ip1B', ip2B', T1P1_wB', T1P2_wB', C_truth);
%     [RC, TC, errC] = SRPnLReal61(ip1C', ip2C', T1P1_wC', T1P2_wC', C_truth);
%     [RD, TD, errD] = SRPnLReal61(ip1D', ip2D', T1P1_wD', T1P2_wD', C_truth);
    [RE1, TE1, errE1] = SRPnLReal(ip1', ip2', T1P1_w', T1P2_w',C_truth);

    
    W1 = T1P1_wA(1:2,:)';
    W2 = T1P2_wA(1:2,:)';


    p = [pn1 pn2];
    nLine = length(p);
    W = [W1 W2];
    p = [p; ones(1,nLine)];
    
    [~,n] = max(abs(W2(:,1) - W1(:,1)));

    [RE2, TE2, errE2] = SRPnLReal61(ip1A', ip2A', T1P1_wA', T1P2_wA',C_truth,n);


    clearvars apoint aPointsN aPointsNt aPointsNs apointsIni apointsInis apointsInit apointsIniu apointsIniv aPointsNu aPointsNv T1P1_wA T1P2_wA T1P1_wB T1P2_wB T1P1_wC T1P2_wC T1P1_wD T1P2_wD Rf Tf Rtf Ttf Rsf Tsf 
    tic;
    count = 1; 
    apoint = [-1,-1,-1,-1];

    T1P1_wA = [P1_w(1,:);P2_w(1,:);P1_w(1,:)];
    T1P2_wA = [P1_w(2,:);P2_w(2,:);P2_w(2,:)];

    T1P1_wB = [P1_w(1,:);P2_w(1,:);P1_w(2,:)];
    T1P2_wB = [P1_w(2,:);P2_w(2,:);P2_w(1,:)];

    [F1, G] = P2LMGA61(T1p1A, T1p2A, T1P1_wA, T1P2_wA, T1p1B, T1p2B, T1P1_wB, T1P2_wB, A, C_truth, Lheight);
    errAmin = Inf;
    errBmin = Inf;
    errmin = Inf;
    for i = 1:length(F1)
        if F1(i).ValObjective(1) > 1 || F1(i).ValObjective(2) > 1 || F1(i).ValObjective(3) > 10 || F1(i).ValObjective(4) > 10
    %         if F1(i).ValObjective(1) + F1(i).ValObjective(2) > 2
             continue
        end
        if sum(F1(i).ValObjective)< errmin 
            apoint(1) = F1(i).Val(1);
            apoint(2) = F1(i).Val(2);
            apoint(3) = F1(i).Val(3);
            apoint(4) = F1(i).Val(4);
            RAt = F1(i).RA;
            TAt = F1(i).TA;
            RBt = F1(i).RB;
            TBt = F1(i).TB;
            errmin = sum(F1(i).ValObjective);
            errAmin = F1(i).ValObjective(1);
            errBmin = F1(i).ValObjective(2);
        end
%         if F1(i).ValObjective(1) < errAmin 
%             apoint(1) = F1(i).Val(1);
%             apoint(2) = F1(i).Val(2);
%             RAt = F1(i).RA;
%             TAt = F1(i).TA;
%             errAmin = F1(i).ValObjective(1);
%         end 
%         if F1(i).ValObjective(2) < errBmin 
%             apoint(3) = F1(i).Val(3);
%             apoint(4) = F1(i).Val(4);
%             RBt = F1(i).RB;
%             TBt = F1(i).TB;
%             errBmin = F1(i).ValObjective(2);
%         end 
    end  

    
    
%     [Rt,aPoints] = CalculateTandIbyR(p1, p2, W1, W2, C_truth, RAt);

    [~,n] = max(abs(W2(:,1) - W1(:,1)));

%     L1 = (T1P2_wA(1,:)- T1P1_wA(1,:))/norm(T1P2_wA(1,:)- T1P1_wA(1,:));
%     L2 = (T1P2_wA(2,:)- T1P1_wA(2,:))/norm(T1P2_wA(2,:)- T1P1_wA(2,:));
    
%     I = imread('d3038501401941f8afe4e9e2df5eda73_i0_3.jpg');
% 
%     imshow(I); hold on;
    
    if n == 1
        if errAmin < errBmin
            [EE,GG,CC] = RefineEquA61A1(p, W);
            apointsIni = [apoint(1) apoint(4)]';
            solutionO = [Cayley(RAt);apointsIni]; 
            [solution, obj_cur] = RefineGaussNewtonRandApT61A1(solutionO, p, W, EE, GG, CC, Lheight);


            if size(solution) ~= 11
                aPointsN = solution(4:5);
                s1=solution(1); s2=solution(2); s3=solution(3); 
                s10 = solution(4); s11=solution(5); 
            else
                aPointsN = solution(10:11);
                s1=solution(1); s2=solution(2); s3=solution(3); 
                s10 = solution(10); s11=solution(11); 
            end
        else
            [EE,GG,CC] = RefineEquA61B1(p, W);
            apointsIni = [apoint(2) apoint(3)]';
            solutionO = [Cayley(RBt);apointsIni]; 
            [solution, obj_cur] = RefineGaussNewtonRandApT61B1(solutionO, p, W, EE, GG, CC, Lheight);

            if size(solution) ~= 11
                aPointsN = solution(4:5);
                s1=solution(1); s2=solution(2); s3=solution(3); 
                s10 = solution(4); s11=solution(5); 
            else
                aPointsN = solution(10:11);
                s1=solution(1); s2=solution(2); s3=solution(3); 
                s10 = solution(10); s11=solution(11); 
            end
        end
        s4=s1^2; s5=s1*s2; s6=s1*s3;
        s7=s2^2; s8=s2*s3; s9=s3^2;
        w=[1,s1,s2,s3,s4,s5,s6,s7,s8,s9,...
           s10,s10*s2,s10*s3,s10*s4,s10*s5,s10*s6,s10*s7,s10*s9,...
           s11,s11*s2,s11*s3,s11*s4,s11*s5,s11*s6,s11*s7,s11*s9,...
           s1^2,s1*s2,s1*s3,s2^2,s2*s3,s3^2]';
        factor=1/(1+s1^2+s2^2+s3^2);     
        Rf=factor*[1+s1^2-s2^2-s3^2,2*s1*s2-2*s3,2*s2+2*s1*s3;
                       2*s3+2*s1*s2,1-s1^2+s2^2-s3^2,2*s2*s3-2*s1;
                       2*s1*s3-2*s2,2*s1+2*s2*s3,1-s1^2-s2^2+s3^2];
        Tf =  factor*CC*w; 

        if errAmin < errBmin
            tempW = P_W1;
            tempW(1,n) = aPointsN(1);
            tempW(2,n) = aPointsN(2);
            Pa = tempW;
            p2a = p_w1;
            [errep710{j},Urep710{j}]=reprojection_error_usingRT(Pa,p2a,Rf,Tf,A);
%             Urep710 = cell2mat(Urep710);
%             line([Urep710(1,1),p1(2,1)],[Urep710(1,2),p1(2,2)],'Color','Red','LineWidth',3);
%             line([p2(1,1),Urep710(2,1)],[p2(1,2),Urep710(2,2)],'Color','Red','LineWidth',3);
        else
            tempW = P_W2;
            tempW(1,n) = aPointsN(1);
            tempW(2,n) = aPointsN(2);
            Pa = tempW;
            p2a = p_w2;
            [errep710{j},Urep710{j}]=reprojection_error_usingRT(Pa,p2a,Rf,Tf,A);
%             Urep710 = cell2mat(Urep710);
%             line([Urep710(2,1),p1(1,1)],[Urep710(2,2),p1(1,2)],'Color','Red','LineWidth',3);
%             line([p2(2,1),Urep710(1,1)],[p2(2,2),Urep710(1,2)],'Color','Red','LineWidth',3);
        end


        R71{j} = Rf;
        T71{j} = Tf;
        error71{j} = obj_cur; 
        errRf71{j} = cal_rotation_err(Rf,Rtruth);
        errTf71{j} = cal_translation_err(Tf,Ttruth);


        [EEt,GGt,CCt] = RefineEquA61A1(p, W);
        apointsInit = [apoint(1) apoint(4)]';
        solutionOt = [Cayley(RAt);apointsInit]; 
        [solutiont, obj_cur] = RefineGaussNewtonRandApT61A1(solutionOt, p, W, EEt, GGt, CCt, Lheight);


        if size(solutiont) ~= 11
            aPointsNt = solutiont(4:5);
            s1=solutiont(1); s2=solutiont(2); s3=solutiont(3); 
            s10 = solutiont(4); s11=solutiont(5); 
        else
            aPointsNt = solutiont(10:11);
            s1=solutiont(1); s2=solutiont(2); s3=solutiont(3); 
            s10 = solutiont(10); s11=solutiont(11); 
        end

        s4=s1^2; s5=s1*s2; s6=s1*s3;
        s7=s2^2; s8=s2*s3; s9=s3^2;
        w=[1,s1,s2,s3,s4,s5,s6,s7,s8,s9,...
           s10,s10*s2,s10*s3,s10*s4,s10*s5,s10*s6,s10*s7,s10*s9,...
           s11,s11*s2,s11*s3,s11*s4,s11*s5,s11*s6,s11*s7,s11*s9,...
           s1^2,s1*s2,s1*s3,s2^2,s2*s3,s3^2]';
        factor=1/(1+s1^2+s2^2+s3^2);     
        Rtf=factor*[1+s1^2-s2^2-s3^2,2*s1*s2-2*s3,2*s2+2*s1*s3;
                       2*s3+2*s1*s2,1-s1^2+s2^2-s3^2,2*s2*s3-2*s1;
                       2*s1*s3-2*s2,2*s1+2*s2*s3,1-s1^2-s2^2+s3^2];
        Ttf =  factor*CCt*w; 


        tempW = P_W1;
        tempW(1,n) = aPointsNt(1);
        tempW(2,n) = aPointsNt(2);
        Pa = tempW;
        p2a = p_w1;
        [errep711{j},Urep711{j}]=reprojection_error_usingRT(Pa,p2a,Rtf,Ttf,A);

        Rth71{j} = Rtf;
        Tth71{j} = Ttf;
        errortf71{j} = obj_cur; 
        errRtf71{j} = cal_rotation_err(Rtf,Rtruth);
        errTtf71{j} = cal_translation_err(Ttf,Ttruth);


  
        [EEs,GGs,CCs] = RefineEquA61B1(p, W);
        apointsInis = [apoint(2) apoint(3)]';
        solutionOs = [Cayley(RBt);apointsInis]; 
        [solutions, obj_cur] = RefineGaussNewtonRandApT61B1(solutionOs, p, W, EEs, GGs, CCs, Lheight);

        if size(solutions) ~= 11
            aPointsNs = solutions(4:5);
            s1=solutions(1); s2=solutions(2); s3=solutions(3); 
            s10 = solutions(4); s11=solutions(5); 
        else
            aPointsNs = solutions(10:11);
            s1=solutions(1); s2=solutions(2); s3=solutions(3); 
            s10 = solutions(10); s11=solutions(11); 
        end

        s4=s1^2; s5=s1*s2; s6=s1*s3;
        s7=s2^2; s8=s2*s3; s9=s3^2;
        w=[1,s1,s2,s3,s4,s5,s6,s7,s8,s9,...
           s10,s10*s2,s10*s3,s10*s4,s10*s5,s10*s6,s10*s7,s10*s9,...
           s11,s11*s2,s11*s3,s11*s4,s11*s5,s11*s6,s11*s7,s11*s9,...
           s1^2,s1*s2,s1*s3,s2^2,s2*s3,s3^2]';
        factor=1/(1+s1^2+s2^2+s3^2);     
        Rfs=factor*[1+s1^2-s2^2-s3^2,2*s1*s2-2*s3,2*s2+2*s1*s3;
                       2*s3+2*s1*s2,1-s1^2+s2^2-s3^2,2*s2*s3-2*s1;
                       2*s1*s3-2*s2,2*s1+2*s2*s3,1-s1^2-s2^2+s3^2];
        Tfs =  factor*CCs*w; 



        tempW = P_W2;
        tempW(1,n) = aPointsNs(1);
        tempW(2,n) = aPointsNs(2);
        Pa = tempW;
        p2a = p_w2;
        [errep712{j},Urep712{j}]=reprojection_error_usingRT(Pa,p2a,Rfs,Tfs,A);

        Rts71{j} = Rfs;
        Tts71{j} = Tfs;
        errortfs71{j} = obj_cur; 
        errRtfs71{j} = cal_rotation_err(Rfs,Rtruth);
        errTtfs71{j} = cal_translation_err(Tfs,Ttruth);
        
    elseif n == 2
        if errAmin < errBmin
            [EE,GG,CC] = RefineEquA61A2(p, W);
            apointsIni = [apoint(1) apoint(4)]';
            solutionO = [Cayley(RAt);apointsIni]; 
            [solution, obj_cur] = RefineGaussNewtonRandApT61A2(solutionO, p, W, EE, GG, CC, Lheight);


            if size(solution) ~= 11
                aPointsN = solution(4:5);
                s1=solution(1); s2=solution(2); s3=solution(3); 
                s10 = solution(4); s11=solution(5); 
            else
                aPointsN = solution(10:11);
                s1=solution(1); s2=solution(2); s3=solution(3); 
                s10 = solution(10); s11=solution(11); 
            end

        else
            [EE,GG,CC] = RefineEquA61B2(p, W);
            apointsIni = [apoint(2) apoint(3)]';
            solutionO = [Cayley(RBt);apointsIni]; 
            [solution, obj_cur] = RefineGaussNewtonRandApT61B2(solutionO, p, W, EE, GG, CC, Lheight);

            if size(solution) ~= 11
                aPointsN = solution(4:5);
                s1=solution(1); s2=solution(2); s3=solution(3); 
                s10 = solution(4); s11=solution(5); 
            else
                aPointsN = solution(10:11);
                s1=solution(1); s2=solution(2); s3=solution(3); 
                s10 = solution(10); s11=solution(11); 
            end
        end
        s4=s1^2; s5=s1*s2; s6=s1*s3;
        s7=s2^2; s8=s2*s3; s9=s3^2;
        w=[1,s1,s2,s3,s4,s5,s6,s7,s8,s9,...
           s10,s10*s1,s10*s3,s10*s4,s10*s5,s10*s7,s10*s8,s10*s9,...
           s11,s11*s1,s11*s3,s11*s4,s11*s5,s11*s7,s11*s8,s11*s9,...
           s1^2,s1*s2,s1*s3,s2^2,s2*s3,s3^2]';
        factor=1/(1+s1^2+s2^2+s3^2);     
        Rf=factor*[1+s1^2-s2^2-s3^2,2*s1*s2-2*s3,2*s2+2*s1*s3;
                       2*s3+2*s1*s2,1-s1^2+s2^2-s3^2,2*s2*s3-2*s1;
                       2*s1*s3-2*s2,2*s1+2*s2*s3,1-s1^2-s2^2+s3^2];
        Tf =  factor*CC*w;  
        if errAmin < errBmin
            tempW = P_W1;
            tempW(1,n) = aPointsN(1);
            tempW(2,n) = aPointsN(2);
            Pa = tempW;
            p2a = p_w1;
            [errep710{j},Urep710{j}]=reprojection_error_usingRT(Pa,p2a,Rf,Tf,A);
%             Urep710 = cell2mat(Urep710);
%             line([Urep710(1,1),p1(2,1)],[Urep710(1,2),p1(2,2)],'Color','Red','LineWidth',3);
%             line([p2(1,1),Urep710(2,1)],[p2(1,2),Urep710(2,2)],'Color','Red','LineWidth',3);
        else
            tempW = P_W2;
            tempW(1,n) = aPointsN(1);
            tempW(2,n) = aPointsN(2);
            Pa = tempW;
            p2a = p_w2;
            [errep710{j},Urep710{j}]=reprojection_error_usingRT(Pa,p2a,Rf,Tf,A);
%             Urep710 = cell2mat(Urep710);
%             line([Urep710(2,1),p1(1,1)],[Urep710(2,2),p1(1,2)],'Color','Red','LineWidth',3);
%             line([p2(2,1),Urep710(1,1)],[p2(2,2),Urep710(1,2)],'Color','Red','LineWidth',3);
        end


        R71{j} = Rf;
        T71{j} = Tf;
        error71{j} = obj_cur; 
        errRf71{j} = cal_rotation_err(Rf,Rtruth);
        errTf71{j} = cal_translation_err(Tf,Ttruth);


        [EEt,GGt,CCt] = RefineEquA61A2(p, W);
        apointsInit = [apoint(1) apoint(4)]';
        solutionOt = [Cayley(RAt);apointsInit]; 
        [solutiont, obj_cur] = RefineGaussNewtonRandApT61A2(solutionOt, p, W, EEt, GGt, CCt, Lheight);


        if size(solutiont) ~= 11
            aPointsNt = solutiont(4:5);
            s1=solutiont(1); s2=solutiont(2); s3=solutiont(3); 
            s10 = solutiont(4); s11=solutiont(5); 
        else
            aPointsNt = solutiont(10:11);
            s1=solutiont(1); s2=solutiont(2); s3=solutiont(3); 
            s10 = solutiont(10); s11=solutiont(11); 
        end

        s4=s1^2; s5=s1*s2; s6=s1*s3;
        s7=s2^2; s8=s2*s3; s9=s3^2;
        w=[1,s1,s2,s3,s4,s5,s6,s7,s8,s9,...
           s10,s10*s1,s10*s3,s10*s4,s10*s5,s10*s7,s10*s8,s10*s9,...
           s11,s11*s1,s11*s3,s11*s4,s11*s5,s11*s7,s11*s8,s11*s9,...
           s1^2,s1*s2,s1*s3,s2^2,s2*s3,s3^2]';
        factor=1/(1+s1^2+s2^2+s3^2);     
        Rtf=factor*[1+s1^2-s2^2-s3^2,2*s1*s2-2*s3,2*s2+2*s1*s3;
                       2*s3+2*s1*s2,1-s1^2+s2^2-s3^2,2*s2*s3-2*s1;
                       2*s1*s3-2*s2,2*s1+2*s2*s3,1-s1^2-s2^2+s3^2];
        Ttf =  factor*CCt*w;  


        tempW = P_W1;
        tempW(1,n) = aPointsNt(1);
        tempW(2,n) = aPointsNt(2);
        Pa = tempW;
        p2a = p_w1;
        [errep711{j},Urep711{j}]=reprojection_error_usingRT(Pa,p2a,Rtf,Ttf,A);

        Rth71{j} = Rtf;
        Tth71{j} = Ttf;
        errortf71{j} = obj_cur; 
        errRtf71{j} = cal_rotation_err(Rtf,Rtruth);
        errTtf71{j} = cal_translation_err(Ttf,Ttruth);


  
        [EEs,GGs,CCs] = RefineEquA61B2(p, W);
        apointsInis = [apoint(2) apoint(3)]';
        solutionOs = [Cayley(RBt);apointsInis]; 
        [solutions, obj_cur] = RefineGaussNewtonRandApT61B2(solutionOs, p, W, EEs, GGs, CCs, Lheight);

        if size(solutions) ~= 11
            aPointsNs = solutions(4:5);
            s1=solutions(1); s2=solutions(2); s3=solutions(3); 
            s10 = solutions(4); s11=solutions(5); 
        else
            aPointsNs = solutions(10:11);
            s1=solutions(1); s2=solutions(2); s3=solutions(3); 
            s10 = solutions(10); s11=solutions(11); 
        end

        s4=s1^2; s5=s1*s2; s6=s1*s3;
        s7=s2^2; s8=s2*s3; s9=s3^2;
        w=[1,s1,s2,s3,s4,s5,s6,s7,s8,s9,...
           s10,s10*s1,s10*s3,s10*s4,s10*s5,s10*s7,s10*s8,s10*s9,...
           s11,s11*s1,s11*s3,s11*s4,s11*s5,s11*s7,s11*s8,s11*s9,...
           s1^2,s1*s2,s1*s3,s2^2,s2*s3,s3^2]';
        factor=1/(1+s1^2+s2^2+s3^2);     
        Rfs=factor*[1+s1^2-s2^2-s3^2,2*s1*s2-2*s3,2*s2+2*s1*s3;
                       2*s3+2*s1*s2,1-s1^2+s2^2-s3^2,2*s2*s3-2*s1;
                       2*s1*s3-2*s2,2*s1+2*s2*s3,1-s1^2-s2^2+s3^2];
        Tfs =  factor*CCs*w;  



        tempW = P_W2;
        tempW(1,n) = aPointsNs(1);
        tempW(2,n) = aPointsNs(2);
        Pa = tempW;
        p2a = p_w2;
        [errep712{j},Urep712{j}]=reprojection_error_usingRT(Pa,p2a,Rfs,Tfs,A);

        Rts71{j} = Rfs;
        Tts71{j} = Tfs;
        errortfs71{j} = obj_cur; 
        errRtfs71{j} = cal_rotation_err(Rfs,Rtruth);
        errTtfs71{j} = cal_translation_err(Tfs,Ttruth);
    elseif n == 3
        if errAmin < errBmin
            [EE,GG,CC] = RefineEquA61A3(p, W);
            apointsIni = [apoint(1) apoint(4)]';
            solutionO = [Cayley(RAt);apointsIni]; 
            [solution, obj_cur] = RefineGaussNewtonRandApT61A3(solutionO, p, W, EE, GG, CC, Lheight);


            if size(solution) ~= 11
                aPointsN = solution(4:5);
                s1=solution(1); s2=solution(2); s3=solution(3); 
                s10 = solution(4); s11=solution(5); 
            else
                aPointsN = solution(10:11);
                s1=solution(1); s2=solution(2); s3=solution(3); 
                s10 = solution(10); s11=solution(11); 
            end
        else
            [EE,GG,CC] = RefineEquA61B3(p, W);
            apointsIni = [apoint(2) apoint(3)]';
            solutionO = [Cayley(RBt);apointsIni]; 
            [solution, obj_cur] = RefineGaussNewtonRandApT61B3(solutionO, p, W, EE, GG, CC, Lheight);

            if size(solution) ~= 11
                aPointsN = solution(4:5);
                s1=solution(1); s2=solution(2); s3=solution(3); 
                s10 = solution(4); s11=solution(5); 
            else
                aPointsN = solution(10:11);
                s1=solution(1); s2=solution(2); s3=solution(3); 
                s10 = solution(10); s11=solution(11); 
            end
        end

        s4=s1^2; s5=s1*s2; s6=s1*s3;
        s7=s2^2; s8=s2*s3; s9=s3^2;
        w=[1,s1,s2,s3,s4,s5,s6,s7,s8,s9,...
           s10,s10*s1,s10*s2,s10*s4,s10*s6,s10*s7,s10*s8,s10*s9,...
           s11,s11*s1,s11*s2,s11*s4,s11*s6,s11*s7,s11*s8,s11*s9,...
           s1^2,s1*s2,s1*s3,s2^2,s2*s3,s3^2]';
        factor=1/(1+s1^2+s2^2+s3^2);     
        Rf=factor*[1+s1^2-s2^2-s3^2,2*s1*s2-2*s3,2*s2+2*s1*s3;
                       2*s3+2*s1*s2,1-s1^2+s2^2-s3^2,2*s2*s3-2*s1;
                       2*s1*s3-2*s2,2*s1+2*s2*s3,1-s1^2-s2^2+s3^2];
        Tf =  factor*CC*w; 
        


        if errAmin < errBmin
            tempW = P_W1;
            tempW(1,n) = aPointsN(1);
            tempW(2,n) = aPointsN(2);
            Pa = tempW;
            p2a = p_w1;
            [errep710{j},Urep710{j}]=reprojection_error_usingRT(Pa,p2a,Rf,Tf,A);
%             Urep710 = cell2mat(Urep710);
%             line([Urep710(1,1),p1(2,1)],[Urep710(1,2),p1(2,2)],'Color','Red','LineWidth',3);
%             line([p2(1,1),Urep710(2,1)],[p2(1,2),Urep710(2,2)],'Color','Red','LineWidth',3);

        else
            tempW = P_W2;
            tempW(1,n) = aPointsN(1);
            tempW(2,n) = aPointsN(2);
            Pa = tempW;
            p2a = p_w2;
            [errep710{j},Urep710{j}]=reprojection_error_usingRT(Pa,p2a,Rf,Tf,A);
%             Urep710 = cell2mat(Urep710);
%             line([Urep710(2,1),p1(1,1)],[Urep710(2,2),p1(1,2)],'Color','Red','LineWidth',3);
%             line([p2(2,1),Urep710(1,1)],[p2(2,2),Urep710(1,2)],'Color','Red','LineWidth',3);

        end

      
        
        R71{j} = Rf;
        T71{j} = Tf;
        error71{j} = obj_cur; 
        errRf71{j} = cal_rotation_err(Rf,Rtruth);
        errTf71{j} = cal_translation_err(Tf,Ttruth);


        [EEt,GGt,CCt] = RefineEquA61A3(p, W);
        apointsInit = [apoint(1) apoint(4)]';
        solutionOt = [Cayley(RAt);apointsInit]; 
        [solutiont, obj_cur] = RefineGaussNewtonRandApT61A3(solutionOt, p, W, EEt, GGt, CCt, Lheight);


        if size(solutiont) ~= 11
            aPointsNt = solutiont(4:5);
            s1=solutiont(1); s2=solutiont(2); s3=solutiont(3); 
            s10 = solutiont(4); s11=solutiont(5); 
        else
            aPointsNt = solutiont(10:11);
            s1=solutiont(1); s2=solutiont(2); s3=solutiont(3); 
            s10 = solutiont(10); s11=solutiont(11); 
        end

        s4=s1^2; s5=s1*s2; s6=s1*s3;
        s7=s2^2; s8=s2*s3; s9=s3^2;
        w=[1,s1,s2,s3,s4,s5,s6,s7,s8,s9,...
           s10,s10*s1,s10*s2,s10*s4,s10*s6,s10*s7,s10*s8,s10*s9,...
           s11,s11*s1,s11*s2,s11*s4,s11*s6,s11*s7,s11*s8,s11*s9,...
           s1^2,s1*s2,s1*s3,s2^2,s2*s3,s3^2]';
        factor=1/(1+s1^2+s2^2+s3^2);     
        Rtf=factor*[1+s1^2-s2^2-s3^2,2*s1*s2-2*s3,2*s2+2*s1*s3;
                       2*s3+2*s1*s2,1-s1^2+s2^2-s3^2,2*s2*s3-2*s1;
                       2*s1*s3-2*s2,2*s1+2*s2*s3,1-s1^2-s2^2+s3^2];
        Ttf =  factor*CCt*w;  


        tempW = P_W1;
        tempW(1,n) = aPointsNt(1);
        tempW(2,n) = aPointsNt(2);
        Pa = tempW;
        p2a = p_w1;
        [errep711{j},Urep711{j}]=reprojection_error_usingRT(Pa,p2a,Rtf,Ttf,A);

        Rth71{j} = Rtf;
        Tth71{j} = Ttf;
        errortf71{j} = obj_cur; 
        errRtf71{j} = cal_rotation_err(Rtf,Rtruth);
        errTtf71{j} = cal_translation_err(Ttf,Ttruth);


  
        [EEs,GGs,CCs] = RefineEquA61B3(p, W);
        apointsInis = [apoint(2) apoint(3)]';
        solutionOs = [Cayley(RBt);apointsInis]; 
        [solutions, obj_cur] = RefineGaussNewtonRandApT61B3(solutionOs, p, W, EEs, GGs, CCs, Lheight);

        if size(solutions) ~= 11
            aPointsNs = solutions(4:5);
            s1=solutions(1); s2=solutions(2); s3=solutions(3); 
            s10 = solutions(4); s11=solutions(5); 
        else
            aPointsNs = solutions(10:11);
            s1=solutions(1); s2=solutions(2); s3=solutions(3); 
            s10 = solutions(10); s11=solutions(11); 
        end

        s4=s1^2; s5=s1*s2; s6=s1*s3;
        s7=s2^2; s8=s2*s3; s9=s3^2;
        w=[1,s1,s2,s3,s4,s5,s6,s7,s8,s9,...
           s10,s10*s1,s10*s2,s10*s4,s10*s6,s10*s7,s10*s8,s10*s9,...
           s11,s11*s1,s11*s2,s11*s4,s11*s6,s11*s7,s11*s8,s11*s9,...
           s1^2,s1*s2,s1*s3,s2^2,s2*s3,s3^2]';
        factor=1/(1+s1^2+s2^2+s3^2);     
        Rfs=factor*[1+s1^2-s2^2-s3^2,2*s1*s2-2*s3,2*s2+2*s1*s3;
                       2*s3+2*s1*s2,1-s1^2+s2^2-s3^2,2*s2*s3-2*s1;
                       2*s1*s3-2*s2,2*s1+2*s2*s3,1-s1^2-s2^2+s3^2];
        Tfs =  factor*CCs*w; 

        tempW = P_W2;
        tempW(1,n) = aPointsNs(1);
        tempW(2,n) = aPointsNs(2);
        Pa = tempW;
        p2a = p_w2;
        [errep712{j},Urep712{j}]=reprojection_error_usingRT(Pa,p2a,Rfs,Tfs,A);

        Rts71{j} = Rfs;
        Tts71{j} = Tfs;
        errortfs71{j} = obj_cur; 
        errRtfs71{j} = cal_rotation_err(Rfs,Rtruth);
        errTtfs71{j} = cal_translation_err(Tfs,Ttruth);
    end
    
    toc;
    time71{j} = toc; 



    
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    clearvars apointa apointb aPointsN aPointsNt aPointsNs apointsIni apointsInis apointsInit apointsIniu apointsIniv aPointsNu aPointsNv T1P1_wA T1P2_wA T1P1_wB T1P2_wB T1P1_wC T1P2_wC T1P1_wD T1P2_wD Rf Tf Rtf Ttf Rfs Tfs
    tic;

    T1P1_wA = [P1_w(1,:);P2_w(1,:);P1_w(1,:)];
    T1P2_wA = [P1_w(2,:);P2_w(2,:);P2_w(2,:)];

    T1P1_wC = [P1_w(1,:);P2_w(1,:);P1_w(1,:)];
    T1P2_wC = [P1_w(2,:);P2_w(2,:);P2_w(1,:)];

    [F1, G] = P2LMGA62(T1p1A, T1p2A, T1P1_wA, T1P2_wA, T1p1C, T1p2C, T1P1_wC, T1P2_wC, cH, A, C_truth, Lheight);
    errAmin = Inf;
    errBmin = Inf;
    apointa = [-1,-1,-1];
    apointb = [-1,-1,-1];
    for i = 1:length(F1)
        if F1(i).ValObjective(1) > 1 || F1(i).ValObjective(2) > 1 || F1(i).ValObjective(3) > 10 || F1(i).ValObjective(4) > 10
    %         if F1(i).ValObjective(1) + F1(i).ValObjective(2)> 2
             continue
        end
%             apoints(count,:) =  F1(i).Val;   
%             count = count + 1;
        if F1(i).ValObjective(1) < errAmin 
            apointa(1) = F1(i).Val(1);
            apointa(3) = F1(i).Val(3);
            RAt = F1(i).RA;
            TAt = F1(i).TA;
            errAmin = F1(i).ValObjective(1);
        end
        if F1(i).ValObjective(2) < errBmin 
            apointb(1) = F1(i).Val(1);
            apointb(2) = F1(i).Val(2);
            RBt = F1(i).RB;
            TBt = F1(i).TB;
            errBmin = F1(i).ValObjective(2);
        end
    end  
    
    
    [~,n] = max(abs(W2(:,1) - W1(:,1)));

    L1 = (T1P2_wA(1,:)- T1P1_wA(1,:))/norm(T1P2_wA(1,:)- T1P1_wA(1,:));
    L2 = (T1P2_wA(2,:)- T1P1_wA(2,:))/norm(T1P2_wA(2,:)- T1P1_wA(2,:));
    
    if n == 1
        if errAmin < errBmin

            [EE,GG,CC] = RefineEquA62A1(p, W);
            apointsIni = [apointa(1) apointa(3)]';
            solutionO = [Cayley(RAt);apointsIni]; 
            [solution, obj_cur] = RefineGaussNewtonRandApT62A1(solutionO, p, W, EE, GG, CC, Lheight);


            if size(solution) ~= 11
                aPointsN = solution(4:5);
                s1=solution(1); s2=solution(2); s3=solution(3); 
                s10 = solution(4); s11=solution(5); 
            else
                aPointsN = solution(10:11);
                s1=solution(1); s2=solution(2); s3=solution(3); 
                s10 = solution(10); s11=solution(11); 
            end 
        else
            [EE,GG,CC] = RefineEquA62B1(p, W);
            apointsIni = [apointb(1) apointb(2)]';
            solutionO = [Cayley(RBt);apointsIni]; 
            [solution, obj_cur] = RefineGaussNewtonRandApT62B1(solutionO, p, W, EE, GG, CC, Lheight);

            if size(solution) ~= 11
                aPointsN = solution(4:5);
                s1=solution(1); s2=solution(2); s3=solution(3); 
                s10 = solution(4); s11=solution(5); 
            else
                aPointsN = solution(10:11);
                s1=solution(1); s2=solution(2); s3=solution(3); 
                s10 = solution(10); s11=solution(11); 
            end 
        end

        s4=s1^2; s5=s1*s2; s6=s1*s3;
        s7=s2^2; s8=s2*s3; s9=s3^2;
        w=[1,s1,s2,s3,s4,s5,s6,s7,s8,s9,...
           s10,s10*s2,s10*s3,s10*s4,s10*s5,s10*s6,s10*s7,s10*s9,...
           s11,s11*s2,s11*s3,s11*s4,s11*s5,s11*s6,s11*s7,s11*s9,...
           s1^2,s1*s2,s1*s3,s2^2,s2*s3,s3^2]';
        factor=1/(1+s1^2+s2^2+s3^2);     
        Rf=factor*[1+s1^2-s2^2-s3^2,2*s1*s2-2*s3,2*s2+2*s1*s3;
                       2*s3+2*s1*s2,1-s1^2+s2^2-s3^2,2*s2*s3-2*s1;
                       2*s1*s3-2*s2,2*s1+2*s2*s3,1-s1^2-s2^2+s3^2];
        Tf =  factor*CC*w; 

        if errAmin < errBmin
            tempW = P_W1;
            tempW(1,n) = aPointsN(1);
            tempW(2,n) = aPointsN(2);
            Pa = tempW;
            p2a = p_w1;
            [errep720{j},Urep720{j}]=reprojection_error_usingRT(Pa,p2a,Rf,Tf,A);
        else
            tempW = P_W3;
            tempW(1,n) = aPointsN(1);
            tempW(2,n) = aPointsN(2);
            Pa = tempW;
            p2a = p_w3;
            [errep720{j},Urep720{j}]=reprojection_error_usingRT(Pa,p2a,Rf,Tf,A);
        end


        R72{j} = Rf;
        T72{j} = Tf;
        error72{j} = obj_cur; 
        errRf72{j} = cal_rotation_err(Rf,Rtruth);
        errTf72{j} = cal_translation_err(Tf,Ttruth);


        [EEt,GGt,CCt] = RefineEquA62A1(p, W);
        apointsInit = [apointa(1) apointa(3)]';
        solutionOt = [Cayley(RAt);apointsInit]; 
        [solutiont, obj_curt] = RefineGaussNewtonRandApT62A1(solutionOt, p, W, EEt, GGt, CCt, Lheight);


        if size(solutiont) ~= 11
            aPointsNt = solutiont(4:5);
            s1=solutiont(1); s2=solutiont(2); s3=solutiont(3); 
            s10 = solutiont(4); s11=solutiont(5); 
        else
            aPointsNt = solutiont(10:11);
            s1=solutiont(1); s2=solutiont(2); s3=solutiont(3); 
            s10 = solutiont(10); s11=solutiont(11); 
        end


        s4=s1^2; s5=s1*s2; s6=s1*s3;
        s7=s2^2; s8=s2*s3; s9=s3^2;
        w=[1,s1,s2,s3,s4,s5,s6,s7,s8,s9,...
           s10,s10*s2,s10*s3,s10*s4,s10*s5,s10*s6,s10*s7,s10*s9,...
           s11,s11*s2,s11*s3,s11*s4,s11*s5,s11*s6,s11*s7,s11*s9,...
           s1^2,s1*s2,s1*s3,s2^2,s2*s3,s3^2]';
        factor=1/(1+s1^2+s2^2+s3^2);     
        Rtf=factor*[1+s1^2-s2^2-s3^2,2*s1*s2-2*s3,2*s2+2*s1*s3;
                       2*s3+2*s1*s2,1-s1^2+s2^2-s3^2,2*s2*s3-2*s1;
                       2*s1*s3-2*s2,2*s1+2*s2*s3,1-s1^2-s2^2+s3^2];
        Ttf =  factor*CCt*w; 

        tempW = P_W1;
        tempW(1,n) = aPointsNt(1);
        tempW(2,n) = aPointsNt(2);
        Pa = tempW;
        p2a = p_w1;
        [errep721{j},Urep721{j}]=reprojection_error_usingRT(Pa,p2a,Rtf,Ttf,A);

        Rth72{j} = Rtf;
        Tth72{j} = Ttf;
        errortf72{j} = obj_cur; 
        errRtf72{j} = cal_rotation_err(Rtf,Rtruth);
        errTtf72{j} = cal_translation_err(Ttf,Ttruth);



        [EEs,GGs,CCs] = RefineEquA62B1(p, W);
        apointsInis = [apointb(1) apointb(2)]';
        solutionOs = [Cayley(RBt);apointsInis]; 
        [solutions, obj_cur] = RefineGaussNewtonRandApT62B1(solutionOs, p, W, EEs, GGs, CCs, Lheight);

        if size(solutions) ~= 11
            aPointsNs = solutions(4:5);
            s1=solutions(1); s2=solutions(2); s3=solutions(3); 
            s10 = solutions(4); s11=solutions(5); 
        else
            aPointsNs = solutions(10:11);
            s1=solutions(1); s2=solutions(2); s3=solutions(3); 
            s10 = solutions(10); s11=solutions(11); 
        end 

        s4=s1^2; s5=s1*s2; s6=s1*s3;
        s7=s2^2; s8=s2*s3; s9=s3^2;
        w=[1,s1,s2,s3,s4,s5,s6,s7,s8,s9,...
           s10,s10*s2,s10*s3,s10*s4,s10*s5,s10*s6,s10*s7,s10*s9,...
           s11,s11*s2,s11*s3,s11*s4,s11*s5,s11*s6,s11*s7,s11*s9,...
           s1^2,s1*s2,s1*s3,s2^2,s2*s3,s3^2]';
        factor=1/(1+s1^2+s2^2+s3^2);     
        Rfs=factor*[1+s1^2-s2^2-s3^2,2*s1*s2-2*s3,2*s2+2*s1*s3;
                       2*s3+2*s1*s2,1-s1^2+s2^2-s3^2,2*s2*s3-2*s1;
                       2*s1*s3-2*s2,2*s1+2*s2*s3,1-s1^2-s2^2+s3^2];
        Tfs =  factor*CCs*w; 


        tempW = P_W3;
        tempW(1,n) = aPointsNs(1);
        tempW(2,n) = aPointsNs(2);
        Pa = tempW;
        p2a = p_w3;
        [errep722{j},Urep722{j}]=reprojection_error_usingRT(Pa,p2a,Rfs,Tfs,A);

        Rts72{j} = Rfs;
        Tts72{j} = Tfs;
        errortfs72{j} = obj_cur; 
        errRtfs72{j} = cal_rotation_err(Rfs,Rtruth);
        errTtfs72{j} = cal_translation_err(Tfs,Ttruth);
    elseif n ==2
        if errAmin < errBmin

            [EE,GG,CC] = RefineEquA62A2(p, W);
            apointsIni = [apointa(1) apointa(3)]';
            solutionO = [Cayley(RAt);apointsIni]; 
            [solution, obj_cur] = RefineGaussNewtonRandApT62A2(solutionO, p, W, EE, GG, CC, Lheight);


            if size(solution) ~= 11
                aPointsN = solution(4:5);
                s1=solution(1); s2=solution(2); s3=solution(3); 
                s10 = solution(4); s11=solution(5); 
            else
                aPointsN = solution(10:11);
                s1=solution(1); s2=solution(2); s3=solution(3); 
                s10 = solution(10); s11=solution(11); 
            end 
        else
            [EE,GG,CC] = RefineEquA62B2(p, W);
            apointsIni = [apointb(1) apointb(2)]';
            solutionO = [Cayley(RBt);apointsIni]; 
            [solution, obj_cur] = RefineGaussNewtonRandApT62B2(solutionO, p, W, EE, GG, CC, Lheight);

            if size(solution) ~= 11
                aPointsN = solution(4:5);
                s1=solution(1); s2=solution(2); s3=solution(3); 
                s10 = solution(4); s11=solution(5); 
            else
                aPointsN = solution(10:11);
                s1=solution(1); s2=solution(2); s3=solution(3); 
                s10 = solution(10); s11=solution(11); 
            end 
        end

        s4=s1^2; s5=s1*s2; s6=s1*s3;
        s7=s2^2; s8=s2*s3; s9=s3^2;
        w=[1,s1,s2,s3,s4,s5,s6,s7,s8,s9,...
           s10,s10*s1,s10*s3,s10*s4,s10*s5,s10*s7,s10*s8,s10*s9,...
           s11,s11*s1,s11*s3,s11*s4,s11*s5,s11*s7,s11*s8,s11*s9,...
           s1^2,s1*s2,s1*s3,s2^2,s2*s3,s3^2]';
        factor=1/(1+s1^2+s2^2+s3^2);     
        Rf=factor*[1+s1^2-s2^2-s3^2,2*s1*s2-2*s3,2*s2+2*s1*s3;
                       2*s3+2*s1*s2,1-s1^2+s2^2-s3^2,2*s2*s3-2*s1;
                       2*s1*s3-2*s2,2*s1+2*s2*s3,1-s1^2-s2^2+s3^2];
        Tf =  factor*CC*w; 


        if errAmin < errBmin
            tempW = P_W1;
            tempW(1,n) = aPointsN(1);
            tempW(2,n) = aPointsN(2);
            Pa = tempW;
            p2a = p_w1;
            [errep720{j},Urep720{j}]=reprojection_error_usingRT(Pa,p2a,Rf,Tf,A);
        else
            tempW = P_W3;
            tempW(1,n) = aPointsN(1);
            tempW(2,n) = aPointsN(2);
            Pa = tempW;
            p2a = p_w3;
            [errep720{j},Urep720{j}]=reprojection_error_usingRT(Pa,p2a,Rf,Tf,A);
        end


        R72{j} = Rf;
        T72{j} = Tf;
        error72{j} = obj_cur; 
        errRf72{j} = cal_rotation_err(Rf,Rtruth);
        errTf72{j} = cal_translation_err(Tf,Ttruth);


        [EEt,GGt,CCt] = RefineEquA62A2(p, W);
        apointsInit = [apointa(1) apointa(3)]';
        solutionOt = [Cayley(RAt);apointsInit]; 
        [solutiont, obj_curt] = RefineGaussNewtonRandApT62A2(solutionOt, p, W, EEt, GGt, CCt, Lheight);


        if size(solutiont) ~= 11
            aPointsNt = solutiont(4:5);
            s1=solutiont(1); s2=solutiont(2); s3=solutiont(3); 
            s10 = solutiont(4); s11=solutiont(5); 
        else
            aPointsNt = solutiont(10:11);
            s1=solutiont(1); s2=solutiont(2); s3=solutiont(3); 
            s10 = solutiont(10); s11=solutiont(11); 
        end

        s4=s1^2; s5=s1*s2; s6=s1*s3;
        s7=s2^2; s8=s2*s3; s9=s3^2;
        w=[1,s1,s2,s3,s4,s5,s6,s7,s8,s9,...
           s10,s10*s1,s10*s3,s10*s4,s10*s5,s10*s7,s10*s8,s10*s9,...
           s11,s11*s1,s11*s3,s11*s4,s11*s5,s11*s7,s11*s8,s11*s9,...
           s1^2,s1*s2,s1*s3,s2^2,s2*s3,s3^2]';
        factor=1/(1+s1^2+s2^2+s3^2);     
        Rtf=factor*[1+s1^2-s2^2-s3^2,2*s1*s2-2*s3,2*s2+2*s1*s3;
                       2*s3+2*s1*s2,1-s1^2+s2^2-s3^2,2*s2*s3-2*s1;
                       2*s1*s3-2*s2,2*s1+2*s2*s3,1-s1^2-s2^2+s3^2];
        Ttf =  factor*CCt*w; 

        tempW = P_W1;
        tempW(1,n) = aPointsNt(1);
        tempW(2,n) = aPointsNt(2);
        Pa = tempW;
        p2a = p_w1;
        [errep721{j},Urep721{j}]=reprojection_error_usingRT(Pa,p2a,Rtf,Ttf,A);

        Rth72{j} = Rtf;
        Tth72{j} = Ttf;
        errortf72{j} = obj_cur; 
        errRtf72{j} = cal_rotation_err(Rtf,Rtruth);
        errTtf72{j} = cal_translation_err(Ttf,Ttruth);



        [EEs,GGs,CCs] = RefineEquA62B2(p, W);
        apointsInis = [apointb(1) apointb(2)]';
        solutionOs = [Cayley(RBt);apointsInis]; 
        [solutions, obj_cur] = RefineGaussNewtonRandApT62B2(solutionOs, p, W, EEs, GGs, CCs, Lheight);

        if size(solutions) ~= 11
            aPointsNs = solutions(4:5);
            s1=solutions(1); s2=solutions(2); s3=solutions(3); 
            s10 = solutions(4); s11=solutions(5); 
        else
            aPointsNs = solutions(10:11);
            s1=solutions(1); s2=solutions(2); s3=solutions(3); 
            s10 = solutions(10); s11=solutions(11); 
        end  

        s4=s1^2; s5=s1*s2; s6=s1*s3;
        s7=s2^2; s8=s2*s3; s9=s3^2;
        w=[1,s1,s2,s3,s4,s5,s6,s7,s8,s9,...
           s10,s10*s1,s10*s3,s10*s4,s10*s5,s10*s7,s10*s8,s10*s9,...
           s11,s11*s1,s11*s3,s11*s4,s11*s5,s11*s7,s11*s8,s11*s9,...
           s1^2,s1*s2,s1*s3,s2^2,s2*s3,s3^2]';
        factor=1/(1+s1^2+s2^2+s3^2);     
        Rfs=factor*[1+s1^2-s2^2-s3^2,2*s1*s2-2*s3,2*s2+2*s1*s3;
                       2*s3+2*s1*s2,1-s1^2+s2^2-s3^2,2*s2*s3-2*s1;
                       2*s1*s3-2*s2,2*s1+2*s2*s3,1-s1^2-s2^2+s3^2];
        Tfs =  factor*CCs*w; 


        tempW = P_W3;
        tempW(1,n) = aPointsNs(1);
        tempW(2,n) = aPointsNs(2);
        Pa = tempW;
        p2a = p_w3;
        [errep722{j},Urep722{j}]=reprojection_error_usingRT(Pa,p2a,Rfs,Tfs,A);

        Rts72{j} = Rfs;
        Tts72{j} = Tfs;
        errortfs72{j} = obj_cur; 
        errRtfs72{j} = cal_rotation_err(Rfs,Rtruth);
        errTtfs72{j} = cal_translation_err(Tfs,Ttruth);
    elseif n ==3
        if errAmin < errBmin

            [EE,GG,CC] = RefineEquA62A3(p, W);
            apointsIni = [apointa(1) apointa(3)]';
            solutionO = [Cayley(RAt);apointsIni]; 
            [solution, obj_cur] = RefineGaussNewtonRandApT62A3(solutionO, p, W, EE, GG, CC, Lheight);


            if size(solution) ~= 11
                aPointsN = solution(4:5);
                s1=solution(1); s2=solution(2); s3=solution(3); 
                s10 = solution(4); s11=solution(5); 
            else
                aPointsN = solution(10:11);
                s1=solution(1); s2=solution(2); s3=solution(3); 
                s10 = solution(10); s11=solution(11); 
            end 
        else
            [EE,GG,CC] = RefineEquA62B3(p, W);
            apointsIni = [apointb(1) apointb(2)]';
            solutionO = [Cayley(RBt);apointsIni]; 
            [solution, obj_cur] = RefineGaussNewtonRandApT62B3(solutionO, p, W, EE, GG, CC, Lheight);

            if size(solution) ~= 11
                aPointsN = solution(4:5);
                s1=solution(1); s2=solution(2); s3=solution(3); 
                s10 = solution(4); s11=solution(5); 
            else
                aPointsN = solution(10:11);
                s1=solution(1); s2=solution(2); s3=solution(3); 
                s10 = solution(10); s11=solution(11); 
            end 
        end

        s4=s1^2; s5=s1*s2; s6=s1*s3;
        s7=s2^2; s8=s2*s3; s9=s3^2;
        w=[1,s1,s2,s3,s4,s5,s6,s7,s8,s9,...
           s10,s10*s1,s10*s2,s10*s4,s10*s6,s10*s7,s10*s8,s10*s9,...
           s11,s11*s1,s11*s2,s11*s4,s11*s6,s11*s7,s11*s8,s11*s9,...
           s1^2,s1*s2,s1*s3,s2^2,s2*s3,s3^2]';
        factor=1/(1+s1^2+s2^2+s3^2);     
        Rf=factor*[1+s1^2-s2^2-s3^2,2*s1*s2-2*s3,2*s2+2*s1*s3;
                       2*s3+2*s1*s2,1-s1^2+s2^2-s3^2,2*s2*s3-2*s1;
                       2*s1*s3-2*s2,2*s1+2*s2*s3,1-s1^2-s2^2+s3^2];
        Tf =  factor*CC*w; 


        if errAmin < errBmin
            tempW = P_W1;
            tempW(1,n) = aPointsN(1);
            tempW(2,n) = aPointsN(2);
            Pa = tempW;
            p2a = p_w1;
            [errep720{j},Urep720{j}]=reprojection_error_usingRT(Pa,p2a,Rf,Tf,A);
        else
            tempW = P_W3;
            tempW(1,n) = aPointsN(1);
            tempW(2,n) = aPointsN(2);
            Pa = tempW;
            p2a = p_w3;
            [errep720{j},Urep720{j}]=reprojection_error_usingRT(Pa,p2a,Rf,Tf,A);
        end

        R72{j} = Rf;
        T72{j} = Tf;
        error72{j} = obj_cur; 
        errRf72{j} = cal_rotation_err(Rf,Rtruth);
        errTf72{j} = cal_translation_err(Tf,Ttruth);


        [EEt,GGt,CCt] = RefineEquA62A3(p, W);
        apointsInit = [apointa(1) apointa(3)]';
        solutionOt = [Cayley(RAt);apointsInit]; 
        [solutiont, obj_curt] = RefineGaussNewtonRandApT62A3(solutionOt, p, W, EEt, GGt, CCt, Lheight);


        if size(solutiont) ~= 11
            aPointsNt = solutiont(4:5);
            s1=solutiont(1); s2=solutiont(2); s3=solutiont(3); 
            s10 = solutiont(4); s11=solutiont(5); 
        else
            aPointsNt = solutiont(10:11);
            s1=solutiont(1); s2=solutiont(2); s3=solutiont(3); 
            s10 = solutiont(10); s11=solutiont(11); 
        end

        s4=s1^2; s5=s1*s2; s6=s1*s3;
        s7=s2^2; s8=s2*s3; s9=s3^2;
        w=[1,s1,s2,s3,s4,s5,s6,s7,s8,s9,...
           s10,s10*s1,s10*s2,s10*s4,s10*s6,s10*s7,s10*s8,s10*s9,...
           s11,s11*s1,s11*s2,s11*s4,s11*s6,s11*s7,s11*s8,s11*s9,...
           s1^2,s1*s2,s1*s3,s2^2,s2*s3,s3^2]';
        factor=1/(1+s1^2+s2^2+s3^2);     
        Rtf=factor*[1+s1^2-s2^2-s3^2,2*s1*s2-2*s3,2*s2+2*s1*s3;
                       2*s3+2*s1*s2,1-s1^2+s2^2-s3^2,2*s2*s3-2*s1;
                       2*s1*s3-2*s2,2*s1+2*s2*s3,1-s1^2-s2^2+s3^2];
        Ttf =  factor*CCt*w; 

        tempW = P_W1;
        tempW(1,n) = aPointsNt(1);
        tempW(2,n) = aPointsNt(2);
        Pa = tempW;
        p2a = p_w1;
        [errep721{j},Urep721{j}]=reprojection_error_usingRT(Pa,p2a,Rtf,Ttf,A);

        Rth72{j} = Rtf;
        Tth72{j} = Ttf;
        errortf72{j} = obj_cur; 
        errRtf72{j} = cal_rotation_err(Rtf,Rtruth);
        errTtf72{j} = cal_translation_err(Ttf,Ttruth);



        [EEs,GGs,CCs] = RefineEquA62B3(p, W);
        apointsInis = [apointb(1) apointb(2)]';
        solutionOs = [Cayley(RBt);apointsInis]; 
        [solutions, obj_cur] = RefineGaussNewtonRandApT62B3(solutionOs, p, W, EEs, GGs, CCs, Lheight);

        if size(solutions) ~= 11
            aPointsNs = solutions(4:5);
            s1=solutions(1); s2=solutions(2); s3=solutions(3); 
            s10 = solutions(4); s11=solutions(5); 
        else
            aPointsNs = solutions(10:11);
            s1=solutions(1); s2=solutions(2); s3=solutions(3); 
            s10 = solutions(10); s11=solutions(11); 
        end  

        s4=s1^2; s5=s1*s2; s6=s1*s3;
        s7=s2^2; s8=s2*s3; s9=s3^2;
        w=[1,s1,s2,s3,s4,s5,s6,s7,s8,s9,...
           s10,s10*s1,s10*s2,s10*s4,s10*s6,s10*s7,s10*s8,s10*s9,...
           s11,s11*s1,s11*s2,s11*s4,s11*s6,s11*s7,s11*s8,s11*s9,...
           s1^2,s1*s2,s1*s3,s2^2,s2*s3,s3^2]';
        factor=1/(1+s1^2+s2^2+s3^2);     
        Rfs=factor*[1+s1^2-s2^2-s3^2,2*s1*s2-2*s3,2*s2+2*s1*s3;
                       2*s3+2*s1*s2,1-s1^2+s2^2-s3^2,2*s2*s3-2*s1;
                       2*s1*s3-2*s2,2*s1+2*s2*s3,1-s1^2-s2^2+s3^2];
        Tfs =  factor*CCs*w; 


        tempW = P_W3;
        tempW(1,n) = aPointsNs(1);
        tempW(2,n) = aPointsNs(2);
        Pa = tempW;
        p2a = p_w3;
        [errep722{j},Urep722{j}]=reprojection_error_usingRT(Pa,p2a,Rfs,Tfs,A);

        Rts72{j} = Rfs;
        Tts72{j} = Tfs;
        errortfs72{j} = obj_cur; 
        errRtfs72{j} = cal_rotation_err(Rfs,Rtruth);
        errTtfs72{j} = cal_translation_err(Tfs,Ttruth);
    end
    
    toc;
    time72{j} = toc; 


    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    clearvars  apointa apointb aPointsN aPointsNt aPointsNs apointsIni apointsInis apointsInit apointsIniu apointsIniv aPointsNu aPointsNv T1P1_wA T1P2_wA T1P1_wB T1P2_wB T1P1_wC T1P2_wC T1P1_wD T1P2_wD Rf Tf Rtf Ttf Rfs Tfs
    tic;
    T1P1_wA = [P1_w(1,:);P2_w(1,:);P1_w(1,:)];
    T1P2_wA = [P1_w(2,:);P2_w(2,:);P2_w(2,:)];


    T1P1_wD = [P1_w(1,:);P2_w(1,:);P1_w(2,:)];
    T1P2_wD = [P1_w(2,:);P2_w(2,:);P2_w(2,:)];

    [F1, G] = P2LMGA63(T1p1A, T1p2A, T1P1_wA, T1P2_wA, T1p1D, T1p2D, T1P1_wD, T1P2_wD, cH, A, C_truth, Lheight);
    errAmin = Inf;
    errBmin = Inf;
    apointa = [-1,-1,-1];
    apointb = [-1,-1,-1];
    for i = 1:length(F1)
%         if F1(i).ValObjective(1) > 5 || F1(i).ValObjective(2) > 5 || F1(i).ValObjective(3) > 50 || F1(i).ValObjective(4) > 50
%     %         if F1(i).ValObjective(1) + F1(i).ValObjective(2) > 2
%             continue
%         end
    %         apoints(count,:) =  F1(i).Val;   
    %         count = count + 1;
        if F1(i).ValObjective(1) < errAmin 
            apointa(1) = F1(i).Val(1);
            apointa(3) = F1(i).Val(3);
            RAt = F1(i).RA;
            TAt = F1(i).TA;
            errAmin = F1(i).ValObjective(1);
        end
        if F1(i).ValObjective(2) < errBmin 
            apointb(2) = F1(i).Val(2);
            apointb(3) = F1(i).Val(3);
            RBt = F1(i).RB;
            TBt = F1(i).TB;
            errBmin = F1(i).ValObjective(2);
        end 
    end  

    [~,n] = max(abs(W2(:,1) - W1(:,1)));
    
    if n == 1
        if errAmin < errBmin

            [EE,GG,CC] = RefineEquA63A1(p, W);
            apointsIni = [apointa(1) apointa(3)]';
            solutionO = [Cayley(RAt);apointsIni]; 
            [solution, obj_cur] = RefineGaussNewtonRandApT63A1(solutionO, p, W, EE, GG, CC, Lheight);


            if size(solution) ~= 11
                aPointsN = solution(4:5);
                s1=solution(1); s2=solution(2); s3=solution(3); 
                s10 = solution(4); s11=solution(5); 
            else
                aPointsN = solution(10:11);
                s1=solution(1); s2=solution(2); s3=solution(3); 
                s10 = solution(10); s11=solution(11); 
            end 
        else
            [EE,GG,CC] = RefineEquA63B1(p, W);
            apointsIni = [apointb(2) apointb(3)]';
            solutionO = [Cayley(RBt);apointsIni]; 
            [solution, obj_cur] = RefineGaussNewtonRandApT63B1(solutionO, p, W, EE, GG, CC, Lheight);

            if size(solution) ~= 11
                aPointsN = solution(4:5);
                s1=solution(1); s2=solution(2); s3=solution(3); 
                s10 = solution(4); s11=solution(5); 
            else
                aPointsN = solution(10:11);
                s1=solution(1); s2=solution(2); s3=solution(3); 
                s10 = solution(10); s11=solution(11); 
            end  
        end
        
        s4=s1^2; s5=s1*s2; s6=s1*s3;
        s7=s2^2; s8=s2*s3; s9=s3^2;
        w=[1,s1,s2,s3,s4,s5,s6,s7,s8,s9,...
           s10,s10*s2,s10*s3,s10*s4,s10*s5,s10*s6,s10*s7,s10*s9,...
           s11,s11*s2,s11*s3,s11*s4,s11*s5,s11*s6,s11*s7,s11*s9,...
           s1^2,s1*s2,s1*s3,s2^2,s2*s3,s3^2]';
        factor=1/(1+s1^2+s2^2+s3^2);     
        Rf=factor*[1+s1^2-s2^2-s3^2,2*s1*s2-2*s3,2*s2+2*s1*s3;
                       2*s3+2*s1*s2,1-s1^2+s2^2-s3^2,2*s2*s3-2*s1;
                       2*s1*s3-2*s2,2*s1+2*s2*s3,1-s1^2-s2^2+s3^2];
        Tf =  factor*CC*w;

        if errAmin < errBmin
            tempW = P_W1;
            tempW(1,n) = aPointsN(1);
            tempW(2,n) = aPointsN(2);
            Pa = tempW;
            p2a = p_w1;
            [errep730{j},Urep730{j}]=reprojection_error_usingRT(Pa,p2a,Rf,Tf,A);
        else
            tempW = P_W4;
            tempW(1,n) = aPointsN(1);
            tempW(2,n) = aPointsN(2);
            Pa = tempW;
            p2a = p_w4;
            [errep730{j},Urep730{j}]=reprojection_error_usingRT(Pa,p2a,Rf,Tf,A);
        end


        R73{j} = Rf;
        T73{j} = Tf;
        error73{j} = obj_cur; 
        errRf73{j} = cal_rotation_err(Rf,Rtruth);
        errTf73{j} = cal_translation_err(Tf,Ttruth);



        [EEt,GGt,CCt] = RefineEquA63A1(p, W);
        apointsInit = [apointa(1) apointa(3)]';
        solutionOt = [Cayley(RAt);apointsInit]; 
        [solutiont, obj_cur] = RefineGaussNewtonRandApT63A1(solutionOt, p, W, EEt, GGt, CCt, Lheight);


        if size(solutiont) ~= 11
            aPointsNt = solutiont(4:5);
            s1=solutiont(1); s2=solutiont(2); s3=solutiont(3); 
            s10 = solutiont(4); s11=solutiont(5); 
        else
            aPointsNt = solutiont(10:11);
            s1=solutiont(1); s2=solutiont(2); s3=solutiont(3); 
            s10 = solutiont(10); s11=solutiont(11); 
        end

        s4=s1^2; s5=s1*s2; s6=s1*s3;
        s7=s2^2; s8=s2*s3; s9=s3^2;
        w=[1,s1,s2,s3,s4,s5,s6,s7,s8,s9,...
           s10,s10*s2,s10*s3,s10*s4,s10*s5,s10*s6,s10*s7,s10*s9,...
           s11,s11*s2,s11*s3,s11*s4,s11*s5,s11*s6,s11*s7,s11*s9,...
           s1^2,s1*s2,s1*s3,s2^2,s2*s3,s3^2]';
        factor=1/(1+s1^2+s2^2+s3^2);     
        Rtf=factor*[1+s1^2-s2^2-s3^2,2*s1*s2-2*s3,2*s2+2*s1*s3;
                       2*s3+2*s1*s2,1-s1^2+s2^2-s3^2,2*s2*s3-2*s1;
                       2*s1*s3-2*s2,2*s1+2*s2*s3,1-s1^2-s2^2+s3^2];
        Ttf =  factor*CCt*w;            
    

        tempW = P_W1;
        tempW(1,n) = aPointsNt(1);
        tempW(2,n) = aPointsNt(2);
        Pa = tempW;
        p2a = p_w1;
        [errep731{j},Urep731{j}]=reprojection_error_usingRT(Pa,p2a,Rtf,Ttf,A);

        Rth73{j} = Rtf;
        Tth73{j} = Ttf;
        errortf73{j} = obj_cur; 
        errRtf73{j} = cal_rotation_err(Rtf,Rtruth);
        errTtf73{j} = cal_translation_err(Ttf,Ttruth);



        [EEs,GGs,CCs] = RefineEquA63B1(p, W);
        apointsInis = [apointb(2) apointb(3)]';
        solutionOs = [Cayley(RBt);apointsInis]; 
        [solutions, obj_cur] = RefineGaussNewtonRandApT63B1(solutionOs, p, W, EEs, GGs, CCs, Lheight);

        if size(solutions) ~= 11
            aPointsNs = solutions(4:5);
            s1=solutions(1); s2=solutions(2); s3=solutions(3); 
            s10 = solutions(4); s11=solutions(5); 
        else
            aPointsNs = solutions(10:11);
            s1=solutions(1); s2=solutions(2); s3=solutions(3); 
            s10 = solutions(10); s11=solutions(11); 
        end  

        s4=s1^2; s5=s1*s2; s6=s1*s3;
        s7=s2^2; s8=s2*s3; s9=s3^2;
        w=[1,s1,s2,s3,s4,s5,s6,s7,s8,s9,...
           s10,s10*s2,s10*s3,s10*s4,s10*s5,s10*s6,s10*s7,s10*s9,...
           s11,s11*s2,s11*s3,s11*s4,s11*s5,s11*s6,s11*s7,s11*s9,...
           s1^2,s1*s2,s1*s3,s2^2,s2*s3,s3^2]';
        factor=1/(1+s1^2+s2^2+s3^2);     
        Rfs=factor*[1+s1^2-s2^2-s3^2,2*s1*s2-2*s3,2*s2+2*s1*s3;
                       2*s3+2*s1*s2,1-s1^2+s2^2-s3^2,2*s2*s3-2*s1;
                       2*s1*s3-2*s2,2*s1+2*s2*s3,1-s1^2-s2^2+s3^2];
        Tfs =  factor*CCs*w;  

        tempW = P_W4;
        tempW(1,n) = aPointsNs(1);
        tempW(2,n) = aPointsNs(2);
        Pa = tempW;
        p2a = p_w4;
        [errep732{j},Urep732{j}]=reprojection_error_usingRT(Pa,p2a,Rfs,Tfs,A);

        Rts73{j} = Rfs;
        Tts73{j} = Tfs;
        errortfs73{j} = obj_cur; 
        errRtfs73{j} = cal_rotation_err(Rfs,Rtruth);
        errTtfs73{j} = cal_translation_err(Tfs,Ttruth);
        
    elseif n ==2        
        if errAmin < errBmin

            [EE,GG,CC] = RefineEquA63A2(p, W);
            apointsIni = [apointa(1) apointa(3)]';
            solutionO = [Cayley(RAt);apointsIni]; 
            [solution, obj_cur] = RefineGaussNewtonRandApT63A2(solutionO, p, W, EE, GG, CC, Lheight);


            if size(solution) ~= 11
                aPointsN = solution(4:5);
                s1=solution(1); s2=solution(2); s3=solution(3); 
                s10 = solution(4); s11=solution(5); 
            else
                aPointsN = solution(10:11);
                s1=solution(1); s2=solution(2); s3=solution(3); 
                s10 = solution(10); s11=solution(11); 
            end 
        else
            [EE,GG,CC] = RefineEquA63B2(p, W);
            apointsIni = [apointb(2) apointb(3)]';
            solutionO = [Cayley(RBt);apointsIni]; 
            [solution, obj_cur] = RefineGaussNewtonRandApT63B2(solutionO, p, W, EE, GG, CC, Lheight);

            if size(solution) ~= 11
                aPointsN = solution(4:5);
                s1=solution(1); s2=solution(2); s3=solution(3); 
                s10 = solution(4); s11=solution(5); 
            else
                aPointsN = solution(10:11);
                s1=solution(1); s2=solution(2); s3=solution(3); 
                s10 = solution(10); s11=solution(11); 
            end  
        end
        
        s4=s1^2; s5=s1*s2; s6=s1*s3;
        s7=s2^2; s8=s2*s3; s9=s3^2;
        w=[1,s1,s2,s3,s4,s5,s6,s7,s8,s9,...
           s10,s10*s1,s10*s3,s10*s4,s10*s5,s10*s7,s10*s8,s10*s9,...
           s11,s11*s1,s11*s3,s11*s4,s11*s5,s11*s7,s11*s8,s11*s9,...
           s1^2,s1*s2,s1*s3,s2^2,s2*s3,s3^2]';
        factor=1/(1+s1^2+s2^2+s3^2);     
        Rf=factor*[1+s1^2-s2^2-s3^2,2*s1*s2-2*s3,2*s2+2*s1*s3;
                       2*s3+2*s1*s2,1-s1^2+s2^2-s3^2,2*s2*s3-2*s1;
                       2*s1*s3-2*s2,2*s1+2*s2*s3,1-s1^2-s2^2+s3^2];
        Tf =  factor*CC*w;

        if errAmin < errBmin
            tempW = P_W1;
            tempW(1,n) = aPointsN(1);
            tempW(2,n) = aPointsN(2);
            Pa = tempW;
            p2a = p_w1;
            [errep730{j},Urep730{j}]=reprojection_error_usingRT(Pa,p2a,Rf,Tf,A);
        else
            tempW = P_W4;
            tempW(1,n) = aPointsN(1);
            tempW(2,n) = aPointsN(2);
            Pa = tempW;
            p2a = p_w4;
            [errep730{j},Urep730{j}]=reprojection_error_usingRT(Pa,p2a,Rf,Tf,A);
        end
 

        R73{j} = Rf;
        T73{j} = Tf;
        error73{j} = obj_cur; 
        errRf73{j} = cal_rotation_err(Rf,Rtruth);
        errTf73{j} = cal_translation_err(Tf,Ttruth);



        [EEt,GGt,CCt] = RefineEquA63A2(p, W);
        apointsInit = [apointa(1) apointa(3)]';
        solutionOt = [Cayley(RAt);apointsInit]; 
        [solutiont, obj_cur] = RefineGaussNewtonRandApT63A2(solutionOt, p, W, EEt, GGt, CCt, Lheight);


        if size(solutiont) ~= 11
            aPointsNt = solutiont(4:5);
            s1=solutiont(1); s2=solutiont(2); s3=solutiont(3); 
            s10 = solutiont(4); s11=solutiont(5); 
        else
            aPointsNt = solutiont(10:11);
            s1=solutiont(1); s2=solutiont(2); s3=solutiont(3); 
            s10 = solutiont(10); s11=solutiont(11); 
        end

        s4=s1^2; s5=s1*s2; s6=s1*s3;
        s7=s2^2; s8=s2*s3; s9=s3^2;
        w=[1,s1,s2,s3,s4,s5,s6,s7,s8,s9,...
           s10,s10*s1,s10*s3,s10*s4,s10*s5,s10*s7,s10*s8,s10*s9,...
           s11,s11*s1,s11*s3,s11*s4,s11*s5,s11*s7,s11*s8,s11*s9,...
           s1^2,s1*s2,s1*s3,s2^2,s2*s3,s3^2]';
        factor=1/(1+s1^2+s2^2+s3^2);     
        Rtf=factor*[1+s1^2-s2^2-s3^2,2*s1*s2-2*s3,2*s2+2*s1*s3;
                       2*s3+2*s1*s2,1-s1^2+s2^2-s3^2,2*s2*s3-2*s1;
                       2*s1*s3-2*s2,2*s1+2*s2*s3,1-s1^2-s2^2+s3^2];
        Ttf =  factor*CCt*w;            
    

        tempW = P_W1;
        tempW(1,n) = aPointsNt(1);
        tempW(2,n) = aPointsNt(2);
        Pa = tempW;
        p2a = p_w1;
        [errep731{j},Urep731{j}]=reprojection_error_usingRT(Pa,p2a,Rtf,Ttf,A);

        Rth73{j} = Rtf;
        Tth73{j} = Ttf;
        errortf73{j} = obj_cur; 
        errRtf73{j} = cal_rotation_err(Rtf,Rtruth);
        errTtf73{j} = cal_translation_err(Ttf,Ttruth);



        [EEs,GGs,CCs] = RefineEquA63B2(p, W);
        apointsInis = [apointb(2) apointb(3)]';
        solutionOs = [Cayley(RBt);apointsInis]; 
        [solutions, obj_cur] = RefineGaussNewtonRandApT63B2(solutionOs, p, W, EEs, GGs, CCs, Lheight);

        if size(solutions) ~= 11
            aPointsNs = solutions(4:5);
            s1=solutions(1); s2=solutions(2); s3=solutions(3); 
            s10 = solutions(4); s11=solutions(5); 
        else
            aPointsNs = solutions(10:11);
            s1=solutions(1); s2=solutions(2); s3=solutions(3); 
            s10 = solutions(10); s11=solutions(11); 
        end  

        s4=s1^2; s5=s1*s2; s6=s1*s3;
        s7=s2^2; s8=s2*s3; s9=s3^2;
        w=[1,s1,s2,s3,s4,s5,s6,s7,s8,s9,...
           s10,s10*s1,s10*s3,s10*s4,s10*s5,s10*s7,s10*s8,s10*s9,...
           s11,s11*s1,s11*s3,s11*s4,s11*s5,s11*s7,s11*s8,s11*s9,...
           s1^2,s1*s2,s1*s3,s2^2,s2*s3,s3^2]';
        factor=1/(1+s1^2+s2^2+s3^2);     
        Rfs=factor*[1+s1^2-s2^2-s3^2,2*s1*s2-2*s3,2*s2+2*s1*s3;
                       2*s3+2*s1*s2,1-s1^2+s2^2-s3^2,2*s2*s3-2*s1;
                       2*s1*s3-2*s2,2*s1+2*s2*s3,1-s1^2-s2^2+s3^2];
        Tfs =  factor*CCs*w;  

        tempW = P_W4;
        tempW(1,n) = aPointsNs(1);
        tempW(2,n) = aPointsNs(2);
        Pa = tempW;
        p2a = p_w4;
        [errep732{j},Urep732{j}]=reprojection_error_usingRT(Pa,p2a,Rfs,Tfs,A);

        Rts73{j} = Rfs;
        Tts73{j} = Tfs;
        errortfs73{j} = obj_cur; 
        errRtfs73{j} = cal_rotation_err(Rfs,Rtruth);
        errTtfs73{j} = cal_translation_err(Tfs,Ttruth);
    elseif n ==3
        if errAmin < errBmin

            [EE,GG,CC] = RefineEquA63A3(p, W);
            apointsIni = [apointa(1) apointa(3)]';
            solutionO = [Cayley(RAt);apointsIni]; 
            [solution, obj_cur] = RefineGaussNewtonRandApT63A2(solutionO, p, W, EE, GG, CC, Lheight);


            if size(solution) ~= 11
                aPointsN = solution(4:5);
                s1=solution(1); s2=solution(2); s3=solution(3); 
                s10 = solution(4); s11=solution(5); 
            else
                aPointsN = solution(10:11);
                s1=solution(1); s2=solution(2); s3=solution(3); 
                s10 = solution(10); s11=solution(11); 
            end 
        else
            [EE,GG,CC] = RefineEquA63B3(p, W);
            [~,s2] = size(apointb);
            if s2 ~= 3
                apointsIni = [0 0]';
            else
                apointsIni = [apointb(2) apointb(3)]';
            end          
            solutionO = [Cayley(RBt);apointsIni]; 
            [solution, obj_cur] = RefineGaussNewtonRandApT63B2(solutionO, p, W, EE, GG, CC, Lheight);

            if size(solution) ~= 11
                aPointsN = solution(4:5);
                s1=solution(1); s2=solution(2); s3=solution(3); 
                s10 = solution(4); s11=solution(5); 
            else
                aPointsN = solution(10:11);
                s1=solution(1); s2=solution(2); s3=solution(3); 
                s10 = solution(10); s11=solution(11); 
            end  
        end
        
        s4=s1^2; s5=s1*s2; s6=s1*s3;
        s7=s2^2; s8=s2*s3; s9=s3^2;
        w=[1,s1,s2,s3,s4,s5,s6,s7,s8,s9,...
           s10,s10*s1,s10*s2,s10*s4,s10*s6,s10*s7,s10*s8,s10*s9,...
           s11,s11*s1,s11*s2,s11*s4,s11*s6,s11*s7,s11*s8,s11*s9,...
           s1^2,s1*s2,s1*s3,s2^2,s2*s3,s3^2]';
        factor=1/(1+s1^2+s2^2+s3^2);     
        Rf=factor*[1+s1^2-s2^2-s3^2,2*s1*s2-2*s3,2*s2+2*s1*s3;
                       2*s3+2*s1*s2,1-s1^2+s2^2-s3^2,2*s2*s3-2*s1;
                       2*s1*s3-2*s2,2*s1+2*s2*s3,1-s1^2-s2^2+s3^2];
        Tf =  factor*CC*w;

        if errAmin < errBmin
            tempW = P_W1;
            tempW(1,n) = aPointsN(1);
            tempW(2,n) = aPointsN(2);
            Pa = tempW;
            p2a = p_w1;
            [errep730{j},Urep730{j}]=reprojection_error_usingRT(Pa,p2a,Rf,Tf,A);
        else
            tempW = P_W4;
            tempW(1,n) = aPointsN(1);
            tempW(2,n) = aPointsN(2);
            Pa = tempW;
            p2a = p_w4;
            [errep730{j},Urep730{j}]=reprojection_error_usingRT(Pa,p2a,Rf,Tf,A);
        end


        R73{j} = Rf;
        T73{j} = Tf;
        error73{j} = obj_cur; 
        errRf73{j} = cal_rotation_err(Rf,Rtruth);
        errTf73{j} = cal_translation_err(Tf,Ttruth);



        [EEt,GGt,CCt] = RefineEquA63A3(p, W);
        [~,sa] = size(apointa);
        if sa ~= 3
            apointsInit = [0 0]';
        else
            apointsInit = [apointa(1) apointa(3)]';
        end 
        solutionOt = [Cayley(RAt);apointsInit]; 
        [solutiont, obj_cur] = RefineGaussNewtonRandApT63A3(solutionOt, p, W, EEt, GGt, CCt, Lheight);


        if size(solutiont) ~= 11
            aPointsNt = solutiont(4:5);
            s1=solutiont(1); s2=solutiont(2); s3=solutiont(3); 
            s10 = solutiont(4); s11=solutiont(5); 
        else
            aPointsNt = solutiont(10:11);
            s1=solutiont(1); s2=solutiont(2); s3=solutiont(3); 
            s10 = solutiont(10); s11=solutiont(11); 
        end

        s4=s1^2; s5=s1*s2; s6=s1*s3;
        s7=s2^2; s8=s2*s3; s9=s3^2;
        w=[1,s1,s2,s3,s4,s5,s6,s7,s8,s9,...
           s10,s10*s1,s10*s2,s10*s4,s10*s6,s10*s7,s10*s8,s10*s9,...
           s11,s11*s1,s11*s2,s11*s4,s11*s6,s11*s7,s11*s8,s11*s9,...
           s1^2,s1*s2,s1*s3,s2^2,s2*s3,s3^2]';
        factor=1/(1+s1^2+s2^2+s3^2);     
        Rtf=factor*[1+s1^2-s2^2-s3^2,2*s1*s2-2*s3,2*s2+2*s1*s3;
                       2*s3+2*s1*s2,1-s1^2+s2^2-s3^2,2*s2*s3-2*s1;
                       2*s1*s3-2*s2,2*s1+2*s2*s3,1-s1^2-s2^2+s3^2];
        Ttf =  factor*CCt*w;            
    

        tempW = P_W1;
        tempW(1,n) = aPointsNt(1);
        tempW(2,n) = aPointsNt(2);
        Pa = tempW;
        p2a = p_w1;
        [errep731{j},Urep731{j}]=reprojection_error_usingRT(Pa,p2a,Rtf,Ttf,A);

        Rth73{j} = Rtf;
        Tth73{j} = Ttf;
        errortf73{j} = obj_cur; 
        errRtf73{j} = cal_rotation_err(Rtf,Rtruth);
        errTtf73{j} = cal_translation_err(Ttf,Ttruth);



        [EEs,GGs,CCs] = RefineEquA63B3(p, W);
        apointsInis = [apointb(2) apointb(3)]';
        solutionOs = [Cayley(RBt);apointsInis]; 
        [solutions, obj_cur] = RefineGaussNewtonRandApT63B3(solutionOs, p, W, EEs, GGs, CCs, Lheight);

        if size(solutions) ~= 11
            aPointsNs = solutions(4:5);
            s1=solutions(1); s2=solutions(2); s3=solutions(3); 
            s10 = solutions(4); s11=solutions(5); 
        else
            aPointsNs = solutions(10:11);
            s1=solutions(1); s2=solutions(2); s3=solutions(3); 
            s10 = solutions(10); s11=solutions(11); 
        end  

        s4=s1^2; s5=s1*s2; s6=s1*s3;
        s7=s2^2; s8=s2*s3; s9=s3^2;
        w=[1,s1,s2,s3,s4,s5,s6,s7,s8,s9,...
           s10,s10*s1,s10*s2,s10*s4,s10*s6,s10*s7,s10*s8,s10*s9,...
           s11,s11*s1,s11*s2,s11*s4,s11*s6,s11*s7,s11*s8,s11*s9,...
           s1^2,s1*s2,s1*s3,s2^2,s2*s3,s3^2]';
        factor=1/(1+s1^2+s2^2+s3^2);     
        Rfs=factor*[1+s1^2-s2^2-s3^2,2*s1*s2-2*s3,2*s2+2*s1*s3;
                       2*s3+2*s1*s2,1-s1^2+s2^2-s3^2,2*s2*s3-2*s1;
                       2*s1*s3-2*s2,2*s1+2*s2*s3,1-s1^2-s2^2+s3^2];
        Tfs =  factor*CCs*w;  

        tempW = P_W4;
        tempW(1,n) = aPointsNs(1);
        tempW(2,n) = aPointsNs(2);
        Pa = tempW;
        p2a = p_w4;
        [errep732{j},Urep732{j}]=reprojection_error_usingRT(Pa,p2a,Rfs,Tfs,A);

        Rts73{j} = Rfs;
        Tts73{j} = Tfs;
        errortfs73{j} = obj_cur; 
        errRtfs73{j} = cal_rotation_err(Rfs,Rtruth);
        errTtfs73{j} = cal_translation_err(Tfs,Ttruth);
    end
    
    toc;
    time73{j} = toc; 

%      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    clearvars apointa apointb aPointsN aPointsNt aPointsNs apointsIni apointsInis apointsInit apointsIniu apointsIniv aPointsNu aPointsNv T1P1_wA T1P2_wA T1P1_wB T1P2_wB T1P1_wC T1P2_wC T1P1_wD T1P2_wD Rf Tf Rtf Ttf Rfs Tfs
    tic; 
    T1P1_wB = [P1_w(1,:);P2_w(1,:);P1_w(2,:)];
    T1P2_wB = [P1_w(2,:);P2_w(2,:);P2_w(1,:)];

    T1P1_wC = [P1_w(1,:);P2_w(1,:);P1_w(1,:)];
    T1P2_wC = [P1_w(2,:);P2_w(2,:);P2_w(1,:)]; 

    [F1, G] = P2LMGA64(T1p1B, T1p2B, T1P1_wB, T1P2_wB, T1p1C, T1p2C, T1P1_wC, T1P2_wC, cH, A, C_truth, Lheight);
    errAmin = Inf;
    errBmin = Inf;
    
    apointa = [-1,-1,-1];
    apointb = [-1,-1,-1];
    for i = 1:length(F1)
    %         if F1(i).ValObjective(1) > 2 || F1(i).ValObjective(2) > 2 || F1(i).ValObjective(3) > 2 || F1(i).ValObjective(4) > 2
        % + F1(i).ValObjective(3) + F1(i).ValObjective(4)
%         if F1(i).ValObjective(1) > 5 || F1(i).ValObjective(2) > 5 || F1(i).ValObjective(3) > 50 || F1(i).ValObjective(4) > 50
%     %         if F1(i).ValObjective(1) + F1(i).ValObjective(2) > 2
%             continue
%         end
    %         apoints(count,:) =  F1(i).Val;   
    %         count = count + 1;
        if F1(i).ValObjective(1) < errAmin 
            apointa(2) = F1(i).Val(2);
            apointa(3) = F1(i).Val(3);
            RAt = F1(i).RA;
            TAt = F1(i).TA;
            errAmin = F1(i).ValObjective(1);
        end
        if F1(i).ValObjective(2) < errBmin 
            apointb(1) = F1(i).Val(1);
            apointb(2) = F1(i).Val(2);
            RBt = F1(i).RB;
            TBt = F1(i).TB;
            errBmin = F1(i).ValObjective(2);
        end 
    end  
    
    [~,n] = max(abs(W2(:,1) - W1(:,1)));

    if n==1
        if errAmin < errBmin

            [EE,GG,CC] = RefineEquA64A1(p, W);
            apointsIni = [apointa(2) apointa(3)]';
            solutionO = [Cayley(RAt);apointsIni]; 
            [solution, obj_cur] = RefineGaussNewtonRandApT64A1(solutionO, p, W, EE, GG, CC, Lheight);


            if size(solution) ~= 11
                aPointsN = solution(4:5);
                s1=solution(1); s2=solution(2); s3=solution(3); 
                s10 = solution(4); s11=solution(5); 
            else
                aPointsN = solution(10:11);
                s1=solution(1); s2=solution(2); s3=solution(3); 
                s10 = solution(10); s11=solution(11); 
            end 
        else
            [EE,GG,CC] = RefineEquA64B1(p, W);
            apointsIni = [apointb(1) apointb(2)]';
            solutionO = [Cayley(RBt);apointsIni]; 
            [solution, obj_cur] = RefineGaussNewtonRandApT64B1(solutionO, p, W, EE, GG, CC, Lheight);

            if size(solution) ~= 11
                aPointsN = solution(4:5);
                s1=solution(1); s2=solution(2); s3=solution(3); 
                s10 = solution(4); s11=solution(5); 
            else
                aPointsN = solution(10:11);
                s1=solution(1); s2=solution(2); s3=solution(3); 
                s10 = solution(10); s11=solution(11); 
            end 
        end
        
        s4=s1^2; s5=s1*s2; s6=s1*s3;
        s7=s2^2; s8=s2*s3; s9=s3^2;
        w=[1,s1,s2,s3,s4,s5,s6,s7,s8,s9,...
           s10,s10*s2,s10*s3,s10*s4,s10*s5,s10*s6,s10*s7,s10*s9,...
           s11,s11*s2,s11*s3,s11*s4,s11*s5,s11*s6,s11*s7,s11*s9,...
           s1^2,s1*s2,s1*s3,s2^2,s2*s3,s3^2]';
        factor=1/(1+s1^2+s2^2+s3^2);     
        Rf=factor*[1+s1^2-s2^2-s3^2,2*s1*s2-2*s3,2*s2+2*s1*s3;
                       2*s3+2*s1*s2,1-s1^2+s2^2-s3^2,2*s2*s3-2*s1;
                       2*s1*s3-2*s2,2*s1+2*s2*s3,1-s1^2-s2^2+s3^2];
        Tf =  factor*CC*w;


        if errAmin < errBmin
            tempW = P_W2;
            tempW(1,n) = aPointsN(1);
            tempW(2,n) = aPointsN(2);
            Pa = tempW;
            p2a = p_w2;
            [errep740{j},Urep740{j}]=reprojection_error_usingRT(Pa,p2a,Rf,Tf,A);
        else
            tempW = P_W3;
            tempW(1,n) = aPointsN(1);
            tempW(2,n) = aPointsN(2);
            Pa = tempW;
            p2a = p_w3;
            [errep740{j},Urep740{j}]=reprojection_error_usingRT(Pa,p2a,Rf,Tf,A);
        end


        R74{j} = Rf;
        T74{j} = Tf;
        error74{j} = obj_cur; 
        errRf74{j} = cal_rotation_err(Rf,Rtruth);
        errTf74{j} = cal_translation_err(Tf,Ttruth);


        [EEt,GGt,CCt] = RefineEquA64A1(p, W);
        apointsInit = [apointa(2) apointa(3)]';
        solutionOt = [Cayley(RAt);apointsInit]; 
        [solutiont, obj_cur] = RefineGaussNewtonRandApT64A1(solutionOt, p, W, EEt, GGt, CCt, Lheight);


        if size(solutiont) ~= 11
            aPointsNt = solutiont(4:5);
            s1=solutiont(1); s2=solutiont(2); s3=solutiont(3); 
            s10 = solutiont(4); s11=solutiont(5); 
        else
            aPointsNt = solutiont(10:11);
            s1=solutiont(1); s2=solutiont(2); s3=solutiont(3); 
            s10 = solutiont(10); s11=solutiont(11); 
        end
        
        
        s4=s1^2; s5=s1*s2; s6=s1*s3;
        s7=s2^2; s8=s2*s3; s9=s3^2;
        w=[1,s1,s2,s3,s4,s5,s6,s7,s8,s9,...
           s10,s10*s2,s10*s3,s10*s4,s10*s5,s10*s6,s10*s7,s10*s9,...
           s11,s11*s2,s11*s3,s11*s4,s11*s5,s11*s6,s11*s7,s11*s9,...
           s1^2,s1*s2,s1*s3,s2^2,s2*s3,s3^2]';
        factor=1/(1+s1^2+s2^2+s3^2);     
        Rtf=factor*[1+s1^2-s2^2-s3^2,2*s1*s2-2*s3,2*s2+2*s1*s3;
                       2*s3+2*s1*s2,1-s1^2+s2^2-s3^2,2*s2*s3-2*s1;
                       2*s1*s3-2*s2,2*s1+2*s2*s3,1-s1^2-s2^2+s3^2];
        Ttf =  factor*CCt*w;  

        tempW = P_W2;
        tempW(1,n) = aPointsNt(1);
        tempW(2,n) = aPointsNt(2);
        Pa = tempW;
        p2a = p_w2;
        [errep741{j},Urep741{j}]=reprojection_error_usingRT(Pa,p2a,Rtf,Ttf,A);

        Rth74{j} = Rtf;
        Tth74{j} = Ttf;
        errortf74{j} = obj_cur; 
        errRtf74{j} = cal_rotation_err(Rtf,Rtruth);
        errTtf74{j} = cal_translation_err(Ttf,Ttruth);



        [EEs,GGs,CCs] = RefineEquA64B1(p, W);
        apointsInis = [apointb(1) apointb(2)]';
        solutionOs = [Cayley(RBt);apointsInis]; 
        [solutions, obj_cur] = RefineGaussNewtonRandApT64B1(solutionOs, p, W, EEs, GGs, CCs, Lheight);

        if size(solutions) ~= 11
            aPointsNs = solutions(4:5);
            s1=solutions(1); s2=solutions(2); s3=solutions(3); 
            s10 = solutions(4); s11=solutions(5); 
        else
            aPointsNs = solutions(10:11);
            s1=solutions(1); s2=solutions(2); s3=solutions(3); 
            s10 = solutions(10); s11=solutions(11); 
        end  

        s4=s1^2; s5=s1*s2; s6=s1*s3;
        s7=s2^2; s8=s2*s3; s9=s3^2;
        w=[1,s1,s2,s3,s4,s5,s6,s7,s8,s9,...
           s10,s10*s2,s10*s3,s10*s4,s10*s5,s10*s6,s10*s7,s10*s9,...
           s11,s11*s2,s11*s3,s11*s4,s11*s5,s11*s6,s11*s7,s11*s9,...
           s1^2,s1*s2,s1*s3,s2^2,s2*s3,s3^2]';
        factor=1/(1+s1^2+s2^2+s3^2);     
        Rfs=factor*[1+s1^2-s2^2-s3^2,2*s1*s2-2*s3,2*s2+2*s1*s3;
                       2*s3+2*s1*s2,1-s1^2+s2^2-s3^2,2*s2*s3-2*s1;
                       2*s1*s3-2*s2,2*s1+2*s2*s3,1-s1^2-s2^2+s3^2];
        Tfs =  factor*CCs*w; 
 

        tempW = P_W3;
        tempW(1,n) = aPointsNs(1);
        tempW(2,n) = aPointsNs(2);
        Pa = tempW;
        p2a = p_w3;
        [errep742{j},Urep742{j}]=reprojection_error_usingRT(Pa,p2a,Rfs,Tfs,A);

        Rts74{j} = Rfs;
        Tts74{j} = Tfs;
        errortfs74{j} = obj_cur; 
        errRtfs74{j} = cal_rotation_err(Rfs,Rtruth);
        errTtfs74{j} = cal_translation_err(Tfs,Ttruth);
    elseif  n==2

        if errAmin < errBmin

            [EE,GG,CC] = RefineEquA64A2(p, W);
            apointsIni = [apointa(2) apointa(3)]';
            solutionO = [Cayley(RAt);apointsIni]; 
            [solution, obj_cur] = RefineGaussNewtonRandApT64A2(solutionO, p, W, EE, GG, CC, Lheight);


            if size(solution) ~= 11
                aPointsN = solution(4:5);
                s1=solution(1); s2=solution(2); s3=solution(3); 
                s10 = solution(4); s11=solution(5); 
            else
                aPointsN = solution(10:11);
                s1=solution(1); s2=solution(2); s3=solution(3); 
                s10 = solution(10); s11=solution(11); 
            end 
        else
            [EE,GG,CC] = RefineEquA64B2(p, W);
            apointsIni = [apointb(1) apointb(2)]';
            solutionO = [Cayley(RBt);apointsIni]; 
            [solution, obj_cur] = RefineGaussNewtonRandApT64B2(solutionO, p, W, EE, GG, CC, Lheight);

            if size(solution) ~= 11
                aPointsN = solution(4:5);
                s1=solution(1); s2=solution(2); s3=solution(3); 
                s10 = solution(4); s11=solution(5); 
            else
                aPointsN = solution(10:11);
                s1=solution(1); s2=solution(2); s3=solution(3); 
                s10 = solution(10); s11=solution(11); 
            end 
        end
        
        s4=s1^2; s5=s1*s2; s6=s1*s3;
        s7=s2^2; s8=s2*s3; s9=s3^2;
        w=[1,s1,s2,s3,s4,s5,s6,s7,s8,s9,...
           s10,s10*s1,s10*s3,s10*s4,s10*s5,s10*s7,s10*s8,s10*s9,...
           s11,s11*s1,s11*s3,s11*s4,s11*s5,s11*s7,s11*s8,s11*s9,...
           s1^2,s1*s2,s1*s3,s2^2,s2*s3,s3^2]';
        factor=1/(1+s1^2+s2^2+s3^2);     
        Rf=factor*[1+s1^2-s2^2-s3^2,2*s1*s2-2*s3,2*s2+2*s1*s3;
                       2*s3+2*s1*s2,1-s1^2+s2^2-s3^2,2*s2*s3-2*s1;
                       2*s1*s3-2*s2,2*s1+2*s2*s3,1-s1^2-s2^2+s3^2];
        Tf =  factor*CC*w;


        if errAmin < errBmin
            tempW = P_W2;
            tempW(1,n) = aPointsN(1);
            tempW(2,n) = aPointsN(2);
            Pa = tempW;
            p2a = p_w2;
            [errep740{j},Urep740{j}]=reprojection_error_usingRT(Pa,p2a,Rf,Tf,A);
        else
            tempW = P_W3;
            tempW(1,n) = aPointsN(1);
            tempW(2,n) = aPointsN(2);
            Pa = tempW;
            p2a = p_w3;
            [errep740{j},Urep740{j}]=reprojection_error_usingRT(Pa,p2a,Rf,Tf,A);
        end


        R74{j} = Rf;
        T74{j} = Tf;
        error74{j} = obj_cur; 
        errRf74{j} = cal_rotation_err(Rf,Rtruth);
        errTf74{j} = cal_translation_err(Tf,Ttruth);


        [EEt,GGt,CCt] = RefineEquA64A2(p, W);
        apointsInit = [apointa(2) apointa(3)]';
        solutionOt = [Cayley(RAt);apointsInit]; 
        [solutiont, obj_cur] = RefineGaussNewtonRandApT64A2(solutionOt, p, W, EEt, GGt, CCt, Lheight);


        if size(solutiont) ~= 11
            aPointsNt = solutiont(4:5);
            s1=solutiont(1); s2=solutiont(2); s3=solutiont(3); 
            s10 = solutiont(4); s11=solutiont(5); 
        else
            aPointsNt = solutiont(10:11);
            s1=solutiont(1); s2=solutiont(2); s3=solutiont(3); 
            s10 = solutiont(10); s11=solutiont(11); 
        end
        
        
        s4=s1^2; s5=s1*s2; s6=s1*s3;
        s7=s2^2; s8=s2*s3; s9=s3^2;
        w=[1,s1,s2,s3,s4,s5,s6,s7,s8,s9,...
           s10,s10*s1,s10*s3,s10*s4,s10*s5,s10*s7,s10*s8,s10*s9,...
           s11,s11*s1,s11*s3,s11*s4,s11*s5,s11*s7,s11*s8,s11*s9,...
           s1^2,s1*s2,s1*s3,s2^2,s2*s3,s3^2]';
        factor=1/(1+s1^2+s2^2+s3^2);     
        Rtf=factor*[1+s1^2-s2^2-s3^2,2*s1*s2-2*s3,2*s2+2*s1*s3;
                       2*s3+2*s1*s2,1-s1^2+s2^2-s3^2,2*s2*s3-2*s1;
                       2*s1*s3-2*s2,2*s1+2*s2*s3,1-s1^2-s2^2+s3^2];
        Ttf =  factor*CCt*w;  

        tempW = P_W2;
        tempW(1,n) = aPointsNt(1);
        tempW(2,n) = aPointsNt(2);
        Pa = tempW;
        p2a = p_w2;
        [errep741{j},Urep741{j}]=reprojection_error_usingRT(Pa,p2a,Rtf,Ttf,A);

        Rth74{j} = Rtf;
        Tth74{j} = Ttf;
        errortf74{j} = obj_cur; 
        errRtf74{j} = cal_rotation_err(Rtf,Rtruth);
        errTtf74{j} = cal_translation_err(Ttf,Ttruth);



        [EEs,GGs,CCs] = RefineEquA64B2(p, W);
        apointsInis = [apointb(1) apointb(2)]';
        solutionOs = [Cayley(RBt);apointsInis]; 
        [solutions, obj_cur] = RefineGaussNewtonRandApT64B2(solutionOs, p, W, EEs, GGs, CCs, Lheight);

        if size(solutions) ~= 11
            aPointsNs = solutions(4:5);
            s1=solutions(1); s2=solutions(2); s3=solutions(3); 
            s10 = solutions(4); s11=solutions(5); 
        else
            aPointsNs = solutions(10:11);
            s1=solutions(1); s2=solutions(2); s3=solutions(3); 
            s10 = solutions(10); s11=solutions(11); 
        end  

        s4=s1^2; s5=s1*s2; s6=s1*s3;
        s7=s2^2; s8=s2*s3; s9=s3^2;
        w=[1,s1,s2,s3,s4,s5,s6,s7,s8,s9,...
           s10,s10*s1,s10*s3,s10*s4,s10*s5,s10*s7,s10*s8,s10*s9,...
           s11,s11*s1,s11*s3,s11*s4,s11*s5,s11*s7,s11*s8,s11*s9,...
           s1^2,s1*s2,s1*s3,s2^2,s2*s3,s3^2]';
        factor=1/(1+s1^2+s2^2+s3^2);     
        Rfs=factor*[1+s1^2-s2^2-s3^2,2*s1*s2-2*s3,2*s2+2*s1*s3;
                       2*s3+2*s1*s2,1-s1^2+s2^2-s3^2,2*s2*s3-2*s1;
                       2*s1*s3-2*s2,2*s1+2*s2*s3,1-s1^2-s2^2+s3^2];
        Tfs =  factor*CCs*w; 
 

        tempW = P_W3;
        tempW(1,n) = aPointsNs(1);
        tempW(2,n) = aPointsNs(2);
        Pa = tempW;
        p2a = p_w3;
        [errep742{j},Urep742{j}]=reprojection_error_usingRT(Pa,p2a,Rfs,Tfs,A);

        Rts74{j} = Rfs;
        Tts74{j} = Tfs;
        errortfs74{j} = obj_cur; 
        errRtfs74{j} = cal_rotation_err(Rfs,Rtruth);
        errTtfs74{j} = cal_translation_err(Tfs,Ttruth);
    elseif n==3
        if errAmin < errBmin

            [EE,GG,CC] = RefineEquA64A3(p, W);
            apointsIni = [apointa(2) apointa(3)]';
            solutionO = [Cayley(RAt);apointsIni]; 
            [solution, obj_cur] = RefineGaussNewtonRandApT64A3(solutionO, p, W, EE, GG, CC, Lheight);


            if size(solution) ~= 11
                aPointsN = solution(4:5);
                s1=solution(1); s2=solution(2); s3=solution(3); 
                s10 = solution(4); s11=solution(5); 
            else
                aPointsN = solution(10:11);
                s1=solution(1); s2=solution(2); s3=solution(3); 
                s10 = solution(10); s11=solution(11); 
            end 
        else
            [EE,GG,CC] = RefineEquA64B3(p, W);
            apointsIni = [apointb(1) apointb(2)]';
            solutionO = [Cayley(RBt);apointsIni]; 
            [solution, obj_cur] = RefineGaussNewtonRandApT64B3(solutionO, p, W, EE, GG, CC, Lheight);

            if size(solution) ~= 11
                aPointsN = solution(4:5);
                s1=solution(1); s2=solution(2); s3=solution(3); 
                s10 = solution(4); s11=solution(5); 
            else
                aPointsN = solution(10:11);
                s1=solution(1); s2=solution(2); s3=solution(3); 
                s10 = solution(10); s11=solution(11); 
            end 
        end
        
        s4=s1^2; s5=s1*s2; s6=s1*s3;
        s7=s2^2; s8=s2*s3; s9=s3^2;
        w=[1,s1,s2,s3,s4,s5,s6,s7,s8,s9,...
           s10,s10*s1,s10*s2,s10*s4,s10*s6,s10*s7,s10*s8,s10*s9,...
           s11,s11*s1,s11*s2,s11*s4,s11*s6,s11*s7,s11*s8,s11*s9,...
           s1^2,s1*s2,s1*s3,s2^2,s2*s3,s3^2]';
        factor=1/(1+s1^2+s2^2+s3^2);     
        Rf=factor*[1+s1^2-s2^2-s3^2,2*s1*s2-2*s3,2*s2+2*s1*s3;
                       2*s3+2*s1*s2,1-s1^2+s2^2-s3^2,2*s2*s3-2*s1;
                       2*s1*s3-2*s2,2*s1+2*s2*s3,1-s1^2-s2^2+s3^2];
        Tf =  factor*CC*w;


        if errAmin < errBmin
            tempW = P_W2;
            tempW(1,n) = aPointsN(1);
            tempW(2,n) = aPointsN(2);
            Pa = tempW;
            p2a = p_w2;
            [errep740{j},Urep740{j}]=reprojection_error_usingRT(Pa,p2a,Rf,Tf,A);
        else
            tempW = P_W3;
            tempW(1,n) = aPointsN(1);
            tempW(2,n) = aPointsN(2);
            Pa = tempW;
            p2a = p_w3;
            [errep740{j},Urep740{j}]=reprojection_error_usingRT(Pa,p2a,Rf,Tf,A);
        end


        R74{j} = Rf;
        T74{j} = Tf;
        error74{j} = obj_cur; 
        errRf74{j} = cal_rotation_err(Rf,Rtruth);
        errTf74{j} = cal_translation_err(Tf,Ttruth);


        [EEt,GGt,CCt] = RefineEquA64A3(p, W);
        apointsInit = [apointa(2) apointa(3)]';
        solutionOt = [Cayley(RAt);apointsInit]; 
        [solutiont, obj_cur] = RefineGaussNewtonRandApT64A3(solutionOt, p, W, EEt, GGt, CCt, Lheight);


        if size(solutiont) ~= 11
            aPointsNt = solutiont(4:5);
            s1=solutiont(1); s2=solutiont(2); s3=solutiont(3); 
            s10 = solutiont(4); s11=solutiont(5); 
        else
            aPointsNt = solutiont(10:11);
            s1=solutiont(1); s2=solutiont(2); s3=solutiont(3); 
            s10 = solutiont(10); s11=solutiont(11); 
        end
        
        
        s4=s1^2; s5=s1*s2; s6=s1*s3;
        s7=s2^2; s8=s2*s3; s9=s3^2;
        w=[1,s1,s2,s3,s4,s5,s6,s7,s8,s9,...
           s10,s10*s1,s10*s2,s10*s4,s10*s6,s10*s7,s10*s8,s10*s9,...
           s11,s11*s1,s11*s2,s11*s4,s11*s6,s11*s7,s11*s8,s11*s9,...
           s1^2,s1*s2,s1*s3,s2^2,s2*s3,s3^2]';
        factor=1/(1+s1^2+s2^2+s3^2);     
        Rtf=factor*[1+s1^2-s2^2-s3^2,2*s1*s2-2*s3,2*s2+2*s1*s3;
                       2*s3+2*s1*s2,1-s1^2+s2^2-s3^2,2*s2*s3-2*s1;
                       2*s1*s3-2*s2,2*s1+2*s2*s3,1-s1^2-s2^2+s3^2];
        Ttf =  factor*CCt*w;  

        tempW = P_W2;
        tempW(1,n) = aPointsNt(1);
        tempW(2,n) = aPointsNt(2);
        Pa = tempW;
        p2a = p_w2;
        [errep741{j},Urep741{j}]=reprojection_error_usingRT(Pa,p2a,Rtf,Ttf,A);

        Rth74{j} = Rtf;
        Tth74{j} = Ttf;
        errortf74{j} = obj_cur; 
        errRtf74{j} = cal_rotation_err(Rtf,Rtruth);
        errTtf74{j} = cal_translation_err(Ttf,Ttruth);



        [EEs,GGs,CCs] = RefineEquA64B3(p, W);
        apointsInis = [apointb(1) apointb(2)]';
        solutionOs = [Cayley(RBt);apointsInis]; 
        [solutions, obj_cur] = RefineGaussNewtonRandApT64B3(solutionOs, p, W, EEs, GGs, CCs, Lheight);

        if size(solutions) ~= 11
            aPointsNs = solutions(4:5);
            s1=solutions(1); s2=solutions(2); s3=solutions(3); 
            s10 = solutions(4); s11=solutions(5); 
        else
            aPointsNs = solutions(10:11);
            s1=solutions(1); s2=solutions(2); s3=solutions(3); 
            s10 = solutions(10); s11=solutions(11); 
        end  

        s4=s1^2; s5=s1*s2; s6=s1*s3;
        s7=s2^2; s8=s2*s3; s9=s3^2;
        w=[1,s1,s2,s3,s4,s5,s6,s7,s8,s9,...
           s10,s10*s1,s10*s2,s10*s4,s10*s6,s10*s7,s10*s8,s10*s9,...
           s11,s11*s1,s11*s2,s11*s4,s11*s6,s11*s7,s11*s8,s11*s9,...
           s1^2,s1*s2,s1*s3,s2^2,s2*s3,s3^2]';
        factor=1/(1+s1^2+s2^2+s3^2);     
        Rfs=factor*[1+s1^2-s2^2-s3^2,2*s1*s2-2*s3,2*s2+2*s1*s3;
                       2*s3+2*s1*s2,1-s1^2+s2^2-s3^2,2*s2*s3-2*s1;
                       2*s1*s3-2*s2,2*s1+2*s2*s3,1-s1^2-s2^2+s3^2];
        Tfs =  factor*CCs*w; 
 

        tempW = P_W3;
        tempW(1,n) = aPointsNs(1);
        tempW(2,n) = aPointsNs(2);
        Pa = tempW;
        p2a = p_w3;
        [errep742{j},Urep742{j}]=reprojection_error_usingRT(Pa,p2a,Rfs,Tfs,A);

        Rts74{j} = Rfs;
        Tts74{j} = Tfs;
        errortfs74{j} = obj_cur; 
        errRtfs74{j} = cal_rotation_err(Rfs,Rtruth);
        errTtfs74{j} = cal_translation_err(Tfs,Ttruth);
    end
    
    toc;
    time74{j} = toc; 

%     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    clearvars apointa apointb aPointsN aPointsNt aPointsNs apointsIni apointsInis apointsInit apointsIniu apointsIniv aPointsNu aPointsNv T1P1_wA T1P2_wA T1P1_wB T1P2_wB T1P1_wC T1P2_wC T1P1_wD T1P2_wD Rf Tf Rtf Ttf Rfs Tfs
    tic;

    T1P1_wB = [P1_w(1,:);P2_w(1,:);P1_w(2,:)];
    T1P2_wB = [P1_w(2,:);P2_w(2,:);P2_w(1,:)];


    T1P1_wD = [P1_w(1,:);P2_w(1,:);P1_w(2,:)];
    T1P2_wD = [P1_w(2,:);P2_w(2,:);P2_w(2,:)];

    [F1, G] = P2LMGA65(T1p1B, T1p2B, T1P1_wB, T1P2_wB, T1p1D, T1p2D, T1P1_wD, T1P2_wD, cH, A, C_truth, Lheight);
    errAmin = Inf;
    errBmin = Inf;
    apointa = [-1,-1,-1];
    apointb = [-1,-1,-1];
    for i = 1:length(F1)
%     %         if F1(i).ValObjective(1) > 0.5 || F1(i).ValObjective(2) > 0.5 || F1(i).ValObjective(3) > 0.5 || F1(i).ValObjective(4) > 0.5
%         if F1(i).ValObjective(1) > 5 || F1(i).ValObjective(2) > 5 || F1(i).ValObjective(3) > 50 || F1(i).ValObjective(4) > 50
%     %         if F1(i).ValObjective(1) + F1(i).ValObjective(2) > 2
%             continue
%         end
    %         apoints(count,:) =  F1(i).Val;   
    %         count = count + 1;
        if F1(i).ValObjective(1) < errAmin 
            apointa(1) = F1(i).Val(1);
            apointa(2) = F1(i).Val(2);
            RAt = F1(i).RA;
            TAt = F1(i).TA;
            errAmin = F1(i).ValObjective(1);
        end
        if F1(i).ValObjective(2) < errBmin 
            apointb(2) = F1(i).Val(2);
            apointb(3) = F1(i).Val(3);
            RBt = F1(i).RB;
            TBt = F1(i).TB;
            errBmin = F1(i).ValObjective(2);
        end 
    end  
    [~,n] = max(abs(W2(:,1) - W1(:,1)));

    if n==1
        if errAmin < errBmin

            [EE,GG,CC] = RefineEquA65A1(p, W);
            apointsIni = [apointa(1) apointa(2)]';
            solutionO = [Cayley(RAt);apointsIni]; 
            [solution, obj_cur] = RefineGaussNewtonRandApT65A1(solutionO, p, W, EE, GG, CC, Lheight);


            if size(solution) ~= 11
                aPointsN = solution(4:5);
                s1=solution(1); s2=solution(2); s3=solution(3); 
                s10 = solution(4); s11=solution(5); 
            else
                aPointsN = solution(10:11);
                s1=solution(1); s2=solution(2); s3=solution(3); 
                s10 = solution(10); s11=solution(11); 
            end 
        else
            [EE,GG,CC] = RefineEquA65B1(p, W);
            apointsIni = [apointb(2) apointb(3)]';
            solutionO = [Cayley(RBt);apointsIni]; 
            [solution, obj_cur] = RefineGaussNewtonRandApT65B1(solutionO, p, W, EE, GG, CC, Lheight);

            if size(solution) ~= 11
                aPointsN = solution(4:5);
                s1=solution(1); s2=solution(2); s3=solution(3); 
                s10 = solution(4); s11=solution(5); 
            else
                aPointsN = solution(10:11);
                s1=solution(1); s2=solution(2); s3=solution(3); 
                s10 = solution(10); s11=solution(11); 
            end 
        end
        
        s4=s1^2; s5=s1*s2; s6=s1*s3;
        s7=s2^2; s8=s2*s3; s9=s3^2;
        w=[1,s1,s2,s3,s4,s5,s6,s7,s8,s9,...
           s10,s10*s2,s10*s3,s10*s4,s10*s5,s10*s6,s10*s7,s10*s9,...
           s11,s11*s2,s11*s3,s11*s4,s11*s5,s11*s6,s11*s7,s11*s9,...
           s1^2,s1*s2,s1*s3,s2^2,s2*s3,s3^2]';
        factor=1/(1+s1^2+s2^2+s3^2);     
        Rf=factor*[1+s1^2-s2^2-s3^2,2*s1*s2-2*s3,2*s2+2*s1*s3;
                       2*s3+2*s1*s2,1-s1^2+s2^2-s3^2,2*s2*s3-2*s1;
                       2*s1*s3-2*s2,2*s1+2*s2*s3,1-s1^2-s2^2+s3^2];
        Tf =  factor*CC*w;
        
        
        if errAmin < errBmin
            tempW = P_W2;
            tempW(1,n) = aPointsN(1);
            tempW(2,n) = aPointsN(2);
            Pa = tempW;
            p2a = p_w2;
            [errep750{j},Urep750{j}]=reprojection_error_usingRT(Pa,p2a,Rf,Tf,A);
        else
            tempW = P_W4;
            tempW(1,n) = aPointsN(1);
            tempW(2,n) = aPointsN(2);
            Pa = tempW;
            p2a = p_w4;
            [errep750{j},Urep750{j}]=reprojection_error_usingRT(Pa,p2a,Rf,Tf,A);
        end


        R75{j} = Rf;
        T75{j} = Tf;
        error75{j} = obj_cur; 
        errRf75{j} = cal_rotation_err(Rf,Rtruth);
        errTf75{j} = cal_translation_err(Tf,Ttruth);


        [EEt,GGt,CCt] = RefineEquA65A1(p, W);
        apointsInit = [apointa(1) apointa(2)]';
        solutionOt = [Cayley(RAt);apointsInit]; 
        [solutiont, obj_curt] = RefineGaussNewtonRandApT65A1(solutionOt, p, W, EEt, GGt, CCt, Lheight);


       if size(solutiont) ~= 11
            aPointsNt = solutiont(4:5);
            s1=solutiont(1); s2=solutiont(2); s3=solutiont(3); 
            s10 = solutiont(4); s11=solutiont(5); 
        else
            aPointsNt = solutiont(10:11);
            s1=solutiont(1); s2=solutiont(2); s3=solutiont(3); 
            s10 = solutiont(10); s11=solutiont(11); 
        end
        
        
        s4=s1^2; s5=s1*s2; s6=s1*s3;
        s7=s2^2; s8=s2*s3; s9=s3^2;
        w=[1,s1,s2,s3,s4,s5,s6,s7,s8,s9,...
           s10,s10*s2,s10*s3,s10*s4,s10*s5,s10*s6,s10*s7,s10*s9,...
           s11,s11*s2,s11*s3,s11*s4,s11*s5,s11*s6,s11*s7,s11*s9,...
           s1^2,s1*s2,s1*s3,s2^2,s2*s3,s3^2]';
        factor=1/(1+s1^2+s2^2+s3^2);     
        Rtf=factor*[1+s1^2-s2^2-s3^2,2*s1*s2-2*s3,2*s2+2*s1*s3;
                       2*s3+2*s1*s2,1-s1^2+s2^2-s3^2,2*s2*s3-2*s1;
                       2*s1*s3-2*s2,2*s1+2*s2*s3,1-s1^2-s2^2+s3^2];
        Ttf =  factor*CCt*w;  

        tempW = P_W2;
        tempW(1,n) = aPointsNt(1);
        tempW(2,n) = aPointsNt(2);
        Pa = tempW;
        p2a = p_w2;
        [errep751{j},Urep751{j}]=reprojection_error_usingRT(Pa,p2a,Rtf,Ttf,A);

        Rth75{j} = Rtf;
        Tth75{j} = Ttf;
        errortf75{j} = obj_cur; 
        errRtf75{j} = cal_rotation_err(Rtf,Rtruth);
        errTtf75{j} = cal_translation_err(Ttf,Ttruth);



        [EEs,GGs,CCs] = RefineEquA65B1(p, W);
        apointsInis = [apointb(2) apointb(3)]';
        solutionOs = [Cayley(RBt);apointsInis]; 
        [solutions, obj_cur] = RefineGaussNewtonRandApT65B1(solutionOs, p, W, EEs, GGs, CCs, Lheight);

        if size(solutions) ~= 11
            aPointsNs = solutions(4:5);
            s1=solutions(1); s2=solutions(2); s3=solutions(3); 
            s10 = solutions(4); s11=solutions(5); 
        else
            aPointsNs = solutions(10:11);
            s1=solutions(1); s2=solutions(2); s3=solutions(3); 
            s10 = solutions(10); s11=solutions(11); 
        end  

        s4=s1^2; s5=s1*s2; s6=s1*s3;
        s7=s2^2; s8=s2*s3; s9=s3^2;
        w=[1,s1,s2,s3,s4,s5,s6,s7,s8,s9,...
           s10,s10*s2,s10*s3,s10*s4,s10*s5,s10*s6,s10*s7,s10*s9,...
           s11,s11*s2,s11*s3,s11*s4,s11*s5,s11*s6,s11*s7,s11*s9,...
           s1^2,s1*s2,s1*s3,s2^2,s2*s3,s3^2]';
        factor=1/(1+s1^2+s2^2+s3^2);     
        Rfs=factor*[1+s1^2-s2^2-s3^2,2*s1*s2-2*s3,2*s2+2*s1*s3;
                       2*s3+2*s1*s2,1-s1^2+s2^2-s3^2,2*s2*s3-2*s1;
                       2*s1*s3-2*s2,2*s1+2*s2*s3,1-s1^2-s2^2+s3^2];
        Tfs =  factor*CCs*w; 

        tempW = P_W4;
        tempW(1,n) = aPointsNs(1);
        tempW(2,n) = aPointsNs(2);
        Pa = tempW;
        p2a = p_w4;
        [errep752{j},Urep752{j}]=reprojection_error_usingRT(Pa,p2a,Rfs,Tfs,A);

        Rts75{j} = Rfs;
        Tts75{j} = Tfs;
        errortfs75{j} = obj_cur; 
        errRtfs75{j} = cal_rotation_err(Rfs,Rtruth);
        errTtfs75{j} = cal_translation_err(Tfs,Ttruth);
    elseif n ==2
        if errAmin < errBmin

            [EE,GG,CC] = RefineEquA65A2(p, W);
            apointsIni = [apointa(1) apointa(2)]';
            solutionO = [Cayley(RAt);apointsIni]; 
            [solution, obj_cur] = RefineGaussNewtonRandApT65A2(solutionO, p, W, EE, GG, CC, Lheight);


            if size(solution) ~= 11
                aPointsN = solution(4:5);
                s1=solution(1); s2=solution(2); s3=solution(3); 
                s10 = solution(4); s11=solution(5); 
            else
                aPointsN = solution(10:11);
                s1=solution(1); s2=solution(2); s3=solution(3); 
                s10 = solution(10); s11=solution(11); 
            end 
        else
            [EE,GG,CC] = RefineEquA65B2(p, W);
            apointsIni = [apointb(2) apointb(3)]';
            solutionO = [Cayley(RBt);apointsIni]; 
            [solution, obj_cur] = RefineGaussNewtonRandApT65B2(solutionO, p, W, EE, GG, CC, Lheight);

            if size(solution) ~= 11
                aPointsN = solution(4:5);
                s1=solution(1); s2=solution(2); s3=solution(3); 
                s10 = solution(4); s11=solution(5); 
            else
                aPointsN = solution(10:11);
                s1=solution(1); s2=solution(2); s3=solution(3); 
                s10 = solution(10); s11=solution(11); 
            end 
        end
        
        s4=s1^2; s5=s1*s2; s6=s1*s3;
        s7=s2^2; s8=s2*s3; s9=s3^2;
        w=[1,s1,s2,s3,s4,s5,s6,s7,s8,s9,...
           s10,s10*s1,s10*s3,s10*s4,s10*s5,s10*s7,s10*s8,s10*s9,...
           s11,s11*s1,s11*s3,s11*s4,s11*s5,s11*s7,s11*s8,s11*s9,...
           s1^2,s1*s2,s1*s3,s2^2,s2*s3,s3^2]';
        factor=1/(1+s1^2+s2^2+s3^2);     
        Rf=factor*[1+s1^2-s2^2-s3^2,2*s1*s2-2*s3,2*s2+2*s1*s3;
                       2*s3+2*s1*s2,1-s1^2+s2^2-s3^2,2*s2*s3-2*s1;
                       2*s1*s3-2*s2,2*s1+2*s2*s3,1-s1^2-s2^2+s3^2];
        Tf =  factor*CC*w;
        
        
        if errAmin < errBmin
            tempW = P_W2;
            tempW(1,n) = aPointsN(1);
            tempW(2,n) = aPointsN(2);
            Pa = tempW;
            p2a = p_w2;
            [errep750{j},Urep750{j}]=reprojection_error_usingRT(Pa,p2a,Rf,Tf,A);
        else
            tempW = P_W4;
            tempW(1,n) = aPointsN(1);
            tempW(2,n) = aPointsN(2);
            Pa = tempW;
            p2a = p_w4;
            [errep750{j},Urep750{j}]=reprojection_error_usingRT(Pa,p2a,Rf,Tf,A);
        end



        R75{j} = Rf;
        T75{j} = Tf;
        error75{j} = obj_cur; 
        errRf75{j} = cal_rotation_err(Rf,Rtruth);
        errTf75{j} = cal_translation_err(Tf,Ttruth);


        [EEt,GGt,CCt] = RefineEquA65A2(p, W);
        apointsInit = [apointa(1) apointa(2)]';
        solutionOt = [Cayley(RAt);apointsInit]; 
        [solutiont, obj_curt] = RefineGaussNewtonRandApT65A2(solutionOt, p, W, EEt, GGt, CCt, Lheight);


       if size(solutiont) ~= 11
            aPointsNt = solutiont(4:5);
            s1=solutiont(1); s2=solutiont(2); s3=solutiont(3); 
            s10 = solutiont(4); s11=solutiont(5); 
        else
            aPointsNt = solutiont(10:11);
            s1=solutiont(1); s2=solutiont(2); s3=solutiont(3); 
            s10 = solutiont(10); s11=solutiont(11); 
        end
        
        
        s4=s1^2; s5=s1*s2; s6=s1*s3;
        s7=s2^2; s8=s2*s3; s9=s3^2;
        w=[1,s1,s2,s3,s4,s5,s6,s7,s8,s9,...
           s10,s10*s1,s10*s3,s10*s4,s10*s5,s10*s7,s10*s8,s10*s9,...
           s11,s11*s1,s11*s3,s11*s4,s11*s5,s11*s7,s11*s8,s11*s9,...
           s1^2,s1*s2,s1*s3,s2^2,s2*s3,s3^2]';
        factor=1/(1+s1^2+s2^2+s3^2);     
        Rtf=factor*[1+s1^2-s2^2-s3^2,2*s1*s2-2*s3,2*s2+2*s1*s3;
                       2*s3+2*s1*s2,1-s1^2+s2^2-s3^2,2*s2*s3-2*s1;
                       2*s1*s3-2*s2,2*s1+2*s2*s3,1-s1^2-s2^2+s3^2];
        Ttf =  factor*CCt*w;  

        tempW = P_W2;
        tempW(1,n) = aPointsNt(1);
        tempW(2,n) = aPointsNt(2);
        Pa = tempW;
        p2a = p_w2;
        [errep751{j},Urep751{j}]=reprojection_error_usingRT(Pa,p2a,Rtf,Ttf,A);

        Rth75{j} = Rtf;
        Tth75{j} = Ttf;
        errortf75{j} = obj_cur; 
        errRtf75{j} = cal_rotation_err(Rtf,Rtruth);
        errTtf75{j} = cal_translation_err(Ttf,Ttruth);



        [EEs,GGs,CCs] = RefineEquA65B2(p, W);
        apointsInis = [apointb(2) apointb(3)]';
        solutionOs = [Cayley(RBt);apointsInis]; 
        [solutions, obj_cur] = RefineGaussNewtonRandApT65B2(solutionOs, p, W, EEs, GGs, CCs, Lheight);

        if size(solutions) ~= 11
            aPointsNs = solutions(4:5);
            s1=solutions(1); s2=solutions(2); s3=solutions(3); 
            s10 = solutions(4); s11=solutions(5); 
        else
            aPointsNs = solutions(10:11);
            s1=solutions(1); s2=solutions(2); s3=solutions(3); 
            s10 = solutions(10); s11=solutions(11); 
        end  

        s4=s1^2; s5=s1*s2; s6=s1*s3;
        s7=s2^2; s8=s2*s3; s9=s3^2;
        w=[1,s1,s2,s3,s4,s5,s6,s7,s8,s9,...
           s10,s10*s1,s10*s3,s10*s4,s10*s5,s10*s7,s10*s8,s10*s9,...
           s11,s11*s1,s11*s3,s11*s4,s11*s5,s11*s7,s11*s8,s11*s9,...
           s1^2,s1*s2,s1*s3,s2^2,s2*s3,s3^2]';
        factor=1/(1+s1^2+s2^2+s3^2);     
        Rfs=factor*[1+s1^2-s2^2-s3^2,2*s1*s2-2*s3,2*s2+2*s1*s3;
                       2*s3+2*s1*s2,1-s1^2+s2^2-s3^2,2*s2*s3-2*s1;
                       2*s1*s3-2*s2,2*s1+2*s2*s3,1-s1^2-s2^2+s3^2];
        Tfs =  factor*CCs*w; 

        tempW = P_W4;
        tempW(1,n) = aPointsNs(1);
        tempW(2,n) = aPointsNs(2);
        Pa = tempW;
        p2a = p_w4;
        [errep752{j},Urep752{j}]=reprojection_error_usingRT(Pa,p2a,Rfs,Tfs,A);

        Rts75{j} = Rfs;
        Tts75{j} = Tfs;
        errortfs75{j} = obj_cur; 
        errRtfs75{j} = cal_rotation_err(Rfs,Rtruth);
        errTtfs75{j} = cal_translation_err(Tfs,Ttruth);
    elseif n==3
        if errAmin < errBmin

            [EE,GG,CC] = RefineEquA65A3(p, W);
            apointsIni = [apointa(1) apointa(2)]';
            solutionO = [Cayley(RAt);apointsIni]; 
            [solution, obj_cur] = RefineGaussNewtonRandApT65A3(solutionO, p, W, EE, GG, CC, Lheight);


            if size(solution) ~= 11
                aPointsN = solution(4:5);
                s1=solution(1); s2=solution(2); s3=solution(3); 
                s10 = solution(4); s11=solution(5); 
            else
                aPointsN = solution(10:11);
                s1=solution(1); s2=solution(2); s3=solution(3); 
                s10 = solution(10); s11=solution(11); 
            end 
        else
            [EE,GG,CC] = RefineEquA65B3(p, W);
            apointsIni = [apointb(2) apointb(3)]';
            solutionO = [Cayley(RBt);apointsIni]; 
            [solution, obj_cur] = RefineGaussNewtonRandApT65B3(solutionO, p, W, EE, GG, CC, Lheight);

            if size(solution) ~= 11
                aPointsN = solution(4:5);
                s1=solution(1); s2=solution(2); s3=solution(3); 
                s10 = solution(4); s11=solution(5); 
            else
                aPointsN = solution(10:11);
                s1=solution(1); s2=solution(2); s3=solution(3); 
                s10 = solution(10); s11=solution(11); 
            end 
        end
        
        s4=s1^2; s5=s1*s2; s6=s1*s3;
        s7=s2^2; s8=s2*s3; s9=s3^2;
        w=[1,s1,s2,s3,s4,s5,s6,s7,s8,s9,...
           s10,s10*s1,s10*s2,s10*s4,s10*s6,s10*s7,s10*s8,s10*s9,...
           s11,s11*s1,s11*s2,s11*s4,s11*s6,s11*s7,s11*s8,s11*s9,...
           s1^2,s1*s2,s1*s3,s2^2,s2*s3,s3^2]';
        factor=1/(1+s1^2+s2^2+s3^2);     
        Rf=factor*[1+s1^2-s2^2-s3^2,2*s1*s2-2*s3,2*s2+2*s1*s3;
                       2*s3+2*s1*s2,1-s1^2+s2^2-s3^2,2*s2*s3-2*s1;
                       2*s1*s3-2*s2,2*s1+2*s2*s3,1-s1^2-s2^2+s3^2];
        Tf =  factor*CC*w;
        
        
        if errAmin < errBmin
            tempW = P_W2;
            tempW(1,n) = aPointsN(1);
            tempW(2,n) = aPointsN(2);
            Pa = tempW;
            p2a = p_w2;
            [errep750{j},Urep750{j}]=reprojection_error_usingRT(Pa,p2a,Rf,Tf,A);
        else
            tempW = P_W4;
            tempW(1,n) = aPointsN(1);
            tempW(2,n) = aPointsN(2);
            Pa = tempW;
            p2a = p_w4;
            [errep750{j},Urep750{j}]=reprojection_error_usingRT(Pa,p2a,Rf,Tf,A);
        end

        R75{j} = Rf;
        T75{j} = Tf;
        error75{j} = obj_cur; 
        errRf75{j} = cal_rotation_err(Rf,Rtruth);
        errTf75{j} = cal_translation_err(Tf,Ttruth);


        [EEt,GGt,CCt] = RefineEquA65A3(p, W);
        apointsInit = [apointa(1) apointa(2)]';
        solutionOt = [Cayley(RAt);apointsInit]; 
        [solutiont, obj_curt] = RefineGaussNewtonRandApT65A3(solutionOt, p, W, EEt, GGt, CCt, Lheight);


       if size(solutiont) ~= 11
            aPointsNt = solutiont(4:5);
            s1=solutiont(1); s2=solutiont(2); s3=solutiont(3); 
            s10 = solutiont(4); s11=solutiont(5); 
        else
            aPointsNt = solutiont(10:11);
            s1=solutiont(1); s2=solutiont(2); s3=solutiont(3); 
            s10 = solutiont(10); s11=solutiont(11); 
        end
        
        
        s4=s1^2; s5=s1*s2; s6=s1*s3;
        s7=s2^2; s8=s2*s3; s9=s3^2;
        w=[1,s1,s2,s3,s4,s5,s6,s7,s8,s9,...
           s10,s10*s1,s10*s2,s10*s4,s10*s6,s10*s7,s10*s8,s10*s9,...
           s11,s11*s1,s11*s2,s11*s4,s11*s6,s11*s7,s11*s8,s11*s9,...
           s1^2,s1*s2,s1*s3,s2^2,s2*s3,s3^2]';
        factor=1/(1+s1^2+s2^2+s3^2);     
        Rtf=factor*[1+s1^2-s2^2-s3^2,2*s1*s2-2*s3,2*s2+2*s1*s3;
                       2*s3+2*s1*s2,1-s1^2+s2^2-s3^2,2*s2*s3-2*s1;
                       2*s1*s3-2*s2,2*s1+2*s2*s3,1-s1^2-s2^2+s3^2];
        Ttf =  factor*CCt*w;  

        tempW = P_W2;
        tempW(1,n) = aPointsNt(1);
        tempW(2,n) = aPointsNt(2);
        Pa = tempW;
        p2a = p_w2;
        [errep751{j},Urep751{j}]=reprojection_error_usingRT(Pa,p2a,Rtf,Ttf,A);

        Rth75{j} = Rtf;
        Tth75{j} = Ttf;
        errortf75{j} = obj_cur; 
        errRtf75{j} = cal_rotation_err(Rtf,Rtruth);
        errTtf75{j} = cal_translation_err(Ttf,Ttruth);



        [EEs,GGs,CCs] = RefineEquA65B3(p, W);
        apointsInis = [apointb(2) apointb(3)]';
        solutionOs = [Cayley(RBt);apointsInis]; 
        [solutions, obj_cur] = RefineGaussNewtonRandApT65B3(solutionOs, p, W, EEs, GGs, CCs, Lheight);

        if size(solutions) ~= 11
            aPointsNs = solutions(4:5);
            s1=solutions(1); s2=solutions(2); s3=solutions(3); 
            s10 = solutions(4); s11=solutions(5); 
        else
            aPointsNs = solutions(10:11);
            s1=solutions(1); s2=solutions(2); s3=solutions(3); 
            s10 = solutions(10); s11=solutions(11); 
        end  

        s4=s1^2; s5=s1*s2; s6=s1*s3;
        s7=s2^2; s8=s2*s3; s9=s3^2;
        w=[1,s1,s2,s3,s4,s5,s6,s7,s8,s9,...
           s10,s10*s1,s10*s2,s10*s4,s10*s6,s10*s7,s10*s8,s10*s9,...
           s11,s11*s1,s11*s2,s11*s4,s11*s6,s11*s7,s11*s8,s11*s9,...
           s1^2,s1*s2,s1*s3,s2^2,s2*s3,s3^2]';
        factor=1/(1+s1^2+s2^2+s3^2);     
        Rfs=factor*[1+s1^2-s2^2-s3^2,2*s1*s2-2*s3,2*s2+2*s1*s3;
                       2*s3+2*s1*s2,1-s1^2+s2^2-s3^2,2*s2*s3-2*s1;
                       2*s1*s3-2*s2,2*s1+2*s2*s3,1-s1^2-s2^2+s3^2];
        Tfs =  factor*CCs*w; 

        tempW = P_W4;
        tempW(1,n) = aPointsNs(1);
        tempW(2,n) = aPointsNs(2);
        Pa = tempW;
        p2a = p_w4;
        [errep752{j},Urep752{j}]=reprojection_error_usingRT(Pa,p2a,Rfs,Tfs,A);

        Rts75{j} = Rfs;
        Tts75{j} = Tfs;
        errortfs75{j} = obj_cur; 
        errRtfs75{j} = cal_rotation_err(Rfs,Rtruth);
        errTtfs75{j} = cal_translation_err(Tfs,Ttruth);
    end
    
    toc;
    time75{j} = toc; 
% 
%     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    clearvars apoint aPointsN aPointsNt aPointsNs apointsIni apointsInis apointsInit apointsIniu apointsIniv aPointsNu aPointsNv T1P1_wA T1P2_wA T1P1_wB T1P2_wB T1P1_wC T1P2_wC T1P1_wD T1P2_wD Rf Tf Rtf Ttf Rfs Tfs F1
    tic;
    T1P1_wC = [P1_w(1,:);P2_w(1,:);P1_w(1,:)];
    T1P2_wC = [P1_w(2,:);P2_w(2,:);P2_w(1,:)]; 

    T1P1_wD = [P1_w(1,:);P2_w(1,:);P1_w(2,:)];
    T1P2_wD = [P1_w(2,:);P2_w(2,:);P2_w(2,:)];

    [F1, G] = P2LMGA66(T1p1C, T1p2C, T1P1_wC, T1P2_wC, T1p1D, T1p2D, T1P1_wD, T1P2_wD, A, C_truth, Lheight);
    errAmin = Inf;
    errBmin = Inf;
    apoint = [-1,-1,-1,-1];
    for i = 1:length(F1)
    %         if F1(i).ValObjective(1) > 0.5 || F1(i).ValObjective(2) > 0.5 || F1(i).ValObjective(3) > 0.5 || F1(i).ValObjective(4) > 0.5
%         if F1(i).ValObjective(1) > 5 || F1(i).ValObjective(2) > 5 || F1(i).ValObjective(3) > 50 || F1(i).ValObjective(4) > 50
%     %         if F1(i).ValObjective(1) + F1(i).ValObjective(2)  > 2
%             continue
%         end
    %         apoints(count,:) =  F1(i).Val;   
    %         count = count + 1;
        if F1(i).ValObjective(1) < errAmin 
            apoint(1) = F1(i).Val(1);
            apoint(2) = F1(i).Val(2);
            RAt = F1(i).RA;
            TAt = F1(i).TA;
            errAmin = F1(i).ValObjective(1);
        end
        if F1(i).ValObjective(2) < errBmin 
            apoint(3) = F1(i).Val(3);
            apoint(4) = F1(i).Val(4);
            RBt = F1(i).RB;
            TBt = F1(i).TB;
            errBmin = F1(i).ValObjective(2);
        end 
    end  
    
    [~,n] = max(abs(W2(:,1) - W1(:,1)));

    if n==1
        if errAmin < errBmin

            [EE,GG,CC] = RefineEquA66A1(p, W);
            apointsIni = [apoint(1) apoint(2)]';
            solutionO = [Cayley(RAt);apointsIni]; 
            [solution, obj_cur] = RefineGaussNewtonRandApT66A1(solutionO, p, W, EE, GG, CC, Lheight);


            if size(solution) ~= 11
                aPointsN = solution(4:5);
                s1=solution(1); s2=solution(2); s3=solution(3); 
                s10 = solution(4); s11=solution(5); 
            else
                aPointsN = solution(10:11);
                s1=solution(1); s2=solution(2); s3=solution(3); 
                s10 = solution(10); s11=solution(11); 
            end 
        else
            [EE,GG,CC] = RefineEquA66B1(p, W);
            apointsIni = [apoint(3) apoint(4)]';
            solutionO = [Cayley(RBt);apointsIni]; 
            [solution, obj_cur] = RefineGaussNewtonRandApT66B1(solutionO, p, W, EE, GG, CC, Lheight);

            if size(solution) ~= 11
                aPointsN = solution(4:5);
                s1=solution(1); s2=solution(2); s3=solution(3); 
                s10 = solution(4); s11=solution(5); 
            else
                aPointsN = solution(10:11);
                s1=solution(1); s2=solution(2); s3=solution(3); 
                s10 = solution(10); s11=solution(11); 
            end 
        end 
        
        s4=s1^2; s5=s1*s2; s6=s1*s3;
        s7=s2^2; s8=s2*s3; s9=s3^2;
        w=[1,s1,s2,s3,s4,s5,s6,s7,s8,s9,...
           s10,s10*s2,s10*s3,s10*s4,s10*s5,s10*s6,s10*s7,s10*s9,...
           s11,s11*s2,s11*s3,s11*s4,s11*s5,s11*s6,s11*s7,s11*s9,...
           s1^2,s1*s2,s1*s3,s2^2,s2*s3,s3^2]';
        factor=1/(1+s1^2+s2^2+s3^2);     
        Rf=factor*[1+s1^2-s2^2-s3^2,2*s1*s2-2*s3,2*s2+2*s1*s3;
                       2*s3+2*s1*s2,1-s1^2+s2^2-s3^2,2*s2*s3-2*s1;
                       2*s1*s3-2*s2,2*s1+2*s2*s3,1-s1^2-s2^2+s3^2];
        Tf =  factor*CC*w;

        if errAmin < errBmin
            tempW = P_W3;
            tempW(1,n) = aPointsN(1);
            tempW(2,n) = aPointsN(2);
            Pa = tempW;
            p2a = p_w3;
            [errep760{j},Urep760{j}]=reprojection_error_usingRT(Pa,p2a,Rf,Tf,A);
        else
            tempW = P_W4;
            tempW(1,n) = aPointsN(1);
            tempW(2,n) = aPointsN(2);
            Pa = tempW;
            p2a = p_w4;
            [errep760{j},Urep760{j}]=reprojection_error_usingRT(Pa,p2a,Rf,Tf,A);
        end

         

        R76{j} = Rf;
        T76{j} = Tf;
        error76{j} = obj_cur; 
        errRf76{j} = cal_rotation_err(Rf,Rtruth);
        errTf76{j} = cal_translation_err(Tf,Ttruth);


        [EEt,GGt,CCt] = RefineEquA66A1(p, W);
        apointsInit = [apoint(1) apoint(2)]';
        solutionOt = [Cayley(RAt);apointsInit]; 
        [solutiont, obj_curt] = RefineGaussNewtonRandApT66A1(solutionOt, p, W, EEt, GGt, CCt, Lheight);


        if size(solutiont) ~= 11
            aPointsNt = solutiont(4:5);
            s1=solutiont(1); s2=solutiont(2); s3=solutiont(3); 
            s10 = solutiont(4); s11=solutiont(5); 
        else
            aPointsNt = solutiont(10:11);
            s1=solutiont(1); s2=solutiont(2); s3=solutiont(3); 
            s10 = solutiont(10); s11=solutiont(11); 
        end
        
        
        s4=s1^2; s5=s1*s2; s6=s1*s3;
        s7=s2^2; s8=s2*s3; s9=s3^2;
        w=[1,s1,s2,s3,s4,s5,s6,s7,s8,s9,...
           s10,s10*s2,s10*s3,s10*s4,s10*s5,s10*s6,s10*s7,s10*s9,...
           s11,s11*s2,s11*s3,s11*s4,s11*s5,s11*s6,s11*s7,s11*s9,...
           s1^2,s1*s2,s1*s3,s2^2,s2*s3,s3^2]';
        factor=1/(1+s1^2+s2^2+s3^2);     
        Rtf=factor*[1+s1^2-s2^2-s3^2,2*s1*s2-2*s3,2*s2+2*s1*s3;
                       2*s3+2*s1*s2,1-s1^2+s2^2-s3^2,2*s2*s3-2*s1;
                       2*s1*s3-2*s2,2*s1+2*s2*s3,1-s1^2-s2^2+s3^2];
        Ttf =  factor*CCt*w;  

        tempW = P_W3;
        tempW(1,n) = aPointsNt(1);
        tempW(2,n) = aPointsNt(2);
        Pa = tempW;
        p2a = p_w3;
        [errep761{j},Urep761{j}]=reprojection_error_usingRT(Pa,p2a,Rtf,Ttf,A);

        Rth76{j} = Rtf;
        Tth76{j} = Ttf;
        errortf76{j} = obj_cur; 
        errRtf76{j} = cal_rotation_err(Rtf,Rtruth);
        errTtf76{j} = cal_translation_err(Ttf,Ttruth);


        [EEs,GGs,CCs] = RefineEquA66B1(p, W);
        apointsInis = [apoint(3) apoint(4)]';
        solutionOs = [Cayley(RBt);apointsInis]; 
        [solutions, obj_cur] = RefineGaussNewtonRandApT66B1(solutionOs, p, W, EEs, GGs, CCs, Lheight);

        if size(solutions) ~= 11
            aPointsNs = solutions(4:5);
            s1=solutions(1); s2=solutions(2); s3=solutions(3); 
            s10 = solutions(4); s11=solutions(5); 
        else
            aPointsNs = solutions(10:11);
            s1=solutions(1); s2=solutions(2); s3=solutions(3); 
            s10 = solutions(10); s11=solutions(11); 
        end  

        s4=s1^2; s5=s1*s2; s6=s1*s3;
        s7=s2^2; s8=s2*s3; s9=s3^2;
        w=[1,s1,s2,s3,s4,s5,s6,s7,s8,s9,...
           s10,s10*s2,s10*s3,s10*s4,s10*s5,s10*s6,s10*s7,s10*s9,...
           s11,s11*s2,s11*s3,s11*s4,s11*s5,s11*s6,s11*s7,s11*s9,...
           s1^2,s1*s2,s1*s3,s2^2,s2*s3,s3^2]';
        factor=1/(1+s1^2+s2^2+s3^2);     
        Rfs=factor*[1+s1^2-s2^2-s3^2,2*s1*s2-2*s3,2*s2+2*s1*s3;
                       2*s3+2*s1*s2,1-s1^2+s2^2-s3^2,2*s2*s3-2*s1;
                       2*s1*s3-2*s2,2*s1+2*s2*s3,1-s1^2-s2^2+s3^2];
        Tfs =  factor*CCs*w; 

        tempW = P_W4;
        tempW(1,n) = aPointsNs(1);
        tempW(2,n) = aPointsNs(2);
        Pa = tempW;
        p2a = p_w4;
        [errep762{j},Urep762{j}]=reprojection_error_usingRT(Pa,p2a,Rfs,Tfs,A);

        Rts76{j} = Rfs;
        Tts76{j} = Tfs;
        errortfs76{j} = obj_cur; 
        errRtfs76{j} = cal_rotation_err(Rfs,Rtruth);
        errTtfs76{j} = cal_translation_err(Tfs,Ttruth);
    elseif n==2
    
        if errAmin < errBmin

            [EE,GG,CC] = RefineEquA66A2(p, W);
            apointsIni = [apoint(1) apoint(2)]';
            solutionO = [Cayley(RAt);apointsIni]; 
            [solution, obj_cur] = RefineGaussNewtonRandApT66A2(solutionO, p, W, EE, GG, CC, Lheight);


            if size(solution) ~= 11
                aPointsN = solution(4:5);
                s1=solution(1); s2=solution(2); s3=solution(3); 
                s10 = solution(4); s11=solution(5); 
            else
                aPointsN = solution(10:11);
                s1=solution(1); s2=solution(2); s3=solution(3); 
                s10 = solution(10); s11=solution(11); 
            end 
        else
            [EE,GG,CC] = RefineEquA66B2(p, W);
            apointsIni = [apoint(3) apoint(4)]';
            solutionO = [Cayley(RBt);apointsIni]; 
            [solution, obj_cur] = RefineGaussNewtonRandApT66B2(solutionO, p, W, EE, GG, CC, Lheight);

            if size(solution) ~= 11
                aPointsN = solution(4:5);
                s1=solution(1); s2=solution(2); s3=solution(3); 
                s10 = solution(4); s11=solution(5); 
            else
                aPointsN = solution(10:11);
                s1=solution(1); s2=solution(2); s3=solution(3); 
                s10 = solution(10); s11=solution(11); 
            end 
        end 
        
        s4=s1^2; s5=s1*s2; s6=s1*s3;
        s7=s2^2; s8=s2*s3; s9=s3^2;
        w=[1,s1,s2,s3,s4,s5,s6,s7,s8,s9,...
           s10,s10*s1,s10*s3,s10*s4,s10*s5,s10*s7,s10*s8,s10*s9,...
           s11,s11*s1,s11*s3,s11*s4,s11*s5,s11*s7,s11*s8,s11*s9,...
           s1^2,s1*s2,s1*s3,s2^2,s2*s3,s3^2]';
        factor=1/(1+s1^2+s2^2+s3^2);     
        Rf=factor*[1+s1^2-s2^2-s3^2,2*s1*s2-2*s3,2*s2+2*s1*s3;
                       2*s3+2*s1*s2,1-s1^2+s2^2-s3^2,2*s2*s3-2*s1;
                       2*s1*s3-2*s2,2*s1+2*s2*s3,1-s1^2-s2^2+s3^2];
        Tf =  factor*CC*w;

        if errAmin < errBmin
            tempW = P_W3;
            tempW(1,n) = aPointsN(1);
            tempW(2,n) = aPointsN(2);
            Pa = tempW;
            p2a = p_w3;
            [errep760{j},Urep760{j}]=reprojection_error_usingRT(Pa,p2a,Rf,Tf,A);
        else
            tempW = P_W4;
            tempW(1,n) = aPointsN(1);
            tempW(2,n) = aPointsN(2);
            Pa = tempW;
            p2a = p_w4;
            [errep760{j},Urep760{j}]=reprojection_error_usingRT(Pa,p2a,Rf,Tf,A);
        end

 

        R76{j} = Rf;
        T76{j} = Tf;
        error76{j} = obj_cur; 
        errRf76{j} = cal_rotation_err(Rf,Rtruth);
        errTf76{j} = cal_translation_err(Tf,Ttruth);


        [EEt,GGt,CCt] = RefineEquA66A2(p, W);
        apointsInit = [apoint(1) apoint(2)]';
        solutionOt = [Cayley(RAt);apointsInit]; 
        [solutiont, obj_curt] = RefineGaussNewtonRandApT66A2(solutionOt, p, W, EEt, GGt, CCt, Lheight);


        if size(solutiont) ~= 11
            aPointsNt = solutiont(4:5);
            s1=solutiont(1); s2=solutiont(2); s3=solutiont(3); 
            s10 = solutiont(4); s11=solutiont(5); 
        else
            aPointsNt = solutiont(10:11);
            s1=solutiont(1); s2=solutiont(2); s3=solutiont(3); 
            s10 = solutiont(10); s11=solutiont(11); 
        end
        
        
        s4=s1^2; s5=s1*s2; s6=s1*s3;
        s7=s2^2; s8=s2*s3; s9=s3^2;
        w=[1,s1,s2,s3,s4,s5,s6,s7,s8,s9,...
           s10,s10*s1,s10*s3,s10*s4,s10*s5,s10*s7,s10*s8,s10*s9,...
           s11,s11*s1,s11*s3,s11*s4,s11*s5,s11*s7,s11*s8,s11*s9,...
           s1^2,s1*s2,s1*s3,s2^2,s2*s3,s3^2]';
        factor=1/(1+s1^2+s2^2+s3^2);     
        Rtf=factor*[1+s1^2-s2^2-s3^2,2*s1*s2-2*s3,2*s2+2*s1*s3;
                       2*s3+2*s1*s2,1-s1^2+s2^2-s3^2,2*s2*s3-2*s1;
                       2*s1*s3-2*s2,2*s1+2*s2*s3,1-s1^2-s2^2+s3^2];
        Ttf =  factor*CCt*w;  

        tempW = P_W3;
        tempW(1,n) = aPointsNt(1);
        tempW(2,n) = aPointsNt(2);
        Pa = tempW;
        p2a = p_w3;
        [errep761{j},Urep761{j}]=reprojection_error_usingRT(Pa,p2a,Rtf,Ttf,A);

        Rth76{j} = Rtf;
        Tth76{j} = Ttf;
        errortf76{j} = obj_cur; 
        errRtf76{j} = cal_rotation_err(Rtf,Rtruth);
        errTtf76{j} = cal_translation_err(Ttf,Ttruth);


        [EEs,GGs,CCs] = RefineEquA66B2(p, W);
        apointsInis = [apoint(3) apoint(4)]';
        solutionOs = [Cayley(RBt);apointsInis]; 
        [solutions, obj_cur] = RefineGaussNewtonRandApT66B2(solutionOs, p, W, EEs, GGs, CCs, Lheight);

        if size(solutions) ~= 11
            aPointsNs = solutions(4:5);
            s1=solutions(1); s2=solutions(2); s3=solutions(3); 
            s10 = solutions(4); s11=solutions(5); 
        else
            aPointsNs = solutions(10:11);
            s1=solutions(1); s2=solutions(2); s3=solutions(3); 
            s10 = solutions(10); s11=solutions(11); 
        end  

        s4=s1^2; s5=s1*s2; s6=s1*s3;
        s7=s2^2; s8=s2*s3; s9=s3^2;
        w=[1,s1,s2,s3,s4,s5,s6,s7,s8,s9,...
           s10,s10*s1,s10*s3,s10*s4,s10*s5,s10*s7,s10*s8,s10*s9,...
           s11,s11*s1,s11*s3,s11*s4,s11*s5,s11*s7,s11*s8,s11*s9,...
           s1^2,s1*s2,s1*s3,s2^2,s2*s3,s3^2]';
        factor=1/(1+s1^2+s2^2+s3^2);     
        Rfs=factor*[1+s1^2-s2^2-s3^2,2*s1*s2-2*s3,2*s2+2*s1*s3;
                       2*s3+2*s1*s2,1-s1^2+s2^2-s3^2,2*s2*s3-2*s1;
                       2*s1*s3-2*s2,2*s1+2*s2*s3,1-s1^2-s2^2+s3^2];
        Tfs =  factor*CCs*w; 

        tempW = P_W4;
        tempW(1,n) = aPointsNs(1);
        tempW(2,n) = aPointsNs(2);
        Pa = tempW;
        p2a = p_w4;
        [errep762{j},Urep762{j}]=reprojection_error_usingRT(Pa,p2a,Rfs,Tfs,A);

        Rts76{j} = Rfs;
        Tts76{j} = Tfs;
        errortfs76{j} = obj_cur; 
        errRtfs76{j} = cal_rotation_err(Rfs,Rtruth);
        errTtfs76{j} = cal_translation_err(Tfs,Ttruth);
    elseif n==3
        if errAmin < errBmin

            [EE,GG,CC] = RefineEquA66A3(p, W);
            apointsIni = [apoint(1) apoint(2)]';
            solutionO = [Cayley(RAt);apointsIni]; 
            [solution, obj_cur] = RefineGaussNewtonRandApT66A3(solutionO, p, W, EE, GG, CC, Lheight);


            if size(solution) ~= 11
                aPointsN = solution(4:5);
                s1=solution(1); s2=solution(2); s3=solution(3); 
                s10 = solution(4); s11=solution(5); 
            else
                aPointsN = solution(10:11);
                s1=solution(1); s2=solution(2); s3=solution(3); 
                s10 = solution(10); s11=solution(11); 
            end 
        else
            [EE,GG,CC] = RefineEquA66B3(p, W);
            apointsIni = [apoint(3) apoint(4)]';
            solutionO = [Cayley(RBt);apointsIni]; 
            [solution, obj_cur] = RefineGaussNewtonRandApT66B3(solutionO, p, W, EE, GG, CC, Lheight);

            if size(solution) ~= 11
                aPointsN = solution(4:5);
                s1=solution(1); s2=solution(2); s3=solution(3); 
                s10 = solution(4); s11=solution(5); 
            else
                aPointsN = solution(10:11);
                s1=solution(1); s2=solution(2); s3=solution(3); 
                s10 = solution(10); s11=solution(11); 
            end 
        end 
        
        s4=s1^2; s5=s1*s2; s6=s1*s3;
        s7=s2^2; s8=s2*s3; s9=s3^2;
        w=[1,s1,s2,s3,s4,s5,s6,s7,s8,s9,...
           s10,s10*s1,s10*s2,s10*s4,s10*s6,s10*s7,s10*s8,s10*s9,...
           s11,s11*s1,s11*s2,s11*s4,s11*s6,s11*s7,s11*s8,s11*s9,...
           s1^2,s1*s2,s1*s3,s2^2,s2*s3,s3^2]';
        factor=1/(1+s1^2+s2^2+s3^2);     
        Rf=factor*[1+s1^2-s2^2-s3^2,2*s1*s2-2*s3,2*s2+2*s1*s3;
                       2*s3+2*s1*s2,1-s1^2+s2^2-s3^2,2*s2*s3-2*s1;
                       2*s1*s3-2*s2,2*s1+2*s2*s3,1-s1^2-s2^2+s3^2];
        Tf =  factor*CC*w;

        if errAmin < errBmin
            tempW = P_W3;
            tempW(1,n) = aPointsN(1);
            tempW(2,n) = aPointsN(2);
            Pa = tempW;
            p2a = p_w3;
            [errep760{j},Urep760{j}]=reprojection_error_usingRT(Pa,p2a,Rf,Tf,A);
        else
            tempW = P_W4;
            tempW(1,n) = aPointsN(1);
            tempW(2,n) = aPointsN(2);
            Pa = tempW;
            p2a = p_w4;
            [errep760{j},Urep760{j}]=reprojection_error_usingRT(Pa,p2a,Rf,Tf,A);
        end



        R76{j} = Rf;
        T76{j} = Tf;
        error76{j} = obj_cur; 
        errRf76{j} = cal_rotation_err(Rf,Rtruth);
        errTf76{j} = cal_translation_err(Tf,Ttruth);


        [EEt,GGt,CCt] = RefineEquA66A3(p, W);
        apointsInit = [apoint(1) apoint(2)]';
        solutionOt = [Cayley(RAt);apointsInit]; 
        [solutiont, obj_curt] = RefineGaussNewtonRandApT66A3(solutionOt, p, W, EEt, GGt, CCt, Lheight);


        if size(solutiont) ~= 11
            aPointsNt = solutiont(4:5);
            s1=solutiont(1); s2=solutiont(2); s3=solutiont(3); 
            s10 = solutiont(4); s11=solutiont(5); 
        else
            aPointsNt = solutiont(10:11);
            s1=solutiont(1); s2=solutiont(2); s3=solutiont(3); 
            s10 = solutiont(10); s11=solutiont(11); 
        end
        
        
        s4=s1^2; s5=s1*s2; s6=s1*s3;
        s7=s2^2; s8=s2*s3; s9=s3^2;
        w=[1,s1,s2,s3,s4,s5,s6,s7,s8,s9,...
           s10,s10*s1,s10*s2,s10*s4,s10*s6,s10*s7,s10*s8,s10*s9,...
           s11,s11*s1,s11*s2,s11*s4,s11*s6,s11*s7,s11*s8,s11*s9,...
           s1^2,s1*s2,s1*s3,s2^2,s2*s3,s3^2]';
        factor=1/(1+s1^2+s2^2+s3^2);     
        Rtf=factor*[1+s1^2-s2^2-s3^2,2*s1*s2-2*s3,2*s2+2*s1*s3;
                       2*s3+2*s1*s2,1-s1^2+s2^2-s3^2,2*s2*s3-2*s1;
                       2*s1*s3-2*s2,2*s1+2*s2*s3,1-s1^2-s2^2+s3^2];
        Ttf =  factor*CCt*w;  

        tempW = P_W3;
        tempW(1,n) = aPointsNt(1);
        tempW(2,n) = aPointsNt(2);
        Pa = tempW;
        p2a = p_w3;
        [errep761{j},Urep761{j}]=reprojection_error_usingRT(Pa,p2a,Rtf,Ttf,A);

        Rth76{j} = Rtf;
        Tth76{j} = Ttf;
        errortf76{j} = obj_cur; 
        errRtf76{j} = cal_rotation_err(Rtf,Rtruth);
        errTtf76{j} = cal_translation_err(Ttf,Ttruth);


        [EEs,GGs,CCs] = RefineEquA66B3(p, W);
        apointsInis = [apoint(3) apoint(4)]';
        solutionOs = [Cayley(RBt);apointsInis]; 
        [solutions, obj_cur] = RefineGaussNewtonRandApT66B3(solutionOs, p, W, EEs, GGs, CCs, Lheight);

        if size(solutions) ~= 11
            aPointsNs = solutions(4:5);
            s1=solutions(1); s2=solutions(2); s3=solutions(3); 
            s10 = solutions(4); s11=solutions(5); 
        else
            aPointsNs = solutions(10:11);
            s1=solutions(1); s2=solutions(2); s3=solutions(3); 
            s10 = solutions(10); s11=solutions(11); 
        end  

        s4=s1^2; s5=s1*s2; s6=s1*s3;
        s7=s2^2; s8=s2*s3; s9=s3^2;
        w=[1,s1,s2,s3,s4,s5,s6,s7,s8,s9,...
           s10,s10*s1,s10*s2,s10*s4,s10*s6,s10*s7,s10*s8,s10*s9,...
           s11,s11*s1,s11*s2,s11*s4,s11*s6,s11*s7,s11*s8,s11*s9,...
           s1^2,s1*s2,s1*s3,s2^2,s2*s3,s3^2]';
        factor=1/(1+s1^2+s2^2+s3^2);     
        Rfs=factor*[1+s1^2-s2^2-s3^2,2*s1*s2-2*s3,2*s2+2*s1*s3;
                       2*s3+2*s1*s2,1-s1^2+s2^2-s3^2,2*s2*s3-2*s1;
                       2*s1*s3-2*s2,2*s1+2*s2*s3,1-s1^2-s2^2+s3^2];
        Tfs =  factor*CCs*w; 

        tempW = P_W4;
        tempW(1,n) = aPointsNs(1);
        tempW(2,n) = aPointsNs(2);
        Pa = tempW;
        p2a = p_w4;
        [errep762{j},Urep762{j}]=reprojection_error_usingRT(Pa,p2a,Rfs,Tfs,A);

        Rts76{j} = Rfs;
        Tts76{j} = Tfs;
        errortfs76{j} = obj_cur; 
        errRtfs76{j} = cal_rotation_err(Rfs,Rtruth);
        errTtfs76{j} = cal_translation_err(Tfs,Ttruth);      
    end
    
    toc;
    time76{j} = toc;

%     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%     clearvars apointa apointb aPointsN aPointsNt aPointsNs apointsIni apointsInis apointsInit apointsIniu apointsIniv aPointsNu aPointsNv T1P1_wA T1P2_wA T1P1_wB T1P2_wB T1P1_wC T1P2_wC T1P1_wD T1P2_wD Rf Tf Rtf Ttf Rfs Tfs Ruf Tuf Rfv Tfv F1
%     tic;
% 
%     T1P1_wA = [P1_w(1,:);P2_w(1,:);P1_w(1,:)];
%     T1P2_wA = [P1_w(2,:);P2_w(2,:);P2_w(2,:)];
% 
%     T1P1_wB = [P1_w(1,:);P2_w(1,:);P1_w(2,:)];
%     T1P2_wB = [P1_w(2,:);P2_w(2,:);P2_w(1,:)];
% 
%     T1P1_wC = [P1_w(1,:);P2_w(1,:);P1_w(1,:)];
%     T1P2_wC = [P1_w(2,:);P2_w(2,:);P2_w(1,:)]; 
% 
%     T1P1_wD = [P1_w(1,:);P2_w(1,:);P1_w(2,:)];
%     T1P2_wD = [P1_w(2,:);P2_w(2,:);P2_w(2,:)];
% 
%     [F1, G] = P2LMGA67(T1p1A, T1p2A, T1P1_wA, T1P2_wA, T1p1B, T1p2B, T1P1_wB, T1P2_wB, T1p1C, T1p2C, T1P1_wC, T1P2_wC, T1p1D, T1p2D, T1P1_wD, T1P2_wD, A, C_truth, Lheight);
%     errAmin = Inf;
%     errBmin = Inf;
%     errCmin = Inf;
%     errDmin = Inf;
%     Ri = eye(3,3);
%     Ti = [0,0,0];
%     apointa = [-1,-1];
%     apointb = [-1,-1];
%     for i = 1:length(F1)
% %             if F1(i).ValObjective(1) > 1 || F1(i).ValObjective(2) > 1 || F1(i).ValObjective(3) > 1 || F1(i).ValObjective(4) > 1
% %         if F1(i).ValObjective(1) > 5 || F1(i).ValObjective(2) > 5 || F1(i).ValObjective(3) > 5 || F1(i).ValObjective(4) > 5 || F1(i).ValObjective(5) > 50 || F1(i).ValObjective(6) > 50
% %     %         if F1(i).ValObjective(1) + F1(i).ValObjective(2) > 2
% %             continue
% %         end
%     %         apoints(count,:) =  F1(i).Val;   
%     %         count = count + 1;
%         if F1(i).ValObjective(1) < errAmin 
%             apointa(1) = F1(i).Val(1);
%             apointa(4) = F1(i).Val(4);
%             RAt = F1(i).RA;
%             TAt = F1(i).TA;
%             errAmin = F1(i).ValObjective(1);
%         end
%         if F1(i).ValObjective(2) < errBmin 
%             apointa(2) = F1(i).Val(2);
%             apointa(3) = F1(i).Val(3);
%             RBt = F1(i).RB;
%             TBt = F1(i).TB;
%             errBmin = F1(i).ValObjective(2);
%         end 
%         if F1(i).ValObjective(3) < errCmin 
%             apointb(1) = F1(i).Val(1);
%             apointb(2) = F1(i).Val(2);
%             RCt = F1(i).RC;
%             TCt = F1(i).TC;
%             errCmin = F1(i).ValObjective(3);
%         end
%         if F1(i).ValObjective(4) < errDmin 
%             apointb(3) = F1(i).Val(3);
%             apointb(4) = F1(i).Val(4);
%             RDt = F1(i).RD;
%             TDt = F1(i).TD;
%             errDmin = F1(i).ValObjective(4);
%         end 
%     end
%     
%     [~,n] = max(abs(W2(:,1) - W1(:,1)));
%     
%     if n == 1
%         errg = [errAmin,errBmin,errCmin,errDmin];
%         %         errOriRf67 = cal_rotation_err(Ri,Rtruth);
%         %         errOriTf67 = cal_translation_err(Ti,Ttruth);
%         [~,flag] = min(errg);
% 
%         if flag  == 1
% 
%             [EE,GG,CC] = RefineEquA67A1(p, W);
%             apointsIni = [apointa(1) apointa(4)]';
%             solutionO = [Cayley(RAt);apointsIni]; 
%             [solution, obj_cur] = RefineGaussNewtonRandApT67A1(solutionO, p, W, EE, GG, CC, Lheight);
% 
% 
%             if size(solution) ~= 11
%                 aPointsN = solution(4:5);
%                 s1=solution(1); s2=solution(2); s3=solution(3); 
%                 s10 = solution(4); s11=solution(5); 
%             else
%                 aPointsN = solution(10:11);
%                 s1=solution(1); s2=solution(2); s3=solution(3); 
%                 s10 = solution(10); s11=solution(11); 
%             end
%         elseif flag == 2
%             [EE,GG,CC] = RefineEquA67B1(p, W);
%             apointsIni = [apointa(2) apointa(3)]';
%             solutionO = [Cayley(RBt);apointsIni]; 
%             [solution, obj_cur] = RefineGaussNewtonRandApT67B1(solutionO, p, W, EE, GG, CC, Lheight);
% 
%             if size(solution) ~= 11
%                 aPointsN = solution(4:5);
%                 s1=solution(1); s2=solution(2); s3=solution(3); 
%                 s10 = solution(4); s11=solution(5); 
%             else
%                 aPointsN = solution(10:11);
%                 s1=solution(1); s2=solution(2); s3=solution(3); 
%                 s10 = solution(10); s11=solution(11); 
%             end
%         elseif flag == 3
%             [EE,GG,CC] = RefineEquA67C1(p, W);
%             apointsIni = [apointb(1) apointb(2)]';
%             solutionO = [Cayley(RCt);apointsIni]; 
%             [solution, obj_cur] = RefineGaussNewtonRandApT67C1(solutionO, p, W, EE, GG, CC, Lheight);
% 
%             if size(solution) ~= 11
%                 aPointsN = solution(4:5);
%                 s1=solution(1); s2=solution(2); s3=solution(3); 
%                 s10 = solution(4); s11=solution(5); 
%             else
%                 aPointsN = solution(10:11);
%                 s1=solution(1); s2=solution(2); s3=solution(3); 
%                 s10 = solution(10); s11=solution(11); 
%             end
%         elseif flag == 4
%             [EE,GG,CC] = RefineEquA67D1(p, W);
%             apointsIni = [apointb(3) apointb(4)]';
%             solutionO = [Cayley(RDt);apointsIni]; 
%             [solution, obj_cur] = RefineGaussNewtonRandApT67D1(solutionO, p, W, EE, GG, CC, Lheight);
% 
%             if size(solution) ~= 11
%                 aPointsN = solution(4:5);
%                 s1=solution(1); s2=solution(2); s3=solution(3); 
%                 s10 = solution(4); s11=solution(5); 
%             else
%                 aPointsN = solution(10:11);
%                 s1=solution(1); s2=solution(2); s3=solution(3); 
%                 s10 = solution(10); s11=solution(11); 
%             end
%         end
%         
%         s4=s1^2; s5=s1*s2; s6=s1*s3;
%         s7=s2^2; s8=s2*s3; s9=s3^2;
%         w=[1,s1,s2,s3,s4,s5,s6,s7,s8,s9,...
%            s10,s10*s2,s10*s3,s10*s4,s10*s5,s10*s6,s10*s7,s10*s9,...
%            s11,s11*s2,s11*s3,s11*s4,s11*s5,s11*s6,s11*s7,s11*s9,...
%            s1^2,s1*s2,s1*s3,s2^2,s2*s3,s3^2]';
%         factor=1/(1+s1^2+s2^2+s3^2);     
%         Rf=factor*[1+s1^2-s2^2-s3^2,2*s1*s2-2*s3,2*s2+2*s1*s3;
%                        2*s3+2*s1*s2,1-s1^2+s2^2-s3^2,2*s2*s3-2*s1;
%                        2*s1*s3-2*s2,2*s1+2*s2*s3,1-s1^2-s2^2+s3^2];
%         Tf =  factor*CC*w;
% 
%         if flag  == 1
%             tempW = P_W1;
%             tempW(1,n) = aPointsN(1);
%             tempW(2,n) = aPointsN(2);
%             Pa = tempW;
%             p2a = p_w1;
%             [errep770{j},Urep770{j}]=reprojection_error_usingRT(Pa,p2a,Rf,Tf,A);
%         elseif flag  == 2
%             tempW = P_W2;
%             tempW(1,n) = aPointsN(1);
%             tempW(2,n) = aPointsN(2);
%             Pa = tempW;
%             p2a = p_w2;
%             [errep770{j},Urep770{j}]=reprojection_error_usingRT(Pa,p2a,Rf,Tf,A);
%         elseif flag == 3
%             tempW = P_W3;
%             tempW(1,n) = aPointsN(1);
%             tempW(2,n) = aPointsN(2);
%             Pa = tempW;
%             p2a = p_w3;
%             [errep770{j},Urep770{j}]=reprojection_error_usingRT(Pa,p2a,Rf,Tf,A);
%         else
%             tempW = P_W4;
%             tempW(1,n) = aPointsN(1);
%             tempW(2,n) = aPointsN(2);
%             Pa = tempW;
%             p2a = p_w4;
%             [errep770{j},Urep770{j}]=reprojection_error_usingRT(Pa,p2a,Rf,Tf,A);
%         end
% 
% 
% 
%         R77{j} = Rf;
%         T77{j} = Tf;
%         error77{j} = obj_cur; 
%         errRf77{j} = cal_rotation_err(Rf,Rtruth);
%         errTf77{j} = cal_translation_err(Tf,Ttruth);
% 
%         [EEt,GGt,CCt] = RefineEquA67A1(p, W);
%         apointsInit = [apointa(1) apointa(4)]';
%         solutionOt = [Cayley(RAt);apointsInit]; 
%         [solutiont, obj_curt] = RefineGaussNewtonRandApT67A1(solutionOt, p, W, EEt, GGt, CCt, Lheight);
% 
% 
%         if size(solutiont) ~= 11
%             aPointsNt = solutiont(4:5);
%             s1=solutiont(1); s2=solutiont(2); s3=solutiont(3); 
%             s10 = solutiont(4); s11=solutiont(5); 
%         else
%             aPointsNt = solutiont(10:11);
%             s1=solutiont(1); s2=solutiont(2); s3=solutiont(3); 
%             s10 = solutiont(10); s11=solutiont(11); 
%         end
%         
%         
%         s4=s1^2; s5=s1*s2; s6=s1*s3;
%         s7=s2^2; s8=s2*s3; s9=s3^2;
%         w=[1,s1,s2,s3,s4,s5,s6,s7,s8,s9,...
%            s10,s10*s2,s10*s3,s10*s4,s10*s5,s10*s6,s10*s7,s10*s9,...
%            s11,s11*s2,s11*s3,s11*s4,s11*s5,s11*s6,s11*s7,s11*s9,...
%            s1^2,s1*s2,s1*s3,s2^2,s2*s3,s3^2]';
%         factor=1/(1+s1^2+s2^2+s3^2);     
%         Rtf=factor*[1+s1^2-s2^2-s3^2,2*s1*s2-2*s3,2*s2+2*s1*s3;
%                        2*s3+2*s1*s2,1-s1^2+s2^2-s3^2,2*s2*s3-2*s1;
%                        2*s1*s3-2*s2,2*s1+2*s2*s3,1-s1^2-s2^2+s3^2];
%         Ttf =  factor*CCt*w;
% 
% 
%         tempW = P_W1;
%         tempW(1,n) = aPointsNt(1);
%         tempW(2,n) = aPointsNt(2);
%         Pa = tempW;
%         p2a = p_w1;
%         [errep771{j},Urep771{j}]=reprojection_error_usingRT(Pa,p2a,Rtf,Ttf,A);
% 
%         Rth77{j} = Rtf;
%         Tth77{j} = Ttf;
%         errortf77{j} = obj_cur; 
%         errRtf77{j} = cal_rotation_err(Rtf,Rtruth);
%         errTtf77{j} = cal_translation_err(Ttf,Ttruth);
% 
% 
% 
%         [EEs,GGs,CCs] = RefineEquA67B1(p, W);
%         apointsInis = [apointa(2) apointa(3)]';
%         solutionOs = [Cayley(RBt);apointsInis]; 
%         [solutions, obj_cur] = RefineGaussNewtonRandApT67B1(solutionOs, p, W, EEs, GGs, CCs, Lheight);
% 
%         if size(solutions) ~= 11
%             aPointsNs = solutions(4:5);
%             s1=solutions(1); s2=solutions(2); s3=solutions(3); 
%             s10 = solutions(4); s11=solutions(5); 
%         else
%             aPointsNs = solutions(10:11);
%             s1=solutions(1); s2=solutions(2); s3=solutions(3); 
%             s10 = solutions(10); s11=solutions(11); 
%         end  
% 
%         s4=s1^2; s5=s1*s2; s6=s1*s3;
%         s7=s2^2; s8=s2*s3; s9=s3^2;
%         w=[1,s1,s2,s3,s4,s5,s6,s7,s8,s9,...
%            s10,s10*s2,s10*s3,s10*s4,s10*s5,s10*s6,s10*s7,s10*s9,...
%            s11,s11*s2,s11*s3,s11*s4,s11*s5,s11*s6,s11*s7,s11*s9,...
%            s1^2,s1*s2,s1*s3,s2^2,s2*s3,s3^2]';
%         factor=1/(1+s1^2+s2^2+s3^2);     
%         Rfs=factor*[1+s1^2-s2^2-s3^2,2*s1*s2-2*s3,2*s2+2*s1*s3;
%                        2*s3+2*s1*s2,1-s1^2+s2^2-s3^2,2*s2*s3-2*s1;
%                        2*s1*s3-2*s2,2*s1+2*s2*s3,1-s1^2-s2^2+s3^2];
%         Tfs =  factor*CCs*w; 
% 
%         tempW = P_W2;
%         tempW(1,2) = aPointsNs(1);
%         tempW(2,2) = aPointsNs(2);
%         Pa = tempW;
%         p2a = p_w2;
%         [errep772{j},Urep772{j}]=reprojection_error_usingRT(Pa,p2a,Rfs,Tfs,A);
% 
%         Rts77{j} = Rfs;
%         Tts77{j} = Tfs;
%         errortfs77{j} = obj_cur; 
%         errRtfs77{j} = cal_rotation_err(Rfs,Rtruth);
%         errTtfs77{j} = cal_translation_err(Tfs,Ttruth);
% 
% 
%         [EEu,GGu,CCu] = RefineEquA67C1(p, W);
%         apointsIniu = [apointb(1) apointb(2)]';
%         solutionOu = [Cayley(RCt);apointsIniu]; 
%         [solutionu, obj_curt] = RefineGaussNewtonRandApT67C1(solutionOu, p, W, EEu, GGu, CCu, Lheight);
% 
%     %     aPointsNu = [apoint(1) apoint(2)]';
% 
%         if size(solutionu) ~= 11
%             aPointsNu = solutionu(4:5);
%             s1=solutionu(1); s2=solutionu(2); s3=solutionu(3); 
%             s10 = solutionu(4); s11=solutionu(5); 
%         else
%             aPointsNu = solutionu(10:11);
%             s1=solutionu(1); s2=solutionu(2); s3=solutionu(3); 
%             s10 = solutionu(10); s11=solutionu(11); 
%         end  
% 
%         s4=s1^2; s5=s1*s2; s6=s1*s3;
%         s7=s2^2; s8=s2*s3; s9=s3^2;
%         w=[1,s1,s2,s3,s4,s5,s6,s7,s8,s9,...
%            s10,s10*s2,s10*s3,s10*s4,s10*s5,s10*s6,s10*s7,s10*s9,...
%            s11,s11*s2,s11*s3,s11*s4,s11*s5,s11*s6,s11*s7,s11*s9,...
%            s1^2,s1*s2,s1*s3,s2^2,s2*s3,s3^2]';
%         factor=1/(1+s1^2+s2^2+s3^2);     
%         Ruf=factor*[1+s1^2-s2^2-s3^2,2*s1*s2-2*s3,2*s2+2*s1*s3;
%                        2*s3+2*s1*s2,1-s1^2+s2^2-s3^2,2*s2*s3-2*s1;
%                        2*s1*s3-2*s2,2*s1+2*s2*s3,1-s1^2-s2^2+s3^2];
%         Tuf =  factor*CCs*w; 
%         
%         tempW = P_W3;
%         tempW(1,n) = aPointsNu(1);
%         tempW(2,n) = aPointsNu(2);
%         Pa = tempW;
%         p2a = p_w3;
%         [errep773{j},Urep773{j}]=reprojection_error_usingRT(Pa,p2a,Ruf,Tuf,A);
% 
%         Rthu77{j} = Ruf;
%         Tthu77{j} = Tuf;
%         errortfu77(j) = obj_cur; 
%         errRtfu77(j) = cal_rotation_err(Ruf,Rtruth);
%         errTtfu77(j) = cal_translation_err(Tuf,Ttruth);
% 
% 
% 
% 
%         [EEv,GGv,CCv] = RefineEquA67D1(p, W);
%         apointsIniv = [apointb(3) apointb(4)]';
%         solutionOv = [Cayley(RDt);apointsIniv]; 
%         [solutionv, obj_cur] = RefineGaussNewtonRandApT67D1(solutionOv, p, W, EEv, GGv, CCv, Lheight);
% 
%         if size(solutionv) ~= 11
%             aPointsNv = solutionv(4:5);
%             s1=solutionv(1); s2=solutionv(2); s3=solutionv(3); 
%             s10 = solutionv(4); s11=solutionv(5); 
%         else
%             aPointsNv = solutionv(10:11);
%             s1=solutionv(1); s2=solutionv(2); s3=solutionv(3); 
%             s10 = solutionv(10); s11=solutionv(11); 
%         end  
% 
%         s4=s1^2; s5=s1*s2; s6=s1*s3;
%         s7=s2^2; s8=s2*s3; s9=s3^2;
%         w=[1,s1,s2,s3,s4,s5,s6,s7,s8,s9,...
%            s10,s10*s2,s10*s3,s10*s4,s10*s5,s10*s6,s10*s7,s10*s9,...
%            s11,s11*s2,s11*s3,s11*s4,s11*s5,s11*s6,s11*s7,s11*s9,...
%            s1^2,s1*s2,s1*s3,s2^2,s2*s3,s3^2]';
%         factor=1/(1+s1^2+s2^2+s3^2);     
%         Rfv=factor*[1+s1^2-s2^2-s3^2,2*s1*s2-2*s3,2*s2+2*s1*s3;
%                        2*s3+2*s1*s2,1-s1^2+s2^2-s3^2,2*s2*s3-2*s1;
%                        2*s1*s3-2*s2,2*s1+2*s2*s3,1-s1^2-s2^2+s3^2];
%         Tfv =  factor*CCs*w; 
%         
% 
%         tempW = P_W4;
%         tempW(1,n) = aPointsNv(1);
%         tempW(2,n) = aPointsNv(2);
%         Pa = tempW;
%         p2a = p_w4;
%         [errep774{j},Urep774{j}]=reprojection_error_usingRT(Pa,p2a,Rfv,Tfv,A);
% 
%         Rtv77{j} = Rfv;
%         Ttv77{j} = Tfv;
%         errortfv77(j) = obj_cur; 
%         errRtfv77(j) = cal_rotation_err(Rfv,Rtruth);
%         errTtfv77(j) = cal_translation_err(Tfv,Ttruth);
%        
%     elseif n == 2
% 
%         errg = [errAmin,errBmin,errCmin,errDmin];
%         %         errOriRf67 = cal_rotation_err(Ri,Rtruth);
%         %         errOriTf67 = cal_translation_err(Ti,Ttruth);
%         [~,flag] = min(errg);
% 
%         if flag  == 1
% 
%             [EE,GG,CC] = RefineEquA67A2(p, W);
%             apointsIni = [apointa(1) apointa(4)]';
%             solutionO = [Cayley(RAt);apointsIni]; 
%             [solution, obj_cur] = RefineGaussNewtonRandApT67A2(solutionO, p, W, EE, GG, CC, Lheight);
% 
% 
%             if size(solution) ~= 11
%                 aPointsN = solution(4:5);
%                 s1=solution(1); s2=solution(2); s3=solution(3); 
%                 s10 = solution(4); s11=solution(5); 
%             else
%                 aPointsN = solution(10:11);
%                 s1=solution(1); s2=solution(2); s3=solution(3); 
%                 s10 = solution(10); s11=solution(11); 
%             end
%         elseif flag == 2
%             [EE,GG,CC] = RefineEquA67B2(p, W);
%             apointsIni = [apointa(2) apointa(3)]';
%             solutionO = [Cayley(RBt);apointsIni]; 
%             [solution, obj_cur] = RefineGaussNewtonRandApT67B2(solutionO, p, W, EE, GG, CC, Lheight);
% 
%             if size(solution) ~= 11
%                 aPointsN = solution(4:5);
%                 s1=solution(1); s2=solution(2); s3=solution(3); 
%                 s10 = solution(4); s11=solution(5); 
%             else
%                 aPointsN = solution(10:11);
%                 s1=solution(1); s2=solution(2); s3=solution(3); 
%                 s10 = solution(10); s11=solution(11); 
%             end
%         elseif flag == 3
%             [EE,GG,CC] = RefineEquA67C2(p, W);
%             apointsIni = [apointb(1) apointb(2)]';
%             solutionO = [Cayley(RCt);apointsIni]; 
%             [solution, obj_cur] = RefineGaussNewtonRandApT67C2(solutionO, p, W, EE, GG, CC, Lheight);
% 
%             if size(solution) ~= 11
%                 aPointsN = solution(4:5);
%                 s1=solution(1); s2=solution(2); s3=solution(3); 
%                 s10 = solution(4); s11=solution(5); 
%             else
%                 aPointsN = solution(10:11);
%                 s1=solution(1); s2=solution(2); s3=solution(3); 
%                 s10 = solution(10); s11=solution(11); 
%             end
%         elseif flag == 4
%             [EE,GG,CC] = RefineEquA67D2(p, W);
%             apointsIni = [apointb(3) apointb(4)]';
%             solutionO = [Cayley(RDt);apointsIni]; 
%             [solution, obj_cur] = RefineGaussNewtonRandApT67D2(solutionO, p, W, EE, GG, CC, Lheight);
% 
%             if size(solution) ~= 11
%                 aPointsN = solution(4:5);
%                 s1=solution(1); s2=solution(2); s3=solution(3); 
%                 s10 = solution(4); s11=solution(5); 
%             else
%                 aPointsN = solution(10:11);
%                 s1=solution(1); s2=solution(2); s3=solution(3); 
%                 s10 = solution(10); s11=solution(11); 
%             end
%         end
%         
%         s4=s1^2; s5=s1*s2; s6=s1*s3;
%         s7=s2^2; s8=s2*s3; s9=s3^2;
%         w=[1,s1,s2,s3,s4,s5,s6,s7,s8,s9,...
%            s10,s10*s1,s10*s3,s10*s4,s10*s5,s10*s7,s10*s8,s10*s9,...
%            s11,s11*s1,s11*s3,s11*s4,s11*s5,s11*s7,s11*s8,s11*s9,...
%            s1^2,s1*s2,s1*s3,s2^2,s2*s3,s3^2]';
%         factor=1/(1+s1^2+s2^2+s3^2);     
%         Rf=factor*[1+s1^2-s2^2-s3^2,2*s1*s2-2*s3,2*s2+2*s1*s3;
%                        2*s3+2*s1*s2,1-s1^2+s2^2-s3^2,2*s2*s3-2*s1;
%                        2*s1*s3-2*s2,2*s1+2*s2*s3,1-s1^2-s2^2+s3^2];
%         Tf =  factor*CC*w;
% 
%         if flag  == 1
%             tempW = P_W1;
%             tempW(1,n) = aPointsN(1);
%             tempW(2,n) = aPointsN(2);
%             Pa = tempW;
%             p2a = p_w1;
%             [errep770{j},Urep770{j}]=reprojection_error_usingRT(Pa,p2a,Rf,Tf,A);
%         elseif flag  == 2
%             tempW = P_W2;
%             tempW(1,n) = aPointsN(1);
%             tempW(2,n) = aPointsN(2);
%             Pa = tempW;
%             p2a = p_w2;
%             [errep770{j},Urep770{j}]=reprojection_error_usingRT(Pa,p2a,Rf,Tf,A);
%         elseif flag == 3
%             tempW = P_W3;
%             tempW(1,n) = aPointsN(1);
%             tempW(2,n) = aPointsN(2);
%             Pa = tempW;
%             p2a = p_w3;
%             [errep770{j},Urep770{j}]=reprojection_error_usingRT(Pa,p2a,Rf,Tf,A);
%         else
%             tempW = P_W4;
%             tempW(1,n) = aPointsN(1);
%             tempW(2,n) = aPointsN(2);
%             Pa = tempW;
%             p2a = p_w4;
%             [errep770{j},Urep770{j}]=reprojection_error_usingRT(Pa,p2a,Rf,Tf,A);
%         end
% 
% 
%         R77{j} = Rf;
%         T77{j} = Tf;
%         error77{j} = obj_cur; 
%         errRf77{j} = cal_rotation_err(Rf,Rtruth);
%         errTf77{j} = cal_translation_err(Tf,Ttruth);
% 
%         [EEt,GGt,CCt] = RefineEquA67A2(p, W);
%         apointsInit = [apointa(1) apointa(4)]';
%         solutionOt = [Cayley(RAt);apointsInit]; 
%         [solutiont, obj_curt] = RefineGaussNewtonRandApT67A2(solutionOt, p, W, EEt, GGt, CCt, Lheight);
% 
% 
%         if size(solutiont) ~= 11
%             aPointsNt = solutiont(4:5);
%             s1=solutiont(1); s2=solutiont(2); s3=solutiont(3); 
%             s10 = solutiont(4); s11=solutiont(5); 
%         else
%             aPointsNt = solutiont(10:11);
%             s1=solutiont(1); s2=solutiont(2); s3=solutiont(3); 
%             s10 = solutiont(10); s11=solutiont(11); 
%         end
%         
%         
%         s4=s1^2; s5=s1*s2; s6=s1*s3;
%         s7=s2^2; s8=s2*s3; s9=s3^2;
%         w=[1,s1,s2,s3,s4,s5,s6,s7,s8,s9,...
%            s10,s10*s1,s10*s3,s10*s4,s10*s5,s10*s7,s10*s8,s10*s9,...
%            s11,s11*s1,s11*s3,s11*s4,s11*s5,s11*s7,s11*s8,s11*s9,...
%            s1^2,s1*s2,s1*s3,s2^2,s2*s3,s3^2]';
%         factor=1/(1+s1^2+s2^2+s3^2);     
%         Rtf=factor*[1+s1^2-s2^2-s3^2,2*s1*s2-2*s3,2*s2+2*s1*s3;
%                        2*s3+2*s1*s2,1-s1^2+s2^2-s3^2,2*s2*s3-2*s1;
%                        2*s1*s3-2*s2,2*s1+2*s2*s3,1-s1^2-s2^2+s3^2];
%         Ttf =  factor*CCt*w;
% 
% 
%         tempW = P_W1;
%         tempW(1,n) = aPointsNt(1);
%         tempW(2,n) = aPointsNt(2);
%         Pa = tempW;
%         p2a = p_w1;
%         [errep771{j},Urep771{j}]=reprojection_error_usingRT(Pa,p2a,Rtf,Ttf,A);
% 
%         Rth77{j} = Rtf;
%         Tth77{j} = Ttf;
%         errortf77{j} = obj_cur; 
%         errRtf77{j} = cal_rotation_err(Rtf,Rtruth);
%         errTtf77{j} = cal_translation_err(Ttf,Ttruth);
% 
% 
% 
%         [EEs,GGs,CCs] = RefineEquA67B2(p, W);
%         apointsInis = [apointa(2) apointa(3)]';
%         solutionOs = [Cayley(RBt);apointsInis]; 
%         [solutions, obj_cur] = RefineGaussNewtonRandApT67B2(solutionOs, p, W, EEs, GGs, CCs, Lheight);
% 
%         if size(solutions) ~= 11
%             aPointsNs = solutions(4:5);
%             s1=solutions(1); s2=solutions(2); s3=solutions(3); 
%             s10 = solutions(4); s11=solutions(5); 
%         else
%             aPointsNs = solutions(10:11);
%             s1=solutions(1); s2=solutions(2); s3=solutions(3); 
%             s10 = solutions(10); s11=solutions(11); 
%         end  
% 
%         s4=s1^2; s5=s1*s2; s6=s1*s3;
%         s7=s2^2; s8=s2*s3; s9=s3^2;
%         w=[1,s1,s2,s3,s4,s5,s6,s7,s8,s9,...
%            s10,s10*s1,s10*s3,s10*s4,s10*s5,s10*s7,s10*s8,s10*s9,...
%            s11,s11*s1,s11*s3,s11*s4,s11*s5,s11*s7,s11*s8,s11*s9,...
%            s1^2,s1*s2,s1*s3,s2^2,s2*s3,s3^2]';
%         factor=1/(1+s1^2+s2^2+s3^2);     
%         Rfs=factor*[1+s1^2-s2^2-s3^2,2*s1*s2-2*s3,2*s2+2*s1*s3;
%                        2*s3+2*s1*s2,1-s1^2+s2^2-s3^2,2*s2*s3-2*s1;
%                        2*s1*s3-2*s2,2*s1+2*s2*s3,1-s1^2-s2^2+s3^2];
%         Tfs =  factor*CCs*w; 
% 
%         tempW = P_W2;
%         tempW(1,2) = aPointsNs(1);
%         tempW(2,2) = aPointsNs(2);
%         Pa = tempW;
%         p2a = p_w2;
%         [errep772{j},Urep772{j}]=reprojection_error_usingRT(Pa,p2a,Rfs,Tfs,A);
% 
%         Rts77{j} = Rfs;
%         Tts77{j} = Tfs;
%         errortfs77{j} = obj_cur; 
%         errRtfs77{j} = cal_rotation_err(Rfs,Rtruth);
%         errTtfs77{j} = cal_translation_err(Tfs,Ttruth);
% 
% 
%         [EEu,GGu,CCu] = RefineEquA67C2(p, W);
%         apointsIniu = [apointb(1) apointb(2)]';
%         solutionOu = [Cayley(RCt);apointsIniu]; 
%         [solutionu, obj_curt] = RefineGaussNewtonRandApT67C2(solutionOu, p, W, EEu, GGu, CCu, Lheight);
% 
%     %     aPointsNu = [apoint(1) apoint(2)]';
% 
%         if size(solutionu) ~= 11
%             aPointsNu = solutionu(4:5);
%             s1=solutionu(1); s2=solutionu(2); s3=solutionu(3); 
%             s10 = solutionu(4); s11=solutionu(5); 
%         else
%             aPointsNu = solutionu(10:11);
%             s1=solutionu(1); s2=solutionu(2); s3=solutionu(3); 
%             s10 = solutionu(10); s11=solutionu(11); 
%         end  
% 
%         s4=s1^2; s5=s1*s2; s6=s1*s3;
%         s7=s2^2; s8=s2*s3; s9=s3^2;
%         w=[1,s1,s2,s3,s4,s5,s6,s7,s8,s9,...
%            s10,s10*s1,s10*s3,s10*s4,s10*s5,s10*s7,s10*s8,s10*s9,...
%            s11,s11*s1,s11*s3,s11*s4,s11*s5,s11*s7,s11*s8,s11*s9,...
%            s1^2,s1*s2,s1*s3,s2^2,s2*s3,s3^2]';
%         factor=1/(1+s1^2+s2^2+s3^2);     
%         Ruf=factor*[1+s1^2-s2^2-s3^2,2*s1*s2-2*s3,2*s2+2*s1*s3;
%                        2*s3+2*s1*s2,1-s1^2+s2^2-s3^2,2*s2*s3-2*s1;
%                        2*s1*s3-2*s2,2*s1+2*s2*s3,1-s1^2-s2^2+s3^2];
%         Tuf =  factor*CCs*w; 
%         
%         tempW = P_W3;
%         tempW(1,n) = aPointsNu(1);
%         tempW(2,n) = aPointsNu(2);
%         Pa = tempW;
%         p2a = p_w3;
%         [errep773{j},Urep773{j}]=reprojection_error_usingRT(Pa,p2a,Ruf,Tuf,A);
% 
%         Rthu77{j} = Ruf;
%         Tthu77{j} = Tuf;
%         errortfu77(j) = obj_cur; 
%         errRtfu77(j) = cal_rotation_err(Ruf,Rtruth);
%         errTtfu77(j) = cal_translation_err(Tuf,Ttruth);
% 
% 
% 
% 
%         [EEv,GGv,CCv] = RefineEquA67D2(p, W);
%         apointsIniv = [apointb(3) apointb(4)]';
%         solutionOv = [Cayley(RDt);apointsIniv]; 
%         [solutionv, obj_cur] = RefineGaussNewtonRandApT67D2(solutionOv, p, W, EEv, GGv, CCv, Lheight);
% 
%         if size(solutionv) ~= 11
%             aPointsNv = solutionv(4:5);
%             s1=solutionv(1); s2=solutionv(2); s3=solutionv(3); 
%             s10 = solutionv(4); s11=solutionv(5); 
%         else
%             aPointsNv = solutionv(10:11);
%             s1=solutionv(1); s2=solutionv(2); s3=solutionv(3); 
%             s10 = solutionv(10); s11=solutionv(11); 
%         end  
% 
%         s4=s1^2; s5=s1*s2; s6=s1*s3;
%         s7=s2^2; s8=s2*s3; s9=s3^2;
%         w=[1,s1,s2,s3,s4,s5,s6,s7,s8,s9,...
%            s10,s10*s1,s10*s3,s10*s4,s10*s5,s10*s7,s10*s8,s10*s9,...
%            s11,s11*s1,s11*s3,s11*s4,s11*s5,s11*s7,s11*s8,s11*s9,...
%            s1^2,s1*s2,s1*s3,s2^2,s2*s3,s3^2]';
%         factor=1/(1+s1^2+s2^2+s3^2);     
%         Rfv=factor*[1+s1^2-s2^2-s3^2,2*s1*s2-2*s3,2*s2+2*s1*s3;
%                        2*s3+2*s1*s2,1-s1^2+s2^2-s3^2,2*s2*s3-2*s1;
%                        2*s1*s3-2*s2,2*s1+2*s2*s3,1-s1^2-s2^2+s3^2];
%         Tfv =  factor*CCs*w; 
%         
% 
%         tempW = P_W4;
%         tempW(1,n) = aPointsNv(1);
%         tempW(2,n) = aPointsNv(2);
%         Pa = tempW;
%         p2a = p_w4;
%         [errep774{j},Urep774{j}]=reprojection_error_usingRT(Pa,p2a,Rfv,Tfv,A);
% 
%         Rtv77{j} = Rfv;
%         Ttv77{j} = Tfv;
%         errortfv77(j) = obj_cur; 
%         errRtfv77(j) = cal_rotation_err(Rfv,Rtruth);
%         errTtfv77(j) = cal_translation_err(Tfv,Ttruth);
%     elseif n == 3
%         errg = [errAmin,errBmin,errCmin,errDmin];
%         %         errOriRf67 = cal_rotation_err(Ri,Rtruth);
%         %         errOriTf67 = cal_translation_err(Ti,Ttruth);
%         [~,flag] = min(errg);
% 
%         if flag  == 1
% 
%             [EE,GG,CC] = RefineEquA67A3(p, W);
%             apointsIni = [apointa(1) apointa(4)]';
%             solutionO = [Cayley(RAt);apointsIni]; 
%             [solution, obj_cur] = RefineGaussNewtonRandApT67A3(solutionO, p, W, EE, GG, CC, Lheight);
% 
% 
%             if size(solution) ~= 11
%                 aPointsN = solution(4:5);
%                 s1=solution(1); s2=solution(2); s3=solution(3); 
%                 s10 = solution(4); s11=solution(5); 
%             else
%                 aPointsN = solution(10:11);
%                 s1=solution(1); s2=solution(2); s3=solution(3); 
%                 s10 = solution(10); s11=solution(11); 
%             end
%         elseif flag == 2
%             [EE,GG,CC] = RefineEquA67B3(p, W);
%             apointsIni = [apointa(2) apointa(3)]';
%             solutionO = [Cayley(RBt);apointsIni]; 
%             [solution, obj_cur] = RefineGaussNewtonRandApT67B3(solutionO, p, W, EE, GG, CC, Lheight);
% 
%             if size(solution) ~= 11
%                 aPointsN = solution(4:5);
%                 s1=solution(1); s2=solution(2); s3=solution(3); 
%                 s10 = solution(4); s11=solution(5); 
%             else
%                 aPointsN = solution(10:11);
%                 s1=solution(1); s2=solution(2); s3=solution(3); 
%                 s10 = solution(10); s11=solution(11); 
%             end
%         elseif flag == 3
%             [EE,GG,CC] = RefineEquA67C3(p, W);
%             apointsIni = [apointb(1) apointb(2)]';
%             solutionO = [Cayley(RCt);apointsIni]; 
%             [solution, obj_cur] = RefineGaussNewtonRandApT67C3(solutionO, p, W, EE, GG, CC, Lheight);
% 
%             if size(solution) ~= 11
%                 aPointsN = solution(4:5);
%                 s1=solution(1); s2=solution(2); s3=solution(3); 
%                 s10 = solution(4); s11=solution(5); 
%             else
%                 aPointsN = solution(10:11);
%                 s1=solution(1); s2=solution(2); s3=solution(3); 
%                 s10 = solution(10); s11=solution(11); 
%             end
%         elseif flag == 4
%             [EE,GG,CC] = RefineEquA67D3(p, W);
%             apointsIni = [apointb(3) apointb(4)]';
%             solutionO = [Cayley(RDt);apointsIni]; 
%             [solution, obj_cur] = RefineGaussNewtonRandApT67D3(solutionO, p, W, EE, GG, CC, Lheight);
% 
%             if size(solution) ~= 11
%                 aPointsN = solution(4:5);
%                 s1=solution(1); s2=solution(2); s3=solution(3); 
%                 s10 = solution(4); s11=solution(5); 
%             else
%                 aPointsN = solution(10:11);
%                 s1=solution(1); s2=solution(2); s3=solution(3); 
%                 s10 = solution(10); s11=solution(11); 
%             end
%         end
%         
%         s4=s1^2; s5=s1*s2; s6=s1*s3;
%         s7=s2^2; s8=s2*s3; s9=s3^2;
%         w=[1,s1,s2,s3,s4,s5,s6,s7,s8,s9,...
%            s10,s10*s1,s10*s2,s10*s4,s10*s6,s10*s7,s10*s8,s10*s9,...
%            s11,s11*s1,s11*s2,s11*s4,s11*s6,s11*s7,s11*s8,s11*s9,...
%            s1^2,s1*s2,s1*s3,s2^2,s2*s3,s3^2]';
%         factor=1/(1+s1^2+s2^2+s3^2);     
%         Rf=factor*[1+s1^2-s2^2-s3^2,2*s1*s2-2*s3,2*s2+2*s1*s3;
%                        2*s3+2*s1*s2,1-s1^2+s2^2-s3^2,2*s2*s3-2*s1;
%                        2*s1*s3-2*s2,2*s1+2*s2*s3,1-s1^2-s2^2+s3^2];
%         Tf =  factor*CC*w;
% 
%         if flag  == 1
%             tempW = P_W1;
%             tempW(1,n) = aPointsN(1);
%             tempW(2,n) = aPointsN(2);
%             Pa = tempW;
%             p2a = p_w1;
%             [errep770{j},Urep770{j}]=reprojection_error_usingRT(Pa,p2a,Rf,Tf,A);
%         elseif flag  == 2
%             tempW = P_W2;
%             tempW(1,n) = aPointsN(1);
%             tempW(2,n) = aPointsN(2);
%             Pa = tempW;
%             p2a = p_w2;
%             [errep770{j},Urep770{j}]=reprojection_error_usingRT(Pa,p2a,Rf,Tf,A);
%         elseif flag == 3
%             tempW = P_W3;
%             tempW(1,n) = aPointsN(1);
%             tempW(2,n) = aPointsN(2);
%             Pa = tempW;
%             p2a = p_w3;
%             [errep770{j},Urep770{j}]=reprojection_error_usingRT(Pa,p2a,Rf,Tf,A);
%         else
%             tempW = P_W4;
%             tempW(1,n) = aPointsN(1);
%             tempW(2,n) = aPointsN(2);
%             Pa = tempW;
%             p2a = p_w4;
%             [errep770{j},Urep770{j}]=reprojection_error_usingRT(Pa,p2a,Rf,Tf,A);
%         end
% 
% 
% 
%         R77{j} = Rf;
%         T77{j} = Tf;
%         error77{j} = obj_cur; 
%         errRf77{j} = cal_rotation_err(Rf,Rtruth);
%         errTf77{j} = cal_translation_err(Tf,Ttruth);
% 
%         [EEt,GGt,CCt] = RefineEquA67A3(p, W);
%         apointsInit = [apointa(1) apointa(4)]';
%         solutionOt = [Cayley(RAt);apointsInit]; 
%         [solutiont, obj_curt] = RefineGaussNewtonRandApT67A3(solutionOt, p, W, EEt, GGt, CCt, Lheight);
% 
% 
%         if size(solutiont) ~= 11
%             aPointsNt = solutiont(4:5);
%             s1=solutiont(1); s2=solutiont(2); s3=solutiont(3); 
%             s10 = solutiont(4); s11=solutiont(5); 
%         else
%             aPointsNt = solutiont(10:11);
%             s1=solutiont(1); s2=solutiont(2); s3=solutiont(3); 
%             s10 = solutiont(10); s11=solutiont(11); 
%         end
%         
%         
%         s4=s1^2; s5=s1*s2; s6=s1*s3;
%         s7=s2^2; s8=s2*s3; s9=s3^2;
%         w=[1,s1,s2,s3,s4,s5,s6,s7,s8,s9,...
%            s10,s10*s1,s10*s2,s10*s4,s10*s6,s10*s7,s10*s8,s10*s9,...
%            s11,s11*s1,s11*s2,s11*s4,s11*s6,s11*s7,s11*s8,s11*s9,...
%            s1^2,s1*s2,s1*s3,s2^2,s2*s3,s3^2]';
%         factor=1/(1+s1^2+s2^2+s3^2);     
%         Rtf=factor*[1+s1^2-s2^2-s3^2,2*s1*s2-2*s3,2*s2+2*s1*s3;
%                        2*s3+2*s1*s2,1-s1^2+s2^2-s3^2,2*s2*s3-2*s1;
%                        2*s1*s3-2*s2,2*s1+2*s2*s3,1-s1^2-s2^2+s3^2];
%         Ttf =  factor*CCt*w;
% 
% 
%         tempW = P_W1;
%         tempW(1,n) = aPointsNt(1);
%         tempW(2,n) = aPointsNt(2);
%         Pa = tempW;
%         p2a = p_w1;
%         [errep771{j},Urep771{j}]=reprojection_error_usingRT(Pa,p2a,Rtf,Ttf,A);
% 
%         Rth77{j} = Rtf;
%         Tth77{j} = Ttf;
%         errortf77{j} = obj_cur; 
%         errRtf77{j} = cal_rotation_err(Rtf,Rtruth);
%         errTtf77{j} = cal_translation_err(Ttf,Ttruth);
% 
% 
% 
%         [EEs,GGs,CCs] = RefineEquA67B3(p, W);
%         apointsInis = [apointa(2) apointa(3)]';
%         solutionOs = [Cayley(RBt);apointsInis]; 
%         [solutions, obj_cur] = RefineGaussNewtonRandApT67B3(solutionOs, p, W, EEs, GGs, CCs, Lheight);
% 
%         if size(solutions) ~= 11
%             aPointsNs = solutions(4:5);
%             s1=solutions(1); s2=solutions(2); s3=solutions(3); 
%             s10 = solutions(4); s11=solutions(5); 
%         else
%             aPointsNs = solutions(10:11);
%             s1=solutions(1); s2=solutions(2); s3=solutions(3); 
%             s10 = solutions(10); s11=solutions(11); 
%         end  
% 
%         s4=s1^2; s5=s1*s2; s6=s1*s3;
%         s7=s2^2; s8=s2*s3; s9=s3^2;
%         w=[1,s1,s2,s3,s4,s5,s6,s7,s8,s9,...
%            s10,s10*s1,s10*s2,s10*s4,s10*s6,s10*s7,s10*s8,s10*s9,...
%            s11,s11*s1,s11*s2,s11*s4,s11*s6,s11*s7,s11*s8,s11*s9,...
%            s1^2,s1*s2,s1*s3,s2^2,s2*s3,s3^2]';
%         factor=1/(1+s1^2+s2^2+s3^2);     
%         Rfs=factor*[1+s1^2-s2^2-s3^2,2*s1*s2-2*s3,2*s2+2*s1*s3;
%                        2*s3+2*s1*s2,1-s1^2+s2^2-s3^2,2*s2*s3-2*s1;
%                        2*s1*s3-2*s2,2*s1+2*s2*s3,1-s1^2-s2^2+s3^2];
%         Tfs =  factor*CCs*w; 
% 
%         tempW = P_W2;
%         tempW(1,2) = aPointsNs(1);
%         tempW(2,2) = aPointsNs(2);
%         Pa = tempW;
%         p2a = p_w2;
%         [errep772{j},Urep772{j}]=reprojection_error_usingRT(Pa,p2a,Rfs,Tfs,A);
% 
%         Rts77{j} = Rfs;
%         Tts77{j} = Tfs;
%         errortfs77{j} = obj_cur; 
%         errRtfs77{j} = cal_rotation_err(Rfs,Rtruth);
%         errTtfs77{j} = cal_translation_err(Tfs,Ttruth);
% 
% 
%         [EEu,GGu,CCu] = RefineEquA67C3(p, W);
%         apointsIniu = [apointb(1) apointb(2)]';
%         solutionOu = [Cayley(RCt);apointsIniu]; 
%         [solutionu, obj_curt] = RefineGaussNewtonRandApT67C3(solutionOu, p, W, EEu, GGu, CCu, Lheight);
% 
%     %     aPointsNu = [apoint(1) apoint(2)]';
% 
%         if size(solutionOu) ~= 11
%             aPointsNu = solutionu(4:5);
%             s1=solutionu(1); s2=solutionu(2); s3=solutionu(3); 
%             s10 = solutionu(4); s11=solutionu(5); 
%         else
%             aPointsNu = solutionu(10:11);
%             s1=solutionu(1); s2=solutionu(2); s3=solutionu(3); 
%             s10 = solutionu(10); s11=solutionu(11); 
%         end  
% 
%         s4=s1^2; s5=s1*s2; s6=s1*s3;
%         s7=s2^2; s8=s2*s3; s9=s3^2;
%         w=[1,s1,s2,s3,s4,s5,s6,s7,s8,s9,...
%            s10,s10*s1,s10*s2,s10*s4,s10*s6,s10*s7,s10*s8,s10*s9,...
%            s11,s11*s1,s11*s2,s11*s4,s11*s6,s11*s7,s11*s8,s11*s9,...
%            s1^2,s1*s2,s1*s3,s2^2,s2*s3,s3^2]';
%         factor=1/(1+s1^2+s2^2+s3^2);     
%         Ruf=factor*[1+s1^2-s2^2-s3^2,2*s1*s2-2*s3,2*s2+2*s1*s3;
%                        2*s3+2*s1*s2,1-s1^2+s2^2-s3^2,2*s2*s3-2*s1;
%                        2*s1*s3-2*s2,2*s1+2*s2*s3,1-s1^2-s2^2+s3^2];
%         Tuf =  factor*CCs*w; 
%         
%         tempW = P_W3;
%         tempW(1,n) = aPointsNu(1);
%         tempW(2,n) = aPointsNu(2);
%         Pa = tempW;
%         p2a = p_w3;
%         [errep773{j},Urep773{j}]=reprojection_error_usingRT(Pa,p2a,Ruf,Tuf,A);
% 
%         Rthu77{j} = Ruf;
%         Tthu77{j} = Tuf;
%         errortfu77(j) = obj_cur; 
%         errRtfu77(j) = cal_rotation_err(Ruf,Rtruth);
%         errTtfu77(j) = cal_translation_err(Tuf,Ttruth);
% 
% 
% 
% 
%         [EEv,GGv,CCv] = RefineEquA67D3(p, W);
%         apointsIniv = [apointb(3) apointb(4)]';
%         solutionOv = [Cayley(RDt);apointsIniv]; 
%         [solutionv, obj_cur] = RefineGaussNewtonRandApT67D3(solutionOv, p, W, EEv, GGv, CCv, Lheight);
% 
%         if size(solutionv) ~= 11
%             aPointsNv = solutionv(4:5);
%             s1=solutionv(1); s2=solutionv(2); s3=solutionv(3); 
%             s10 = solutionv(4); s11=solutionv(5); 
%         else
%             aPointsNv = solutionv(10:11);
%             s1=solutionv(1); s2=solutionv(2); s3=solutionv(3); 
%             s10 = solutionv(10); s11=solutionv(11); 
%         end  
% 
%         s4=s1^2; s5=s1*s2; s6=s1*s3;
%         s7=s2^2; s8=s2*s3; s9=s3^2;
%         w=[1,s1,s2,s3,s4,s5,s6,s7,s8,s9,...
%            s10,s10*s1,s10*s2,s10*s4,s10*s6,s10*s7,s10*s8,s10*s9,...
%            s11,s11*s1,s11*s2,s11*s4,s11*s6,s11*s7,s11*s8,s11*s9,...
%            s1^2,s1*s2,s1*s3,s2^2,s2*s3,s3^2]';
%         factor=1/(1+s1^2+s2^2+s3^2);     
%         Rfv=factor*[1+s1^2-s2^2-s3^2,2*s1*s2-2*s3,2*s2+2*s1*s3;
%                        2*s3+2*s1*s2,1-s1^2+s2^2-s3^2,2*s2*s3-2*s1;
%                        2*s1*s3-2*s2,2*s1+2*s2*s3,1-s1^2-s2^2+s3^2];
%         Tfv =  factor*CCs*w; 
%         
% 
%         tempW = P_W4;
%         tempW(1,n) = aPointsNv(1);
%         tempW(2,n) = aPointsNv(2);
%         Pa = tempW;
%         p2a = p_w4;
%         [errep774{j},Urep774{j}]=reprojection_error_usingRT(Pa,p2a,Rfv,Tfv,A);
% 
%         Rtv77{j} = Rfv;
%         Ttv77{j} = Tfv;
%         errortfv77(j) = obj_cur; 
%         errRtfv77(j) = cal_rotation_err(Rfv,Rtruth);
%         errTtfv77(j) = cal_translation_err(Tfv,Ttruth);
%     end
%     
%     toc;
%     time77{j} = toc; 

end

Avg_errep710 = mean(cell2mat(errep710));
Avg_errep711 = mean(cell2mat(errep711));
Avg_errep712 = mean(cell2mat(errep712));
Avg_errep720 = mean(cell2mat(errep720));
Avg_errep721 = mean(cell2mat(errep721));
Avg_errep722 = mean(cell2mat(errep722));
Avg_errep730 = mean(cell2mat(errep730));
Avg_errep731 = mean(cell2mat(errep731));
Avg_errep732 = mean(cell2mat(errep732));
Avg_errep740 = mean(cell2mat(errep740));
Avg_errep741 = mean(cell2mat(errep741));
Avg_errep742 = mean(cell2mat(errep742));
Avg_errep750 = mean(cell2mat(errep750));
Avg_errep751 = mean(cell2mat(errep751));
Avg_errep752 = mean(cell2mat(errep752));
Avg_errep760 = mean(cell2mat(errep760));
Avg_errep761 = mean(cell2mat(errep761));
Avg_errep762 = mean(cell2mat(errep762));
Avg_errep770 = mean(cell2mat(errep770));
Avg_errep771 = mean(cell2mat(errep771));
Avg_errep772 = mean(cell2mat(errep772));
Avg_errep773 = mean(cell2mat(errep773));
Avg_errep774 = mean(cell2mat(errep774));

% 
% 
% 
