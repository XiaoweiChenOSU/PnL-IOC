close all;
clc;
clear;






%Type6 Room Camera Pose Estimation

noise=1;
rateOutlier=0;
genN = 100;
Data = LayoutDataGen(6,noise,rateOutlier,genN);
Height = 256;
for j = 1:genN
    pointS = Data{j}.pointS;
    pointE = Data{j}.pointE;
    pointSNoNoise = Data{j}.pointSt;
    pointENoNoise = Data{j}.pointEt;
    cH = Data{j}.Cc(2);
    Rtruth = Data{j}.R;
    Ttruth = Data{j}.T;
    
%     pointS = [0,42,0,106.073270196147,319.533247696052;0,250,0,121.076075751357,0.839111782022997];
%     pointE = [256,55,0,470.953822601509,318.672340177603;256,255,0,487.044378287856,35.0985604654714];
%     cH = 43.974741393027320;
%     Rtruth = [0.998148090644185,0.0484477473797083,0.0367859336866626;0.0482386873198666,-0.998814359641701,0.00655011596802260;0.0370596571627925,-0.00476348059409948,-0.999301701721560];
%     Ttruth = [-37.4863513174134;41.6433591557567;118.532445881522];
    
    
    p1 = pointS(:,4:5);
    p2 = pointE(:,4:5);
   
    A = [180 0 160;0 180 320;0   0   1];
    
 
    


%     pno1 = pointSNoNoise(:,4:5);
%     pno2 = pointENoNoise(:,4:5);
    
    P1_w = pointS(:,1:3);
    P2_w = pointE(:,1:3);
    

%     
    T1p1 = [p1(1,:);p2(1,:);p1(1,:);p1(2,:);p1(1,:);p2(1,:)];
    T1p2 = [p1(2,:);p2(2,:);p2(1,:);p2(2,:);p2(2,:);p1(2,:)];
    
    T1p1A = [p1(1,:);p2(1,:);p1(1,:)];
    T1p2A = [p1(2,:);p2(2,:);p2(2,:)];
    
    T1p1B = [p1(1,:);p2(1,:);p1(2,:)];
    T1p2B = [p1(2,:);p2(2,:);p2(1,:)];
    
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
    
    pn1 = ip1A(1:2,:)';
    pn2 = ip2A(1:2,:)';
    
    
    T1P1_w = [P1_w(1,:);P2_w(1,:);P1_w(1,:);P1_w(2,:);P1_w(1,:);P2_w(1,:)];
    T1P2_w = [P1_w(2,:);P2_w(2,:);P2_w(1,:);P2_w(2,:);P2_w(2,:);P1_w(2,:)];
    T1P1_wA = [P1_w(1,:);P2_w(1,:);P1_w(1,:)];
    T1P2_wA = [P1_w(2,:);P2_w(2,:);P2_w(2,:)];

    T1P1_wB = [P1_w(1,:);P2_w(1,:);P1_w(2,:)];
    T1P2_wB = [P1_w(2,:);P2_w(2,:);P2_w(1,:)];
    
    W1 = T1P1_wA(1:2,:)';
    W2 = T1P2_wA(1:2,:)';
    
    [R3, T3, err] = SRPnL61(ip1A', ip2A', T1P1_wA', T1P2_wA', cH);
    [R4, T4, err4] = SRPnL61(ip1B', ip2B', T1P1_wB', T1P2_wB', cH);
    [R5, T5, err5] = SRPnL6(ip1', ip2', T1P1_w', T1P2_w', cH);
    
    Rs{j} = R3;
    
    Ts{j} = T3;
    
    errOtt(j) = err;

     
    errRt(j) = cal_rotation_err(R3,Rtruth);
    

     
    errTt(j) = cal_translation_err(T3,Ttruth);
    
    
    
    
    p = [pn1 pn2];
    nLine = length(p);
    W = [W1 W2];
    p = [p; ones(1,nLine)];
    
    tic;
    count = 1;        
    [F1, G] = P2LMGA61(T1p1A, T1p2A, T1P1_wA, T1P2_wA, T1p1B, T1p2B, T1P1_wB, T1P2_wB, cH);
    errmin = Inf;
    Ri = eye(3,3);
    Ti = [0,0,0];
    flag = 0;
    for i = 1:length(F1)
        if F1(i).ValObjective(1) > 0.5 || F1(i).ValObjective(2) > 0.5 || F1(i).ValObjective(3) > 0.5 || F1(i).ValObjective(4) > 0.5
            continue
        end
%         apoints(count,:) =  F1(i).Val;   
%         count = count + 1;
        if F1(i).ValObjective(1) + F1(i).ValObjective(2) < errmin
            RiA = F1(i).RA;
            TiA =  F1(i).TA;
            RiB = F1(i).RB;
            TiB =  F1(i).TB;
            errmin = F1(i).ValObjective(1) + F1(i).ValObjective(2);
            apoint = F1(i).Val;
%             if F1(i).ValObjective(1) < F1(i).ValObjective(2)
%                 RiA = F1(i).RA;
%                 TiA =  F1(i).TA;
%                 errmin = F1(i).ValObjective(1);
%             else
%                 RiB = F1(i).RB;
%                 TiB =  F1(i).TB;
%                 errmin = F1(i).ValObjective(2);
%             end
%             apoint = F1(i).Val;
        end
    end  


    errOriRf(j) = cal_rotation_err(Ri,Rtruth);
    errOriTf(j) = cal_translation_err(Ti,Ttruth);

    for k  = 1:5
            
        [EEA,GGA,CCA] = RefineEquA61A(p, W);
        apointsIniA = [apoint(1) apoint(2)]';
        solutionOA = [Cayley(RiA);apointsIniA]; 
        [solutionA, obj_cur] = RefineGaussNewtonRandApT61A(solutionOA, p, W, EEA, GGA, CCA);

        s1=solutionA(1); s2=solutionA(2); s3=solutionA(3); 
        factor=1/(1+s1^2+s2^2+s3^2); 
        RiA=factor*[1+s1^2-s2^2-s3^2,2*s1*s2-2*s3,2*s2+2*s1*s3;
                       2*s3+2*s1*s2,1-s1^2+s2^2-s3^2,2*s2*s3-2*s1;
                       2*s1*s3-2*s2,2*s1+2*s2*s3,1-s1^2-s2^2+s3^2];

        [EEB,GGB,CCB] = RefineEquA61B(p, W);
        apointsIniB = [apoint(4) apoint(3)]';
        solutionOB = [Cayley(RiB);apointsIniB]; 
        [solutionB, obj_cur] = RefineGaussNewtonRandApT61B(solutionOB, p, W, EEB, GGB, CCB); 
        s1=solutionB(1); s2=solutionB(2); s3=solutionB(3); 
        factor=1/(1+s1^2+s2^2+s3^2); 
        RiB=factor*[1+s1^2-s2^2-s3^2,2*s1*s2-2*s3,2*s2+2*s1*s3;
                       2*s3+2*s1*s2,1-s1^2+s2^2-s3^2,2*s2*s3-2*s1;
                       2*s1*s3-2*s2,2*s1+2*s2*s3,1-s1^2-s2^2+s3^2];     
    end
    
    if size(solutionA) ~= 11 
        aPointsN(1:2) = solutionA(4:5);
    else
        aPointsN(1:2) = solutionA(10:11);
    end
    if size(solutionB) ~= 11 
        aPointsN(3:4) = solutionB(4:5);
    else
        aPointsN(3:4) = solutionB(10:11);
    end

    
%     T1P1_wA(1,2) =  aPointsN(2)-50;
%     T1P1_wA(2,2) =  aPointsN(2)-50;
%     T1P1_wA(3,2) =  aPointsN(1);
%     T1P2_wA(1,2) =  aPointsN(1)+50;
%     T1P2_wA(2,2) =  aPointsN(2);
%     T1P2_wA(3,2) =  aPointsN(2);
%     [Rf, Tf, errf] = SRPnL61(ip1A', ip2A', T1P1_wA', T1P2_wA', cH);


    T1P1_w(1,2) =  aPointsN(1);
    T1P1_w(2,2) =  aPointsN(3);
    T1P1_w(3,2) =  aPointsN(1);
    T1P1_w(4,2) =  aPointsN(4);
    T1P1_w(5,2) =  aPointsN(1);
    T1P1_w(6,2) =  aPointsN(3);
    T1P2_w(1,2) =  aPointsN(4);
    T1P2_w(2,2) =  aPointsN(2);
    T1P2_w(3,2) =  aPointsN(3);
    T1P2_w(4,2) =  aPointsN(2);
    T1P2_w(5,2) =  aPointsN(2);
    T1P2_w(6,2) =  aPointsN(4);
    [Rf, Tf, errf] = SRPnL6(ip1', ip2', T1P1_w', T1P2_w', cH);


%     T1P1_wA(1,2) =  aPointsN(1);
%     T1P1_wA(2,2) =  aPointsN(2)-50;
%     T1P1_wA(3,2) =  aPointsN(1);
%     T1P2_wA(1,2) =  aPointsN(1)+50;
%     T1P2_wA(2,2) =  aPointsN(2);
%     T1P2_wA(3,2) =  aPointsN(2);
    
    
    R{j} = Rf;
    T{j} = Tf;
    error(j) = errf; 
    errRf(j) = cal_rotation_err(Rf,Rtruth);
    errTf(j) = cal_translation_err(Tf,Ttruth);
    toc;
    time(j) = toc; 
end  

errSrpnlRm = mean(errRt);
errSrpnlRme = median(errRt);
errSrpnlTm = mean(errTt);
errSrpnlTme = median(errTt);
errSrpnlOtm = mean(errOtt);
errSrpnlOtme = median(errOtt);


errGApnlRm = mean(errRf);
errGApnlRme = median(errRf);
errGApnlTm = mean(errTf);
errGApnlTme = median(errTf);

avgTime = mean(time);




