close all;
clc;
clear;






%Type6 Room Camera Pose Estimation

noise=0;
rateOutlier=0;
genN = 20;
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
   
    A = [200 0 160;0 200 320;0   0   1];
    
 
    


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

    T1P1_wB = [P1_w(1,:);P2_w(1,:);P1_w(2,:)];
    T1P2_wB = [P1_w(2,:);P2_w(2,:);P2_w(1,:)];
    
    T1P1_wC = [P1_w(1,:);P2_w(1,:);P1_w(1,:)];
    T1P2_wC = [P1_w(2,:);P2_w(2,:);P2_w(1,:)];
    
    T1P1_wD = [P1_w(1,:);P2_w(1,:);P1_w(2,:)];
    T1P2_wD = [P1_w(2,:);P2_w(2,:);P2_w(2,:)];
    
    W1 = T1P1_wA(1:2,:)';
    W2 = T1P2_wA(1:2,:)';
    
    [R3, T3, err] = SRPnL61(ip1A', ip2A', T1P1_wA', T1P2_wA', cH);
    [R4, T4, err4] = SRPnL61(ip1B', ip2B', T1P1_wB', T1P2_wB', cH);
    [R6, T6, err6] = SRPnL61(ip1C', ip2C', T1P1_wC', T1P2_wC', cH);
    [R7, T7, err7] = SRPnL61(ip1D', ip2D', T1P1_wD', T1P2_wD', cH);
    
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
    [F1, G] = P2LMGA67(T1p1A, T1p2A, T1P1_wA, T1P2_wA, T1p1B, T1p2B, T1P1_wB, T1P2_wB, T1p1C, T1p2C, T1P1_wC, T1P2_wC, T1p1D, T1p2D, T1P1_wD, T1P2_wD, cH);
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
        if F1(i).ValObjective(1) + F1(i).ValObjective(2) + F1(i).ValObjective(3) + F1(i).ValObjective(4)< errmin
            [m,n] = min(F1(i).ValObjective);
            if n == 1
                Ri = F1(i).RA;
                Ti =  F1(i).TA;
                errmin = F1(i).ValObjective(1);
                flag = 1;
            elseif n == 2
                Ri = F1(i).RB;
                Ti =  F1(i).TB;
                errmin = F1(i).ValObjective(2);
                flag = 2;
            elseif n == 3
                Ri = F1(i).RC;
                Ti =  F1(i).TC;
                errmin = F1(i).ValObjective(3);
                flag = 3;
            elseif n == 4
                Ri = F1(i).RD;
                Ti =  F1(i).TD;
                errmin = F1(i).ValObjective(4);
                flag = 4;
            end
            RAt = F1(i).RA;
            TAt = F1(i).TA;
            RBt = F1(i).RB;
            TBt = F1(i).TB;
            RCt = F1(i).RC;
            TCt = F1(i).TC;
            RDt = F1(i).RD;
            TDt = F1(i).TD;           
            apoint = F1(i).Val;
        end
    end  


    errOriRf67(j) = cal_rotation_err(Ri,Rtruth);
    errOriTf67(j) = cal_translation_err(Ti,Ttruth);

    
    if flag  == 1
        
        [EE,GG,CC] = RefineEquA67A(p, W);
        apointsIni = [apoint(1) apoint(4)]';
        solutionO = [Cayley(Ri);apointsIni]; 
        [solution, obj_cur] = RefineGaussNewtonRandApT67A(solutionO, p, W, EE, GG, CC);


        if size(solution) ~= 11
            aPointsN = solution(4:5);
        else
            aPointsN = solution(10:11);
        end

        T1P1_wA(1,2) =  aPointsN(1);
        T1P1_wA(2,2) =  aPointsN(2)-50;
        T1P1_wA(3,2) =  aPointsN(1);
        T1P2_wA(1,2) =  aPointsN(1)+50;
        T1P2_wA(2,2) =  aPointsN(2);
        T1P2_wA(3,2) =  aPointsN(2);
        [Rf, Tf, errf] = SRPnL61(ip1A', ip2A', T1P1_wA', T1P2_wA', cH);
    elseif flag == 2
        [EE,GG,CC] = RefineEquA67B(p, W);
        apointsIni = [apoint(2) apoint(3)]';
        solutionO = [Cayley(Ri);apointsIni]; 
        [solution, obj_cur] = RefineGaussNewtonRandApT67B(solutionO, p, W, EE, GG, CC);

        if size(solution) ~= 11
            aPointsN = solution(4:5);
        else
            aPointsN = solution(10:11);
        end

        T1P1_wB(1,2) =  aPointsN(2)-50;
        T1P1_wB(2,2) =  aPointsN(1);
        T1P1_wB(3,2) =  aPointsN(2);
        T1P2_wB(1,2) =  aPointsN(2);
        T1P2_wB(2,2) =  aPointsN(1)+50;
        T1P2_wB(3,2) =  aPointsN(1);
        [Rf, Tf, errf] = SRPnL61(ip1B', ip2B', T1P1_wB', T1P2_wB', cH); 
    elseif flag == 3
        [EE,GG,CC] = RefineEquA67C(p, W);
        apointsIni = [apoint(3) apoint(4)]';
        solutionO = [Cayley(Ri);apointsIni]; 
        [solution, obj_cur] = RefineGaussNewtonRandApT67C(solutionO, p, W, EE, GG, CC);

        if size(solution) ~= 11
            aPointsN = solution(4:5);
        else
            aPointsN = solution(10:11);
        end

        T1P1_wB(1,2) =  aPointsN(1);
        T1P1_wB(2,2) =  aPointsN(2);
        T1P1_wB(3,2) =  aPointsN(1);
        T1P2_wB(1,2) =  aPointsN(1)+ 50;
        T1P2_wB(2,2) =  aPointsN(2)+ 50;
        T1P2_wB(3,2) =  aPointsN(2);
        [Rf, Tf, errf] = SRPnL61(ip1C', ip2C', T1P1_wC', T1P2_wC', cH); 
    elseif flag == 4
        [EE,GG,CC] = RefineEquA67D(p, W);
        apointsIni = [apoint(3) apoint(4)]';
        solutionO = [Cayley(Ri);apointsIni]; 
        [solution, obj_cur] = RefineGaussNewtonRandApT67D(solutionO, p, W, EE, GG, CC);

        if size(solution) ~= 11
            aPointsN = solution(4:5);
        else
            aPointsN = solution(10:11);
        end

        T1P1_wB(1,2) =  aPointsN(1)-50;
        T1P1_wB(2,2) =  aPointsN(2)-50;
        T1P1_wB(3,2) =  aPointsN(1);
        T1P2_wB(1,2) =  aPointsN(1);
        T1P2_wB(2,2) =  aPointsN(2);
        T1P2_wB(3,2) =  aPointsN(2);
        [Rf, Tf, errf] = SRPnL61(ip1D', ip2D', T1P1_wD', T1P2_wD', cH); 
    end
    
    R67{j} = Rf;
    T67{j} = Tf;
    error67(j) = errf; 
    errRf67(j) = cal_rotation_err(Rf,Rtruth);
    errTf67(j) = cal_translation_err(Tf,Ttruth);
    toc;
    time67(j) = toc; 
    
    [EEt,GGt,CCt] = RefineEquA67A(p, W);
    apointsInit = [apoint(1) apoint(4)]';
    solutionOt = [Cayley(RAt);apointsInit]; 
    [solutiont, obj_curt] = RefineGaussNewtonRandApT67A(solutionOt, p, W, EEt, GGt, CCt);


    if size(solutiont) ~= 11
        aPointsNt = solutiont(4:5);
    else
        aPointsNt = solutiont(10:11);
    end

    T1P1_wA(1,2) =  aPointsNt(1);
    T1P1_wA(2,2) =  aPointsNt(2)-50;
    T1P1_wA(3,2) =  aPointsNt(1);
    T1P2_wA(1,2) =  aPointsNt(1)+50;
    T1P2_wA(2,2) =  aPointsNt(2);
    T1P2_wA(3,2) =  aPointsNt(2);
    [Rtf, Ttf, errftf] = SRPnL61(ip1A', ip2A', T1P1_wA', T1P2_wA', cH);
    Rth67{j} = Rtf;
    Tth67{j} = Ttf;
    errortf67(j) = errftf; 
    errRtf67(j) = cal_rotation_err(Rtf,Rtruth);
    errTtf67(j) = cal_translation_err(Ttf,Ttruth);
    
    
    
    [EEs,GGs,CCs] = RefineEquA67B(p, W);
    apointsInis = [apoint(2) apoint(3)]';
    solutionOs = [Cayley(RBt);apointsInis]; 
    [solutions, obj_cur] = RefineGaussNewtonRandApT67B(solutionOs, p, W, EE, GG, CC);

    if size(solutions) ~= 11
        aPointsNs = solutions(4:5);
    else
        aPointsNs = solutions(10:11);
    end

    T1P1_wB(1,2) =  aPointsNs(2)-50;
    T1P1_wB(2,2) =  aPointsNs(1);
    T1P1_wB(3,2) =  aPointsNs(2);
    T1P2_wB(1,2) =  aPointsNs(2);
    T1P2_wB(2,2) =  aPointsNs(1)+50;
    T1P2_wB(3,2) =  aPointsNs(1);
    [Rfs, Tfs, errfs] = SRPnL61(ip1B', ip2B', T1P1_wB', T1P2_wB', cH); 
    
    Rts67{j} = Rfs;
    Tts67{j} = Tfs;
    errortfs67(j) = errfs; 
    errRtfs67(j) = cal_rotation_err(Rfs,Rtruth);
    errTtfs67(j) = cal_translation_err(Tfs,Ttruth);
    
    
    [EEt,GGt,CCt] = RefineEquA67C(p, W);
    apointsIniu = [apoint(1) apoint(2)]';
    solutionOu = [Cayley(RCt);apointsIniu]; 
    [solutionu, obj_curt] = RefineGaussNewtonRandApT67C(solutionOu, p, W, EEt, GGt, CCt);


    if size(solutionu) ~= 11
        aPointsNu = solutionu(4:5);
    else
        aPointsNu = solutionu(10:11);
    end

    T1P1_wC(1,2) =  aPointsNu(1);
    T1P1_wC(2,2) =  aPointsNu(2);
    T1P1_wC(3,2) =  aPointsNu(1);
    T1P2_wC(1,2) =  aPointsNu(1)+50;
    T1P2_wC(2,2) =  aPointsNu(2)+50;
    T1P2_wC(3,2) =  aPointsNu(2);
    [Ruf, Tuf, errfuf] = SRPnL61(ip1C', ip2C', T1P1_wC', T1P2_wC', cH);
    Rthu67{j} = Ruf;
    Tthu67{j} = Tuf;
    errortfu67(j) = errfuf; 
    errRtfu67(j) = cal_rotation_err(Ruf,Rtruth);
    errTtfu67(j) = cal_translation_err(Tuf,Ttruth);
    
    
    
    [EEs,GGs,CCs] = RefineEquA67D(p, W);
    apointsIniv = [apoint(3) apoint(4)]';
    solutionOv = [Cayley(RDt);apointsIniv]; 
    [solutionv, obj_cur] = RefineGaussNewtonRandApT67D(solutionOv, p, W, EE, GG, CC);

    if size(solutionv) ~= 11
        aPointsNv = solutionv(4:5);
    else
        aPointsNv = solutionv(10:11);
    end

    T1P1_wD(1,2) =  aPointsNv(1)-50;
    T1P1_wD(2,2) =  aPointsNv(2)-50;
    T1P1_wD(3,2) =  aPointsNv(1);
    T1P2_wD(1,2) =  aPointsNv(1);
    T1P2_wD(2,2) =  aPointsNv(2);
    T1P2_wD(3,2) =  aPointsNv(2);
    [Rfv, Tfv, errfv] = SRPnL61(ip1D', ip2D', T1P1_wD', T1P2_wD', cH); 

    Rtv66{j} = Rfv;
    Ttv66{j} = Tfv;
    errortfv67(j) = errfs; 
    errRtfv67(j) = cal_rotation_err(Rfv,Rtruth);
    errTtfv67(j) = cal_translation_err(Tfv,Ttruth);

%     T1P1_wA(1,2) =  aPointsN(1);
%     T1P1_wA(2,2) =  aPointsN(2)-50;
%     T1P1_wA(3,2) =  aPointsN(1);
%     T1P2_wA(1,2) =  aPointsN(1)+50;
%     T1P2_wA(2,2) =  aPointsN(2);
%     T1P2_wA(3,2) =  aPointsN(2);

%     T1P1_w(1,2) = apoint(1);
%     T1P1_w(2,2) = apoint(4);
%     T1P1_w(3,2) = apoint(1);
%     T1P1_w(4,2) = apoint(3);
%     T1P1_w(5,2) = apoint(1);
%     T1P1_w(6,2) = apoint(4);
%     
%     T1P2_w(1,2) = apoint(3);
%     T1P2_w(2,2) = apoint(2);
%     T1P2_w(3,2) = apoint(4);
%     T1P2_w(4,2) = apoint(2);
%     T1P2_w(5,2) = apoint(2);
%     T1P2_w(6,2) = apoint(3);
%     
%     [Rtg, Ttg, errtg] = SRPnL6(ip1', ip2', T1P1_w', T1P2_w', cH);
%     
%     Rtg{j} = Rtg;
%     Ttg{j} = Ttg;
%     errortfg(j) = errtg; 
%     errRtfg(j) = cal_rotation_err(Rtg,Rtruth);
%     errTtfg(j) = cal_translation_err(Ttg,Ttruth);

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


errGAtpnlRm = mean(errRtf);
errGAtpnlRme = median(errRtf);
errGAtpnlTm = mean(errTtf);
errGAtpnlTme = median(errTtf);


errGAspnlRm = mean(errRtfs);
errGAspnlRme = median(errRtfs);
errGAspnlTm = mean(errTtfs);
errGAspnlTme = median(errTtfs);

% errGAgpnlRm = mean(errRtfg);
% errGAgpnlRme = median(errRtfg);
% errGAgpnlTm = mean(errTtfg);
% errGAgpnlTme = median(errTtfg);

avgTime = mean(time);




