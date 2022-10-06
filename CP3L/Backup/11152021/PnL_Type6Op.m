close all;
clc;
clear;






%Type6 Room Camera Pose Estimation

noise=0;
rateOutlier=0;
genN = 50;
Data = LayoutDataGen(6,noise,rateOutlier,genN);
Height = 256;
for j = 1:genN
    pointS = Data{j}.pointS;
    pointE = Data{j}.pointE;
    pointSNoNoise = Data{j}.pointSt;
    pointENoNoise = Data{j}.pointEt;
    cH = Data{j}.Cc(2);
    
    p1 = pointS(:,4:5);
    p2 = pointE(:,4:5);
   
    A = [180 0 160;0 180 320;0   0   1];
    
    Rtruth = Data{j}.R;
    Ttruth = Data{j}.T;
    

    pno1 = pointSNoNoise(:,4:5);
    pno2 = pointENoNoise(:,4:5);
    
    P1_w = pointS(:,1:3);
    P2_w = pointE(:,1:3);
    

    
    T1p1 = [p1(1,:);p2(1,:);p1(1,:);p1(2,:)];
    T1p2 = [p1(2,:);p2(2,:);p2(1,:);p2(2,:)];
    
    for i = 1:length(T1p1)
        temp1 = inv(A)*[T1p1(i,:) 1].';
        temp1 = (temp1/temp1(3)).';
        ip1(i,:) = temp1(:,1:2);
        temp2 = inv(A)*[T1p2(i,:) 1].';
        temp2 = (temp2/temp2(3)).';
        ip2(i,:) = temp2(:,1:2);
    end
    
    pn1 = ip1(1:2,:)';
    pn2 = ip2(1:2,:)';
    
    
    T1P1_w = [P1_w(1,:);P2_w(1,:);P1_w(1,:);P1_w(2,:)];
    T1P2_w = [P1_w(2,:);P2_w(2,:);P2_w(1,:);P2_w(2,:)];
    
    W1 = T1P1_w(1:2,:)';
    W2 = T1P2_w(1:2,:)';
    
    [R3, T3, err] = SRPnL6(ip1', ip2', T1P1_w', T1P2_w', cH);
    
    Rs{j} = R3;
    
    Ts{j} = T3;
    
    errOtt(j) = err;

     
    errRt(j) = cal_rotation_err(R3,Rtruth);
    

     
    errTt(j) = cal_translation_err(T3,Ttruth);
    
    p = [pn1 pn2];
    nLine = length(p);
    W = [W1 W2];
    p = [p; ones(1,nLine)];
    
    ProblemParametersOption = 0;
    k = 0;
%     [R,T,Ierr] = P2LMCMC(ip1, ip2, T1P1_w, T1P2_w, cH);
    ProblemParameters = SetProblemParameters(ProblemParametersOption, k);
    [F1, G] = P2LMGA(T1p1, T1p2, T1P1_w, T1P2_w, cH, ProblemParameters);
%         aPoints = P2LSGA(T1p1, T1p2, T1P1_w, T1P2_w, cH, Height, k);
    if isValid == 1
        ResF{k} = F1;
        [nR,~] = size(F1);
        minerr = inf;
        for n = 1:nR
            if F1(n).ValObjective(1) < minerr
                apoints = F1(n).Val;
                minerr =  F1(n).ValObjective(1);
            end
        end      


        W(2,1) = apoints(1);
        W(2,2) = apoints(3);
        W(2,3) = apoints(2);
        W(2,4) = apoints(4);

        pn1 = [p(1:2,1) p(1:2,1) p(1:2,1) p(1:2,2) p(1:2,2) p(1:2,3)];
        pn2 = [p(1:2,2) p(1:2,3) p(1:2,4) p(1:2,3) p(1:2,4) p(1:2,4)];
        Pn1_w = [W(:,1) W(:,1) W(:,1) W(:,2) W(:,2) W(:,3)];
        Pn2_w = [W(:,2) W(:,3) W(:,4) W(:,3) W(:,4) W(:,4)];

        [Rt, Tt, errt] = SRPnL6(pn1, pn2, Pn1_w, Pn2_w, cH);



        [EE,GG,CC] = RefineEquA6(p, W);
        apointsIni = [apoints(1) apoints(3) apoints(2) apoints(4)]';
        solutionO = [Cayley(Rt);apointsIni]; 
        [solution, obj_cur] = RefineGaussNewtonRandApT6(solutionO, EE, GG, CC, pn1, pn2, Pn1_w, Pn2_w);
        if size(solution) ~= 13
            aPointsN = solution(4:7);
        else
            aPointsN = solution(10:13);
        end

        W(2,1) = aPointsN(1);
        W(2,2) = aPointsN(2);
        W(2,3) = aPointsN(3);
        W(2,4) = aPointsN(4);
        Pn1_w = [W(:,1) W(:,1) W(:,1) W(:,2) W(:,2) W(:,3)];
        Pn2_w = [W(:,2) W(:,3) W(:,4) W(:,3) W(:,4) W(:,4)];

        [Rg, Tg, errg] = SRPnL6(pn1(1:2,:), pn2(1:2,:), Pn1_w, Pn2_w, cH);

        R{k} = Rg;
        T{k} = Tg;
        err(k) = errg; 
    else
        R{k}=eye(3,3);
        T{k}=[0 0 0]';
        err(k) = inf;      
    end 
end

errSrpnlRm = mean(errRt);
errSrpnlRme = median(errRt);
errSrpnlTm = mean(errTt);
errSrpnlTme = median(errTt);
errSrpnlOtm = mean(errOtt);
errSrpnlOtme = median(errOtt);





