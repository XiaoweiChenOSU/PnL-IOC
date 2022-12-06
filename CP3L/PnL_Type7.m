close all;
clc;
clear;

addpath('SRPnL');
addpath('others');
addpath('RPnL');
addpath('LPnL-Bar-ENull');
addpath('PnL-IOC');
addpath('TypeReal71MultiGA');
addpath('TypeReal72MultiGA');
addpath('TypeReal73MultiGA');
addpath('TypeReal74MultiGA');
addpath('TypeReal75MultiGA');
addpath('TypeReal76MultiGA');
addpath('TypeReal77MultiGA');

%Type6 Room Camera Pose Estimation

for no = 10:10

    noise=no;
    rateOutlier=0;
    genN = 10;
    Data = LayoutDataGen(7,noise,rateOutlier,genN);
    Height = 256;
    for j = 1:genN
        pointS = Data{j}.pointS;
        pointE = Data{j}.pointE;
        pointSNoNoise = Data{j}.pointSt;
        pointENoNoise = Data{j}.pointEt;
        cH = Data{j}.Cc(2);
        Rtruth = Data{j}.R;
        Ttruth = Data{j}.T;




        p1 = pointS(:,4:5);
        p2 = pointE(:,4:5);

        A = [100 0 320;0 100 160;0  0  1];





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


        W1 = T1P1_wA(1:2,:)';
        W2 = T1P2_wA(1:2,:)';

        [R3, T3, err] = SRPnL71(ip1A', ip2A', T1P1_wA', T1P2_wA', cH,Height);
        [R4, T4, err4] = SRPnL71(ip1B', ip2B', T1P1_wB', T1P2_wB', cH,Height);
        [R5, T5, err5] = SRPnL7(ip1', ip2', T1P1_w', T1P2_w');


        Rs{j} = R3;

        Ts{j} = T3;

        errOtt(j) = err;


        errRt(j) = cal_rotation_err(R3,Rtruth);



        errTt(j) = cal_translation_err(T3,Ttruth);




        p = [pn1 pn2];
        nLine = length(p);
        W = [W1 W2];
        p = [p; ones(1,nLine)];

        clearvars apoint aPointsN aPointsNt aPointsNs apointsIni apointsInis apointsInit apointsIniu apointsIniv aPointsNu aPointsNv T1P1_wA T1P2_wA T1P1_wB T1P2_wB T1P1_wC T1P2_wC T1P1_wD T1P2_wD
        tic;
        count = 1; 

        T1P1_wA = [P1_w(1,:);P2_w(1,:);P1_w(1,:)];
        T1P2_wA = [P1_w(2,:);P2_w(2,:);P2_w(2,:)];

        T1P1_wB = [P1_w(1,:);P2_w(1,:);P1_w(2,:)];
        T1P2_wB = [P1_w(2,:);P2_w(2,:);P2_w(1,:)];

        [F1, G] = P2LMGA71(T1p1A, T1p2A, T1P1_wA, T1P2_wA, T1p1B, T1p2B, T1P1_wB, T1P2_wB, cH);
        errAmin = Inf;
        errBmin = Inf;
        for i = 1:length(F1)
            if F1(i).ValObjective(1) > 10 || F1(i).ValObjective(2) > 10 || F1(i).ValObjective(3) > 100 || F1(i).ValObjective(4) > 100
    %         if F1(i).ValObjective(1) + F1(i).ValObjective(2) > 2
                 continue
            end
    %         apoints(count,:) =  F1(i).Val;   
    %         count = count + 1;
            if F1(i).ValObjective(1) < errAmin 
                apoint(1) = F1(i).Val(1);
                apoint(4) = F1(i).Val(4);
                RAt = F1(i).RA;
                TAt = F1(i).TA;
                errAmin = F1(i).ValObjective(1);
            end
            if F1(i).ValObjective(2) < errBmin 
                apoint(2) = F1(i).Val(2);
                apoint(3) = F1(i).Val(3);
                RBt = F1(i).RB;
                TBt = F1(i).TB;
                errBmin = F1(i).ValObjective(2);
            end 
        end  


        if errAmin < errBmin

            [EE,GG,CC] = RefineEquA71A(p, W);
            apointsIni = [apoint(1) apoint(4)]';
            solutionO = [Cayley(RAt);apointsIni]; 
            [solution, obj_cur] = RefineGaussNewtonRandApT71A(solutionO, p, W, EE, GG, CC);


            if size(solution) ~= 11
                aPointsN = solution(4:5);
            else
                aPointsN = solution(10:11);
            end
    %         aPointsN = [apoint(1) apoint(4)]';

            T1P1_wA(1,1) =  aPointsN(1);
            T1P1_wA(2,1) =  aPointsN(2)-50;
            T1P1_wA(3,1) =  aPointsN(1);
            T1P2_wA(1,1) =  aPointsN(1)+50;
            T1P2_wA(2,1) =  aPointsN(2);
            T1P2_wA(3,1) =  aPointsN(2);
            [Rf, Tf, errf] = SRPnL71(ip1A', ip2A', T1P1_wA', T1P2_wA', cH);
        else
            [EE,GG,CC] = RefineEquA71B(p, W);
            apointsIni = [apoint(2) apoint(3)]';
            solutionO = [Cayley(RBt);apointsIni]; 
            [solution, obj_cur] = RefineGaussNewtonRandApT71B(solutionO, p, W, EE, GG, CC);

            if size(solution) ~= 11
                aPointsN = solution(4:5);
            else
                aPointsN = solution(10:11);
            end

    %         aPointsN = [apoint(2) apoint(3)]';

            T1P1_wB(1,1) =  aPointsN(2)-50;
            T1P1_wB(2,1) =  aPointsN(1);
            T1P1_wB(3,1) =  aPointsN(2);
            T1P2_wB(1,1) =  aPointsN(2);
            T1P2_wB(2,1) =  aPointsN(1)+50;
            T1P2_wB(3,1) =  aPointsN(1);
            [Rf, Tf, errf] = SRPnL71(ip1B', ip2B', T1P1_wB', T1P2_wB', cH,Height); 
        end

        R71{j} = Rf;
        T71{j} = Tf;
        error71(j) = errf; 
        errRf71(j) = cal_rotation_err(Rf,Rtruth);
        errTf71(j) = cal_translation_err(Tf,Ttruth);
        toc;
        time71(j) = toc; 


        [EEt,GGt,CCt] = RefineEquA71A(p, W);
        apointsInit = [apoint(1) apoint(4)]';
        solutionOt = [Cayley(RAt);apointsInit]; 
        [solutiont, obj_curt] = RefineGaussNewtonRandApT71A(solutionOt, p, W, EEt, GGt, CCt);


        if size(solutiont) ~= 11
            aPointsNt = solutiont(4:5);
        else
            aPointsNt = solutiont(10:11);
        end

    %     aPointsNt = [apoint(1) apoint(4)]';

        T1P1_wA(1,1) =  aPointsNt(1);
        T1P1_wA(2,1) =  aPointsNt(2)-50;
        T1P1_wA(3,1) =  aPointsNt(1);
        T1P2_wA(1,1) =  aPointsNt(1)+50;
        T1P2_wA(2,1) =  aPointsNt(2);
        T1P2_wA(3,1) =  aPointsNt(2);
        [Rtf, Ttf, errftf] = SRPnL71(ip1A', ip2A', T1P1_wA', T1P2_wA', cH,Height);
        Rth71{j} = Rtf;
        Tth71{j} = Ttf;
        errortf71(j) = errftf; 
        errRtf71(j) = cal_rotation_err(Rtf,Rtruth);
        errTtf71(j) = cal_translation_err(Ttf,Ttruth);



        [EEs,GGs,CCs] = RefineEquA71B(p, W);
        apointsInis = [apoint(2) apoint(3)]';
        solutionOs = [Cayley(RBt);apointsInis]; 
        [solutions, obj_cur] = RefineGaussNewtonRandApT71B(solutionOs, p, W, EEs, GGs, CCs);

        if size(solutions) ~= 11
            aPointsNs = solutions(4:5);
        else
            aPointsNs = solutions(10:11);
        end

    %     aPointsNs = [apoint(2) apoint(3)]';

        T1P1_wB(1,1) =  aPointsNs(2)-50;
        T1P1_wB(2,1) =  aPointsNs(1);
        T1P1_wB(3,1) =  aPointsNs(2);
        T1P2_wB(1,1) =  aPointsNs(2);
        T1P2_wB(2,1) =  aPointsNs(1)+50;
        T1P2_wB(3,1) =  aPointsNs(1);
        [Rfs, Tfs, errfs] = SRPnL71(ip1B', ip2B', T1P1_wB', T1P2_wB', cH,Height); 

        Rts71{j} = Rfs;
        Tts71{j} = Tfs;
        errortfs71(j) = errfs; 
        errRtfs71(j) = cal_rotation_err(Rfs,Rtruth);
        errTtfs71(j) = cal_translation_err(Tfs,Ttruth);


        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        clearvars apointa apointb aPointsN aPointsNt aPointsNs apointsIni apointsInis apointsInit apointsIniu apointsIniv aPointsNu aPointsNv T1P1_wA T1P2_wA T1P1_wB T1P2_wB T1P1_wC T1P2_wC T1P1_wD T1P2_wD
        tic;

        T1P1_wA = [P1_w(1,:);P2_w(1,:);P1_w(1,:)];
        T1P2_wA = [P1_w(2,:);P2_w(2,:);P2_w(2,:)];


        T1P1_wC = [P1_w(1,:);P2_w(1,:);P1_w(1,:)];
        T1P2_wC = [P1_w(2,:);P2_w(2,:);P2_w(1,:)];

        [F1, G] = P2LMGA72(T1p1A, T1p2A, T1P1_wA, T1P2_wA, T1p1C, T1p2C, T1P1_wC, T1P2_wC, cH);
        errAmin = Inf;
        errBmin = Inf;
        for i = 1:length(F1)
            if F1(i).ValObjective(1) > 10 || F1(i).ValObjective(2) > 10 || F1(i).ValObjective(3) > 100 || F1(i).ValObjective(4) > 100
    %         if F1(i).ValObjective(1) + F1(i).ValObjective(2)> 2
                 continue
            end
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
                apointb(1) = F1(i).Val(1);
                apointb(2) = F1(i).Val(2);
                RBt = F1(i).RB;
                TBt = F1(i).TB;
                errBmin = F1(i).ValObjective(2);
            end
        end  

        if errAmin < errBmin

            [EE,GG,CC] = RefineEquA72A(p, W);
            apointsIni = [apointa(1) apointa(3)]';
            solutionO = [Cayley(RAt);apointsIni]; 
            [solution, obj_cur] = RefineGaussNewtonRandApT72A(solutionO, p, W, EE, GG, CC);


            if size(solution) ~= 11
                aPointsN = solution(4:5);
            else
                aPointsN = solution(10:11);
            end

    %         aPointsN = [apoint(1) apoint(3)]';

            T1P1_wA(1,1) =  aPointsN(1);
            T1P1_wA(2,1) =  aPointsN(2)-50;
            T1P1_wA(3,1) =  aPointsN(1);
            T1P2_wA(1,1) =  aPointsN(1)+50;
            T1P2_wA(2,1) =  aPointsN(2);
            T1P2_wA(3,1) =  aPointsN(2);
            [Rf, Tf, errf] = SRPnL71(ip1A', ip2A', T1P1_wA', T1P2_wA', cH,Height);
        else
            [EE,GG,CC] = RefineEquA72B(p, W);
            apointsIni = [apointb(1) apointb(2)]';
            solutionO = [Cayley(RBt);apointsIni]; 
            [solution, obj_cur] = RefineGaussNewtonRandApT72B(solutionO, p, W, EE, GG, CC);

            if size(solution) ~= 11
                aPointsN = solution(4:5);
            else
                aPointsN = solution(10:11);
            end

    %         aPointsN = [apoint(1) apoint(2)]';

            T1P1_wC(1,1) =  aPointsN(1);
            T1P1_wC(2,1) =  aPointsN(1);
            T1P1_wC(3,1) =  aPointsN(1);
            T1P2_wC(1,1) =  aPointsN(1)+50;
            T1P2_wC(2,1) =  aPointsN(2)+50;
            T1P2_wC(3,1) =  aPointsN(2);
            [Rf, Tf, errf] = SRPnL71(ip1C', ip2C', T1P1_wC', T1P2_wC', cH,Height); 
        end

        R72{j} = Rf;
        T72{j} = Tf;
        error72(j) = errf; 
        errRf72(j) = cal_rotation_err(Rf,Rtruth);
        errTf72(j) = cal_translation_err(Tf,Ttruth);
        toc;
        time72(j) = toc; 


        [EEt,GGt,CCt] = RefineEquA72A(p, W);
        apointsInit = [apointa(1) apointa(3)]';
        solutionOt = [Cayley(RAt);apointsInit]; 
        [solutiont, obj_curt] = RefineGaussNewtonRandApT72A(solutionOt, p, W, EEt, GGt, CCt);


        if size(solutiont) ~= 11
            aPointsNt = solutiont(4:5);
        else
            aPointsNt = solutiont(10:11);
        end

    %     aPointsNt = [apoint(1) apoint(3)]';


        T1P1_wA(1,1) =  aPointsNt(1);
        T1P1_wA(2,1) =  aPointsNt(2)-50;
        T1P1_wA(3,1) =  aPointsNt(1);
        T1P2_wA(1,1) =  aPointsNt(1)+50;
        T1P2_wA(2,1) =  aPointsNt(2);
        T1P2_wA(3,1) =  aPointsNt(2);
        [Rtf, Ttf, errftf] = SRPnL71(ip1A', ip2A', T1P1_wA', T1P2_wA', cH,Height);
        Rth72{j} = Rtf;
        Tth72{j} = Ttf;
        errortf72(j) = errftf; 
        errRtf72(j) = cal_rotation_err(Rtf,Rtruth);
        errTtf72(j) = cal_translation_err(Ttf,Ttruth);



        [EEs,GGs,CCs] = RefineEquA72B(p, W);
        apointsInis = [apointb(1) apointb(2)]';
        solutionOs = [Cayley(RBt);apointsInis]; 
        [solutions, obj_cur] = RefineGaussNewtonRandApT72B(solutionOs, p, W, EEs, GGs, CCs);

        if size(solutions) ~= 11
            aPointsNs = solutions(4:5);
        else
            aPointsNs = solutions(10:11);
        end

    %     aPointsNs = [apoint(1) apoint(2)]';

        T1P1_wC(1,1) =  aPointsNs(1);
        T1P1_wC(2,1) =  aPointsNs(2);
        T1P1_wC(3,1) =  aPointsNs(1);
        T1P2_wC(1,1) =  aPointsNs(1)+50;
        T1P2_wC(2,1) =  aPointsNs(2)+50;
        T1P2_wC(3,1) =  aPointsNs(2);
        [Rfs, Tfs, errfs] = SRPnL71(ip1C', ip2C', T1P1_wC', T1P2_wC', cH,Height); 

        Rts72{j} = Rfs;
        Tts72{j} = Tfs;
        errortfs72(j) = errfs; 
        errRtfs72(j) = cal_rotation_err(Rfs,Rtruth);
        errTtfs72(j) = cal_translation_err(Tfs,Ttruth);


        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        clearvars apointa apointb aPointsN aPointsNt aPointsNs apointsIni apointsInis apointsInit apointsIniu apointsIniv aPointsNu aPointsNv T1P1_wA T1P2_wA T1P1_wB T1P2_wB T1P1_wC T1P2_wC T1P1_wD T1P2_wD
        tic;
        T1P1_wA = [P1_w(1,:);P2_w(1,:);P1_w(1,:)];
        T1P2_wA = [P1_w(2,:);P2_w(2,:);P2_w(2,:)];


        T1P1_wD = [P1_w(1,:);P2_w(1,:);P1_w(2,:)];
        T1P2_wD = [P1_w(2,:);P2_w(2,:);P2_w(2,:)];

        [F1, G] = P2LMGA73(T1p1A, T1p2A, T1P1_wA, T1P2_wA, T1p1D, T1p2D, T1P1_wD, T1P2_wD, cH);
        errAmin = Inf;
        errBmin = Inf;
        for i = 1:length(F1)
            if F1(i).ValObjective(1) > 10 || F1(i).ValObjective(2) > 10 || F1(i).ValObjective(3) > 100 || F1(i).ValObjective(4) > 100
    %         if F1(i).ValObjective(1) + F1(i).ValObjective(2) > 2
                continue
            end
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


        if errAmin < errBmin

            [EE,GG,CC] = RefineEquA73A(p, W);
            apointsIni = [apointa(1) apointa(3)]';
            solutionO = [Cayley(RAt);apointsIni]; 
            [solution, obj_cur] = RefineGaussNewtonRandApT73A(solutionO, p, W, EE, GG, CC);


            if size(solution) ~= 11
                aPointsN = solution(4:5);
            else
                aPointsN = solution(10:11);
            end

    %         aPointsN = [apoint(1) apoint(3)]';

            T1P1_wA(1,1) =  aPointsN(1);
            T1P1_wA(2,1) =  aPointsN(2)-50;
            T1P1_wA(3,1) =  aPointsN(1);
            T1P2_wA(1,1) =  aPointsN(1)+50;
            T1P2_wA(2,1) =  aPointsN(2);
            T1P2_wA(3,1) =  aPointsN(2);
            [Rf, Tf, errf] = SRPnL71(ip1A', ip2A', T1P1_wA', T1P2_wA', cH,Height);
        else
            [EE,GG,CC] = RefineEquA73B(p, W);
            apointsIni = [apointb(2) apointb(3)]';
            solutionO = [Cayley(RBt);apointsIni]; 
            [solution, obj_cur] = RefineGaussNewtonRandApT73B(solutionO, p, W, EE, GG, CC);

            if size(solution) ~= 11
                aPointsN = solution(4:5);
            else
                aPointsN = solution(10:11);
            end

    %         aPointsN = [apoint(2) apoint(3)]';

            T1P1_wD(1,1) =  aPointsN(1)-50;
            T1P1_wD(2,1) =  aPointsN(2)-50;
            T1P1_wD(3,1) =  aPointsN(1);
            T1P2_wD(1,1) =  aPointsN(1);
            T1P2_wD(2,1) =  aPointsN(2);
            T1P2_wD(3,1) =  aPointsN(2);
            [Rf, Tf, errf] = SRPnL71(ip1D', ip2D', T1P1_wD', T1P2_wD', cH,Height); 
        end

        R73{j} = Rf;
        T73{j} = Tf;
        error73(j) = errf; 
        errRf73(j) = cal_rotation_err(Rf,Rtruth);
        errTf73(j) = cal_translation_err(Tf,Ttruth);
        toc;
        time73(j) = toc; 


        [EEt,GGt,CCt] = RefineEquA73A(p, W);
        apointsInit = [apointa(1) apointa(3)]';
        solutionOt = [Cayley(RAt);apointsInit]; 
        [solutiont, obj_curt] = RefineGaussNewtonRandApT73A(solutionOt, p, W, EEt, GGt, CCt);


        if size(solutiont) ~= 11
            aPointsNt = solutiont(4:5);
        else
            aPointsNt = solutiont(10:11);
        end

    %     aPointsNt = [apoint(1) apoint(3)]';

        T1P1_wA(1,1) =  aPointsNt(1);
        T1P1_wA(2,1) =  aPointsNt(2)-50;
        T1P1_wA(3,1) =  aPointsNt(1);
        T1P2_wA(1,1) =  aPointsNt(1)+50;
        T1P2_wA(2,1) =  aPointsNt(2);
        T1P2_wA(3,1) =  aPointsNt(2);
        [Rtf, Ttf, errftf] = SRPnL71(ip1A', ip2A', T1P1_wA', T1P2_wA', cH,Height);
        Rth73{j} = Rtf;
        Tth73{j} = Ttf;
        errortf73(j) = errftf; 
        errRtf73(j) = cal_rotation_err(Rtf,Rtruth);
        errTtf73(j) = cal_translation_err(Ttf,Ttruth);



        [EEs,GGs,CCs] = RefineEquA73B(p, W);
        apointsInis = [apointb(2) apointb(3)]';
        solutionOs = [Cayley(RBt);apointsInis]; 
        [solutions, obj_cur] = RefineGaussNewtonRandApT73B(solutionOs, p, W, EEs, GGs, CCs);

        if size(solutions) ~= 11
            aPointsNs = solutions(4:5);
        else
            aPointsNs = solutions(10:11);
        end

    %     aPointsNs = [apoint(2) apoint(3)]';

        T1P1_wD(1,1) =  aPointsNs(1)-50;
        T1P1_wD(2,1) =  aPointsNs(2)-50;
        T1P1_wD(3,1) =  aPointsNs(1);
        T1P2_wD(1,1) =  aPointsNs(1);
        T1P2_wD(2,1) =  aPointsNs(2);
        T1P2_wD(3,1) =  aPointsNs(2);
        [Rfs, Tfs, errfs] = SRPnL71(ip1D', ip2D', T1P1_wD', T1P2_wD', cH,Height); 

        Rts73{j} = Rfs;
        Tts73{j} = Tfs;
        errortfs73(j) = errfs; 
        errRtfs73(j) = cal_rotation_err(Rfs,Rtruth);
        errTtfs73(j) = cal_translation_err(Tfs,Ttruth);


         %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        clearvars apointa apointb aPointsN aPointsNt aPointsNs apointsIni apointsInis apointsInit apointsIniu apointsIniv aPointsNu aPointsNv T1P1_wA T1P2_wA T1P1_wB T1P2_wB T1P1_wC T1P2_wC T1P1_wD T1P2_wD
        tic; 
        T1P1_wB = [P1_w(1,:);P2_w(1,:);P1_w(2,:)];
        T1P2_wB = [P1_w(2,:);P2_w(2,:);P2_w(1,:)];

        T1P1_wC = [P1_w(1,:);P2_w(1,:);P1_w(1,:)];
        T1P2_wC = [P1_w(2,:);P2_w(2,:);P2_w(1,:)]; 

        [F1, G] = P2LMGA74(T1p1B, T1p2B, T1P1_wB, T1P2_wB, T1p1C, T1p2C, T1P1_wC, T1P2_wC, cH);
        errAmin = Inf;
        errBmin = Inf;
        for i = 1:length(F1)
    %         if F1(i).ValObjective(1) > 2 || F1(i).ValObjective(2) > 2 || F1(i).ValObjective(3) > 2 || F1(i).ValObjective(4) > 2
            % + F1(i).ValObjective(3) + F1(i).ValObjective(4)
            if F1(i).ValObjective(1) > 10 || F1(i).ValObjective(2) > 10 || F1(i).ValObjective(3) > 100 || F1(i).ValObjective(4) > 100
    %         if F1(i).ValObjective(1) + F1(i).ValObjective(2) > 2
                continue
            end
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


        if errAmin < errBmin

            [EE,GG,CC] = RefineEquA74A(p, W);
            apointsIni = [apointa(2) apointa(3)]';
            solutionO = [Cayley(RAt);apointsIni]; 
            [solution, obj_cur] = RefineGaussNewtonRandApT74A(solutionO, p, W, EE, GG, CC);


            if size(solution) ~= 11
                aPointsN = solution(4:5);
            else
                aPointsN = solution(10:11);
            end

    %         aPointsN = [apoint(2) apoint(3)]';


            T1P1_wB(1,1) =  aPointsN(2)-50;
            T1P1_wB(2,1) =  aPointsN(1);
            T1P1_wB(3,1) =  aPointsN(2);
            T1P2_wB(1,1) =  aPointsN(2);
            T1P2_wB(2,1) =  aPointsN(1)+50;
            T1P2_wB(3,1) =  aPointsN(1);
            [Rf, Tf, errf] = SRPnL71(ip1B', ip2B', T1P1_wB', T1P2_wB', cH,Height);
        else
            [EE,GG,CC] = RefineEquA74B(p, W);
            apointsIni = [apointb(1) apointb(2)]';
            solutionO = [Cayley(RBt);apointsIni]; 
            [solution, obj_cur] = RefineGaussNewtonRandApT74B(solutionO, p, W, EE, GG, CC);

            if size(solution) ~= 11
                aPointsN = solution(4:5);
            else
                aPointsN = solution(10:11);
            end

    %         aPointsN = [apoint(1) apoint(2)]';

            T1P1_wC(1,1) =  aPointsN(1);
            T1P1_wC(2,1) =  aPointsN(2);
            T1P1_wC(3,1) =  aPointsN(1);
            T1P2_wC(1,1) =  aPointsN(1)+50;
            T1P2_wC(2,1) =  aPointsN(2)+50;
            T1P2_wC(3,1) =  aPointsN(2);
            [Rf, Tf, errf] = SRPnL71(ip1C', ip2C', T1P1_wC', T1P2_wC', cH,Height); 
        end

        R74{j} = Rf;
        T74{j} = Tf;
        error74(j) = errf; 
        errRf74(j) = cal_rotation_err(Rf,Rtruth);
        errTf74(j) = cal_translation_err(Tf,Ttruth);
        toc;
        time74(j) = toc; 


        [EEt,GGt,CCt] = RefineEquA74A(p, W);
        apointsInit = [apointa(2) apointa(3)]';
        solutionOt = [Cayley(RAt);apointsInit]; 
        [solutiont, obj_curt] = RefineGaussNewtonRandApT74A(solutionOt, p, W, EEt, GGt, CCt);


        if size(solutiont) ~= 11
            aPointsNt = solutiont(4:5);
        else
            aPointsNt = solutiont(10:11);
        end

    %     aPointsNt = [apoint(2) apoint(3)]';

        T1P1_wB(1,1) =  aPointsNt(2)-50;
        T1P1_wB(2,1) =  aPointsNt(1);
        T1P1_wB(3,1) =  aPointsNt(2);
        T1P2_wB(1,1) =  aPointsNt(2);
        T1P2_wB(2,1) =  aPointsNt(1)+50;
        T1P2_wB(3,1) =  aPointsNt(1);
        [Rtf, Ttf, errftf] = SRPnL71(ip1B', ip2B', T1P1_wB', T1P2_wB', cH,Height);
        Rth74{j} = Rtf;
        Tth74{j} = Ttf;
        errortf74(j) = errftf; 
        errRtf74(j) = cal_rotation_err(Rtf,Rtruth);
        errTtf74(j) = cal_translation_err(Ttf,Ttruth);



        [EEs,GGs,CCs] = RefineEquA74B(p, W);
        apointsInis = [apointb(1) apointb(2)]';
        solutionOs = [Cayley(RBt);apointsInis]; 
        [solutions, obj_cur] = RefineGaussNewtonRandApT74B(solutionOs, p, W, EEs, GGs, CCs);

        if size(solutions) ~= 11
            aPointsNs = solutions(4:5);
        else
            aPointsNs = solutions(10:11);
        end

    %     aPointsNs = [apoint(1) apoint(2)]';

        T1P1_wC(1,1) =  aPointsNs(1);
        T1P1_wC(2,1) =  aPointsNs(2);
        T1P1_wC(3,1) =  aPointsNs(1);
        T1P2_wB(1,1) =  aPointsNs(1)+50;
        T1P2_wB(2,1) =  aPointsNs(2)+50;
        T1P2_wB(3,1) =  aPointsNs(2);
        [Rfs, Tfs, errfs] = SRPnL71(ip1C', ip2C', T1P1_wC', T1P2_wC', cH,Height); 

        Rts74{j} = Rfs;
        Tts74{j} = Tfs;
        errortfs74(j) = errfs; 
        errRtfs74(j) = cal_rotation_err(Rfs,Rtruth);
        errTtfs74(j) = cal_translation_err(Tfs,Ttruth);

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        clearvars apointa apointb aPointsN aPointsNt aPointsNs apointsIni apointsInis apointsInit apointsIniu apointsIniv aPointsNu aPointsNv T1P1_wA T1P2_wA T1P1_wB T1P2_wB T1P1_wC T1P2_wC T1P1_wD T1P2_wD
        tic;

        T1P1_wB = [P1_w(1,:);P2_w(1,:);P1_w(2,:)];
        T1P2_wB = [P1_w(2,:);P2_w(2,:);P2_w(1,:)];


        T1P1_wD = [P1_w(1,:);P2_w(1,:);P1_w(2,:)];
        T1P2_wD = [P1_w(2,:);P2_w(2,:);P2_w(2,:)];

        [F1, G] = P2LMGA75(T1p1B, T1p2B, T1P1_wB, T1P2_wB, T1p1D, T1p2D, T1P1_wD, T1P2_wD, cH);
        errAmin = Inf;
        errBmin = Inf;
        for i = 1:length(F1)
    %         if F1(i).ValObjective(1) > 0.5 || F1(i).ValObjective(2) > 0.5 || F1(i).ValObjective(3) > 0.5 || F1(i).ValObjective(4) > 0.5
            if F1(i).ValObjective(1) > 10 || F1(i).ValObjective(2) > 10 || F1(i).ValObjective(3) > 100 || F1(i).ValObjective(4) > 100
    %         if F1(i).ValObjective(1) + F1(i).ValObjective(2) > 2
                continue
            end
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




        if errAmin < errBmin

            [EE,GG,CC] = RefineEquA75A(p, W);
            apointsIni = [apointa(1) apointa(2)]';
            solutionO = [Cayley(RAt);apointsIni]; 
            [solution, obj_cur] = RefineGaussNewtonRandApT75A(solutionO, p, W, EE, GG, CC);


            if size(solution) ~= 11
                aPointsN = solution(4:5);
            else
                aPointsN = solution(10:11);
            end

    %         aPointsN = [apoint(1) apoint(2)]';

            T1P1_wB(1,1) =  aPointsN(2)-50;
            T1P1_wB(2,1) =  aPointsN(1);
            T1P1_wB(3,1) =  aPointsN(2);
            T1P2_wB(1,1) =  aPointsN(2);
            T1P2_wB(2,1) =  aPointsN(1)+50;
            T1P2_wB(3,1) =  aPointsN(1);
            [Rf, Tf, errf] = SRPnL71(ip1B', ip2B', T1P1_wB', T1P2_wB', cH,Height);
        else
            [EE,GG,CC] = RefineEquA75B(p, W);
            apointsIni = [apointb(2) apointb(3)]';
            solutionO = [Cayley(RBt);apointsIni]; 
            [solution, obj_cur] = RefineGaussNewtonRandApT75B(solutionO, p, W, EE, GG, CC);

            if size(solution) ~= 11
                aPointsN = solution(4:5);
            else
                aPointsN = solution(10:11);
            end

    %         aPointsN = [apoint(2) apoint(3)]';

            T1P1_wD(1,1) =  aPointsN(1)-50;
            T1P1_wD(2,1) =  aPointsN(2)-50;
            T1P1_wD(3,1) =  aPointsN(1);
            T1P2_wD(1,1) =  aPointsN(1);
            T1P2_wD(2,1) =  aPointsN(2);
            T1P2_wD(3,1) =  aPointsN(2);
            [Rf, Tf, errf] = SRPnL71(ip1D', ip2D', T1P1_wD', T1P2_wD', cH,Height); 
        end

        R75{j} = Rf;
        T75{j} = Tf;
        error75(j) = errf; 
        errRf75(j) = cal_rotation_err(Rf,Rtruth);
        errTf75(j) = cal_translation_err(Tf,Ttruth);
        toc;
        time75(j) = toc; 


        [EEt,GGt,CCt] = RefineEquA75A(p, W);
        apointsInit = [apointa(1) apointa(2)]';
        solutionOt = [Cayley(RAt);apointsInit]; 
        [solutiont, obj_curt] = RefineGaussNewtonRandApT75A(solutionOt, p, W, EEt, GGt, CCt);


        if size(solutiont) ~= 11
            aPointsNt = solutiont(4:5);
        else
            aPointsNt = solutiont(10:11);
        end

    %     aPointsNt = [apoint(1) apoint(2)]';

        T1P1_wB(1,1) =  aPointsNt(2)-50;
        T1P1_wB(2,1) =  aPointsNt(1);
        T1P1_wB(3,1) =  aPointsNt(2);
        T1P2_wB(1,1) =  aPointsNt(2);
        T1P2_wB(2,1) =  aPointsNt(1)+50;
        T1P2_wB(3,1) =  aPointsNt(1);
        [Rtf, Ttf, errftf] = SRPnL71(ip1B', ip2B', T1P1_wB', T1P2_wB', cH,Height);
        Rth75{j} = Rtf;
        Tth75{j} = Ttf;
        errortf75(j) = errftf; 
        errRtf75(j) = cal_rotation_err(Rtf,Rtruth);
        errTtf75(j) = cal_translation_err(Ttf,Ttruth);



        [EEs,GGs,CCs] = RefineEquA75B(p, W);
        apointsInis = [apointb(2) apointb(3)]';
        solutionOs = [Cayley(RBt);apointsInis]; 
        [solutions, obj_cur] = RefineGaussNewtonRandApT75B(solutionOs, p, W, EEs, GGs, CCs);

        if size(solutions) ~= 11
            aPointsNs = solutions(4:5);
        else
            aPointsNs = solutions(10:11);
        end

    %     aPointsNs = [apoint(2) apoint(3)]';

        T1P1_wD(1,1) =  aPointsNs(1)-50;
        T1P1_wD(2,1) =  aPointsNs(2)-50;
        T1P1_wD(3,1) =  aPointsNs(1);
        T1P2_wD(1,1) =  aPointsNs(1);
        T1P2_wD(2,1) =  aPointsNs(2);
        T1P2_wD(3,1) =  aPointsNs(2);
        [Rfs, Tfs, errfs] = SRPnL71(ip1D', ip2D', T1P1_wD', T1P2_wD', cH,Height); 

        Rts75{j} = Rfs;
        Tts75{j} = Tfs;
        errortfs75(j) = errfs; 
        errRtfs75(j) = cal_rotation_err(Rfs,Rtruth);
        errTtfs75(j) = cal_translation_err(Tfs,Ttruth);    

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        clearvars apoint aPointsN aPointsNt aPointsNs apointsIni apointsInis apointsInit apointsIniu apointsIniv aPointsNu aPointsNv T1P1_wA T1P2_wA T1P1_wB T1P2_wB T1P1_wC T1P2_wC T1P1_wD T1P2_wD
        tic;
        T1P1_wC = [P1_w(1,:);P2_w(1,:);P1_w(1,:)];
        T1P2_wC = [P1_w(2,:);P2_w(2,:);P2_w(1,:)]; 

        T1P1_wD = [P1_w(1,:);P2_w(1,:);P1_w(2,:)];
        T1P2_wD = [P1_w(2,:);P2_w(2,:);P2_w(2,:)];

        [F1, G] = P2LMGA76(T1p1C, T1p2C, T1P1_wC, T1P2_wC, T1p1D, T1p2D, T1P1_wD, T1P2_wD, cH);
        errAmin = Inf;
        errBmin = Inf;
        for i = 1:length(F1)
    %         if F1(i).ValObjective(1) > 0.5 || F1(i).ValObjective(2) > 0.5 || F1(i).ValObjective(3) > 0.5 || F1(i).ValObjective(4) > 0.5
            if F1(i).ValObjective(1) > 10 || F1(i).ValObjective(2) > 10 || F1(i).ValObjective(3) > 100 || F1(i).ValObjective(4) > 100
    %         if F1(i).ValObjective(1) + F1(i).ValObjective(2)  > 2
                continue
            end
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


%         errOriRf76(j) = cal_rotation_err(Ri,Rtruth);
%         errOriTf76(j) = cal_translation_err(Ti,Ttruth);


        if errAmin < errBmin

            [EE,GG,CC] = RefineEquA76A(p, W);
            apointsIni = [apoint(1) apoint(2)]';
            solutionO = [Cayley(RAt);apointsIni]; 
            [solution, obj_cur] = RefineGaussNewtonRandApT76A(solutionO, p, W, EE, GG, CC);


            if size(solution) ~= 11
                aPointsN = solution(4:5);
            else
                aPointsN = solution(10:11);
            end

    %         aPointsN = [apoint(1) apoint(2)]';

            T1P1_wC(1,1) =  aPointsN(1);
            T1P1_wC(2,1) =  aPointsN(2);
            T1P1_wC(3,1) =  aPointsN(1);
            T1P2_wC(1,1) =  aPointsN(1)+50;
            T1P2_wC(2,1) =  aPointsN(2)+50;
            T1P2_wC(3,1) =  aPointsN(2);
            [Rf, Tf, errf] = SRPnL71(ip1C', ip2C', T1P1_wC', T1P2_wC', cH,Height);
        else
            [EE,GG,CC] = RefineEquA76B(p, W);
            apointsIni = [apoint(3) apoint(4)]';
            solutionO = [Cayley(RBt);apointsIni]; 
            [solution, obj_cur] = RefineGaussNewtonRandApT76B(solutionO, p, W, EE, GG, CC);

            if size(solution) ~= 11
                aPointsN = solution(4:5);
            else
                aPointsN = solution(10:11);
            end

    %         aPointsN = [apoint(3) apoint(4)]';

            T1P1_wD(1,1) =  aPointsN(1)-50;
            T1P1_wD(2,1) =  aPointsN(2)-50;
            T1P1_wD(3,1) =  aPointsN(1);
            T1P2_wD(1,1) =  aPointsN(1);
            T1P2_wD(2,1) =  aPointsN(2);
            T1P2_wD(3,1) =  aPointsN(2);
            [Rf, Tf, errf] = SRPnL71(ip1D', ip2D', T1P1_wD', T1P2_wD', cH,Height); 
        end

        R76{j} = Rf;
        T76{j} = Tf;
        error76(j) = errf; 
        errRf76(j) = cal_rotation_err(Rf,Rtruth);
        errTf76(j) = cal_translation_err(Tf,Ttruth);
        toc;
        time76(j) = toc; 


        [EEt,GGt,CCt] = RefineEquA76A(p, W);
        apointsInit = [apoint(1) apoint(2)]';
        solutionOt = [Cayley(RAt);apointsInit]; 
        [solutiont, obj_curt] = RefineGaussNewtonRandApT76A(solutionOt, p, W, EEt, GGt, CCt);


        if size(solutiont) ~= 11
            aPointsNt = solutiont(4:5);
        else
            aPointsNt = solutiont(10:11);
        end

    %     aPointsNt = [apoint(1) apoint(2)]';

        T1P1_wC(1,1) =  aPointsNt(1);
        T1P1_wC(2,1) =  aPointsNt(2);
        T1P1_wC(3,1) =  aPointsNt(1);
        T1P2_wC(1,1) =  aPointsNt(1)+50;
        T1P2_wC(2,1) =  aPointsNt(2)+50;
        T1P2_wC(3,1) =  aPointsNt(2);
        [Rtf, Ttf, errftf] = SRPnL71(ip1C', ip2C', T1P1_wC', T1P2_wC', cH,Height);
        Rth76{j} = Rtf;
        Tth76{j} = Ttf;
        errortf76(j) = errftf; 
        errRtf76(j) = cal_rotation_err(Rtf,Rtruth);
        errTtf76(j) = cal_translation_err(Ttf,Ttruth);



        [EEs,GGs,CCs] = RefineEquA76B(p, W);
        apointsInis = [apoint(3) apoint(4)]';
        solutionOs = [Cayley(RBt);apointsInis]; 
        [solutions, obj_cur] = RefineGaussNewtonRandApT76B(solutionOs, p, W, EEs, GGs, CCs);

        if size(solutions) ~= 11
            aPointsNs = solutions(4:5);
        else
            aPointsNs = solutions(10:11);
        end

    %     aPointsNs = [apoint(3) apoint(4)]';

        T1P1_wD(1,1) =  aPointsNs(1)-50;
        T1P1_wD(2,1) =  aPointsNs(2)-50;
        T1P1_wD(3,1) =  aPointsNs(1);
        T1P2_wD(1,1) =  aPointsNs(1);
        T1P2_wD(2,1) =  aPointsNs(2);
        T1P2_wD(3,1) =  aPointsNs(2);
        [Rfs, Tfs, errfs] = SRPnL71(ip1D', ip2D', T1P1_wD', T1P2_wD', cH,Height); 

        Rts76{j} = Rfs;
        Tts76{j} = Tfs;
        errortfs76(j) = errfs; 
        errRtfs76(j) = cal_rotation_err(Rfs,Rtruth);
        errTtfs76(j) = cal_translation_err(Tfs,Ttruth);


        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        clearvars apointa apointb aPointsN aPointsNt aPointsNs apointsIni apointsInis apointsInit apointsIniu apointsIniv aPointsNu aPointsNv T1P1_wA T1P2_wA T1P1_wB T1P2_wB T1P1_wC T1P2_wC T1P1_wD T1P2_wD
        tic;

        T1P1_wA = [P1_w(1,:);P2_w(1,:);P1_w(1,:)];
        T1P2_wA = [P1_w(2,:);P2_w(2,:);P2_w(2,:)];

        T1P1_wB = [P1_w(1,:);P2_w(1,:);P1_w(2,:)];
        T1P2_wB = [P1_w(2,:);P2_w(2,:);P2_w(1,:)];

        T1P1_wC = [P1_w(1,:);P2_w(1,:);P1_w(1,:)];
        T1P2_wC = [P1_w(2,:);P2_w(2,:);P2_w(1,:)]; 

        T1P1_wD = [P1_w(1,:);P2_w(1,:);P1_w(2,:)];
        T1P2_wD = [P1_w(2,:);P2_w(2,:);P2_w(2,:)];

        [F1, G] = P2LMGA77(T1p1A, T1p2A, T1P1_wA, T1P2_wA, T1p1B, T1p2B, T1P1_wB, T1P2_wB, T1p1C, T1p2C, T1P1_wC, T1P2_wC, T1p1D, T1p2D, T1P1_wD, T1P2_wD, cH);
        errAmin = Inf;
        errBmin = Inf;
        errCmin = Inf;
        errDmin = Inf;
        Ri = eye(3,3);
        Ti = [0,0,0];
        for i = 1:length(F1)
    %         if F1(i).ValObjective(1) > 1 || F1(i).ValObjective(2) > 1 || F1(i).ValObjective(3) > 1 || F1(i).ValObjective(4) > 1
            if F1(i).ValObjective(1) > 10 || F1(i).ValObjective(2) > 10 || F1(i).ValObjective(3) > 100 || F1(i).ValObjective(4) > 100
    %         if F1(i).ValObjective(1) + F1(i).ValObjective(2) > 2
                continue
            end
    %         apoints(count,:) =  F1(i).Val;   
    %         count = count + 1;
            if F1(i).ValObjective(1) < errAmin 
                apointa(1) = F1(i).Val(1);
                apointa(4) = F1(i).Val(4);
                RAt = F1(i).RA;
                TAt = F1(i).TA;
                errAmin = F1(i).ValObjective(1);
            end
            if F1(i).ValObjective(2) < errBmin 
                apointa(2) = F1(i).Val(2);
                apointa(3) = F1(i).Val(3);
                RBt = F1(i).RB;
                TBt = F1(i).TB;
                errBmin = F1(i).ValObjective(2);
            end 
            if F1(i).ValObjective(3) < errCmin 
                apointb(1) = F1(i).Val(1);
                apointb(2) = F1(i).Val(2);
                RCt = F1(i).RC;
                TCt = F1(i).TC;
                errCmin = F1(i).ValObjective(3);
            end
            if F1(i).ValObjective(4) < errDmin 
                apointb(3) = F1(i).Val(3);
                apointb(4) = F1(i).Val(4);
                RDt = F1(i).RD;
                TDt = F1(i).TD;
                errDmin = F1(i).ValObjective(4);
            end 
        end  

        errg = [errAmin,errBmin,errCmin,errDmin];
%         errOriRf77(j) = cal_rotation_err(Ri,Rtruth);
%         errOriTf77(j) = cal_translation_err(Ti,Ttruth);
        [~,flag] = min(errg);

        if flag  == 1

            [EE,GG,CC] = RefineEquA77A(p, W);
            apointsIni = [apointa(1) apointa(4)]';
            solutionO = [Cayley(RAt);apointsIni]; 
            [solution, obj_cur] = RefineGaussNewtonRandApT77A(solutionO, p, W, EE, GG, CC);


            if size(solution) ~= 11
                aPointsN = solution(4:5);
            else
                aPointsN = solution(10:11);
            end

    %         aPointsN = [apoint(1) apoint(4)]';

            T1P1_wA(1,1) =  aPointsN(1);
            T1P1_wA(2,1) =  aPointsN(2)-50;
            T1P1_wA(3,1) =  aPointsN(1);
            T1P2_wA(1,1) =  aPointsN(1)+50;
            T1P2_wA(2,1) =  aPointsN(2);
            T1P2_wA(3,1) =  aPointsN(2);
            [Rf, Tf, errf] = SRPnL71(ip1A', ip2A', T1P1_wA', T1P2_wA', cH,Height);
        elseif flag == 2
            [EE,GG,CC] = RefineEquA77B(p, W);
            apointsIni = [apointa(2) apointa(3)]';
            solutionO = [Cayley(RBt);apointsIni]; 
            [solution, obj_cur] = RefineGaussNewtonRandApT77B(solutionO, p, W, EE, GG, CC);

            if size(solution) ~= 11
                aPointsN = solution(4:5);
            else
                aPointsN = solution(10:11);
            end

    %         aPointsN = [apoint(2) apoint(3)]';

            T1P1_wB(1,1) =  aPointsN(2)-50;
            T1P1_wB(2,1) =  aPointsN(1);
            T1P1_wB(3,1) =  aPointsN(2);
            T1P2_wB(1,1) =  aPointsN(2);
            T1P2_wB(2,1) =  aPointsN(1)+50;
            T1P2_wB(3,1) =  aPointsN(1);
            [Rf, Tf, errf] = SRPnL71(ip1B', ip2B', T1P1_wB', T1P2_wB', cH,Height); 
        elseif flag == 3
            [EE,GG,CC] = RefineEquA77C(p, W);
            apointsIni = [apointb(1) apointb(2)]';
            solutionO = [Cayley(RCt);apointsIni]; 
            [solution, obj_cur] = RefineGaussNewtonRandApT77C(solutionO, p, W, EE, GG, CC);

            if size(solution) ~= 11
                aPointsN = solution(4:5);
            else
                aPointsN = solution(10:11);
            end

    %         aPointsN = [apoint(3) apoint(4)]';

            T1P1_wC(1,1) =  aPointsN(1);
            T1P1_wC(2,1) =  aPointsN(2);
            T1P1_wC(3,1) =  aPointsN(1);
            T1P2_wC(1,1) =  aPointsN(1)+ 50;
            T1P2_wC(2,1) =  aPointsN(2)+ 50;
            T1P2_wC(3,1) =  aPointsN(2);
            [Rf, Tf, errf] = SRPnL71(ip1C', ip2C', T1P1_wC', T1P2_wC', cH,Height); 
        elseif flag == 4
            [EE,GG,CC] = RefineEquA77D(p, W);
            apointsIni = [apointb(3) apointb(4)]';
            solutionO = [Cayley(RDt);apointsIni]; 
            [solution, obj_cur] = RefineGaussNewtonRandApT77D(solutionO, p, W, EE, GG, CC);

            if size(solution) ~= 11
                aPointsN = solution(4:5);
            else
                aPointsN = solution(10:11);
            end

    %         aPointsN = [apoint(3) apoint(4)]';

            T1P1_wD(1,1) =  aPointsN(1)-50;
            T1P1_wD(2,1) =  aPointsN(2)-50;
            T1P1_wD(3,1) =  aPointsN(1);
            T1P2_wD(1,1) =  aPointsN(1);
            T1P2_wD(2,1) =  aPointsN(2);
            T1P2_wD(3,1) =  aPointsN(2);
            [Rf, Tf, errf] = SRPnL71(ip1D', ip2D', T1P1_wD', T1P2_wD', cH,Height); 
        end

        R77{j} = Rf;
        T77{j} = Tf;
        error77(j) = errf; 
        errRf77(j) = cal_rotation_err(Rf,Rtruth);
        errTf77(j) = cal_translation_err(Tf,Ttruth);
        toc;
        time77(j) = toc; 

        [EEt,GGt,CCt] = RefineEquA77A(p, W);
        apointsInit = [apointa(1) apointa(4)]';
        solutionOt = [Cayley(RAt);apointsInit]; 
        [solutiont, obj_curt] = RefineGaussNewtonRandApT77A(solutionOt, p, W, EEt, GGt, CCt);


        if size(solutiont) ~= 11
            aPointsNt = solutiont(4:5);
        else
            aPointsNt = solutiont(10:11);
        end

    %     aPointsNt = [apoint(1) apoint(4)]';

        T1P1_wA(1,1) =  aPointsNt(1);
        T1P1_wA(2,1) =  aPointsNt(2)-50;
        T1P1_wA(3,1) =  aPointsNt(1);
        T1P2_wA(1,1) =  aPointsNt(1)+50;
        T1P2_wA(2,1) =  aPointsNt(2);
        T1P2_wA(3,1) =  aPointsNt(2);
        [Rtf, Ttf, errftf] = SRPnL71(ip1A', ip2A', T1P1_wA', T1P2_wA', cH,Height);
        Rth77{j} = Rtf;
        Tth77{j} = Ttf;
        errortf77(j) = errftf; 
        errRtf77(j) = cal_rotation_err(Rtf,Rtruth);
        errTtf77(j) = cal_translation_err(Ttf,Ttruth);



        [EEs,GGs,CCs] = RefineEquA77B(p, W);
        apointsInis = [apointa(2) apointa(3)]';
        solutionOs = [Cayley(RBt);apointsInis]; 
        [solutions, obj_cur] = RefineGaussNewtonRandApT77B(solutionOs, p, W, EEs, GGs, CCs);

        if size(solutions) ~= 11
            aPointsNs = solutions(4:5);
        else
            aPointsNs = solutions(10:11);
        end

    %     aPointsNs = [apoint(2) apoint(3)]';

        T1P1_wB(1,1) =  aPointsNs(2)-50;
        T1P1_wB(2,1) =  aPointsNs(1);
        T1P1_wB(3,1) =  aPointsNs(2);
        T1P2_wB(1,1) =  aPointsNs(2);
        T1P2_wB(2,1) =  aPointsNs(1)+50;
        T1P2_wB(3,1) =  aPointsNs(1);
        [Rfs, Tfs, errfs] = SRPnL71(ip1B', ip2B', T1P1_wB', T1P2_wB', cH,Height); 

        Rts77{j} = Rfs;
        Tts77{j} = Tfs;
        errortfs77(j) = errfs; 
        errRtfs77(j) = cal_rotation_err(Rfs,Rtruth);
        errTtfs77(j) = cal_translation_err(Tfs,Ttruth);


        [EEu,GGu,CCu] = RefineEquA77C(p, W);
        apointsIniu = [apointb(1) apointb(2)]';
        solutionOu = [Cayley(RCt);apointsIniu]; 
        [solutionu, obj_curt] = RefineGaussNewtonRandApT77C(solutionOu, p, W, EEu, GGu, CCu);


        if size(solutionu) ~= 11
            aPointsNu = solutionu(4:5);
        else
            aPointsNu = solutionu(10:11);
        end

    %     aPointsNu = [apoint(1) apoint(2)]';

        T1P1_wC(1,1) =  aPointsNu(1);
        T1P1_wC(2,1) =  aPointsNu(2);
        T1P1_wC(3,1) =  aPointsNu(1);
        T1P2_wC(1,1) =  aPointsNu(1)+50;
        T1P2_wC(2,1) =  aPointsNu(2)+50;
        T1P2_wC(3,1) =  aPointsNu(2);
        [Ruf, Tuf, errfuf] = SRPnL71(ip1C', ip2C', T1P1_wC', T1P2_wC', cH,Height);
        Rthu77{j} = Ruf;
        Tthu77{j} = Tuf;
        errortfu77(j) = errfuf; 
        errRtfu77(j) = cal_rotation_err(Ruf,Rtruth);
        errTtfu77(j) = cal_translation_err(Tuf,Ttruth);



        [EEv,GGv,CCv] = RefineEquA77D(p, W);
        apointsIniv = [apointb(3) apointb(4)]';
        solutionOv = [Cayley(RDt);apointsIniv]; 
        [solutionv, obj_cur] = RefineGaussNewtonRandApT77D(solutionOv, p, W, EEv, GGv, CCv);

        if size(solutionv) ~= 11
            aPointsNv = solutionv(4:5);
        else
            aPointsNv = solutionv(10:11);
        end

    %     aPointsNv = [apoint(3) apoint(4)]';

        T1P1_wD(1,1) =  aPointsNv(1)-50;
        T1P1_wD(2,1) =  aPointsNv(2)-50;
        T1P1_wD(3,1) =  aPointsNv(1);
        T1P2_wD(1,1) =  aPointsNv(1);
        T1P2_wD(2,1) =  aPointsNv(2);
        T1P2_wD(3,1) =  aPointsNv(2);
        [Rfv, Tfv, errfv] = SRPnL71(ip1D', ip2D', T1P1_wD', T1P2_wD', cH,Height); 

        Rtv77{j} = Rfv;
        Ttv77{j} = Tfv;
        errortfv77(j) = errfs; 
        errRtfv77(j) = cal_rotation_err(Rfv,Rtruth);
        errTtfv77(j) = cal_translation_err(Tfv,Ttruth);



    end  


    err71GApnlRm{no+1} = mean(errRf71);
    err71GApnlRme{no+1} = median(errRf71);
    err71GApnlTm{no+1} = mean(errTf71);
    err71GApnlTme{no+1} = median(errTf71);


    err71GAtpnlRm{no+1} = mean(errRtf71);
    err71GAtpnlRme{no+1} = median(errRtf71);
    err71GAtpnlTm{no+1} = mean(errTtf71);
    err71GAtpnlTme{no+1} = median(errTtf71);


    err71GAspnlRm{no+1} = mean(errRtfs71);
    err71GAspnlRme{no+1} = median(errRtfs71);
    err71GAspnlTm{no+1} = mean(errTtfs71);
    err71GAspnlTme{no+1} = median(errTtfs71);

    err72GApnlRm{no+1} = mean(errRf72);
    err72GApnlRme{no+1} = median(errRf72);
    err72GApnlTm{no+1} = mean(errTf72);
    err72GApnlTme{no+1} = median(errTf72);


    err72GAtpnlRm{no+1} = mean(errRtf72);
    err72GAtpnlRme{no+1} = median(errRtf72);
    err72GAtpnlTm{no+1} = mean(errTtf72);
    err72GAtpnlTme{no+1} = median(errTtf72);


    err72GAspnlRm{no+1} = mean(errRtfs72);
    err72GAspnlRme{no+1} = median(errRtfs72);
    err72GAspnlTm{no+1} = mean(errTtfs72);
    err72GAspnlTme{no+1} = median(errTtfs72);

    err73GApnlRm{no+1} = mean(errRf73);
    err73GApnlRme{no+1} = median(errRf73);
    err73GApnlTm{no+1} = mean(errTf73);
    err73GApnlTme{no+1} = median(errTf73);


    err73GAtpnlRm{no+1} = mean(errRtf73);
    err73GAtpnlRme{no+1} = median(errRtf73);
    err73GAtpnlTm{no+1} = mean(errTtf73);
    err73GAtpnlTme{no+1} = median(errTtf73);


    err73GAspnlRm{no+1} = mean(errRtfs73);
    err73GAspnlRme{no+1} = median(errRtfs73);
    err73GAspnlTm{no+1} = mean(errTtfs73);
    err73GAspnlTme{no+1} = median(errTtfs73);

    err74GApnlRm{no+1} = mean(errRf74);
    err74GApnlRme{no+1} = median(errRf74);
    err74GApnlTm{no+1} = mean(errTf74);
    err74GApnlTme{no+1} = median(errTf74);


    err74GAtpnlRm{no+1} = mean(errRtf74);
    err74GAtpnlRme{no+1} = median(errRtf74);
    err74GAtpnlTm{no+1} = mean(errTtf74);
    err74GAtpnlTme{no+1} = median(errTtf74);


    err74GAspnlRm{no+1} = mean(errRtfs74);
    err74GAspnlRme{no+1} = median(errRtfs74);
    err74GAspnlTm{no+1} = mean(errTtfs74);
    err74GAspnlTme{no+1} = median(errTtfs74);

    err75GApnlRm{no+1} = mean(errRf75);
    err75GApnlRme{no+1} = median(errRf75);
    err75GApnlTm{no+1} = mean(errTf75);
    err75GApnlTme{no+1} = median(errTf75);


    err75GAtpnlRm{no+1} = mean(errRtf75);
    err75GAtpnlRme{no+1} = median(errRtf75);
    err75GAtpnlTm{no+1} = mean(errTtf75);
    err75GAtpnlTme{no+1} = median(errTtf75);


    err75GAspnlRm{no+1} = mean(errRtfs75);
    err75GAspnlRme{no+1} = median(errRtfs75);
    err75GAspnlTm{no+1} = mean(errTtfs75);
    err75GAspnlTme{no+1} = median(errTtfs75);

    err76GApnlRm{no+1} = mean(errRf76);
    err76GApnlRme{no+1} = median(errRf76);
    err76GApnlTm{no+1} = mean(errTf76);
    err76GApnlTme{no+1} = median(errTf76);


    err76GAtpnlRm{no+1} = mean(errRtf76);
    err76GAtpnlRme{no+1} = median(errRtf76);
    err76GAtpnlTm{no+1} = mean(errTtf76);
    err76GAtpnlTme{no+1} = median(errTtf76);


    err76GAspnlRm{no+1} = mean(errRtfs76);
    err76GAspnlRme{no+1} = median(errRtfs76);
    err76GAspnlTm{no+1} = mean(errTtfs76);
    err76GAspnlTme{no+1} = median(errTtfs76);


    err77GApnlRm{no+1} = mean(errRf77);
    err77GApnlRme{no+1} = median(errRf77);
    err77GApnlTm{no+1} = mean(errTf77);
    err77GApnlTme{no+1} = median(errTf77);


    err77GAtpnlRm{no+1} = mean(errRtf77);
    err77GAtpnlRme{no+1} = median(errRtf77);
    err77GAtpnlTm{no+1} = mean(errTtf77);
    err77GAtpnlTme{no+1} = median(errTtf77);


    err77GAspnlRm{no+1} = mean(errRtfs77);
    err77GAspnlRme{no+1} = median(errRtfs77);
    err77GAspnlTm{no+1} = mean(errTtfs77);
    err77GAspnlTme{no+1} = median(errTtfs77);


    err77GAupnlRm{no+1} = mean(errRtfu77);
    err77GAupnlRme{no+1} = median(errRtfu77);
    err77GAupnlTm{no+1} = mean(errTtfu77);
    err77GAupnlTme{no+1} = median(errTtfu77);


    err77GAvpnlRm{no+1} = mean(errRtfv77);
    err77GAvpnlRme{no+1} = median(errRtfv77);
    err77GAvpnlTm{no+1} = mean(errTtfv77);
    err77GAvpnlTme{no+1} = median(errTtfv77);


    avgTime71{no+1} = mean(time71);
    avgTime72{no+1} = mean(time72);
    avgTime73{no+1} = mean(time73);
    avgTime74{no+1} = mean(time74);
    avgTime75{no+1} = mean(time75);
    avgTime76{no+1} = mean(time76);
end    









