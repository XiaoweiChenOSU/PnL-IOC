close all;
clc;
clear;

addpath('SRPnL');
addpath('others');
addpath('ASPnL');
addpath('RPnL');
addpath('LPnL-Bar-ENull');
addpath('Type61MultiGA');
addpath('Type62MultiGA');
addpath('Type63MultiGA');
addpath('Type64MultiGA');
addpath('Type65MultiGA');
addpath('Type66MultiGA');
addpath('Type67MultiGA');


%Type6 Room Camera Pose Estimation

for no = 0:10

    noise=no;
    rateOutlier=0;
    genN = 10;
    Data = LayoutDataGen(6,noise,rateOutlier,genN);
    for j = 1:genN
        pointS = Data{j}.pointS;
        pointE = Data{j}.pointE;
        pointSNoNoise = Data{j}.pointSt;
        pointENoNoise = Data{j}.pointEt;
        cH = Data{j}.Cc(2);
        Rtruth = Data{j}.R;
        Ttruth = Data{j}.T;
%  
%         pointS = [0,40,0,106.510194231831,318.691586659135;0,223,0,101.987426972125,0.358161547535985];
%         pointE = [256,37,0,545.402080668516,319.221741497087;256,219,0,550.476832818212,0.107389707789991];
%         pointSNoNoise = [0,40,0,106.510194231831,318.691586659135;0,223,0,101.987426972125,0.358161547535985];
%         pointENoNoise = [256,37,0,545.402080668516,319.221741497087;256,219,0,550.476832818212,0.107389707789991];
%         cH = 40.512278204906680;
%         Rtruth = [0.999937879662318,-0.010537403563706,-0.003633172520540;-0.010486489613869,-0.999850351853606,0.013758903800780;-0.003777611944949,-0.013719949867128,-0.999898741185145];
%         Ttruth = [-30.997788259482025;39.225467427139506;1.180264517300710e+02];
% 

        Lheight = 256;

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


        W1 = T1P1_wA(1:2,:)';
        W2 = T1P2_wA(1:2,:)';

%         [R3, T3, err] = SRPnL61(ip1A', ip2A', T1P1_wA', T1P2_wA', cH, Lheight);
%         [R4, T4, err4] = SRPnL61(ip1B', ip2B', T1P1_wB', T1P2_wB', cH, Lheight);
%         [R5, T5, err5] = SRPnL6(ip1', ip2', T1P1_w', T1P2_w', cH);


%         Rs{j} = R3;
% 
%         Ts{j} = T3;
% 
%         errOtt(j) = err;
% 
% 
%         errRt(j) = cal_rotation_err(R3,Rtruth);



%         errTt(j) = cal_translation_err(T3,Ttruth);




        p = [pn1 pn2];
        nLine = length(p);
        W = [W1 W2];
        p = [p; ones(1,nLine)];

        clearvars apoint aPointsN aPointsNt aPointsNs apointsIni apointsInis apointsInit apointsIniu apointsIniv aPointsNu aPointsNv T1P1_wA T1P2_wA T1P1_wB T1P2_wB T1P1_wC T1P2_wC T1P1_wD T1P2_wD Rf Tf Rtf Ttf Rsf Tsf 
        tic;
        count = 1; 

        T1P1_wA = [P1_w(1,:);P2_w(1,:);P1_w(1,:)];
        T1P2_wA = [P1_w(2,:);P2_w(2,:);P2_w(2,:)];

        T1P1_wB = [P1_w(1,:);P2_w(1,:);P1_w(2,:)];
        T1P2_wB = [P1_w(2,:);P2_w(2,:);P2_w(1,:)];
        
        [F1, G] = P2LMGA61(T1p1A, T1p2A, T1P1_wA, T1P2_wA, T1p1B, T1p2B, T1P1_wB, T1P2_wB, cH, A, Lheight);
        errAmin = Inf;
        errBmin = Inf;
        errmin = Inf;
        for i = 1:length(F1)
            if F1(i).ValObjective(1) > 5 || F1(i).ValObjective(2) > 5 || F1(i).ValObjective(3) > 100 || F1(i).ValObjective(4) > 100
    %         if F1(i).ValObjective(1) + F1(i).ValObjective(2) > 2
                 continue
            end
%             if F1(i).ValObjective(1) < errAmin 
%                 apoint(1) = F1(i).Val(1);
%                 apoint(4) = F1(i).Val(4);
%                 RAt = F1(i).RA;
%                 TAt = F1(i).TA;
%                 errAmin = F1(i).ValObjective(1);
%             end
%             if F1(i).ValObjective(2) < errBmin 
%                 apoint(2) = F1(i).Val(2);
%                 apoint(3) = F1(i).Val(3);
%                 RBt = F1(i).RB;
%                 TBt = F1(i).TB;
%                 errBmin = F1(i).ValObjective(2);
%             end 
            if sum(F1(i).ValObjective(1)+F1(i).ValObjective(2)) < errmin 
                apoint(1) = F1(i).Val(1);
                apoint(2) = F1(i).Val(2);
                apoint(3) = F1(i).Val(3);
                apoint(4) = F1(i).Val(4);
                RAt = F1(i).RA;
                TAt = F1(i).TA;
                RBt = F1(i).RB;
                TBt = F1(i).TB;
                errAmin = F1(i).ValObjective(1);
                errBmin = F1(i).ValObjective(2);
                errmin = sum(F1(i).ValObjective(1)+F1(i).ValObjective(2));
            end
        end  
%         toc;
%         time61(j) = toc; 

        if errAmin < errBmin

            [EE,GG,CC] = RefineEquA61A(p, W);
            apointsIni = [apoint(1) apoint(4)]';
            solutionO = [Cayley(RAt);apointsIni]; 
            [solution, obj_cur] = RefineGaussNewtonRandApT61A(solutionO, p, W, EE, GG, CC, Lheight);


            if size(solution) ~= 11
                aPointsN = solution(4:5);
            else
                aPointsN = solution(10:11);
            end
    %         aPointsN = [apoint(1) apoint(4)]';

            T1P1_wA(1,2) =  aPointsN(1);
            T1P1_wA(2,2) =  aPointsN(2)-50;
            T1P1_wA(3,2) =  aPointsN(1);
            T1P2_wA(1,2) =  aPointsN(1)+50;
            T1P2_wA(2,2) =  aPointsN(2);
            T1P2_wA(3,2) =  aPointsN(2);
            [Rf, Tf, errf] = SRPnL61(ip1A', ip2A', T1P1_wA', T1P2_wA', cH, Lheight);
        else
            [EE,GG,CC] = RefineEquA61B(p, W);
            apointsIni = [apoint(2) apoint(3)]';
            solutionO = [Cayley(RBt);apointsIni]; 
            [solution, obj_cur] = RefineGaussNewtonRandApT61B(solutionO, p, W, EE, GG, CC, Lheight);

            if size(solution) ~= 11
                aPointsN = solution(4:5);
            else
                aPointsN = solution(10:11);
            end

    %         aPointsN = [apoint(2) apoint(3)]';

            T1P1_wB(1,2) =  aPointsN(2)-50;
            T1P1_wB(2,2) =  aPointsN(1);
            T1P1_wB(3,2) =  aPointsN(2);
            T1P2_wB(1,2) =  aPointsN(2);
            T1P2_wB(2,2) =  aPointsN(1)+50;
            T1P2_wB(3,2) =  aPointsN(1);
            [Rf, Tf, errf] = SRPnL61(ip1B', ip2B', T1P1_wB', T1P2_wB', cH, Lheight); 
        end


        R61{j} = Rf;
        T61{j} = Tf;
        error61(j) = obj_cur; 
        errRf61(j) = cal_rotation_err(Rf,Rtruth);
        errTf61(j) = cal_translation_err(Tf,Ttruth);
        toc;
        time61(j) = toc; 


        [EEt,GGt,CCt] = RefineEquA61A(p, W);
        apointsInit = [apoint(1) apoint(4)]';
        solutionOt = [Cayley(RAt);apointsInit]; 
        [solutiont, obj_cur] = RefineGaussNewtonRandApT61A(solutionOt, p, W, EEt, GGt, CCt, Lheight);


        if size(solutiont) ~= 11
            aPointsNt = solutiont(4:5);
        else
            aPointsNt = solutiont(10:11);
        end

    %     aPointsNt = [apoint(1) apoint(4)]';

        T1P1_wA(1,2) =  aPointsNt(1);
        T1P1_wA(2,2) =  aPointsNt(2)-50;
        T1P1_wA(3,2) =  aPointsNt(1);
        T1P2_wA(1,2) =  aPointsNt(1)+50;
        T1P2_wA(2,2) =  aPointsNt(2);
        T1P2_wA(3,2) =  aPointsNt(2);
        [Rtf, Ttf, errftf] = SRPnL61(ip1A', ip2A', T1P1_wA', T1P2_wA', cH, Lheight);

        Rth61{j} = Rtf;
        Tth61{j} = Ttf;
        errortf61(j) = obj_cur; 
        errRtf61(j) = cal_rotation_err(Rtf,Rtruth);
        errTtf61(j) = cal_translation_err(Ttf,Ttruth);



        [EEs,GGs,CCs] = RefineEquA61B(p, W);
        apointsInis = [apoint(2) apoint(3)]';
        solutionOs = [Cayley(RBt);apointsInis]; 
        [solutions, obj_cur] = RefineGaussNewtonRandApT61B(solutionOs, p, W, EEs, GGs, CCs, Lheight);

        if size(solutions) ~= 11
            aPointsNs = solutions(4:5);
        else
            aPointsNs = solutions(10:11);
        end

    %     aPointsNs = [apoint(2) apoint(3)]';

        T1P1_wB(1,2) =  aPointsNs(2)-50;
        T1P1_wB(2,2) =  aPointsNs(1);
        T1P1_wB(3,2) =  aPointsNs(2);
        T1P2_wB(1,2) =  aPointsNs(2);
        T1P2_wB(2,2) =  aPointsNs(1)+50;
        T1P2_wB(3,2) =  aPointsNs(1);
        [Rfs, Tfs, errfs] = SRPnL61(ip1B', ip2B', T1P1_wB', T1P2_wB', cH, Lheight); 


        Rts61{j} = Rfs;
        Tts61{j} = Tfs;
        errortfs61(j) = obj_cur; 
        errRtfs61(j) = cal_rotation_err(Rfs,Rtruth);
        errTtfs61(j) = cal_translation_err(Tfs,Ttruth);

%         %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        clearvars apointa apointb aPointsN aPointsNt aPointsNs apointsIni apointsInis apointsInit apointsIniu apointsIniv aPointsNu aPointsNv T1P1_wA T1P2_wA T1P1_wB T1P2_wB T1P1_wC T1P2_wC T1P1_wD T1P2_wD Rf Tf Rtf Ttf Rfs Tfs
        tic;

        T1P1_wA = [P1_w(1,:);P2_w(1,:);P1_w(1,:)];
        T1P2_wA = [P1_w(2,:);P2_w(2,:);P2_w(2,:)];


        T1P1_wC = [P1_w(1,:);P2_w(1,:);P1_w(1,:)];
        T1P2_wC = [P1_w(2,:);P2_w(2,:);P2_w(1,:)];

        [F1, G] = P2LMGA62(T1p1A, T1p2A, T1P1_wA, T1P2_wA, T1p1C, T1p2C, T1P1_wC, T1P2_wC, cH, A, Lheight);
        errAmin = Inf;
        errBmin = Inf;
        for i = 1:length(F1)
            if F1(i).ValObjective(1) > 5 || F1(i).ValObjective(2) > 5 || F1(i).ValObjective(3) > 100 || F1(i).ValObjective(4) > 100
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

            [EE,GG,CC] = RefineEquA62A(p, W);
            apointsIni = [apointa(1) apointa(3)]';
            solutionO = [Cayley(RAt);apointsIni]; 
            [solution, obj_cur] = RefineGaussNewtonRandApT62A(solutionO, p, W, EE, GG, CC, Lheight);


            if size(solution) ~= 11
                aPointsN = solution(4:5);
            else
                aPointsN = solution(10:11);
            end

    %         aPointsN = [apoint(1) apoint(3)]';

            T1P1_wA(1,2) =  aPointsN(1);
            T1P1_wA(2,2) =  aPointsN(2)-50;
            T1P1_wA(3,2) =  aPointsN(1);
            T1P2_wA(1,2) =  aPointsN(1)+50;
            T1P2_wA(2,2) =  aPointsN(2);
            T1P2_wA(3,2) =  aPointsN(2);
            [Rf, Tf, errf] = SRPnL61(ip1A', ip2A', T1P1_wA', T1P2_wA', cH, Lheight);
        else
            [EE,GG,CC] = RefineEquA62B(p, W);
            apointsIni = [apointb(1) apointb(2)]';
            solutionO = [Cayley(RBt);apointsIni]; 
            [solution, obj_cur] = RefineGaussNewtonRandApT62B(solutionO, p, W, EE, GG, CC, Lheight);

            if size(solution) ~= 11
                aPointsN = solution(4:5);
            else
                aPointsN = solution(10:11);
            end

    %         aPointsN = [apoint(1) apoint(2)]';

            T1P1_wC(1,2) =  aPointsN(1);
            T1P1_wC(2,2) =  aPointsN(2);
            T1P1_wC(3,2) =  aPointsN(1);
            T1P2_wC(1,2) =  aPointsN(1)+50;
            T1P2_wC(2,2) =  aPointsN(2)+50;
            T1P2_wC(3,2) =  aPointsN(2);
            [Rf, Tf, errf] = SRPnL61(ip1C', ip2C', T1P1_wC', T1P2_wC', cH, Lheight); 
        end
  

        R62{j} = Rf;
        T62{j} = Tf;
        error62(j) = obj_cur; 
        errRf62(j) = cal_rotation_err(Rf,Rtruth);
        errTf62(j) = cal_translation_err(Tf,Ttruth);
        toc;
        time62(j) = toc; 


        [EEt,GGt,CCt] = RefineEquA62A(p, W);
        apointsInit = [apointa(1) apointa(3)]';
        solutionOt = [Cayley(RAt);apointsInit]; 
        [solutiont, obj_cur] = RefineGaussNewtonRandApT62A(solutionOt, p, W, EEt, GGt, CCt, Lheight);


        if size(solutiont) ~= 11
            aPointsNt = solutiont(4:5);
        else
            aPointsNt = solutiont(10:11);
        end

    %     aPointsNt = [apoint(1) apoint(3)]';


        T1P1_wA(1,2) =  aPointsNt(1);
        T1P1_wA(2,2) =  aPointsNt(2)-50;
        T1P1_wA(3,2) =  aPointsNt(1);
        T1P2_wA(1,2) =  aPointsNt(1)+50;
        T1P2_wA(2,2) =  aPointsNt(2);
        T1P2_wA(3,2) =  aPointsNt(2);
        [Rtf, Ttf, errftf] = SRPnL61(ip1A', ip2A', T1P1_wA', T1P2_wA', cH, Lheight);
        
        Rth62{j} = Rtf;
        Tth62{j} = Ttf;
        errortf62(j) = obj_cur; 
        errRtf62(j) = cal_rotation_err(Rtf,Rtruth);
        errTtf62(j) = cal_translation_err(Ttf,Ttruth);



        [EEs,GGs,CCs] = RefineEquA62B(p, W);
        apointsInis = [apointb(1) apointb(2)]';
        solutionOs = [Cayley(RBt);apointsInis]; 
        [solutions, obj_cur] = RefineGaussNewtonRandApT62B(solutionOs, p, W, EEs, GGs, CCs, Lheight);

        if size(solutions) ~= 11
            aPointsNs = solutions(4:5);
        else
            aPointsNs = solutions(10:11);
        end

    %     aPointsNs = [apoint(1) apoint(2)]';

        T1P1_wC(1,2) =  aPointsNs(1);
        T1P1_wC(2,2) =  aPointsNs(2);
        T1P1_wC(3,2) =  aPointsNs(1);
        T1P2_wC(1,2) =  aPointsNs(1)+50;
        T1P2_wC(2,2) =  aPointsNs(2)+50;
        T1P2_wC(3,2) =  aPointsNs(2);
        [Rfs, Tfs, errfs] = SRPnL61(ip1C', ip2C', T1P1_wC', T1P2_wC', cH, Lheight); 

        Rts62{j} = Rfs;
        Tts62{j} = Tfs;
        errortfs62(j) = obj_cur; 
        errRtfs62(j) = cal_rotation_err(Rfs,Rtruth);
        errTtfs62(j) = cal_translation_err(Tfs,Ttruth);


        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        clearvars  apointb aPointsN aPointsNt aPointsNs apointsIni apointsInis apointsInit apointsIniu apointsIniv aPointsNu aPointsNv T1P1_wA T1P2_wA T1P1_wB T1P2_wB T1P1_wC T1P2_wC T1P1_wD T1P2_wD Rf Tf Rtf Ttf Rfs Tfs
        tic;
        T1P1_wA = [P1_w(1,:);P2_w(1,:);P1_w(1,:)];
        T1P2_wA = [P1_w(2,:);P2_w(2,:);P2_w(2,:)];


        T1P1_wD = [P1_w(1,:);P2_w(1,:);P1_w(2,:)];
        T1P2_wD = [P1_w(2,:);P2_w(2,:);P2_w(2,:)];

        [F1, G] = P2LMGA63(T1p1A, T1p2A, T1P1_wA, T1P2_wA, T1p1D, T1p2D, T1P1_wD, T1P2_wD, cH, A, Lheight);
        errAmin = Inf;
        errBmin = Inf;
        for i = 1:length(F1)
            if F1(i).ValObjective(1) > 5 || F1(i).ValObjective(2) > 5 || F1(i).ValObjective(3) > 100 || F1(i).ValObjective(4) > 100
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

            [EE,GG,CC] = RefineEquA63A(p, W);
            apointsIni = [apointa(1) apointa(3)]';
            solutionO = [Cayley(RAt);apointsIni]; 
            [solution, obj_cur] = RefineGaussNewtonRandApT63A(solutionO, p, W, EE, GG, CC, Lheight);


            if size(solution) ~= 11
                aPointsN = solution(4:5);
            else
                aPointsN = solution(10:11);
            end

    %         aPointsN = [apoint(1) apoint(3)]';

            T1P1_wA(1,2) =  aPointsN(1);
            T1P1_wA(2,2) =  aPointsN(2)-50;
            T1P1_wA(3,2) =  aPointsN(1);
            T1P2_wA(1,2) =  aPointsN(1)+50;
            T1P2_wA(2,2) =  aPointsN(2);
            T1P2_wA(3,2) =  aPointsN(2);
            [Rf, Tf, errf] = SRPnL61(ip1A', ip2A', T1P1_wA', T1P2_wA', cH, Lheight);
        else
            [EE,GG,CC] = RefineEquA63B(p, W);
            apointsIni = [apointb(2) apointb(3)]';
            solutionO = [Cayley(RBt);apointsIni]; 
            [solution, obj_cur] = RefineGaussNewtonRandApT63B(solutionO, p, W, EE, GG, CC, Lheight);

            if size(solution) ~= 11
                aPointsN = solution(4:5);
            else
                aPointsN = solution(10:11);
            end

    %         aPointsN = [apoint(2) apoint(3)]';

            T1P1_wD(1,2) =  aPointsN(1)-50;
            T1P1_wD(2,2) =  aPointsN(2)-50;
            T1P1_wD(3,2) =  aPointsN(1);
            T1P2_wD(1,2) =  aPointsN(1);
            T1P2_wD(2,2) =  aPointsN(2);
            T1P2_wD(3,2) =  aPointsN(2);
            [Rf, Tf, errf] = SRPnL61(ip1D', ip2D', T1P1_wD', T1P2_wD', cH, Lheight); 
        end

        R63{j} = Rf;
        T63{j} = Tf;
        error63(j) = obj_cur; 
        errRf63(j) = cal_rotation_err(Rf,Rtruth);
        errTf63(j) = cal_translation_err(Tf,Ttruth);
        toc;
        time63(j) = toc; 


        [EEt,GGt,CCt] = RefineEquA63A(p, W);
        apointsInit = [apointa(1) apointa(3)]';
        solutionOt = [Cayley(RAt);apointsInit]; 
        [solutiont, obj_cur] = RefineGaussNewtonRandApT63A(solutionOt, p, W, EEt, GGt, CCt, Lheight);


        if size(solutiont) ~= 11
            aPointsNt = solutiont(4:5);
        else
            aPointsNt = solutiont(10:11);
        end

    %     aPointsNt = [apoint(1) apoint(3)]';

        T1P1_wA(1,2) =  aPointsNt(1);
        T1P1_wA(2,2) =  aPointsNt(2)-50;
        T1P1_wA(3,2) =  aPointsNt(1);
        T1P2_wA(1,2) =  aPointsNt(1)+50;
        T1P2_wA(2,2) =  aPointsNt(2);
        T1P2_wA(3,2) =  aPointsNt(2);
        [Rtf, Ttf, errftf] = SRPnL61(ip1A', ip2A', T1P1_wA', T1P2_wA', cH, Lheight);            
        
        Rth63{j} = Rtf;
        Tth63{j} = Ttf;
        errortf63(j) = obj_cur; 
        errRtf63(j) = cal_rotation_err(Rtf,Rtruth);
        errTtf63(j) = cal_translation_err(Ttf,Ttruth);



        [EEs,GGs,CCs] = RefineEquA63B(p, W);
        apointsInis = [apointb(2) apointb(3)]';
        solutionOs = [Cayley(RBt);apointsInis]; 
        [solutions, obj_cur] = RefineGaussNewtonRandApT63B(solutionOs, p, W, EEs, GGs, CCs, Lheight);

        if size(solutions) ~= 11
            aPointsNs = solutions(4:5);
        else
            aPointsNs = solutions(10:11);
        end

    %     aPointsNs = [apoint(2) apoint(3)]';

        T1P1_wD(1,2) =  aPointsNs(1)-50;
        T1P1_wD(2,2) =  aPointsNs(2)-50;
        T1P1_wD(3,2) =  aPointsNs(1);
        T1P2_wD(1,2) =  aPointsNs(1);
        T1P2_wD(2,2) =  aPointsNs(2);
        T1P2_wD(3,2) =  aPointsNs(2);
        [Rfs, Tfs, errfs] = SRPnL61(ip1D', ip2D', T1P1_wD', T1P2_wD', cH, Lheight);  

        Rts63{j} = Rfs;
        Tts63{j} = Tfs;
        errortfs63(j) = obj_cur; 
        errRtfs63(j) = cal_rotation_err(Rfs,Rtruth);
        errTtfs63(j) = cal_translation_err(Tfs,Ttruth);


         %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        clearvars apointa apointb aPointsN aPointsNt aPointsNs apointsIni apointsInis apointsInit apointsIniu apointsIniv aPointsNu aPointsNv T1P1_wA T1P2_wA T1P1_wB T1P2_wB T1P1_wC T1P2_wC T1P1_wD T1P2_wD Rf Tf Rtf Ttf Rfs Tfs
        tic; 
        T1P1_wB = [P1_w(1,:);P2_w(1,:);P1_w(2,:)];
        T1P2_wB = [P1_w(2,:);P2_w(2,:);P2_w(1,:)];

        T1P1_wC = [P1_w(1,:);P2_w(1,:);P1_w(1,:)];
        T1P2_wC = [P1_w(2,:);P2_w(2,:);P2_w(1,:)]; 

        [F1, G] = P2LMGA64(T1p1B, T1p2B, T1P1_wB, T1P2_wB, T1p1C, T1p2C, T1P1_wC, T1P2_wC, cH, A, Lheight);
        errAmin = Inf;
        errBmin = Inf;
        for i = 1:length(F1)
    %         if F1(i).ValObjective(1) > 2 || F1(i).ValObjective(2) > 2 || F1(i).ValObjective(3) > 2 || F1(i).ValObjective(4) > 2
            % + F1(i).ValObjective(3) + F1(i).ValObjective(4)
            if F1(i).ValObjective(1) > 5 || F1(i).ValObjective(2) > 5 || F1(i).ValObjective(3) > 100 || F1(i).ValObjective(4) > 100
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

            [EE,GG,CC] = RefineEquA64A(p, W);
            apointsIni = [apointa(2) apointa(3)]';
            solutionO = [Cayley(RAt);apointsIni]; 
            [solution, obj_cur] = RefineGaussNewtonRandApT64A(solutionO, p, W, EE, GG, CC, Lheight);


            if size(solution) ~= 11
                aPointsN = solution(4:5);
            else
                aPointsN = solution(10:11);
            end

    %         aPointsN = [apoint(2) apoint(3)]';


            T1P1_wB(1,2) =  aPointsN(2)-50;
            T1P1_wB(2,2) =  aPointsN(1);
            T1P1_wB(3,2) =  aPointsN(2);
            T1P2_wB(1,2) =  aPointsN(2);
            T1P2_wB(2,2) =  aPointsN(1)+50;
            T1P2_wB(3,2) =  aPointsN(1);
            [Rf, Tf, errf] = SRPnL61(ip1B', ip2B', T1P1_wB', T1P2_wB', cH, Lheight);
        else
            [EE,GG,CC] = RefineEquA64B(p, W);
            apointsIni = [apointb(1) apointb(2)]';
            solutionO = [Cayley(RBt);apointsIni]; 
            [solution, obj_cur] = RefineGaussNewtonRandApT64B(solutionO, p, W, EE, GG, CC, Lheight);

            if size(solution) ~= 11
                aPointsN = solution(4:5);
            else
                aPointsN = solution(10:11);
            end

    %         aPointsN = [apoint(1) apoint(2)]';

            T1P1_wC(1,2) =  aPointsN(1);
            T1P1_wC(2,2) =  aPointsN(2);
            T1P1_wC(3,2) =  aPointsN(1);
            T1P2_wC(1,2) =  aPointsN(1)+50;
            T1P2_wC(2,2) =  aPointsN(2)+50;
            T1P2_wC(3,2) =  aPointsN(2);
            [Rf, Tf, errf] = SRPnL61(ip1C', ip2C', T1P1_wC', T1P2_wC', cH, Lheight); 
        end

        R64{j} = Rf;
        T64{j} = Tf;
        error64(j) = obj_cur; 
        errRf64(j) = cal_rotation_err(Rf,Rtruth);
        errTf64(j) = cal_translation_err(Tf,Ttruth);
        toc;
        time64(j) = toc; 


        [EEt,GGt,CCt] = RefineEquA64A(p, W);
        apointsInit = [apointa(2) apointa(3)]';
        solutionOt = [Cayley(RAt);apointsInit]; 
        [solutiont, obj_cur] = RefineGaussNewtonRandApT64A(solutionOt, p, W, EEt, GGt, CCt, Lheight);


        if size(solutiont) ~= 11
            aPointsNt = solutiont(4:5);
        else
            aPointsNt = solutiont(10:11);
        end

    %     aPointsNt = [apoint(2) apoint(3)]';

        T1P1_wB(1,2) =  aPointsNt(2)-50;
        T1P1_wB(2,2) =  aPointsNt(1);
        T1P1_wB(3,2) =  aPointsNt(2);
        T1P2_wB(1,2) =  aPointsNt(2);
        T1P2_wB(2,2) =  aPointsNt(1)+50;
        T1P2_wB(3,2) =  aPointsNt(1);
        [Rtf, Ttf, errftf] = SRPnL61(ip1B', ip2B', T1P1_wB', T1P2_wB', cH, Lheight);
        
        
        
        Rth64{j} = Rtf;
        Tth64{j} = Ttf;
        errortf64(j) = obj_cur; 
        errRtf64(j) = cal_rotation_err(Rtf,Rtruth);
        errTtf64(j) = cal_translation_err(Ttf,Ttruth);



        [EEs,GGs,CCs] = RefineEquA64B(p, W);
        apointsInis = [apointb(1) apointb(2)]';
        solutionOs = [Cayley(RBt);apointsInis]; 
        [solutions, obj_cur] = RefineGaussNewtonRandApT64B(solutionOs, p, W, EEs, GGs, CCs, Lheight);

        if size(solutions) ~= 11
            aPointsNs = solutions(4:5);
        else
            aPointsNs = solutions(10:11);
        end

    %     aPointsNs = [apoint(1) apoint(2)]';

        T1P1_wC(1,2) =  aPointsNs(1);
        T1P1_wC(2,2) =  aPointsNs(2);
        T1P1_wC(3,2) =  aPointsNs(1);
        T1P2_wC(1,2) =  aPointsNs(1)+50;
        T1P2_wC(2,2) =  aPointsNs(2)+50;
        T1P2_wC(3,2) =  aPointsNs(2);
        [Rfs, Tfs, errfs] = SRPnL61(ip1C', ip2C', T1P1_wC', T1P2_wC', cH, Lheight); 
        
        Rts64{j} = Rfs;
        Tts64{j} = Tfs;
        errortfs64(j) = obj_cur; 
        errRtfs64(j) = cal_rotation_err(Rfs,Rtruth);
        errTtfs64(j) = cal_translation_err(Tfs,Ttruth);

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        clearvars apointa apointb aPointsN aPointsNt aPointsNs apointsIni apointsInis apointsInit apointsIniu apointsIniv aPointsNu aPointsNv T1P1_wA T1P2_wA T1P1_wB T1P2_wB T1P1_wC T1P2_wC T1P1_wD T1P2_wD Rf Tf Rtf Ttf Rfs Tfs
        tic;

        T1P1_wB = [P1_w(1,:);P2_w(1,:);P1_w(2,:)];
        T1P2_wB = [P1_w(2,:);P2_w(2,:);P2_w(1,:)];


        T1P1_wD = [P1_w(1,:);P2_w(1,:);P1_w(2,:)];
        T1P2_wD = [P1_w(2,:);P2_w(2,:);P2_w(2,:)];

        [F1, G] = P2LMGA65(T1p1B, T1p2B, T1P1_wB, T1P2_wB, T1p1D, T1p2D, T1P1_wD, T1P2_wD, cH, A, Lheight);
        errAmin = Inf;
        errBmin = Inf;
        for i = 1:length(F1)
    %         if F1(i).ValObjective(1) > 0.5 || F1(i).ValObjective(2) > 0.5 || F1(i).ValObjective(3) > 0.5 || F1(i).ValObjective(4) > 0.5
            if F1(i).ValObjective(1) > 5 || F1(i).ValObjective(2) > 5 || F1(i).ValObjective(3) > 100 || F1(i).ValObjective(4) > 100
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

            [EE,GG,CC] = RefineEquA65A(p, W);
            apointsIni = [apointa(1) apointa(2)]';
            solutionO = [Cayley(RAt);apointsIni]; 
            [solution, obj_cur] = RefineGaussNewtonRandApT65A(solutionO, p, W, EE, GG, CC, Lheight);


            if size(solution) ~= 11
                aPointsN = solution(4:5);
            else
                aPointsN = solution(10:11);
            end

    %         aPointsN = [apoint(1) apoint(2)]';

            T1P1_wB(1,2) =  aPointsN(2)-50;
            T1P1_wB(2,2) =  aPointsN(1);
            T1P1_wB(3,2) =  aPointsN(2);
            T1P2_wB(1,2) =  aPointsN(2);
            T1P2_wB(2,2) =  aPointsN(1)+50;
            T1P2_wB(3,2) =  aPointsN(1);
            [Rf, Tf, errf] = SRPnL61(ip1B', ip2B', T1P1_wB', T1P2_wB', cH, Lheight);
        else
            [EE,GG,CC] = RefineEquA65B(p, W);
            apointsIni = [apointb(2) apointb(3)]';
            solutionO = [Cayley(RBt);apointsIni]; 
            [solution, obj_cur] = RefineGaussNewtonRandApT65B(solutionO, p, W, EE, GG, CC, Lheight);

            if size(solution) ~= 11
                aPointsN = solution(4:5);
            else
                aPointsN = solution(10:11);
            end

    %         aPointsN = [apoint(2) apoint(3)]';

            T1P1_wD(1,2) =  aPointsN(1)-50;
            T1P1_wD(2,2) =  aPointsN(2)-50;
            T1P1_wD(3,2) =  aPointsN(1);
            T1P2_wD(1,2) =  aPointsN(1);
            T1P2_wD(2,2) =  aPointsN(2);
            T1P2_wD(3,2) =  aPointsN(2);
            [Rf, Tf, errf] = SRPnL61(ip1D', ip2D', T1P1_wD', T1P2_wD', cH, Lheight); 
        end

        
        R65{j} = Rf;
        T65{j} = Tf;
        error65(j) = obj_cur; 
        errRf65(j) = cal_rotation_err(Rf,Rtruth);
        errTf65(j) = cal_translation_err(Tf,Ttruth);
        toc;
        time65(j) = toc; 


        [EEt,GGt,CCt] = RefineEquA65A(p, W);
        apointsInit = [apointa(1) apointa(2)]';
        solutionOt = [Cayley(RAt);apointsInit]; 
        [solutiont, obj_cur] = RefineGaussNewtonRandApT65A(solutionOt, p, W, EEt, GGt, CCt, Lheight);


        if size(solutiont) ~= 11
            aPointsNt = solutiont(4:5);
        else
            aPointsNt = solutiont(10:11);
        end

    %     aPointsNt = [apoint(1) apoint(2)]';

        T1P1_wB(1,2) =  aPointsNt(2)-50;
        T1P1_wB(2,2) =  aPointsNt(1);
        T1P1_wB(3,2) =  aPointsNt(2);
        T1P2_wB(1,2) =  aPointsNt(2);
        T1P2_wB(2,2) =  aPointsNt(1)+50;
        T1P2_wB(3,2) =  aPointsNt(1);
        [Rtf, Ttf, errftf] = SRPnL61(ip1B', ip2B', T1P1_wB', T1P2_wB', cH, Lheight);        
        
        Rth65{j} = Rtf;
        Tth65{j} = Ttf;
        errortf65(j) = obj_cur; 
        errRtf65(j) = cal_rotation_err(Rtf,Rtruth);
        errTtf65(j) = cal_translation_err(Ttf,Ttruth);



        [EEs,GGs,CCs] = RefineEquA65B(p, W);
        apointsInis = [apointb(2) apointb(3)]';
        solutionOs = [Cayley(RBt);apointsInis]; 
        [solutions, obj_cur] = RefineGaussNewtonRandApT65B(solutionOs, p, W, EEs, GGs, CCs, Lheight);

        if size(solutions) ~= 11
            aPointsNs = solutions(4:5);
        else
            aPointsNs = solutions(10:11);
        end

    %     aPointsNs = [apoint(2) apoint(3)]';

        T1P1_wD(1,2) =  aPointsNs(1)-50;
        T1P1_wD(2,2) =  aPointsNs(2)-50;
        T1P1_wD(3,2) =  aPointsNs(1);
        T1P2_wD(1,2) =  aPointsNs(1);
        T1P2_wD(2,2) =  aPointsNs(2);
        T1P2_wD(3,2) =  aPointsNs(2);
        [Rfs, Tfs, errfs] = SRPnL61(ip1D', ip2D', T1P1_wD', T1P2_wD', cH, Lheight); 

        Rts65{j} = Rfs;
        Tts65{j} = Tfs;
        errortfs65(j) = obj_cur; 
        errRtfs65(j) = cal_rotation_err(Rfs,Rtruth);
        errTtfs65(j) = cal_translation_err(Tfs,Ttruth);    

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        clearvars apoint aPointsN aPointsNt aPointsNs apointsIni apointsInis apointsInit apointsIniu apointsIniv aPointsNu aPointsNv T1P1_wA T1P2_wA T1P1_wB T1P2_wB T1P1_wC T1P2_wC T1P1_wD T1P2_wD Rf Tf Rtf Ttf Rfs Tfs F1
        tic;
        T1P1_wC = [P1_w(1,:);P2_w(1,:);P1_w(1,:)];
        T1P2_wC = [P1_w(2,:);P2_w(2,:);P2_w(1,:)]; 

        T1P1_wD = [P1_w(1,:);P2_w(1,:);P1_w(2,:)];
        T1P2_wD = [P1_w(2,:);P2_w(2,:);P2_w(2,:)];

        [F1, G] = P2LMGA66(T1p1C, T1p2C, T1P1_wC, T1P2_wC, T1p1D, T1p2D, T1P1_wD, T1P2_wD, cH, A, Lheight);
        errAmin = Inf;
        errBmin = Inf;
        for i = 1:length(F1)
    %         if F1(i).ValObjective(1) > 0.5 || F1(i).ValObjective(2) > 0.5 || F1(i).ValObjective(3) > 0.5 || F1(i).ValObjective(4) > 0.5
            if F1(i).ValObjective(1) > 5 || F1(i).ValObjective(2) > 5 || F1(i).ValObjective(3) > 100 || F1(i).ValObjective(4) > 100
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


%         errOriRf66(j) = cal_rotation_err(Ri,Rtruth);
%         errOriTf66(j) = cal_translation_err(Ti,Ttruth);


        if errAmin < errBmin

            [EE,GG,CC] = RefineEquA66A(p, W);
            apointsIni = [apoint(1) apoint(2)]';
            solutionO = [Cayley(RAt);apointsIni]; 
            [solution, obj_cur] = RefineGaussNewtonRandApT66A(solutionO, p, W, EE, GG, CC, Lheight);


            if size(solution) ~= 11
                aPointsN = solution(4:5);
            else
                aPointsN = solution(10:11);
            end

    %         aPointsN = [apoint(1) apoint(2)]';

            T1P1_wC(1,2) =  aPointsN(1);
            T1P1_wC(2,2) =  aPointsN(2);
            T1P1_wC(3,2) =  aPointsN(1);
            T1P2_wC(1,2) =  aPointsN(1)+50;
            T1P2_wC(2,2) =  aPointsN(2)+50;
            T1P2_wC(3,2) =  aPointsN(2);
            [Rf, Tf, errf] = SRPnL61(ip1C', ip2C', T1P1_wC', T1P2_wC', cH, Lheight);
        else
            [EE,GG,CC] = RefineEquA66B(p, W);
            apointsIni = [apoint(3) apoint(4)]';
            solutionO = [Cayley(RBt);apointsIni]; 
            [solution, obj_cur] = RefineGaussNewtonRandApT66B(solutionO, p, W, EE, GG, CC, Lheight);

            if size(solution) ~= 11
                aPointsN = solution(4:5);
            else
                aPointsN = solution(10:11);
            end

    %         aPointsN = [apoint(3) apoint(4)]';

            T1P1_wD(1,2) =  aPointsN(1)-50;
            T1P1_wD(2,2) =  aPointsN(2)-50;
            T1P1_wD(3,2) =  aPointsN(1);
            T1P2_wD(1,2) =  aPointsN(1);
            T1P2_wD(2,2) =  aPointsN(2);
            T1P2_wD(3,2) =  aPointsN(2);
            [Rf, Tf, errf] = SRPnL61(ip1D', ip2D', T1P1_wD', T1P2_wD', cH, Lheight); 
        end        

        R66{j} = Rf;
        T66{j} = Tf;
        error66(j) = obj_cur; 
        errRf66(j) = cal_rotation_err(Rf,Rtruth);
        errTf66(j) = cal_translation_err(Tf,Ttruth);
        toc;
        time66(j) = toc; 


        [EEt,GGt,CCt] = RefineEquA66A(p, W);
        apointsInit = [apoint(1) apoint(2)]';
        solutionOt = [Cayley(RAt);apointsInit]; 
        [solutiont, obj_cur] = RefineGaussNewtonRandApT66A(solutionOt, p, W, EEt, GGt, CCt, Lheight);


        if size(solutiont) ~= 11
            aPointsNt = solutiont(4:5);
        else
            aPointsNt = solutiont(10:11);
        end

    %     aPointsNt = [apoint(1) apoint(2)]';

        T1P1_wC(1,2) =  aPointsNt(1);
        T1P1_wC(2,2) =  aPointsNt(2);
        T1P1_wC(3,2) =  aPointsNt(1);
        T1P2_wC(1,2) =  aPointsNt(1)+50;
        T1P2_wC(2,2) =  aPointsNt(2)+50;
        T1P2_wC(3,2) =  aPointsNt(2);
        [Rtf, Ttf, errftf] = SRPnL61(ip1C', ip2C', T1P1_wC', T1P2_wC', cH, Lheight);

        
        
        Rth66{j} = Rtf;
        Tth66{j} = Ttf;
        errortf66(j) = obj_cur; 
        errRtf66(j) = cal_rotation_err(Rtf,Rtruth);
        errTtf66(j) = cal_translation_err(Ttf,Ttruth);



        [EEs,GGs,CCs] = RefineEquA66B(p, W);
        apointsInis = [apoint(3) apoint(4)]';
        solutionOs = [Cayley(RBt);apointsInis]; 
        [solutions, obj_cur] = RefineGaussNewtonRandApT66B(solutionOs, p, W, EEs, GGs, CCs, Lheight);

        if size(solutions) ~= 11
            aPointsNs = solutions(4:5);
        else
            aPointsNs = solutions(10:11);
        end

    %     aPointsNs = [apoint(3) apoint(4)]';

        T1P1_wD(1,2) =  aPointsNs(1)-50;
        T1P1_wD(2,2) =  aPointsNs(2)-50;
        T1P1_wD(3,2) =  aPointsNs(1);
        T1P2_wD(1,2) =  aPointsNs(1);
        T1P2_wD(2,2) =  aPointsNs(2);
        T1P2_wD(3,2) =  aPointsNs(2);
        [Rfs, Tfs, errfs] = SRPnL61(ip1D', ip2D', T1P1_wD', T1P2_wD', cH, Lheight); 


        Rts66{j} = Rfs;
        Tts66{j} = Tfs;
        errortfs66(j) = obj_cur; 
        errRtfs66(j) = cal_rotation_err(Rfs,Rtruth);
        errTtfs66(j) = cal_translation_err(Tfs,Ttruth);


        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        clearvars apointa apointb aPointsN aPointsNt aPointsNs apointsIni apointsInis apointsInit apointsIniu apointsIniv aPointsNu aPointsNv T1P1_wA T1P2_wA T1P1_wB T1P2_wB T1P1_wC T1P2_wC T1P1_wD T1P2_wD Rf Tf Rtf Ttf Rfs Tfs Ruf Tuf Rfv Tfv F1
        tic;

        T1P1_wA = [P1_w(1,:);P2_w(1,:);P1_w(1,:)];
        T1P2_wA = [P1_w(2,:);P2_w(2,:);P2_w(2,:)];

        T1P1_wB = [P1_w(1,:);P2_w(1,:);P1_w(2,:)];
        T1P2_wB = [P1_w(2,:);P2_w(2,:);P2_w(1,:)];

        T1P1_wC = [P1_w(1,:);P2_w(1,:);P1_w(1,:)];
        T1P2_wC = [P1_w(2,:);P2_w(2,:);P2_w(1,:)]; 

        T1P1_wD = [P1_w(1,:);P2_w(1,:);P1_w(2,:)];
        T1P2_wD = [P1_w(2,:);P2_w(2,:);P2_w(2,:)];

        [F1, G] = P2LMGA67(T1p1A, T1p2A, T1P1_wA, T1P2_wA, T1p1B, T1p2B, T1P1_wB, T1P2_wB, T1p1C, T1p2C, T1P1_wC, T1P2_wC, T1p1D, T1p2D, T1P1_wD, T1P2_wD, cH, A, Lheight);
        errAmin = Inf;
        errBmin = Inf;
        errCmin = Inf;
        errDmin = Inf;
        Ri = eye(3,3);
        Ti = [0,0,0];
        for i = 1:length(F1)
    %         if F1(i).ValObjective(1) > 1 || F1(i).ValObjective(2) > 1 || F1(i).ValObjective(3) > 1 || F1(i).ValObjective(4) > 1
            if F1(i).ValObjective(1) > 5 || F1(i).ValObjective(2) > 5 || F1(i).ValObjective(3) > 5 || F1(i).ValObjective(4) > 5
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
%         errOriRf67(j) = cal_rotation_err(Ri,Rtruth);
%         errOriTf67(j) = cal_translation_err(Ti,Ttruth);
        [~,flag] = min(errg);

        if flag  == 1

            [EE,GG,CC] = RefineEquA67A(p, W);
            apointsIni = [apointa(1) apointa(4)]';
            solutionO = [Cayley(RAt);apointsIni]; 
            [solution, obj_cur] = RefineGaussNewtonRandApT67A(solutionO, p, W, EE, GG, CC, Lheight);


            if size(solution) ~= 11
                aPointsN = solution(4:5);
            else
                aPointsN = solution(10:11);
            end

    %         aPointsN = [apoint(1) apoint(4)]';

            T1P1_wA(1,2) =  aPointsN(1);
            T1P1_wA(2,2) =  aPointsN(2)-50;
            T1P1_wA(3,2) =  aPointsN(1);
            T1P2_wA(1,2) =  aPointsN(1)+50;
            T1P2_wA(2,2) =  aPointsN(2);
            T1P2_wA(3,2) =  aPointsN(2);
            [Rf, Tf, errf] = SRPnL61(ip1A', ip2A', T1P1_wA', T1P2_wA', cH, Lheight);
        elseif flag == 2
            [EE,GG,CC] = RefineEquA67B(p, W);
            apointsIni = [apointa(2) apointa(3)]';
            solutionO = [Cayley(RBt);apointsIni]; 
            [solution, obj_cur] = RefineGaussNewtonRandApT67B(solutionO, p, W, EE, GG, CC, Lheight);

            if size(solution) ~= 11
                aPointsN = solution(4:5);
            else
                aPointsN = solution(10:11);
            end

    %         aPointsN = [apoint(2) apoint(3)]';

            T1P1_wB(1,2) =  aPointsN(2)-50;
            T1P1_wB(2,2) =  aPointsN(1);
            T1P1_wB(3,2) =  aPointsN(2);
            T1P2_wB(1,2) =  aPointsN(2);
            T1P2_wB(2,2) =  aPointsN(1)+50;
            T1P2_wB(3,2) =  aPointsN(1);
            [Rf, Tf, errf] = SRPnL61(ip1B', ip2B', T1P1_wB', T1P2_wB', cH, Lheight); 
        elseif flag == 3
            [EE,GG,CC] = RefineEquA67C(p, W);
            apointsIni = [apointb(1) apointb(2)]';
            solutionO = [Cayley(RCt);apointsIni]; 
            [solution, obj_cur] = RefineGaussNewtonRandApT67C(solutionO, p, W, EE, GG, CC, Lheight);

            if size(solution) ~= 11
                aPointsN = solution(4:5);
            else
                aPointsN = solution(10:11);
            end

    %         aPointsN = [apoint(3) apoint(4)]';

            T1P1_wC(1,2) =  aPointsN(1);
            T1P1_wC(2,2) =  aPointsN(2);
            T1P1_wC(3,2) =  aPointsN(1);
            T1P2_wC(1,2) =  aPointsN(1)+ 50;
            T1P2_wC(2,2) =  aPointsN(2)+ 50;
            T1P2_wC(3,2) =  aPointsN(2);
            [Rf, Tf, errf] = SRPnL61(ip1C', ip2C', T1P1_wC', T1P2_wC', cH, Lheight); 
        elseif flag == 4
            [EE,GG,CC] = RefineEquA67D(p, W);
            apointsIni = [apointb(3) apointb(4)]';
            solutionO = [Cayley(RDt);apointsIni]; 
            [solution, obj_cur] = RefineGaussNewtonRandApT67D(solutionO, p, W, EE, GG, CC, Lheight);

            if size(solution) ~= 11
                aPointsN = solution(4:5);
            else
                aPointsN = solution(10:11);
            end

    %         aPointsN = [apoint(3) apoint(4)]';

            T1P1_wD(1,2) =  aPointsN(1)-50;
            T1P1_wD(2,2) =  aPointsN(2)-50;
            T1P1_wD(3,2) =  aPointsN(1);
            T1P2_wD(1,2) =  aPointsN(1);
            T1P2_wD(2,2) =  aPointsN(2);
            T1P2_wB(3,2) =  aPointsN(2);
            [Rf, Tf, errf] = SRPnL61(ip1D', ip2D', T1P1_wD', T1P2_wD', cH, Lheight); 
        end
        
        R67{j} = Rf;
        T67{j} = Tf;
        error67(j) = obj_cur; 
        errRf67(j) = cal_rotation_err(Rf,Rtruth);
        errTf67(j) = cal_translation_err(Tf,Ttruth);
        toc;
        time67(j) = toc; 

        [EEt,GGt,CCt] = RefineEquA67A(p, W);
        apointsInit = [apointa(1) apointa(4)]';
        solutionOt = [Cayley(RAt);apointsInit]; 
        [solutiont, obj_cur] = RefineGaussNewtonRandApT67A(solutionOt, p, W, EEt, GGt, CCt, Lheight);


        if size(solutiont) ~= 11
            aPointsNt = solutiont(4:5);
        else
            aPointsNt = solutiont(10:11);
        end

    %     aPointsNt = [apoint(1) apoint(4)]';

        T1P1_wA(1,2) =  aPointsNt(1);
        T1P1_wA(2,2) =  aPointsNt(2)-50;
        T1P1_wA(3,2) =  aPointsNt(1);
        T1P2_wA(1,2) =  aPointsNt(1)+50;
        T1P2_wA(2,2) =  aPointsNt(2);
        T1P2_wA(3,2) =  aPointsNt(2);
        [Rtf, Ttf, errftf] = SRPnL61(ip1A', ip2A', T1P1_wA', T1P2_wA', cH, Lheight);  
        
        Rth67{j} = Rtf;
        Tth67{j} = Ttf;
        errortf67(j) = obj_cur; 
        errRtf67(j) = cal_rotation_err(Rtf,Rtruth);
        errTtf67(j) = cal_translation_err(Ttf,Ttruth);



        [EEs,GGs,CCs] = RefineEquA67B(p, W);
        apointsInis = [apointa(2) apointa(3)]';
        solutionOs = [Cayley(RBt);apointsInis]; 
        [solutions, obj_cur] = RefineGaussNewtonRandApT67B(solutionOs, p, W, EEs, GGs, CCs, Lheight);

        if size(solutions) ~= 11
            aPointsNs = solutions(4:5);
        else
            aPointsNs = solutions(10:11);
        end

    %     aPointsNs = [apoint(2) apoint(3)]';

        T1P1_wB(1,2) =  aPointsNs(2)-50;
        T1P1_wB(2,2) =  aPointsNs(1);
        T1P1_wB(3,2) =  aPointsNs(2);
        T1P2_wB(1,2) =  aPointsNs(2);
        T1P2_wB(2,2) =  aPointsNs(1)+50;
        T1P2_wB(3,2) =  aPointsNs(1);
        [Rfs, Tfs, errfs] = SRPnL61(ip1B', ip2B', T1P1_wB', T1P2_wB', cH, Lheight); 
        
        Rts67{j} = Rfs;
        Tts67{j} = Tfs;
        errortfs67(j) = obj_cur; 
        errRtfs67(j) = cal_rotation_err(Rfs,Rtruth);
        errTtfs67(j) = cal_translation_err(Tfs,Ttruth);


        [EEu,GGu,CCu] = RefineEquA67C(p, W);
        apointsIniu = [apointb(1) apointb(2)]';
        solutionOu = [Cayley(RCt);apointsIniu]; 
        [solutionu, obj_cur] = RefineGaussNewtonRandApT67C(solutionOu, p, W, EEu, GGu, CCu, Lheight);

        if size(solutionu) ~= 11
            aPointsNu = solutionu(4:5);
        else
            aPointsNu = solutionu(10:11);
        end

    %     aPointsNu = [apoint(1) apoint(2)]';

        T1P1_wC(1,2) =  aPointsNu(1);
        T1P1_wC(2,2) =  aPointsNu(2);
        T1P1_wC(3,2) =  aPointsNu(1);
        T1P2_wC(1,2) =  aPointsNu(1)+50;
        T1P2_wC(2,2) =  aPointsNu(2)+50;
        T1P2_wC(3,2) =  aPointsNu(2);
        [Ruf, Tuf, errfuf] = SRPnL61(ip1C', ip2C', T1P1_wC', T1P2_wC', cH, Lheight);
        
        Rthu67{j} = Ruf;
        Tthu67{j} = Tuf;
        errortfu67(j) = obj_cur; 
        errRtfu67(j) = cal_rotation_err(Ruf,Rtruth);
        errTtfu67(j) = cal_translation_err(Tuf,Ttruth);



        [EEv,GGv,CCv] = RefineEquA67D(p, W);
        apointsIniv = [apointb(3) apointb(4)]';
        solutionOv = [Cayley(RDt);apointsIniv]; 
        [solutionv, obj_cur] = RefineGaussNewtonRandApT67D(solutionOv, p, W, EEv, GGv, CCv, Lheight);

        if size(solutionv) ~= 11
            aPointsNv = solutionv(4:5);
        else
            aPointsNv = solutionv(10:11);
        end

    %     aPointsNv = [apoint(3) apoint(4)]';

        T1P1_wD(1,2) =  aPointsNv(1)-50;
        T1P1_wD(2,2) =  aPointsNv(2)-50;
        T1P1_wD(3,2) =  aPointsNv(1);
        T1P2_wD(1,2) =  aPointsNv(1);
        T1P2_wD(2,2) =  aPointsNv(2);
        T1P2_wD(3,2) =  aPointsNv(2);
        [Rfv, Tfv, errfv] = SRPnL61(ip1D', ip2D', T1P1_wD', T1P2_wD', cH, Lheight);       
        
        

        Rtv66{j} = Rfv;
        Ttv66{j} = Tfv;
        errortfv67(j) = obj_cur; 
        errRtfv67(j) = cal_rotation_err(Rfv,Rtruth);
        errTtfv67(j) = cal_translation_err(Tfv,Ttruth);



    end  


    err61GApnlRm{no+1} = mean(errRf61);
    err61GApnlRme{no+1} = median(errRf61);
    err61GApnlTm{no+1} = mean(errTf61);
    err61GApnlTme{no+1} = median(errTf61);


    err61GAtpnlRm{no+1} = mean(errRtf61);
    err61GAtpnlRme{no+1} = median(errRtf61);
    err61GAtpnlTm{no+1} = mean(errTtf61);
    err61GAtpnlTme{no+1} = median(errTtf61);


    err61GAspnlRm{no+1} = mean(errRtfs61);
    err61GAspnlRme{no+1} = median(errRtfs61);
    err61GAspnlTm{no+1} = mean(errTtfs61);
    err61GAspnlTme{no+1} = median(errTtfs61);

    err62GApnlRm{no+1} = mean(errRf62);
    err62GApnlRme{no+1} = median(errRf62);
    err62GApnlTm{no+1} = mean(errTf62);
    err62GApnlTme{no+1} = median(errTf62);


    err62GAtpnlRm{no+1} = mean(errRtf62);
    err62GAtpnlRme{no+1} = median(errRtf62);
    err62GAtpnlTm{no+1} = mean(errTtf62);
    err62GAtpnlTme{no+1} = median(errTtf62);


    err62GAspnlRm{no+1} = mean(errRtfs62);
    err62GAspnlRme{no+1} = median(errRtfs62);
    err62GAspnlTm{no+1} = mean(errTtfs62);
    err62GAspnlTme{no+1} = median(errTtfs62);

    err63GApnlRm{no+1} = mean(errRf63);
    err63GApnlRme{no+1} = median(errRf63);
    err63GApnlTm{no+1} = mean(errTf63);
    err63GApnlTme{no+1} = median(errTf63);


    err63GAtpnlRm{no+1} = mean(errRtf63);
    err63GAtpnlRme{no+1} = median(errRtf63);
    err63GAtpnlTm{no+1} = mean(errTtf63);
    err63GAtpnlTme{no+1} = median(errTtf63);


    err63GAspnlRm{no+1} = mean(errRtfs63);
    err63GAspnlRme{no+1} = median(errRtfs63);
    err63GAspnlTm{no+1} = mean(errTtfs63);
    err63GAspnlTme{no+1} = median(errTtfs63);

    err64GApnlRm{no+1} = mean(errRf64);
    err64GApnlRme{no+1} = median(errRf64);
    err64GApnlTm{no+1} = mean(errTf64);
    err64GApnlTme{no+1} = median(errTf64);


    err64GAtpnlRm{no+1} = mean(errRtf64);
    err64GAtpnlRme{no+1} = median(errRtf64);
    err64GAtpnlTm{no+1} = mean(errTtf64);
    err64GAtpnlTme{no+1} = median(errTtf64);


    err64GAspnlRm{no+1} = mean(errRtfs64);
    err64GAspnlRme{no+1} = median(errRtfs64);
    err64GAspnlTm{no+1} = mean(errTtfs64);
    err64GAspnlTme{no+1} = median(errTtfs64);

    err65GApnlRm{no+1} = mean(errRf65);
    err65GApnlRme{no+1} = median(errRf65);
    err65GApnlTm{no+1} = mean(errTf65);
    err65GApnlTme{no+1} = median(errTf65);


    err65GAtpnlRm{no+1} = mean(errRtf65);
    err65GAtpnlRme{no+1} = median(errRtf65);
    err65GAtpnlTm{no+1} = mean(errTtf65);
    err65GAtpnlTme{no+1} = median(errTtf65);


    err65GAspnlRm{no+1} = mean(errRtfs65);
    err65GAspnlRme{no+1} = median(errRtfs65);
    err65GAspnlTm{no+1} = mean(errTtfs65);
    err65GAspnlTme{no+1} = median(errTtfs65);

    err66GApnlRm{no+1} = mean(errRf66);
    err66GApnlRme{no+1} = median(errRf66);
    err66GApnlTm{no+1} = mean(errTf66);
    err66GApnlTme{no+1} = median(errTf66);


    err66GAtpnlRm{no+1} = mean(errRtf66);
    err66GAtpnlRme{no+1} = median(errRtf66);
    err66GAtpnlTm{no+1} = mean(errTtf66);
    err66GAtpnlTme{no+1} = median(errTtf66);


    err66GAspnlRm{no+1} = mean(errRtfs66);
    err66GAspnlRme{no+1} = median(errRtfs66);
    err66GAspnlTm{no+1} = mean(errTtfs66);
    err66GAspnlTme{no+1} = median(errTtfs66);


    err67GApnlRm{no+1} = mean(errRf67);
    err67GApnlRme{no+1} = median(errRf67);
    err67GApnlTm{no+1} = mean(errTf67);
    err67GApnlTme{no+1} = median(errTf67);


    err67GAtpnlRm{no+1} = mean(errRtf67);
    err67GAtpnlRme{no+1} = median(errRtf67);
    err67GAtpnlTm{no+1} = mean(errTtf67);
    err67GAtpnlTme{no+1} = median(errTtf67);


    err67GAspnlRm{no+1} = mean(errRtfs67);
    err67GAspnlRme{no+1} = median(errRtfs67);
    err67GAspnlTm{no+1} = mean(errTtfs67);
    err67GAspnlTme{no+1} = median(errTtfs67);


    err67GAupnlRm{no+1} = mean(errRtfu67);
    err67GAupnlRme{no+1} = median(errRtfu67);
    err67GAupnlTm{no+1} = mean(errTtfu67);
    err67GAupnlTme{no+1} = median(errTtfu67);


    err67GAvpnlRm{no+1} = mean(errRtfv67);
    err67GAvpnlRme{no+1} = median(errRtfv67);
    err67GAvpnlTm{no+1} = mean(errTtfv67);
    err67GAvpnlTme{no+1} = median(errTtfv67);


    avgTime61{no+1} = mean(time61);
    avgTime62{no+1} = mean(time62);
    avgTime63{no+1} = mean(time63);
    avgTime64{no+1} = mean(time64);
    avgTime65{no+1} = mean(time65);
    avgTime66{no+1} = mean(time66);
end    




