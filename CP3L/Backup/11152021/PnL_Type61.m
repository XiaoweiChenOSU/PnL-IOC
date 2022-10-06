close all;
clc;
clear;






%Type6 Room Camera Pose Estimation

for no = 2:10

    noise=no;
    rateOutlier=0;
    genN = 10;
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

        clearvars apoint aPointsN aPointsNt aPointsNs apointsIni apointsInis apointsInit apointsIniu apointsIniv aPointsNu aPointsNv T1P1_wA T1P2_wA T1P1_wB T1P2_wB T1P1_wC T1P2_wC T1P1_wD T1P2_wD
        tic;
        count = 1; 

        T1P1_wA = [P1_w(1,:);P2_w(1,:);P1_w(1,:)];
        T1P2_wA = [P1_w(2,:);P2_w(2,:);P2_w(2,:)];

        T1P1_wB = [P1_w(1,:);P2_w(1,:);P1_w(2,:)];
        T1P2_wB = [P1_w(2,:);P2_w(2,:);P2_w(1,:)];

        [F1, G] = P2LMGA61(T1p1A, T1p2A, T1P1_wA, T1P2_wA, T1p1B, T1p2B, T1P1_wB, T1P2_wB, cH);
        errmin = Inf;
        errAmin = Inf;
        errBmin = Inf;
        Ri = eye(3,3);
        Ti = [0,0,0];
        flag = 0;
        for i = 1:length(F1)
            if F1(i).ValObjective(1) > 2 || F1(i).ValObjective(2) > 2 || F1(i).ValObjective(3) > 10 || F1(i).ValObjective(4) > 10
    %         if F1(i).ValObjective(1) + F1(i).ValObjective(2) > 2
                 continue
            end
    %         apoints(count,:) =  F1(i).Val;   
    %         count = count + 1;
            if F1(i).ValObjective(1) + F1(i).ValObjective(2) < errmin
                if F1(i).ValObjective(1) < F1(i).ValObjective(2)
                    Ri = F1(i).RA;
                    Ti =  F1(i).TA;
                    errmin = F1(i).ValObjective(1);
                    flag = 1;
                else
                    Ri = F1(i).RB;
                    Ti =  F1(i).TB;
                    errmin = F1(i).ValObjective(2);
                    flag = 2;
                end
                RAt = F1(i).RA;
                TAt = F1(i).TA;
                RBt = F1(i).RB;
                TBt = F1(i).TB;
                apoint = F1(i).Val;
            end
        end  


        errOriRf61(j) = cal_rotation_err(Ri,Rtruth);
        errOriTf61(j) = cal_translation_err(Ti,Ttruth);


        if flag  == 1

            [EE,GG,CC] = RefineEquA61A(p, W);
            apointsIni = [apoint(1) apoint(4)]';
            solutionO = [Cayley(Ri);apointsIni]; 
            [solution, obj_cur] = RefineGaussNewtonRandApT61A(solutionO, p, W, EE, GG, CC);


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
            [Rf, Tf, errf] = SRPnL61(ip1A', ip2A', T1P1_wA', T1P2_wA', cH);
        elseif flag == 2
            [EE,GG,CC] = RefineEquA61B(p, W);
            apointsIni = [apoint(2) apoint(3)]';
            solutionO = [Cayley(Ri);apointsIni]; 
            [solution, obj_cur] = RefineGaussNewtonRandApT61B(solutionO, p, W, EE, GG, CC);

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
            [Rf, Tf, errf] = SRPnL61(ip1B', ip2B', T1P1_wB', T1P2_wB', cH); 
        end

        R61{j} = Rf;
        T61{j} = Tf;
        error61(j) = errf; 
        errRf61(j) = cal_rotation_err(Rf,Rtruth);
        errTf61(j) = cal_translation_err(Tf,Ttruth);
        toc;
        time61(j) = toc; 


        [EEt,GGt,CCt] = RefineEquA61A(p, W);
        apointsInit = [apoint(1) apoint(4)]';
        solutionOt = [Cayley(RAt);apointsInit]; 
        [solutiont, obj_curt] = RefineGaussNewtonRandApT61A(solutionOt, p, W, EEt, GGt, CCt);


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
        [Rtf, Ttf, errftf] = SRPnL61(ip1A', ip2A', T1P1_wA', T1P2_wA', cH);
        Rth61{j} = Rtf;
        Tth61{j} = Ttf;
        errortf61(j) = errftf; 
        errRtf61(j) = cal_rotation_err(Rtf,Rtruth);
        errTtf61(j) = cal_translation_err(Ttf,Ttruth);



        [EEs,GGs,CCs] = RefineEquA61B(p, W);
        apointsInis = [apoint(2) apoint(3)]';
        solutionOs = [Cayley(RBt);apointsInis]; 
        [solutions, obj_cur] = RefineGaussNewtonRandApT61B(solutionOs, p, W, EEs, GGs, CCs);

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
        [Rfs, Tfs, errfs] = SRPnL61(ip1B', ip2B', T1P1_wB', T1P2_wB', cH); 

        Rts61{j} = Rfs;
        Tts61{j} = Tfs;
        errortfs61(j) = errfs; 
        errRtfs61(j) = cal_rotation_err(Rfs,Rtruth);
        errTtfs61(j) = cal_translation_err(Tfs,Ttruth);


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

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        clearvars apoint aPointsN aPointsNt aPointsNs apointsIni apointsInis apointsInit apointsIniu apointsIniv aPointsNu aPointsNv T1P1_wA T1P2_wA T1P1_wB T1P2_wB T1P1_wC T1P2_wC T1P1_wD T1P2_wD
        tic;

        T1P1_wA = [P1_w(1,:);P2_w(1,:);P1_w(1,:)];
        T1P2_wA = [P1_w(2,:);P2_w(2,:);P2_w(2,:)];


        T1P1_wC = [P1_w(1,:);P2_w(1,:);P1_w(1,:)];
        T1P2_wC = [P1_w(2,:);P2_w(2,:);P2_w(1,:)];

        [F1, G] = P2LMGA62(T1p1A, T1p2A, T1P1_wA, T1P2_wA, T1p1C, T1p2C, T1P1_wC, T1P2_wC, cH);
        errmin = Inf;
        Ri = eye(3,3);
        Ti = [0,0,0];
        flag = 0;
        for i = 1:length(F1)
            if F1(i).ValObjective(1) > 2 || F1(i).ValObjective(2) > 2 || F1(i).ValObjective(3) > 10 || F1(i).ValObjective(4) > 10
    %         if F1(i).ValObjective(1) + F1(i).ValObjective(2)> 2
                 continue
            end
    %         apoints(count,:) =  F1(i).Val;   
    %         count = count + 1;
            if F1(i).ValObjective(1) + F1(i).ValObjective(2) < errmin
                if F1(i).ValObjective(1) < F1(i).ValObjective(2)
                    Ri = F1(i).RA;
                    Ti =  F1(i).TA;
                    errmin = F1(i).ValObjective(1);
                    flag = 1;
                else
                    Ri = F1(i).RB;
                    Ti =  F1(i).TB;
                    errmin = F1(i).ValObjective(2);
                    flag = 2;
                end
                RAt = F1(i).RA;
                TAt = F1(i).TA;
                RBt = F1(i).RB;
                TBt = F1(i).TB;
                apoint = F1(i).Val;
            end
        end  


        errOriRf62(j) = cal_rotation_err(Ri,Rtruth);
        errOriTf62(j) = cal_translation_err(Ti,Ttruth);


        if flag  == 1

            [EE,GG,CC] = RefineEquA62A(p, W);
            apointsIni = [apoint(1) apoint(3)]';
            solutionO = [Cayley(Ri);apointsIni]; 
            [solution, obj_cur] = RefineGaussNewtonRandApT62A(solutionO, p, W, EE, GG, CC);


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
            [Rf, Tf, errf] = SRPnL61(ip1A', ip2A', T1P1_wA', T1P2_wA', cH);
        elseif flag == 2
            [EE,GG,CC] = RefineEquA62B(p, W);
            apointsIni = [apoint(1) apoint(2)]';
            solutionO = [Cayley(Ri);apointsIni]; 
            [solution, obj_cur] = RefineGaussNewtonRandApT62B(solutionO, p, W, EE, GG, CC);

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
            [Rf, Tf, errf] = SRPnL61(ip1C', ip2C', T1P1_wC', T1P2_wC', cH); 
        end

        R62{j} = Rf;
        T62{j} = Tf;
        error62(j) = errf; 
        errRf62(j) = cal_rotation_err(Rf,Rtruth);
        errTf62(j) = cal_translation_err(Tf,Ttruth);
        toc;
        time62(j) = toc; 


        [EEt,GGt,CCt] = RefineEquA62A(p, W);
        apointsInit = [apoint(1) apoint(3)]';
        solutionOt = [Cayley(RAt);apointsInit]; 
        [solutiont, obj_curt] = RefineGaussNewtonRandApT62A(solutionOt, p, W, EEt, GGt, CCt);


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
        [Rtf, Ttf, errftf] = SRPnL61(ip1A', ip2A', T1P1_wA', T1P2_wA', cH);
        Rth62{j} = Rtf;
        Tth62{j} = Ttf;
        errortf62(j) = errftf; 
        errRtf62(j) = cal_rotation_err(Rtf,Rtruth);
        errTtf62(j) = cal_translation_err(Ttf,Ttruth);



        [EEs,GGs,CCs] = RefineEquA62B(p, W);
        apointsInis = [apoint(1) apoint(2)]';
        solutionOs = [Cayley(RBt);apointsInis]; 
        [solutions, obj_cur] = RefineGaussNewtonRandApT62B(solutionOs, p, W, EEs, GGs, CCs);

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
        [Rfs, Tfs, errfs] = SRPnL61(ip1C', ip2C', T1P1_wC', T1P2_wC', cH); 

        Rts62{j} = Rfs;
        Tts62{j} = Tfs;
        errortfs62(j) = errfs; 
        errRtfs62(j) = cal_rotation_err(Rfs,Rtruth);
        errTtfs62(j) = cal_translation_err(Tfs,Ttruth);


        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        clearvars apoint aPointsN aPointsNt aPointsNs apointsIni apointsInis apointsInit apointsIniu apointsIniv aPointsNu aPointsNv T1P1_wA T1P2_wA T1P1_wB T1P2_wB T1P1_wC T1P2_wC T1P1_wD T1P2_wD
        tic;
        T1P1_wA = [P1_w(1,:);P2_w(1,:);P1_w(1,:)];
        T1P2_wA = [P1_w(2,:);P2_w(2,:);P2_w(2,:)];


        T1P1_wD = [P1_w(1,:);P2_w(1,:);P1_w(2,:)];
        T1P2_wD = [P1_w(2,:);P2_w(2,:);P2_w(2,:)];

        [F1, G] = P2LMGA63(T1p1A, T1p2A, T1P1_wA, T1P2_wA, T1p1D, T1p2D, T1P1_wD, T1P2_wD, cH);
        errmin = Inf;
        Ri = eye(3,3);
        Ti = [0,0,0];
        flag = 0;
        for i = 1:length(F1)
            if F1(i).ValObjective(1) > 2 || F1(i).ValObjective(2) > 2 || F1(i).ValObjective(3) > 10 || F1(i).ValObjective(4) > 10
    %         if F1(i).ValObjective(1) + F1(i).ValObjective(2) > 2
                continue
            end
    %         apoints(count,:) =  F1(i).Val;   
    %         count = count + 1;
            if F1(i).ValObjective(1) + F1(i).ValObjective(2) < errmin
                if F1(i).ValObjective(1) < F1(i).ValObjective(2)
                    Ri = F1(i).RA;
                    Ti =  F1(i).TA;
                    errmin = F1(i).ValObjective(1);
                    flag = 1;
                else
                    Ri = F1(i).RB;
                    Ti =  F1(i).TB;
                    errmin = F1(i).ValObjective(2);
                    flag = 2;
                end
                RAt = F1(i).RA;
                TAt = F1(i).TA;
                RBt = F1(i).RB;
                TBt = F1(i).TB;
                apoint = F1(i).Val;
            end
        end  


        errOriRf63(j) = cal_rotation_err(Ri,Rtruth);
        errOriTf63(j) = cal_translation_err(Ti,Ttruth);


        if flag  == 1

            [EE,GG,CC] = RefineEquA63A(p, W);
            apointsIni = [apoint(1) apoint(3)]';
            solutionO = [Cayley(Ri);apointsIni]; 
            [solution, obj_cur] = RefineGaussNewtonRandApT63A(solutionO, p, W, EE, GG, CC);


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
            [Rf, Tf, errf] = SRPnL61(ip1A', ip2A', T1P1_wA', T1P2_wA', cH);
        elseif flag == 2
            [EE,GG,CC] = RefineEquA63B(p, W);
            apointsIni = [apoint(2) apoint(3)]';
            solutionO = [Cayley(Ri);apointsIni]; 
            [solution, obj_cur] = RefineGaussNewtonRandApT63B(solutionO, p, W, EE, GG, CC);

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
            [Rf, Tf, errf] = SRPnL61(ip1D', ip2D', T1P1_wD', T1P2_wD', cH); 
        end

        R63{j} = Rf;
        T63{j} = Tf;
        error63(j) = errf; 
        errRf63(j) = cal_rotation_err(Rf,Rtruth);
        errTf63(j) = cal_translation_err(Tf,Ttruth);
        toc;
        time63(j) = toc; 


        [EEt,GGt,CCt] = RefineEquA63A(p, W);
        apointsInit = [apoint(1) apoint(3)]';
        solutionOt = [Cayley(RAt);apointsInit]; 
        [solutiont, obj_curt] = RefineGaussNewtonRandApT63A(solutionOt, p, W, EEt, GGt, CCt);


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
        [Rtf, Ttf, errftf] = SRPnL61(ip1A', ip2A', T1P1_wA', T1P2_wA', cH);
        Rth63{j} = Rtf;
        Tth63{j} = Ttf;
        errortf63(j) = errftf; 
        errRtf63(j) = cal_rotation_err(Rtf,Rtruth);
        errTtf63(j) = cal_translation_err(Ttf,Ttruth);



        [EEs,GGs,CCs] = RefineEquA63B(p, W);
        apointsInis = [apoint(2) apoint(3)]';
        solutionOs = [Cayley(RBt);apointsInis]; 
        [solutions, obj_cur] = RefineGaussNewtonRandApT63B(solutionOs, p, W, EEs, GGs, CCs);

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
        [Rfs, Tfs, errfs] = SRPnL61(ip1D', ip2D', T1P1_wD', T1P2_wD', cH); 

        Rts63{j} = Rfs;
        Tts63{j} = Tfs;
        errortfs63(j) = errfs; 
        errRtfs63(j) = cal_rotation_err(Rfs,Rtruth);
        errTtfs63(j) = cal_translation_err(Tfs,Ttruth);


         %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        clearvars apoint aPointsN aPointsNt aPointsNs apointsIni apointsInis apointsInit apointsIniu apointsIniv aPointsNu aPointsNv T1P1_wA T1P2_wA T1P1_wB T1P2_wB T1P1_wC T1P2_wC T1P1_wD T1P2_wD
        tic; 
        T1P1_wB = [P1_w(1,:);P2_w(1,:);P1_w(2,:)];
        T1P2_wB = [P1_w(2,:);P2_w(2,:);P2_w(1,:)];

        T1P1_wC = [P1_w(1,:);P2_w(1,:);P1_w(1,:)];
        T1P2_wC = [P1_w(2,:);P2_w(2,:);P2_w(1,:)]; 

        [F1, G] = P2LMGA64(T1p1B, T1p2B, T1P1_wB, T1P2_wB, T1p1C, T1p2C, T1P1_wC, T1P2_wC, cH);
        errmin = Inf;
        Ri = eye(3,3);
        Ti = [0,0,0];
        flag = 0;
        for i = 1:length(F1)
    %         if F1(i).ValObjective(1) > 2 || F1(i).ValObjective(2) > 2 || F1(i).ValObjective(3) > 2 || F1(i).ValObjective(4) > 2
            % + F1(i).ValObjective(3) + F1(i).ValObjective(4)
            if F1(i).ValObjective(1) > 2 || F1(i).ValObjective(2) > 2 || F1(i).ValObjective(3) > 10 || F1(i).ValObjective(4) > 10
    %         if F1(i).ValObjective(1) + F1(i).ValObjective(2) > 2
                continue
            end
    %         apoints(count,:) =  F1(i).Val;   
    %         count = count + 1;
            if F1(i).ValObjective(1) + F1(i).ValObjective(2) < errmin
                if F1(i).ValObjective(1) < F1(i).ValObjective(2)
                    Ri = F1(i).RA;
                    Ti =  F1(i).TA;
                    errmin = F1(i).ValObjective(1);
                    flag = 1;
                else
                    Ri = F1(i).RB;
                    Ti =  F1(i).TB;
                    errmin = F1(i).ValObjective(2);
                    flag = 2;
                end
                RAt = F1(i).RA;
                TAt = F1(i).TA;
                RBt = F1(i).RB;
                TBt = F1(i).TB;
                apoint = F1(i).Val;
            end
        end  


        errOriRf64(j) = cal_rotation_err(Ri,Rtruth);
        errOriTf64(j) = cal_translation_err(Ti,Ttruth);


        if flag  == 1

            [EE,GG,CC] = RefineEquA64A(p, W);
            apointsIni = [apoint(2) apoint(3)]';
            solutionO = [Cayley(Ri);apointsIni]; 
            [solution, obj_cur] = RefineGaussNewtonRandApT64A(solutionO, p, W, EE, GG, CC);


            if size(solution) ~= 11
                aPointsN = solution(4:5);
            else
                aPointsN = solution(10:11);
            end

    %         aPointsN = [apoint(2) apoint(3)]';


            T1P1_wA(1,2) =  aPointsN(2)-50;
            T1P1_wA(2,2) =  aPointsN(1);
            T1P1_wA(3,2) =  aPointsN(2);
            T1P2_wA(1,2) =  aPointsN(2);
            T1P2_wA(2,2) =  aPointsN(1)+50;
            T1P2_wA(3,2) =  aPointsN(1);
            [Rf, Tf, errf] = SRPnL61(ip1B', ip2B', T1P1_wB', T1P2_wB', cH);
        elseif flag == 2
            [EE,GG,CC] = RefineEquA64B(p, W);
            apointsIni = [apoint(1) apoint(2)]';
            solutionO = [Cayley(Ri);apointsIni]; 
            [solution, obj_cur] = RefineGaussNewtonRandApT64B(solutionO, p, W, EE, GG, CC);

            if size(solution) ~= 11
                aPointsN = solution(4:5);
            else
                aPointsN = solution(10:11);
            end

    %         aPointsN = [apoint(1) apoint(2)]';

            T1P1_wB(1,2) =  aPointsN(1);
            T1P1_wB(2,2) =  aPointsN(2);
            T1P1_wB(3,2) =  aPointsN(1);
            T1P2_wB(1,2) =  aPointsN(1)+50;
            T1P2_wB(2,2) =  aPointsN(2)+50;
            T1P2_wB(3,2) =  aPointsN(2);
            [Rf, Tf, errf] = SRPnL61(ip1C', ip2C', T1P1_wC', T1P2_wC', cH); 
        end

        R64{j} = Rf;
        T64{j} = Tf;
        error64(j) = errf; 
        errRf64(j) = cal_rotation_err(Rf,Rtruth);
        errTf64(j) = cal_translation_err(Tf,Ttruth);
        toc;
        time64(j) = toc; 


        [EEt,GGt,CCt] = RefineEquA64A(p, W);
        apointsInit = [apoint(2) apoint(3)]';
        solutionOt = [Cayley(RAt);apointsInit]; 
        [solutiont, obj_curt] = RefineGaussNewtonRandApT64A(solutionOt, p, W, EEt, GGt, CCt);


        if size(solutiont) ~= 11
            aPointsNt = solutiont(4:5);
        else
            aPointsNt = solutiont(10:11);
        end

    %     aPointsNt = [apoint(2) apoint(3)]';

        T1P1_wA(1,2) =  aPointsNt(2)-50;
        T1P1_wA(2,2) =  aPointsNt(1);
        T1P1_wA(3,2) =  aPointsNt(2);
        T1P2_wA(1,2) =  aPointsNt(2);
        T1P2_wA(2,2) =  aPointsNt(1)+50;
        T1P2_wA(3,2) =  aPointsNt(1);
        [Rtf, Ttf, errftf] = SRPnL61(ip1B', ip2B', T1P1_wB', T1P2_wB', cH);
        Rth64{j} = Rtf;
        Tth64{j} = Ttf;
        errortf64(j) = errftf; 
        errRtf64(j) = cal_rotation_err(Rtf,Rtruth);
        errTtf64(j) = cal_translation_err(Ttf,Ttruth);



        [EEs,GGs,CCs] = RefineEquA64B(p, W);
        apointsInis = [apoint(1) apoint(2)]';
        solutionOs = [Cayley(RBt);apointsInis]; 
        [solutions, obj_cur] = RefineGaussNewtonRandApT64B(solutionOs, p, W, EEs, GGs, CCs);

        if size(solutions) ~= 11
            aPointsNs = solutions(4:5);
        else
            aPointsNs = solutions(10:11);
        end

    %     aPointsNs = [apoint(1) apoint(2)]';

        T1P1_wB(1,2) =  aPointsNs(1);
        T1P1_wB(2,2) =  aPointsNs(2);
        T1P1_wB(3,2) =  aPointsNs(1);
        T1P2_wB(1,2) =  aPointsNs(1)+50;
        T1P2_wB(2,2) =  aPointsNs(2)+50;
        T1P2_wB(3,2) =  aPointsNs(2);
        [Rfs, Tfs, errfs] = SRPnL61(ip1C', ip2C', T1P1_wC', T1P2_wC', cH); 

        Rts64{j} = Rfs;
        Tts64{j} = Tfs;
        errortfs64(j) = errfs; 
        errRtfs64(j) = cal_rotation_err(Rfs,Rtruth);
        errTtfs64(j) = cal_translation_err(Tfs,Ttruth);

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        clearvars apoint aPointsN aPointsNt aPointsNs apointsIni apointsInis apointsInit apointsIniu apointsIniv aPointsNu aPointsNv T1P1_wA T1P2_wA T1P1_wB T1P2_wB T1P1_wC T1P2_wC T1P1_wD T1P2_wD
        tic;

        T1P1_wB = [P1_w(1,:);P2_w(1,:);P1_w(2,:)];
        T1P2_wB = [P1_w(2,:);P2_w(2,:);P2_w(1,:)];


        T1P1_wD = [P1_w(1,:);P2_w(1,:);P1_w(2,:)];
        T1P2_wD = [P1_w(2,:);P2_w(2,:);P2_w(2,:)];

        [F1, G] = P2LMGA65(T1p1B, T1p2B, T1P1_wB, T1P2_wB, T1p1D, T1p2D, T1P1_wD, T1P2_wD, cH);
        errmin = Inf;
        Ri = eye(3,3);
        Ti = [0,0,0];
        flag = 0;
        for i = 1:length(F1)
    %         if F1(i).ValObjective(1) > 0.5 || F1(i).ValObjective(2) > 0.5 || F1(i).ValObjective(3) > 0.5 || F1(i).ValObjective(4) > 0.5
            if F1(i).ValObjective(1) > 2 || F1(i).ValObjective(2) > 2 || F1(i).ValObjective(3) > 10 || F1(i).ValObjective(4) > 10
    %         if F1(i).ValObjective(1) + F1(i).ValObjective(2) > 2
                continue
            end
    %         apoints(count,:) =  F1(i).Val;   
    %         count = count + 1;
            if F1(i).ValObjective(1) + F1(i).ValObjective(2) < errmin
                if F1(i).ValObjective(1) < F1(i).ValObjective(2)
                    Ri = F1(i).RA;
                    Ti =  F1(i).TA;
                    errmin = F1(i).ValObjective(1);
                    flag = 1;
                else
                    Ri = F1(i).RB;
                    Ti =  F1(i).TB;
                    errmin = F1(i).ValObjective(2);
                    flag = 2;
                end
                RAt = F1(i).RA;
                TAt = F1(i).TA;
                RBt = F1(i).RB;
                TBt = F1(i).TB;
                apoint = F1(i).Val;
            end
        end  


        errOriRf65(j) = cal_rotation_err(Ri,Rtruth);
        errOriTf65(j) = cal_translation_err(Ti,Ttruth);


        if flag  == 1

            [EE,GG,CC] = RefineEquA65A(p, W);
            apointsIni = [apoint(1) apoint(2)]';
            solutionO = [Cayley(Ri);apointsIni]; 
            [solution, obj_cur] = RefineGaussNewtonRandApT65A(solutionO, p, W, EE, GG, CC);


            if size(solution) ~= 11
                aPointsN = solution(4:5);
            else
                aPointsN = solution(10:11);
            end

    %         aPointsN = [apoint(1) apoint(2)]';

            T1P1_wA(1,2) =  aPointsN(2)-50;
            T1P1_wA(2,2) =  aPointsN(1);
            T1P1_wA(3,2) =  aPointsN(2);
            T1P2_wA(1,2) =  aPointsN(2);
            T1P2_wA(2,2) =  aPointsN(1)+50;
            T1P2_wA(3,2) =  aPointsN(1);
            [Rf, Tf, errf] = SRPnL61(ip1B', ip2B', T1P1_wB', T1P2_wB', cH);
        elseif flag == 2
            [EE,GG,CC] = RefineEquA65B(p, W);
            apointsIni = [apoint(2) apoint(3)]';
            solutionO = [Cayley(Ri);apointsIni]; 
            [solution, obj_cur] = RefineGaussNewtonRandApT65B(solutionO, p, W, EE, GG, CC);

            if size(solution) ~= 11
                aPointsN = solution(4:5);
            else
                aPointsN = solution(10:11);
            end

    %         aPointsN = [apoint(2) apoint(3)]';

            T1P1_wB(1,2) =  aPointsN(1)-50;
            T1P1_wB(2,2) =  aPointsN(2)-50;
            T1P1_wB(3,2) =  aPointsN(1);
            T1P2_wB(1,2) =  aPointsN(1);
            T1P2_wB(2,2) =  aPointsN(2);
            T1P2_wB(3,2) =  aPointsN(2);
            [Rf, Tf, errf] = SRPnL61(ip1D', ip2D', T1P1_wD', T1P2_wD', cH); 
        end

        R65{j} = Rf;
        T65{j} = Tf;
        error65(j) = errf; 
        errRf65(j) = cal_rotation_err(Rf,Rtruth);
        errTf65(j) = cal_translation_err(Tf,Ttruth);
        toc;
        time65(j) = toc; 


        [EEt,GGt,CCt] = RefineEquA65A(p, W);
        apointsInit = [apoint(1) apoint(2)]';
        solutionOt = [Cayley(RAt);apointsInit]; 
        [solutiont, obj_curt] = RefineGaussNewtonRandApT65A(solutionOt, p, W, EEt, GGt, CCt);


        if size(solutiont) ~= 11
            aPointsNt = solutiont(4:5);
        else
            aPointsNt = solutiont(10:11);
        end

    %     aPointsNt = [apoint(1) apoint(2)]';

        T1P1_wA(1,2) =  aPointsNt(2)-50;
        T1P1_wA(2,2) =  aPointsNt(1);
        T1P1_wA(3,2) =  aPointsNt(2);
        T1P2_wA(1,2) =  aPointsNt(2);
        T1P2_wA(2,2) =  aPointsNt(1)+50;
        T1P2_wA(3,2) =  aPointsNt(1);
        [Rtf, Ttf, errftf] = SRPnL61(ip1B', ip2B', T1P1_wB', T1P2_wB', cH);
        Rth65{j} = Rtf;
        Tth65{j} = Ttf;
        errortf65(j) = errftf; 
        errRtf65(j) = cal_rotation_err(Rtf,Rtruth);
        errTtf65(j) = cal_translation_err(Ttf,Ttruth);



        [EEs,GGs,CCs] = RefineEquA65B(p, W);
        apointsInis = [apoint(2) apoint(3)]';
        solutionOs = [Cayley(RBt);apointsInis]; 
        [solutions, obj_cur] = RefineGaussNewtonRandApT65B(solutionOs, p, W, EEs, GGs, CCs);

        if size(solutions) ~= 11
            aPointsNs = solutions(4:5);
        else
            aPointsNs = solutions(10:11);
        end

    %     aPointsNs = [apoint(2) apoint(3)]';

        T1P1_wB(1,2) =  aPointsNs(1)-50;
        T1P1_wB(2,2) =  aPointsNs(2)-50;
        T1P1_wB(3,2) =  aPointsNs(1);
        T1P2_wB(1,2) =  aPointsNs(1);
        T1P2_wB(2,2) =  aPointsNs(2);
        T1P2_wB(3,2) =  aPointsNs(2);
        [Rfs, Tfs, errfs] = SRPnL61(ip1D', ip2D', T1P1_wD', T1P2_wD', cH); 

        Rts65{j} = Rfs;
        Tts65{j} = Tfs;
        errortfs65(j) = errfs; 
        errRtfs65(j) = cal_rotation_err(Rfs,Rtruth);
        errTtfs65(j) = cal_translation_err(Tfs,Ttruth);    

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        clearvars apoint aPointsN aPointsNt aPointsNs apointsIni apointsInis apointsInit apointsIniu apointsIniv aPointsNu aPointsNv T1P1_wA T1P2_wA T1P1_wB T1P2_wB T1P1_wC T1P2_wC T1P1_wD T1P2_wD
        tic;
        T1P1_wC = [P1_w(1,:);P2_w(1,:);P1_w(1,:)];
        T1P2_wC = [P1_w(2,:);P2_w(2,:);P2_w(1,:)]; 

        T1P1_wD = [P1_w(1,:);P2_w(1,:);P1_w(2,:)];
        T1P2_wD = [P1_w(2,:);P2_w(2,:);P2_w(2,:)];

        [F1, G] = P2LMGA66(T1p1C, T1p2C, T1P1_wC, T1P2_wC, T1p1D, T1p2D, T1P1_wD, T1P2_wD, cH);
        errmin = Inf;
        Ri = eye(3,3);
        Ti = [0,0,0];
        flag = 0;
        for i = 1:length(F1)
    %         if F1(i).ValObjective(1) > 0.5 || F1(i).ValObjective(2) > 0.5 || F1(i).ValObjective(3) > 0.5 || F1(i).ValObjective(4) > 0.5
            if F1(i).ValObjective(1) > 2 || F1(i).ValObjective(2) > 2 || F1(i).ValObjective(3) > 10 || F1(i).ValObjective(4) > 10
    %         if F1(i).ValObjective(1) + F1(i).ValObjective(2)  > 2
                continue
            end
    %         apoints(count,:) =  F1(i).Val;   
    %         count = count + 1;
            if F1(i).ValObjective(1) + F1(i).ValObjective(2) < errmin
                if F1(i).ValObjective(1) < F1(i).ValObjective(2)
                    Ri = F1(i).RA;
                    Ti =  F1(i).TA;
                    errmin = F1(i).ValObjective(1);
                    flag = 1;
                else
                    Ri = F1(i).RB;
                    Ti =  F1(i).TB;
                    errmin = F1(i).ValObjective(2);
                    flag = 2;
                end
                RAt = F1(i).RA;
                TAt = F1(i).TA;
                RBt = F1(i).RB;
                TBt = F1(i).TB;
                apoint = F1(i).Val;
            end
        end  


        errOriRf66(j) = cal_rotation_err(Ri,Rtruth);
        errOriTf66(j) = cal_translation_err(Ti,Ttruth);


        if flag  == 1

            [EE,GG,CC] = RefineEquA66A(p, W);
            apointsIni = [apoint(1) apoint(2)]';
            solutionO = [Cayley(Ri);apointsIni]; 
            [solution, obj_cur] = RefineGaussNewtonRandApT66A(solutionO, p, W, EE, GG, CC);


            if size(solution) ~= 11
                aPointsN = solution(4:5);
            else
                aPointsN = solution(10:11);
            end

    %         aPointsN = [apoint(1) apoint(2)]';

            T1P1_wA(1,2) =  aPointsN(1);
            T1P1_wA(2,2) =  aPointsN(2);
            T1P1_wA(3,2) =  aPointsN(1);
            T1P2_wA(1,2) =  aPointsN(1)+50;
            T1P2_wA(2,2) =  aPointsN(2)+50;
            T1P2_wA(3,2) =  aPointsN(2);
            [Rf, Tf, errf] = SRPnL61(ip1C', ip2C', T1P1_wC', T1P2_wC', cH);
        elseif flag == 2
            [EE,GG,CC] = RefineEquA66B(p, W);
            apointsIni = [apoint(3) apoint(4)]';
            solutionO = [Cayley(Ri);apointsIni]; 
            [solution, obj_cur] = RefineGaussNewtonRandApT66B(solutionO, p, W, EE, GG, CC);

            if size(solution) ~= 11
                aPointsN = solution(4:5);
            else
                aPointsN = solution(10:11);
            end

    %         aPointsN = [apoint(3) apoint(4)]';

            T1P1_wB(1,2) =  aPointsN(1)-50;
            T1P1_wB(2,2) =  aPointsN(2)-50;
            T1P1_wB(3,2) =  aPointsN(1);
            T1P2_wB(1,2) =  aPointsN(1);
            T1P2_wB(2,2) =  aPointsN(2);
            T1P2_wB(3,2) =  aPointsN(2);
            [Rf, Tf, errf] = SRPnL61(ip1D', ip2D', T1P1_wD', T1P2_wD', cH); 
        end

        R66{j} = Rf;
        T66{j} = Tf;
        error66(j) = errf; 
        errRf66(j) = cal_rotation_err(Rf,Rtruth);
        errTf66(j) = cal_translation_err(Tf,Ttruth);
        toc;
        time66(j) = toc; 


        [EEt,GGt,CCt] = RefineEquA66A(p, W);
        apointsInit = [apoint(1) apoint(2)]';
        solutionOt = [Cayley(RAt);apointsInit]; 
        [solutiont, obj_curt] = RefineGaussNewtonRandApT66A(solutionOt, p, W, EEt, GGt, CCt);


        if size(solutiont) ~= 11
            aPointsNt = solutiont(4:5);
        else
            aPointsNt = solutiont(10:11);
        end

    %     aPointsNt = [apoint(1) apoint(2)]';

        T1P1_wA(1,2) =  aPointsNt(1);
        T1P1_wA(2,2) =  aPointsNt(2);
        T1P1_wA(3,2) =  aPointsNt(1);
        T1P2_wA(1,2) =  aPointsNt(1)+50;
        T1P2_wA(2,2) =  aPointsNt(2)+50;
        T1P2_wA(3,2) =  aPointsNt(2);
        [Rtf, Ttf, errftf] = SRPnL61(ip1C', ip2C', T1P1_wC', T1P2_wC', cH);
        Rth66{j} = Rtf;
        Tth66{j} = Ttf;
        errortf66(j) = errftf; 
        errRtf66(j) = cal_rotation_err(Rtf,Rtruth);
        errTtf66(j) = cal_translation_err(Ttf,Ttruth);



        [EEs,GGs,CCs] = RefineEquA66B(p, W);
        apointsInis = [apoint(3) apoint(4)]';
        solutionOs = [Cayley(RBt);apointsInis]; 
        [solutions, obj_cur] = RefineGaussNewtonRandApT66B(solutionOs, p, W, EEs, GGs, CCs);

        if size(solutions) ~= 11
            aPointsNs = solutions(4:5);
        else
            aPointsNs = solutions(10:11);
        end

    %     aPointsNs = [apoint(3) apoint(4)]';

        T1P1_wB(1,2) =  aPointsNs(1)-50;
        T1P1_wB(2,2) =  aPointsNs(2)-50;
        T1P1_wB(3,2) =  aPointsNs(1);
        T1P2_wB(1,2) =  aPointsNs(1);
        T1P2_wB(2,2) =  aPointsNs(2);
        T1P2_wB(3,2) =  aPointsNs(2);
        [Rfs, Tfs, errfs] = SRPnL61(ip1D', ip2D', T1P1_wD', T1P2_wD', cH); 

        Rts66{j} = Rfs;
        Tts66{j} = Tfs;
        errortfs66(j) = errfs; 
        errRtfs66(j) = cal_rotation_err(Rfs,Rtruth);
        errTtfs66(j) = cal_translation_err(Tfs,Ttruth);


        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        clearvars apoint aPointsN aPointsNt aPointsNs apointsIni apointsInis apointsInit apointsIniu apointsIniv aPointsNu aPointsNv T1P1_wA T1P2_wA T1P1_wB T1P2_wB T1P1_wC T1P2_wC T1P1_wD T1P2_wD
        tic;

        T1P1_wA = [P1_w(1,:);P2_w(1,:);P1_w(1,:)];
        T1P2_wA = [P1_w(2,:);P2_w(2,:);P2_w(2,:)];

        T1P1_wB = [P1_w(1,:);P2_w(1,:);P1_w(2,:)];
        T1P2_wB = [P1_w(2,:);P2_w(2,:);P2_w(1,:)];

        T1P1_wC = [P1_w(1,:);P2_w(1,:);P1_w(1,:)];
        T1P2_wC = [P1_w(2,:);P2_w(2,:);P2_w(1,:)]; 

        T1P1_wD = [P1_w(1,:);P2_w(1,:);P1_w(2,:)];
        T1P2_wD = [P1_w(2,:);P2_w(2,:);P2_w(2,:)];

        [F1, G] = P2LMGA67(T1p1A, T1p2A, T1P1_wA, T1P2_wA, T1p1B, T1p2B, T1P1_wB, T1P2_wB, T1p1C, T1p2C, T1P1_wC, T1P2_wC, T1p1D, T1p2D, T1P1_wD, T1P2_wD, cH);
        errmin = Inf;
        Ri = eye(3,3);
        Ti = [0,0,0];
        flag = 0;
        for i = 1:length(F1)
    %         if F1(i).ValObjective(1) > 1 || F1(i).ValObjective(2) > 1 || F1(i).ValObjective(3) > 1 || F1(i).ValObjective(4) > 1
            if F1(i).ValObjective(1) > 2 || F1(i).ValObjective(2) > 2 || F1(i).ValObjective(3) > 10 || F1(i).ValObjective(4) > 10
    %         if F1(i).ValObjective(1) + F1(i).ValObjective(2) > 2
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

    %         aPointsN = [apoint(1) apoint(4)]';

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

    %         aPointsN = [apoint(2) apoint(3)]';

            T1P1_wB(1,2) =  aPointsN(2)-50;
            T1P1_wB(2,2) =  aPointsN(1);
            T1P1_wB(3,2) =  aPointsN(2);
            T1P2_wB(1,2) =  aPointsN(2);
            T1P2_wB(2,2) =  aPointsN(1)+50;
            T1P2_wB(3,2) =  aPointsN(1);
            [Rf, Tf, errf] = SRPnL61(ip1B', ip2B', T1P1_wB', T1P2_wB', cH); 
        elseif flag == 3
            [EE,GG,CC] = RefineEquA67C(p, W);
            apointsIni = [apoint(1) apoint(2)]';
            solutionO = [Cayley(Ri);apointsIni]; 
            [solution, obj_cur] = RefineGaussNewtonRandApT67C(solutionO, p, W, EE, GG, CC);

            if size(solution) ~= 11
                aPointsN = solution(4:5);
            else
                aPointsN = solution(10:11);
            end

    %         aPointsN = [apoint(3) apoint(4)]';

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

    %         aPointsN = [apoint(3) apoint(4)]';

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

    %     aPointsNt = [apoint(1) apoint(4)]';

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
        [solutions, obj_cur] = RefineGaussNewtonRandApT67B(solutionOs, p, W, EEs, GGs, CCs);

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
        [Rfs, Tfs, errfs] = SRPnL61(ip1B', ip2B', T1P1_wB', T1P2_wB', cH); 

        Rts67{j} = Rfs;
        Tts67{j} = Tfs;
        errortfs67(j) = errfs; 
        errRtfs67(j) = cal_rotation_err(Rfs,Rtruth);
        errTtfs67(j) = cal_translation_err(Tfs,Ttruth);


        [EEu,GGu,CCu] = RefineEquA67C(p, W);
        apointsIniu = [apoint(1) apoint(2)]';
        solutionOu = [Cayley(RCt);apointsIniu]; 
        [solutionu, obj_curt] = RefineGaussNewtonRandApT67C(solutionOu, p, W, EEu, GGu, CCu);


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
        [Ruf, Tuf, errfuf] = SRPnL61(ip1C', ip2C', T1P1_wC', T1P2_wC', cH);
        Rthu67{j} = Ruf;
        Tthu67{j} = Tuf;
        errortfu67(j) = errfuf; 
        errRtfu67(j) = cal_rotation_err(Ruf,Rtruth);
        errTtfu67(j) = cal_translation_err(Tuf,Ttruth);



        [EEv,GGv,CCv] = RefineEquA67D(p, W);
        apointsIniv = [apoint(3) apoint(4)]';
        solutionOv = [Cayley(RDt);apointsIniv]; 
        [solutionv, obj_cur] = RefineGaussNewtonRandApT67D(solutionOv, p, W, EEv, GGv, CCv);

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
        [Rfv, Tfv, errfv] = SRPnL61(ip1D', ip2D', T1P1_wD', T1P2_wD', cH); 

        Rtv66{j} = Rfv;
        Ttv66{j} = Tfv;
        errortfv67(j) = errfs; 
        errRtfv67(j) = cal_rotation_err(Rfv,Rtruth);
        errTtfv67(j) = cal_translation_err(Tfv,Ttruth);



    end  

    % errSrpnlRm = mean(errRt);
    % errSrpnlRme = median(errRt);
    % errSrpnlTm = mean(errTt);
    % errSrpnlTme = median(errTt);
    % errSrpnlOtm = mean(errOtt);
    % errSrpnlOtme = median(errOtt);
    % 
    % 
    % errGApnlRm61 = mean(errRf61);
    % errGApnlRme61 = median(errRf61);
    % errGApnlTm61 = mean(errTf61);
    % errGApnlTme61 = median(errTf61);
    % 
    % 
    % errGAtpnlRm61 = mean(errRtf61);
    % errGAtpnlRme61 = median(errRtf61);
    % errGAtpnlTm61 = mean(errTtf61);
    % errGAtpnlTme61 = median(errTtf61);
    % 
    % 
    % errGAspnlRm61 = mean(errRtfs61);
    % errGAspnlRme61 = median(errRtfs61);
    % errGAspnlTm61 = mean(errTtfs61);
    % errGAspnlTme61 = median(errTtfs61);
    % 
    % errGApnlRm62 = mean(errRf62);
    % errGApnlRme62 = median(errRf62);
    % errGApnlTm62 = mean(errTf62);
    % errGApnlTme62 = median(errTf62);
    % 
    % 
    % errGAtpnlRm62 = mean(errRtf62);
    % errGAtpnlRme62 = median(errRtf62);
    % errGAtpnlTm62 = mean(errTtf62);
    % errGAtpnlTme62 = median(errTtf62);
    % 
    % 
    % errGAspnlRm62 = mean(errRtfs62);
    % errGAspnlRme62 = median(errRtfs62);
    % errGAspnlTm62 = mean(errTtfs62);
    % errGAspnlTme62 = median(errTtfs62);
    % 
    % errGApnlRm63 = mean(errRf63);
    % errGApnlRme63 = median(errRf63);
    % errGApnlTm63 = mean(errTf63);
    % errGApnlTme63 = median(errTf63);
    % 
    % 
    % errGAtpnlRm63 = mean(errRtf63);
    % errGAtpnlRme63 = median(errRtf63);
    % errGAtpnlTm63 = mean(errTtf63);
    % errGAtpnlTme63 = median(errTtf63);
    % 
    % 
    % errGAspnlRm63 = mean(errRtfs63);
    % errGAspnlRme63 = median(errRtfs63);
    % errGAspnlTm63 = mean(errTtfs63);
    % errGAspnlTme63 = median(errTtfs63);
    % 
    % errGApnlRm64 = mean(errRf64);
    % errGApnlRme64 = median(errRf64);
    % errGApnlTm64 = mean(errTf64);
    % errGApnlTme64 = median(errTf64);
    % 
    % 
    % errGAtpnlRm64 = mean(errRtf64);
    % errGAtpnlRme64 = median(errRtf64);
    % errGAtpnlTm64 = mean(errTtf64);
    % errGAtpnlTme64 = median(errTtf64);
    % 
    % 
    % errGAspnlRm64 = mean(errRtfs64);
    % errGAspnlRme64 = median(errRtfs64);
    % errGAspnlTm64 = mean(errTtfs64);
    % errGAspnlTme64 = median(errTtfs64);
    % 
    % errGApnlRm65 = mean(errRf65);
    % errGApnlRme65 = median(errRf65);
    % errGApnlTm65 = mean(errTf65);
    % errGApnlTme65 = median(errTf65);
    % 
    % 
    % errGAtpnlRm65 = mean(errRtf65);
    % errGAtpnlRme65 = median(errRtf65);
    % errGAtpnlTm65 = mean(errTtf65);
    % errGAtpnlTme65 = median(errTtf65);
    % 
    % 
    % errGAspnlRm65 = mean(errRtfs65);
    % errGAspnlRme65 = median(errRtfs65);
    % errGAspnlTm65 = mean(errTtfs65);
    % errGAspnlTme65 = median(errTtfs65);
    % 
    % errGApnlRm66 = mean(errRf66);
    % errGApnlRme66 = median(errRf66);
    % errGApnlTm66 = mean(errTf66);
    % errGApnlTme66 = median(errTf66);
    % 
    % 
    % errGAtpnlRm66 = mean(errRtf66);
    % errGAtpnlRme66 = median(errRtf66);
    % errGAtpnlTm66 = mean(errTtf66);
    % errGAtpnlTme66 = median(errTtf66);
    % 
    % 
    % errGAspnlRm66 = mean(errRtfs66);
    % errGAspnlRme66 = median(errRtfs66);
    % errGAspnlTm66 = mean(errTtfs66);
    % errGAspnlTme66 = median(errTtfs66);


    % errGAgpnlRm = mean(errRtfg);
    % errGAgpnlRme = median(errRtfg);
    % errGAgpnlTm = mean(errTtfg);
    % errGAgpnlTme = median(errTtfg);

    err61GApnlRm{no} = mean(errRf61);
    err61GApnlRme{no} = median(errRf61);
    err61GApnlTm{no} = mean(errTf61);
    err61GApnlTme{no} = median(errTf61);


    err61GAtpnlRm{no} = mean(errRtf61);
    err61GAtpnlRme{no} = median(errRtf61);
    err61GAtpnlTm{no} = mean(errTtf61);
    err61GAtpnlTme{no} = median(errTtf61);


    err61GAspnlRm{no} = mean(errRtfs61);
    err61GAspnlRme{no} = median(errRtfs61);
    err61GAspnlTm{no} = mean(errTtfs61);
    err61GAspnlTme{no} = median(errTtfs61);

    err62GApnlRm{no} = mean(errRf62);
    err62GApnlRme{no} = median(errRf62);
    err62GApnlTm{no} = mean(errTf62);
    err62GApnlTme{no} = median(errTf62);


    err62GAtpnlRm{no} = mean(errRtf62);
    err62GAtpnlRme{no} = median(errRtf62);
    err62GAtpnlTm{no} = mean(errTtf62);
    err62GAtpnlTme{no} = median(errTtf62);


    err62GAspnlRm{no} = mean(errRtfs62);
    err62GAspnlRme{no} = median(errRtfs62);
    err62GAspnlTm{no} = mean(errTtfs62);
    err62GAspnlTme{no} = median(errTtfs62);

    err63GApnlRm{no} = mean(errRf63);
    err63GApnlRme{no} = median(errRf63);
    err63GApnlTm{no} = mean(errTf63);
    err63GApnlTme{no} = median(errTf63);


    err63GAtpnlRm{no} = mean(errRtf63);
    err63GAtpnlRme{no} = median(errRtf63);
    err63GAtpnlTm{no} = mean(errTtf63);
    err63GAtpnlTme{no} = median(errTtf63);


    err63GAspnlRm{no} = mean(errRtfs63);
    err63GAspnlRme{no} = median(errRtfs63);
    err63GAspnlTm{no} = mean(errTtfs63);
    err63GAspnlTme = median(errTtfs63);

    err64GApnlRm{no} = mean(errRf64);
    err64GApnlRme{no} = median(errRf64);
    err64GApnlTm{no} = mean(errTf64);
    err64GApnlTme{no} = median(errTf64);


    err64GAtpnlRm{no} = mean(errRtf64);
    err64GAtpnlRme{no} = median(errRtf64);
    err64GAtpnlTm{no} = mean(errTtf64);
    err64GAtpnlTme{no} = median(errTtf64);


    err64GAspnlRm{no} = mean(errRtfs64);
    err64GAspnlRme{no} = median(errRtfs64);
    err64GAspnlTm{no} = mean(errTtfs64);
    err64GAspnlTme{no} = median(errTtfs64);

    err65GApnlRm{no} = mean(errRf65);
    err65GApnlRme{no} = median(errRf65);
    err65GApnlTm{no} = mean(errTf65);
    err65GApnlTme{no} = median(errTf65);


    err65GAtpnlRm{no} = mean(errRtf65);
    err65GAtpnlRme{no} = median(errRtf65);
    err65GAtpnlTm{no} = mean(errTtf65);
    err65GAtpnlTme{no} = median(errTtf65);


    err65GAspnlRm{no} = mean(errRtfs65);
    err65GAspnlRme{no} = median(errRtfs65);
    err65GAspnlTm{no} = mean(errTtfs65);
    err65GAspnlTme{no} = median(errTtfs65);

    err66GApnlRm{no} = mean(errRf66);
    err66GApnlRme{no} = median(errRf66);
    err66GApnlTm{no} = mean(errTf66);
    err66GApnlTme{no} = median(errTf66);


    err66GAtpnlRm{no} = mean(errRtf66);
    err66GAtpnlRme{no} = median(errRtf66);
    err66GAtpnlTm{no} = mean(errTtf66);
    err66GAtpnlTme{no} = median(errTtf66);


    err66GAspnlRm{no} = mean(errRtfs66);
    err66GAspnlRme{no} = median(errRtfs66);
    err66GAspnlTm{no} = mean(errTtfs66);
    err66GAspnlTme{no} = median(errTtfs66);


    err67GApnlRm{no} = mean(errRf67);
    err67GApnlRme{no} = median(errRf67);
    err67GApnlTm{no} = mean(errTf67);
    err67GApnlTme{no} = median(errTf67);


    err67GAtpnlRm{no} = mean(errRtf67);
    err67GAtpnlRme{no} = median(errRtf67);
    err67GAtpnlTm{no} = mean(errTtf67);
    err67GAtpnlTme{no} = median(errTtf67);


    err67GAspnlRm{no} = mean(errRtfs67);
    err67GAspnlRme{no} = median(errRtfs67);
    err67GAspnlTm{no} = mean(errTtfs67);
    err67GAspnlTme{no} = median(errTtfs67);


    err67GAupnlRm{no} = mean(errRtfu67);
    err67GAupnlRme{no} = median(errRtfu67);
    err67GAupnlTm{no} = mean(errTtfu67);
    err67GAupnlTme{no} = median(errTtfu67);


    err67GAvpnlRm{no} = mean(errRtfv67);
    err67GAvpnlRme{no} = median(errRtfv67);
    err67GAvpnlTm{no} = mean(errTtfv67);
    err67GAvpnlTme{no} = median(errTtfv67);


    avgTime61{no} = mean(time61);
    avgTime62{no} = mean(time62);
    avgTime63{no} = mean(time63);
    avgTime64{no} = mean(time64);
    avgTime65{no} = mean(time65);
    avgTime66{no} = mean(time66);
end    




