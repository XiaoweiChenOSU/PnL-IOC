%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%   ZDT6
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [f,RA,RB,RC,RD,TA,TB,TC,TD] = CameraPose77(s, T1p1A, T1p2A, Pn1_wA, Pn2_wA, T1p1B, T1p2B, Pn1_wB, Pn2_wB, T1p1C, T1p2C, Pn1_wC, Pn2_wC, T1p1D, T1p2D, Pn1_wD, Pn2_wD,cH)

% function [f,RA,RB,TA,TB] = CameraPose67(s, T1p1A, T1p2A, Pn1_wA, Pn2_wA, T1p1B, T1p2B, Pn1_wB, Pn2_wB,cH)

    Pn1_wA(1,1) =  s(1);
    Pn1_wA(2,1) =  s(4)-50;
    Pn1_wA(3,1) =  s(1);
    Pn2_wA(1,1) =  s(1)+50;
    Pn2_wA(2,1) =  s(4);
    Pn2_wA(3,1) =  s(4);

    Pn1_wB(1,1) =  s(3)-50;
    Pn1_wB(2,1) =  s(2);
    Pn1_wB(3,1) =  s(3);
    Pn2_wB(1,1) =  s(3);
    Pn2_wB(2,1) =  s(2)+50;
    Pn2_wB(3,1) =  s(2);
    
    Pn1_wC(1,1) =  s(1);
    Pn1_wC(2,1) =  s(2);
    Pn1_wC(3,1) =  s(1);
    Pn2_wC(1,1) =  s(1)+50;
    Pn2_wC(2,1) =  s(2)+50;
    Pn2_wC(3,1) =  s(2);
    
    Pn1_wD(1,1) =  s(3)-50;
    Pn1_wD(2,1) =  s(4)-50;
    Pn1_wD(3,1) =  s(3);
    Pn2_wD(1,1) =  s(3);
    Pn2_wD(2,1) =  s(4);
    Pn2_wD(3,1) =  s(4);
%     
    T1A = [100 0 320;0 100 160;0  0  1];

    for i = 1:length(T1p1A)
        temp1 = inv(T1A)*[T1p1A(i,:) 1].';
        temp1 = (temp1/temp1(3)).';
        iT1p1A(i,:) = temp1(:,1:2);
        temp2 = inv(T1A)*[T1p2A(i,:) 1].';
        temp2 = (temp2/temp2(3)).';
        iT1p2A(i,:) = temp2(:,1:2);
    end
    
    for i = 1:length(T1p1B)
        temp1 = inv(T1A)*[T1p1B(i,:) 1].';
        temp1 = (temp1/temp1(3)).';
        iT1p1B(i,:) = temp1(:,1:2);
        temp2 = inv(T1A)*[T1p2B(i,:) 1].';
        temp2 = (temp2/temp2(3)).';
        iT1p2B(i,:) = temp2(:,1:2);
    end
    
       for i = 1:length(T1p1C)
        temp1 = inv(T1A)*[T1p1C(i,:) 1].';
        temp1 = (temp1/temp1(3)).';
        iT1p1C(i,:) = temp1(:,1:2);
        temp2 = inv(T1A)*[T1p2C(i,:) 1].';
        temp2 = (temp2/temp2(3)).';
        iT1p2C(i,:) = temp2(:,1:2);
    end
    
    for i = 1:length(T1p1D)
        temp1 = inv(T1A)*[T1p1D(i,:) 1].';
        temp1 = (temp1/temp1(3)).';
        iT1p1D(i,:) = temp1(:,1:2);
        temp2 = inv(T1A)*[T1p2D(i,:) 1].';
        temp2 = (temp2/temp2(3)).';
        iT1p2D(i,:) = temp2(:,1:2);
    end

    
    [RA, TA, errA] = SRPnL71(iT1p1A', iT1p2A', Pn1_wA', Pn2_wA',cH);
    [RB, TB, errB] = SRPnL71(iT1p1B', iT1p2B', Pn1_wB', Pn2_wB',cH);
    [RC, TC, errC] = SRPnL71(iT1p1C', iT1p2C', Pn1_wC', Pn2_wC',cH);
    [RD, TD, errD] = SRPnL71(iT1p1D', iT1p2D', Pn1_wD', Pn2_wD',cH);


    if isinf(errA) || isinf(errB) || isinf(errC) || isinf(errD)
        err1 = inf;
        err2 = inf;
        err3 = inf;
        err4 = inf;
%         err5 = inf;
        err5 = inf;
        err6 = inf;
        err7 = inf;

    else
%         err1 = errA;
%         err2 = errB;
        err1 = errA;
        err2 = errB;
        err3 = errC;
        err4 = errD;
        err5 = cal_rotation_err(RA,RB);
        err6 = cal_rotation_err(RB,RC);
        err7 = cal_rotation_err(RC,RD);

        
    end  


    f = [err1;err2;err3;err4;err5;err6;err7];
%     f = [err1;err2;err3;err4;err5;err6;err7;err8;err9;err10];
    
%     f = [err1;err2;err3;err4;err5;err6;err7;err8;err9;err10;err11;err12;err13;err14;err15;err16];