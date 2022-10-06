%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%   ZDT6
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [f,RA,RB,RC,RD,TA,TB,TC,TD] = CameraPose67(s, T1p1A, T1p2A, Pn1_wA, Pn2_wA, T1p1B, T1p2B, Pn1_wB, Pn2_wB, T1p1C, T1p2C, Pn1_wC, Pn2_wC, T1p1D, T1p2D, Pn1_wD, Pn2_wD,T1A,C_truth,unknown)

% function [f,RA,RB,TA,TB] = CameraPose67(s, T1p1A, T1p2A, Pn1_wA, Pn2_wA, T1p1B, T1p2B, Pn1_wB, Pn2_wB,cH)

    n = unknown;
    
    L1 = (Pn2_wA(1,:)- Pn1_wA(1,:))/norm(Pn2_wA(1,:)- Pn1_wA(1,:));
    L2 = (Pn2_wA(2,:)- Pn1_wA(2,:))/norm(Pn2_wA(2,:)- Pn1_wA(2,:));
    
    Pn1_wA(1,n) =  s(1);
    Pn1_wA(3,n) =  s(1);
    Pn2_wA(2,n) =  s(4);
    Pn2_wA(3,n) =  s(4);
    Pn1_wA(2,:) =  Pn2_wA(2,:)-L2;
    Pn2_wA(1,:) =  Pn1_wA(1,:)+L1;

    
    Pn1_wB(2,n) =  s(2);
    Pn1_wB(3,n) =  s(3);
    Pn2_wB(1,n) =  s(3);    
    Pn2_wB(3,n) =  s(2);
    Pn1_wB(1,:) =  Pn2_wB(1,:)-L1;
    Pn2_wB(2,:) =  Pn1_wB(2,:)+L2;
    
    Pn1_wC(1,n) =  s(1);
    Pn1_wC(2,n) =  s(2);
    Pn1_wC(3,n) =  s(1);
    Pn2_wC(3,n) =  s(2);
    Pn2_wC(1,:) =  Pn1_wC(1,:)+L1;
    Pn2_wC(2,:) =  Pn1_wC(2,:)+L2;
    
    Pn1_wD(3,n) =  s(3);
    Pn2_wD(1,n) =  s(3);
    Pn2_wD(2,n) =  s(4);
    Pn2_wD(3,n) =  s(4);
    Pn1_wD(1,:) =  Pn2_wD(1,:)-L1;
    Pn1_wD(2,:) =  Pn2_wD(2,:)-L2;
%     
%     T1A = [200 0 160;0 200 320;0 0 1];

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

    
    [RA, TA, errA] = SRPnLReal61(iT1p1A', iT1p2A', Pn1_wA', Pn2_wA',C_truth,unknown);
    [RB, TB, errB] = SRPnLReal61(iT1p1B', iT1p2B', Pn1_wB', Pn2_wB',C_truth,unknown);
    [RC, TC, errC] = SRPnLReal61(iT1p1C', iT1p2C', Pn1_wC', Pn2_wC',C_truth,unknown);
    [RD, TD, errD] = SRPnLReal61(iT1p1D', iT1p2D', Pn1_wD', Pn2_wD',C_truth,unknown);


    if isinf(errA) || isinf(errB) || isinf(errC) || isinf(errD)
        err1 = inf;
        err2 = inf;
        err3 = inf;
        err4 = inf;
%         err5 = inf;
        err5 = inf;
        err6 = inf;
%         err7 = inf;
%         err8 = inf;
%         err9 = inf;
%         err10 =inf;
%         err11 = inf;
%         err12 = inf;
%         err13 = inf;
%         err14 = inf;
%         err15 = inf;
%         err16 = inf;
    else
%         err1 = errA;
%         err2 = errB;
        err1 = errA;
        err2 = errB;
        err3 = errC;
        err4 = errD;
        err5 = cal_rotation_err(RA,RB);
        err6 = cal_rotation_err(RA,RD);
%         err7 = cal_rotation_err(RB,RD);
%         err8 = cal_translation_err(TA,TB);
%         err9 = cal_translation_err(TA,TD);
%         err10 = cal_translation_err(TB,TD);
%         err3 = cal_rotation_err(RA,RB);
%         err4 = cal_translation_err(TA,TB);
%         err5 = cal_rotation_err(RA,RB);
%         err6 = cal_rotation_err(RA,RC);
%         err7 = cal_rotation_err(RA,RD);
%         err8 = cal_rotation_err(RB,RC);
%         err9 = cal_rotation_err(RB,RD);
%         err10 =cal_rotation_err(RC,RD);
%         err11 = cal_translation_err(TA,TB);
%         err12 = cal_translation_err(TA,TC);
%         err13 = cal_translation_err(TA,TD);
%         err14 = cal_translation_err(TB,TC);
%         err15 = cal_translation_err(TB,TD);
%         err16 = cal_translation_err(TC,TD);
%         
%         err4 = cal_rotation_err(RA,RB)+cal_rotation_err(RA,RC)+cal_rotation_err(RB,RC);
%         err5 = cal_translation_err(TA,TB)+cal_translation_err(TA,TC)+cal_translation_err(TB,TC);

%         err5 = cal_rotation_err(RA,RB)+cal_rotation_err(RA,RC)+cal_rotation_err(RA,RD)+cal_rotation_err(RB,RC)+cal_rotation_err(RB,RD)+cal_rotation_err(RC,RD);
%         err6 = cal_translation_err(TA,TB)+cal_translation_err(TA,TC)+cal_translation_err(TA,TD)+cal_translation_err(TB,TC)+cal_translation_err(TB,TD)+cal_translation_err(TC,TD);
%         
%         cal_rotation_err(RA,RB);
%         if errA < errB
%             R = RA;
%             T = TA;
%         else
%             R = RB;
%             T = TB;
%         end
        
    end  

%     if isinf(err1)
%         err2 = inf;
%     else
%         pd = [T1p1(1:2,:);T1p2(1:2,:)];
%         PW = [Pn1_w(1:2,:);Pn2_w(1:2,:)];
% 
%         err2 = reprojection_error_usingRT(PW,pd,R,T,T1A);
%     end

%     pd = [T1p1(1:2,:);T1p2(1:2,:)];
%     PW = [Pn1_w(1:2,:);Pn2_w(1:2,:)];

%     if isinf(errA) || isinf(errB)
%         err1 = inf;
%         err2 = inf;
%     else
%         err1 = errA;
%         err2 = errB;
% %         cal_rotation_err(RA,RB);
%     end  
%     f = [err1;err2;err3;err4];
    f = [err1;err2;err3;err4;err5;err6];
%     f = [err1;err2;err3;err4;err5;err6;err7;err8;err9;err10];
    
%     f = [err1;err2;err3;err4;err5;err6;err7;err8;err9;err10;err11;err12;err13;err14;err15;err16];