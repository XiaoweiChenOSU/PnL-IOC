%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%   ZDT6
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [f,RA,RB,TA,TB] = CameraPose61(s, T1p1A, T1p2A, Pn1_wA, Pn2_wA, T1p1B, T1p2B, Pn1_wB, Pn2_wB,cH,T1A, Lheight)

    Pn1_wA(1,2) =  s(1);
    Pn1_wA(2,2) =  s(4)-1;
    Pn1_wA(3,2) =  s(1);
    Pn2_wA(1,2) =  s(1)+1;
    Pn2_wA(2,2) =  s(4);
    Pn2_wA(3,2) =  s(4);

    Pn1_wB(1,2) =  s(3)-1;
    Pn1_wB(2,2) =  s(2);
    Pn1_wB(3,2) =  s(3);
    Pn2_wB(1,2) =  s(3);
    Pn2_wB(2,2) =  s(2)+1;
    Pn2_wB(3,2) =  s(2);
    
    
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

    
    [RA, TA, errA] = SRPnL61(iT1p1A', iT1p2A', Pn1_wA', Pn2_wA',cH,Lheight);
    [RB, TB, errB] = SRPnL61(iT1p1B', iT1p2B', Pn1_wB', Pn2_wB',cH,Lheight);
    
%     W1 = [Pn1_wA(1,:);Pn2_wA(2,:)];
%     p1 = [T1p1A(1,:);T1p2A(2,:)];
%     [errepA,~]=reprojection_error_usingRT(W1,p1,RA,TA,T1A);
%     
%     W2 = [Pn1_wB(2,:);Pn1_wB(1,:)];
%     p2 = [T1p1B(2,:);T1p2B(1,:)];
%     [errepB,~]=reprojection_error_usingRT(W2,p2,RB,TB,T1A);

%     ecHA = -inv(RA)*TA;
%     ecHB = -inv(RB)*TB;

    if isinf(errA) || isinf(errB)
        err1 = inf;
        err2 = inf;
        err3 = inf;
        err4 = inf;
%         err5 = inf;
    else
        err1 = errA;
        err2 = errB;
        err3 = cal_rotation_err(RA,RB);
        err4 = cal_translation_err(TA,TB);
%         err5 = abs(ecHA(2)-cH) + abs(ecHB(2)-cH); 
    end
    
    
   f = [err1;err2;err3;err4];
    
%    f = [err1;err2;err3;err4;err5];