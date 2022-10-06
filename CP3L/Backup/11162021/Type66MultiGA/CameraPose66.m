%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%   ZDT6
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [f,RA,RB,TA,TB] = CameraPose66(s, T1p1A, T1p2A, Pn1_wA, Pn2_wA, T1p1B, T1p2B, Pn1_wB, Pn2_wB,cH,T1A, Lheight)

    Pn1_wA(1,2) =  s(1);
    Pn1_wA(2,2) =  s(2);
    Pn1_wA(3,2) =  s(1);
    Pn2_wA(1,2) =  s(1)+1;
    Pn2_wA(2,2) =  s(2)+1;
    Pn2_wA(3,2) =  s(2);

    Pn1_wB(1,2) =  s(3)-1;
    Pn1_wB(2,2) =  s(4)-1;
    Pn1_wB(3,2) =  s(3);
    Pn2_wB(1,2) =  s(3);
    Pn2_wB(2,2) =  s(4);
    Pn2_wB(3,2) =  s(4);
    
    

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

    
    [RA, TA, errA] = SRPnL61(iT1p1A', iT1p2A', Pn1_wA', Pn2_wA',cH, Lheight);
    [RB, TB, errB] = SRPnL61(iT1p1B', iT1p2B', Pn1_wB', Pn2_wB',cH, Lheight);


    if isinf(errA) || isinf(errB)
        err1 = inf;
        err2 = inf;
        err3 = inf;
        err4 = inf;
    else
%         err1 = errA;
%         err2 = errB;
        err1 = errA;
        err2 = errB;
        err3 = cal_rotation_err(RA,RB);
        err4 = cal_translation_err(TA,TB);
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

    
    f = [err1;err2;err3;err4];