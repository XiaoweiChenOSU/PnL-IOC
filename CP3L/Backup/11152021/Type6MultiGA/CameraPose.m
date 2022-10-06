%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%   ZDT6
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [f,R,T] = CameraPose(s, T1p1, T1p2, Pn1_w, Pn2_w, cH)

    Pn1_w(1,2) =  s(1);
    Pn1_w(2,2) =  s(3);
    Pn1_w(3,2) =  s(1);
    Pn1_w(4,2) =  s(2);
    Pn2_w(1,2) =  s(2);
    Pn2_w(2,2) =  s(4);
    Pn2_w(3,2) =  s(3);
    Pn2_w(4,2) =  s(4);

    T1A = [180 0 160;0 180 320;0 0 1];

    for i = 1:length(T1p1)
        temp1 = inv(T1A)*[T1p1(i,:) 1].';
        temp1 = (temp1/temp1(3)).';
        iT1p1(i,:) = temp1(:,1:2);
        temp2 = inv(T1A)*[T1p2(i,:) 1].';
        temp2 = (temp2/temp2(3)).';
        iT1p2(i,:) = temp2(:,1:2);
    end

    
    [R, T, err1] = SRPnL6(iT1p1', iT1p2', Pn1_w', Pn2_w', cH);

    if isinf(err1)
        err2 = inf;
    else
        tempCT = -inv(R)*T;
        err2 = (tempCT(2) - cH)^2;
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

    
    f = [err1;err2];