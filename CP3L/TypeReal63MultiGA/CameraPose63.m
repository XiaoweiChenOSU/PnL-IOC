%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%   ZDT6
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [f,RA,RB,TA,TB] = CameraPose63(s, T1p1A, T1p2A, Pn1_wA, Pn2_wA, T1p1B, T1p2B, Pn1_wB, Pn2_wB,cH,T1A,C_truth,unknown)

    n = unknown; 
    L1 = (Pn2_wA(1,:)- Pn1_wA(1,:))/norm(Pn2_wA(1,:)- Pn1_wA(1,:));
    L2 = (Pn2_wA(2,:)- Pn1_wA(2,:))/norm(Pn2_wA(2,:)- Pn1_wA(2,:));
    
    Pn1_wA(1,n) =  s(1);
    Pn1_wA(3,n) =  s(1);
    Pn2_wA(2,n) =  s(3);
    Pn2_wA(3,n) =  s(3);
    Pn1_wA(2,:) =  Pn2_wA(2,:) - L2;
    Pn2_wA(1,:) =  Pn1_wA(1,:) + L1;


    Pn1_wB(3,n) =  s(2);
    Pn2_wB(1,n) =  s(2);
    Pn2_wB(2,n) =  s(3);
    Pn2_wB(3,n) =  s(3);
    Pn1_wB(1,:) =  Pn2_wB(1,:) - L1;
    Pn1_wB(2,:) =  Pn2_wB(2,:) - L2;
    


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

    
    [RA, TA, errA] = SRPnLReal61(iT1p1A', iT1p2A', Pn1_wA', Pn2_wA',C_truth,unknown);
    [RB, TB, errB] = SRPnLReal61(iT1p1B', iT1p2B', Pn1_wB', Pn2_wB',C_truth,unknown);


    if isinf(errA) || isinf(errB)
        err1 = inf;
        err2 = inf;
        err3 = inf;
        err4 = inf;
    else

        err1 = errA;
        err2 = errB;
        err3 = cal_rotation_err(RA,RB);
        err4 = cal_translation_err(TA,TB);

        
    end  
 

    
    f = [err1;err2;err3;err4];