function [error, y, best_id, ercorr] = choose_solution(R1, t1, R, t, XXw, Xc)
    error = inf;
    best_id = -1;
    npt = size(XXw, 2);
    for jjj = 1:size(R1,3)
        tempy = cal_pose_err([R1(:,:,jjj) t1(:,jjj)],[R t]);
        if sum(tempy) < error  
            temp = R1(:,:,jjj) * XXw +  t1(:,jjj) * ones(1,npt);
            for i = 1:size(temp,2)
                temp(:,i) = temp(:,i) /temp(3,i);
            end
            ercorr= mean(sqrt(sum(temp - Xc).^2));
            y     = tempy;
            error = sum(tempy);
            best_id = jjj;
        end
    end
end