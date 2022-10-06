function [v, Cu, cov, nls] = projectPoints_Covs(xxn, homx, f, nl, nlsamples, K, is_outliers)
    if nargin < 7
       is_outliers = false;
    end
    npt = size(xxn, 2);
   

    % isotropic noise
    nnl = round(npt * nlsamples);
    nls = zeros(1,npt);
    id  = 1;
    for idnl = 1:length(nl)
        sigma = nl(idnl);
        nls(id:id + nnl(idnl) - 1) = sigma .* ones(1,nnl(idnl));
        id = id + nnl(idnl);
    end
    noise_2d_inds = randperm(npt);
    nls  = nls (noise_2d_inds);

    v = normc(homx);

    Cu = zeros(2,2,length(nls));
    cov = zeros(9,size(Cu,3));
%     K = diag([f, f, 1]);
%     K = [180 0 320;0 180 320;0   0   1];

    for id = 1:length(nls(1,:))
        Cu(:,:,id) = [nls(id)^2 0; 0 nls(id)^2]/f/f;
        cov_proj = K\diag([nls(id)^2 nls(id)^2 0])/K';    
        J = (eye(3) - (v(:,id)*v(:,id)') / (v(:,id)'*v(:,id))) / norm(homx(:,id));
        evv = J*cov_proj*J';
        cov(:, id) = reshape(evv, 9, 1);            
    end
    
    if is_outliers
    % add outliers    
        out_ids = randsample(npt, 0.5 * npt);
        xxn(:, out_ids) = rand(2, length(out_ids)) * f;
    end

end