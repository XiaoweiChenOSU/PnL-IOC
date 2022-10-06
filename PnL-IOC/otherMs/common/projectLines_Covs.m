function [XXw_s, XXw_e, xx_s_n, xx_e_n, ll, sigmas2dlines] = projectLines_Covs(nl, nlsamples, nln, xx_s, xx_e, XXw_s, XXw_e, f, K)
%     K = [f 0 0
%          0 f 0
%          0 0 1];

%     K = [180 0 320;0 180 320;0   0   1];

    nnl_ln = round(nln * nlsamples);
    nls = zeros(1, nln);        
    id  = 1;
    for idnl = 1:length(nl)
        sigma = nl(idnl);
        nls(id:id+nnl_ln(idnl)-1) = sigma .* ones(1,nnl_ln(idnl));
        id = id+nnl_ln(idnl);
    end                                

    % since len(nls) = len(nln) we can reuse the sigmas 
    line_thr = 25;
    ln_ids = [];
    dx = xx_s - xx_e;
    dxn = sqrt(sum(dx.^2, 1));
    line_mask = dxn > line_thr;
    ln_ids = find(line_mask>0);
    if size(ln_ids) > 1
        ln_ids = ln_ids(1:nln);
    end

    XXw_s = XXw_s(:, ln_ids);
    XXw_e = XXw_e(:, ln_ids);
    xx_s_n = xx_s(:, ln_ids);
    xx_e_n = xx_e(:, ln_ids);


    ll = zeros(3, nln);
    Sigmas2DLines_n = zeros(3,3,nln);        
    sigmas2dlines = zeros(nln, 1);
    for k = 1:nln            
        Sigmas2DLines_n(:, :, k) = get_line2D_covariance(eye(2)*nls(k)^2/f^2, eye(2)*nls(k)^2/f^2, [xx_s_n(:, k)/f;1], [xx_e_n(:, k)/f; 1]);
        sigmas2dlines(k) = nls(k)^2/f^2;
        l_h = cross(K\[xx_s_n(:,k);1], K\[xx_e_n(:,k); 1]);
        ll(:, k) = l_h/norm(l_h(1:2));
    end
end