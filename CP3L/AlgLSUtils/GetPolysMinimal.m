function p = GetPolysMinimal(n,e)
% this file is generated by MinimalCexp.mws
for k = 1:3

    n1 = n(1,k);
    n2 = n(2,k);
    n3 = n(3,k);
    e1 = e(1,k);
    e2 = e(2,k);
    e3 = e(3,k);

    new_a = [n1 * e1 + n3 * e3 + n2 * e2 -2 * n2 * e3 + 2 * n3 * e2 2 * n1 * e3 - 2 * n3 * e1 -2 * n1 * e2 + 2 * n2 * e1 -n2 * e2 - n1 * e1 + n3 * e3 -n2 * e2 + n1 * e1 - n3 * e3 -n3 * e3 - n1 * e1 + n2 * e2 2 * n1 * e2 + 2 * n2 * e1 2 * n3 * e2 + 2 * n2 * e3 2 * n1 * e3 + 2 * n3 * e1];
    p(k).coef = new_a;
    EQexp = [0 1 0 0 0 2 0 1 0 1; 0 0 1 0 0 0 2 1 1 0; 0 0 0 1 2 0 0 0 1 1;];
    p(k).exp = EQexp;
    p(k).m = 10;
end