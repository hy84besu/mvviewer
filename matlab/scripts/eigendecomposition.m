function [V,D] = eigendecomposition(L,noEigs)

    options.disp = 0;
    [V1, D1] = eigs(L,noEigs,'sa',options);
    V = V1(:,end:-1:1);
    D = D1(end:-1:1,end:-1:1);

end