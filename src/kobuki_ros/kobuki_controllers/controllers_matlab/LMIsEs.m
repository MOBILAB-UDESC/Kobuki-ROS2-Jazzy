function [K] = LMIsEs(A, B)

    n  = length(A);
    nu = size(B, 2);

    Q  = sdpvar(n, n);
    Y  = sdpvar(nu, n);

    % LMIs com mudança de variáveis
    % Q > 0
    % QA' + AQ + Y'B' + BY < 0
    LMIs=[];
    LMIs=[LMIs,Q>=0];
    LMIs=[LMIs,A*Q+Q*A'+B*Y+Y'*B'<=0];

    % Solver
    options = sdpsettings;
    options.verbose = 0;
    % options.solver  = 'sedumi';
    solvesdp(LMIs,[],options);

    % Cálculo da matriz de ganhos
    Q=double(Q);
    Y=double(Y);
    K=-Y/Q;
end