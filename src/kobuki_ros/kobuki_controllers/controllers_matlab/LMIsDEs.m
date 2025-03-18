function [K, Y, Q] = LMIsDEs(A, B, reg, QQ, YY)

    n  = length(A);
    nu = size(B, 2);

    Q  = sdpvar(n, n);
    Y  = sdpvar(nu, n);

    % --------------------- REGIÕES -----------------------
    % D = {s ∈ C : L + sM + s∗M′ < 0},
    % s = σ + jω

    % Disco
    % Ld      = [-1 1.5;1.5 -1];
    Ld      = [-2.5 2.5;2.5 -2.5]; % -r c; c -r
    Md      = [0 1;0 0];
    % Cone
    phi     = pi/7;
    Lc      = zeros(2,2);
    Mc      = [sin(phi) cos(phi); -cos(phi) sin(phi)];
    % Semi-plano
    % alpha   = 0.8;
    alpha   = 0.25;
    Lsp     = 2*alpha;
    Msp     = 1;

    if reg == "disco"
        L       = Ld;
        M       = Md;
    elseif reg == "plano"
        L       = Lsp;
        M       = Msp;
    elseif reg == "cono"
        L       = Lc;
        M       = Mc;
    end

    % LMIs para D-Estabilidade
    % ∃Y, Q > 0 : L ⊗ Q + M ⊗ (AQ + BY) + M′ ⊗ (QA′ + Y′B′) < 0,
    LMIs=[];
    LMIs=[LMIs,Q>=0];
    LMIs=[LMIs,(kron(L,Q)+kron(M,(A*Q+B*Y))+kron(M',(Q*A'+Y'*B')))<=0];

    % Solver
    assign(Q,QQ);
    assign(Y,YY);
    options = sdpsettings;
    options.verbose = 0;
    options.solver = 'sedumi';
    options.shift=1e-3;
    options.tol=1e-2;
    solvesdp(LMIs,[],options);

    % Cálculo da matriz de ganhos
    Q=double(Q);
    Y=double(Y);
    K=-Y/Q;
end
