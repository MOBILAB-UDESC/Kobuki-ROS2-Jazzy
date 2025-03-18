function [K, Y, Q, lb] = LMIsCustoGarantido(A, B, x, QQ, YY, lb)

    n   = length(A);
    nu  = size(B, 2);
    
    x   = x(:);
    % Cz  = diag([5, 5, 2]);
    % Dz  = [3, 0; 0, 1; 0, 0];

    Cz  = [sqrt(5) 0 0;0 sqrt(5) 0;0 0 sqrt(5);0 0 0;0 0 0]; % Cz: mx3
    Dz  = [0 0;0 0;0 0;sqrt(1.5) 0;0 sqrt(1.5)]; % Dz: mx2
    Inz = eye(size(Dz, 1));

    Q   = sdpvar(n, n);
    Y   = sdpvar(nu, n);
    lb  = sdpvar(1);

    % LMIs para Custo Garantido
    % ∃Y, Q > 0 : L ⊗ Q + M ⊗ (AQ + BY) + M′ ⊗ (QA′ + Y′B′) < 0,
    LMIs=[];
    % LMIs=[LMIs,Q>=0];
    % LMIs=[LMIs,A*Q+Q*A'+B*Y+Y'*B'<=0];
    LMIs=[LMIs,[lb, x'; x, Q]>=0];
    LMIs=[LMIs,[A*Q+Q*A'+B*Y+Y'*B', Q*Cz'+Y'*Dz'; Cz*Q+Dz*Y, -Inz]<=0];

    % Solver
    assign(Q,QQ);
    assign(Y,YY);
    options = sdpsettings;
    options.verbose = 0;
    options.solver = 'sedumi';
    solvesdp(LMIs,[lb],options);

    % Cálculo da matriz de ganhos
    Q=double(Q);
    Y=double(Y);
    K=-Y/Q;
end
