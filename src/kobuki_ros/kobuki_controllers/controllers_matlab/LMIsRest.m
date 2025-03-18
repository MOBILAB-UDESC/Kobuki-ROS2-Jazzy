function [K, Y, Q] = LMIsRest(A, B, x, v_max, w_max, QQ, YY)
    % Verificar dimensiones
    n  = length(A);          % Número de estados
    nu = size(B, 2);         % Número de entradas

    % Definir variables de decisión
    Q = sdpvar(n, n, 'symmetric'); % Matriz Q simétrica definida positiva
    Y = sdpvar(nu, n);             % Matriz Y intermedia

    % Parámetros de diseño
    mu = [v_max; w_max];     % Límites en las variables de control
    % alpha = 0.35;            % Parámetro para D-estabilidad
    % L = 2 * alpha;           % Matriz L para D-estabilidad
    % M = 1;                   % Matriz M para D-estabilidad

    Cz  = diag([5, 3, 1]);
    Dz  = [2, 0;
            0, 1;
            0, 0];
    Inz = eye(size(Dz, 1));
    lb  = sdpvar(1);

    % Restricciones LMIs
    LMIs = [];
    LMIs = [LMIs, Q >= 0];                            % Q debe ser definida positiva
    LMIs = [LMIs, A*Q + Q*A' + B*Y + Y'*B' <= 0];   % Estabilidad del sistema
    LMIs = [LMIs, [1, x; x', Q] >= 0];              % Restricción en el estado inicial
    LMIs = [LMIs, [Q, Y'; Y, diag(mu.^2)] >= 0];    % Restricción en las variables de control
    % LMIs = [LMIs, (kron(L,Q)+kron(M,(A*Q+B*Y))+kron(M',(Q*A'+Y'*B')))<=0]; % D-estabilidad
    LMIs=[LMIs,[lb x; x' Q]>=0];
    LMIs=[LMIs,[A*Q+Q*A'+B*Y+Y'*B', Q*Cz'+Y'*Dz'; Cz*Q+Dz*Y, -Inz]<=0];

    % Asignar valores iniciales
    assign(Q, QQ);
    assign(Y, YY);

    % Opciones del solver
    options = sdpsettings;
    options.verbose = 0;
    options.solver  = 'sedumi';
    options.shift   = 1e-3;
    options.tol     = 1e-2;

    % Resolver el problema de optimización
    sol = solvesdp(LMIs,[lb],options);

    % Cálculo da matriz de ganhos
    Q=double(Q);
    Y=double(Y);
    K=-Y/Q;
end
