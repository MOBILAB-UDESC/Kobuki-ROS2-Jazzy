function [K, Y, Q, lb] = LMIsCustoGarantido(A, B, x, QQ, YY, llb, Cz, Dz)

    % Get dimensions
    n   = length(A);
    nu  = size(B, 2);
    
    % Ensure x is a column vector
    x   = x(:);

    % Cz  = diag([5, 5, 2]);
    % Dz  = [3, 0; 0, 1; 0, 0];

    Inz = eye(size(Dz, 1));

    % Define LMI variables
    Q   = sdpvar(n, n);
    Y   = sdpvar(nu, n);
    lb  = sdpvar(1);

    % Construct guaranteed cost LMIs
    LMIs = [];
    LMIs = [LMIs, [lb, x'; x, Q] >= 0];
    LMIs = [LMIs, [A*Q+Q*A'+B*Y+Y'*B', Q'*Cz'+Y'*Dz'; Cz*Q+Dz*Y, -Inz] <= 0];

    % Solver configuration
    options = sdpsettings;
    options.verbose = 0;
    options.solver = 'sedumi';
    options.sedumi.eps = 1e-6;        % Relaxed precision for speed
    % options.sedumi.maxiter = 50;      % Limit iterations
    % options.sedumi.theta = 0.9;       % Faster convergence
    options.cachesolvers = 1;         % Cache solver for speed

    % Use warm start if previous solution exists
    if ~isempty(QQ) && any(QQ(:) ~= 0)
        assign(Q, QQ);
        assign(Y, YY);
    end
    
    % Solve with error handling
    try
        result = solvesdp(LMIs, lb, options);
        
        if result.problem == 0
            % Successful solve
            Q = double(Q);
            Y = double(Y);
            K = -Y/Q;
            lb = double(lb);
        else
            % Fallback to previous solution or simple gain
            warning('LMI solver failed, using fallback solution');
            if ~isempty(QQ) && any(QQ(:) ~= 0)
                Q = QQ;
                Y = YY;
                K = -Y/Q;
                lb = llb;
            else
                % Simple fallback gain
                Q = eye(n);
                Y = -[1, 0, 0; 0, 0, 1];
                K = -Y/Q;
                lb = 1;
            end
        end
    catch ME
        warning('LMI computation error: %s', ME.message);
        % Use previous solution or simple gain
        if ~isempty(QQ) && any(QQ(:) ~= 0)
            Q = QQ;
            Y = YY;
            K = -Y/Q;
            lb = llb;
        else
            Q = eye(n);
            Y = -[1, 0, 0; 0, 0, 1];
            K = -Y/Q;
            lb = 1;
        end
    end
end
