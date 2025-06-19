function [K, Y, Q] = LMIsDEs(A, B, reg, r, c, alpha, phi, QQ, YY)

    % Get dimensions
    n   = length(A);
    nu  = size(B, 2);

    Q  = sdpvar(n, n);
    Y  = sdpvar(nu, n);

    % LMIs D-Stability
    % ∃Y, Q > 0 : L ⊗ Q + M ⊗ (AQ + BY) + M′ ⊗ (Q'A′ + Y′B′) < 0,
    LMIs=[];
    LMIs=[LMIs,Q>=0];

    % --------------------- REGIONS -----------------------
    % D = {s ∈ C : L + sM + s∗M′ < 0},
    % s = σ + jω

    regions = split(reg, '-');

    % Apply constraints for each region
    for i = 1:length(regions)
        switch regions{i}
            case "disc"
                L = [-r c;c -r];
                M = [0 1;0 0];
            case "plane"
                L = 2*alpha;
                M = 1;
            case "cone"
                L = zeros(2,2);
                M = [sin(phi) cos(phi); -cos(phi) sin(phi)];
        end
        LMIs = [LMIs, (kron(L,Q) + kron(M,(A*Q+B*Y)) + kron(M',(Q'*A'+Y'*B'))) <= 0];
    end

    % mu = [0.4; 1.91];

    % LMIs = [LMIs, [Q, Y(1,:)'; Y(1,:), mu(1)^2] >= 0];
    % LMIs = [LMIs, [Q, Y(2,:)'; Y(2,:), mu(2)^2] >= 0];

    % Solver configuration
    options = sdpsettings;
    options.verbose = 0;
    options.solver = 'sedumi';
    options.shift=1e-3;
    options.tol=1e-2;
    options.sedumi.eps = 1e-6;        % Relaxed precision for speed
    options.cachesolvers = 1;         % Cache solver for speed

    % Use warm start if previous solution exists
    if ~isempty(QQ) && any(QQ(:) ~= 0)
        assign(Q, QQ);
        assign(Y, YY);
    end

    % Solve with error handling
    try
        result = solvesdp(LMIs, [], options);
        
        if result.problem == 0
            % Successful solve
            Q = double(Q);
            Y = double(Y);
            K = -Y/Q;
        else
            % Fallback to previous solution or simple gain
            warning('LMI solver failed, using fallback solution');
            if ~isempty(QQ) && any(QQ(:) ~= 0)
                Q = QQ;
                Y = YY;
                K = -Y/Q;
            else
                % Simple fallback gain
                Q = eye(n);
                Y = -[1, 0, 0; 0, 0, 1];
                K = -Y/Q;
            end
        end
    catch ME
        warning('LMI computation error: %s', ME.message);
        % Use previous solution or simple gain
        if ~isempty(QQ) && any(QQ(:) ~= 0)
            Q = QQ;
            Y = YY;
            K = -Y/Q;
        else
            Q = eye(n);
            Y = -[1, 0, 0; 0, 0, 1];
            K = -Y/Q;
        end
    end

end
