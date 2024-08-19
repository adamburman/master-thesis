function uout = TubeBasedMPCControllerForModuleSim(currentr, currentx, t)

persistent Controller

if t == 0
    % Compute discrete-time dynamics
    % Corrected state-space using Simulink linearization from differential equations
    %5s
Ad = [
    1, 0, 0, 0;
    0, 0.120021235615572, 0, 0;
    0, 0.000190551076740806, 0.965145286127985, 0.0131775127675528;
    0, 0.0216617463294798, 0.00016242439059922, 0.999835773098934
];

% Define the vector 'Bd'
Bd = [
   -8.9031339031339e-06;
    0.00179387580363326;
    9.56368038926026e-07;
    0.000158727031983218
];
    
    ENom =  24521235.1893945;

    soc_ocv_coefficients = [-24.9533 55.1604 -33.8647 11.9152 41.5814]';
    R0 = 0.000127631043388972;
    deltaT = 5;

    Cd = [0 0 0 0 1; 0 0 1 0 0];
    terminalPenaltyMatrix = [0 0 0 0 1];
    Ts = 1;
    [nx, nu] = size(Bd);

    % Define data for MPC controller
    N = 150; % Prediction horizon
    Q = diag([1e4 1e2]); % Weight for states in objective
    R = 1; % Weight for control input in objective
    W = 1e4; % Weight for terminal state

    % Robust tube-based MPC parameters
    disturbanceBound = .4; % Maximum expected disturbance magnitude for each state

    % Control gain K for robust control using dlqr
    Q_K = diag([1e4 0 1e2 0]); % Q for calculating control gain K
    R_K = R; % Same R for calculating control gain K
    K = dlqr(Ad, Bd, Q_K, R_K); % Control gain

    % Closed-loop dynamics
    A_cl = Ad + Bd * K;

    % Compute maximum deviation due to disturbances
    disturbanceImpact = zeros(nx, 1);
    for i = 1:nx
        disturbanceImpact(i) = sum(abs(A_cl(:,i))) * disturbanceBound;
    end

    % Avoid explosion of internally defined variables in YALMIP
    yalmip('clear')

    % Setup the optimization problem
    u = sdpvar(repmat(nu,1,N),repmat(1,1,N));
    x = sdpvar(repmat(nx+1,1,N+1),repmat(1,1,N+1)); % Include fifth state for SoE
    r = sdpvar(2,1); % Reference for the system output
    constraints = [];
    objective = 0;

    for k = 1:N
        % Objective function
        objective = objective + (r - Cd*x{k})'*Q*(r - Cd*x{k}) + u{k}'*R*u{k};

        % State transition constraints
        % Include the SoE (fifth state) evolution separately
        vT = (x{k}(1).^(4:-1:0))*soc_ocv_coefficients - u{k}*R0 - x{k}(2);
        soeEvolution = x{k}(5) - vT*u{k}*deltaT/ENom;
        constraints = [constraints, x{k+1}(1:4) == Ad*x{k}(1:4)+Bd*u{k}; x{k+1}(5) == soeEvolution];

        if k < N
            % Apply tube-based constraint tightening
            % Constraints on control input changes with disturbance impact
            constraints = [constraints, -1 <= (u{k+1} - u{k}) <= 1];
        else
            % Terminal cost for the last state
            objective = objective + (terminalPenaltyMatrix*x{k})'*W*(terminalPenaltyMatrix*x{k});
        end

        % Apply constraints on the control input
        constraints = [constraints, 0 <= u{k} <= 40];

        % Apply constraints on the third state (T_s) considering disturbance
        constraints = [constraints, x{k+1}(4) <= 40 - disturbanceImpact(4)];

        % Apply constraints on the SoE (fifth state)
        % constraints = [constraints, 0 <= x{k+1}(5) <= 100]; % Example bounds for SoE
    end

    % Define an optimizer object which solves the problem for a particular
    % initial state and reference
    ops = sdpsettings('verbose', 1, 'solver', 'mosek');
    Controller = optimizer(constraints, objective, ops, {r, x{1}}, u{1});

    % And use it here 
    [uout,problem] = Controller({currentr, currentx});
    if problem
        % Handle optimization problems
        % disp('Optimization problem occurred at initialization!');
        uout = 0; % Default output if problem occurs
    end

else    
    % Almost no overhead
    [uout,problem] = Controller({currentr, currentx});
    if problem
        % Handle optimization problems
        % disp('Optimization problem occurred during runtime!');
        uout = 0; % Default output if problem occurs
    end 
end
