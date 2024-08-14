function uout = TubeBasedMPCControllerForModuleSim(currentr, currentx, t)

persistent Controller

if t == 0
    % Compute discrete-time dynamics
    % Corrected state-space using Simulink linearization from differential equations
    Cn = 156; %2.3; % Ah
    Cc = 2059.88437317355; % J/K
    Cs = 31.8813725211613; % J/K
    Ru = 8.86466905698817; % K/W
    Rc = 14.4801919510372; % K/W
    Tf = 20; % Centrigrade
    Em = 4.1897;
    R0 = 0.000127631043388972;
    R1 = 0.00360795873336597;
    C1 = 174.354594213373;
    ENom =  24521235.1893945;

    soc_ocv_coefficients = [-24.9533 55.1604 -33.8647 11.9152 41.5814]';

    Ad = [1 0 0 0;...
          0 0.203994363431329 0 0;...
          0 4.32226620293953e-06 0.994311777865903 0.00215994783323213;...
          0 0.00318808231636143 3.34300810251095e-05 0.999966510718794];
    Bd = [-1.78062678062678e-06;...
          0.00287195548826647;...
          7.39694567424815e-08;...
          7.13485533636988e-05];
    deltaT = 1;

    Cd = [1 0 0 0 0; 0 0 1 0 0];
    terminalPenaltyMatrix = [0 0 0 0 1];
    Ts = 1;
    [nx, nu] = size(Bd);

    % Define data for MPC controller
    N = 40; % Prediction horizon
    Q = diag([100 100]); % Weight for states in objective
    R = 1; % Weight for control input in objective
    W = 1000; % Weight for terminal state

    % Robust tube-based MPC parameters
    disturbanceBound = .4; % Maximum expected disturbance magnitude for each state

    % Control gain K for robust control using dlqr
    Q_K = diag([100 0 100 0]); % Q for calculating control gain K
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
            constraints = [constraints, -1 + disturbanceImpact' <= (u{k+1} - u{k}) <= 1 - disturbanceImpact'];
        else
            % Terminal cost for the last state
            objective = objective + (terminalPenaltyMatrix*x{k})'*W*(terminalPenaltyMatrix*x{k});
        end

        % Apply constraints on the control input
        constraints = [constraints, 0 <= u{k} <= 40];

        % Apply constraints on the third state (T_s) considering disturbance
        constraints = [constraints, 0 + disturbanceImpact(3) <= x{k+1}(3) <= 50 - disturbanceImpact(3)];

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
