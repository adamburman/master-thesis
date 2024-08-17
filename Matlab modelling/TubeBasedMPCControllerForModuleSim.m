function uout = TubeBasedMPCControllerForModuleSim(currentr, currentx, t)

persistent Controller

if t == 0
    % Compute discrete-time dynamics
    % Corrected state-space using Simulink linearization from differential equations
    Cn = 156; % Ah
    Cc = 2059.88437317355; % J/K
    Cs = 31.8813725211613; % J/K
    Ru = 8.86466905698817; % K/W
    Rc = 14.4801919510372; % K/W
    Tf = 20; % Centigrade
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
    deltaT = 100;

                             Ad = [1 0 0 0;
                         0 9.15833994693067e-70 0 0;
                         0 0.000656952709364753 0.56552138106102 0.164795504405946;
                         0 0.00399305972173024 0.00255058338914636 0.996955204802277];
Bd = [-0.000178062678062678 0.00360795873336597 0.000219600401057448 0.00244140524681879]';

    Cd = [0 0 0 0 1; 0 0 0 1 0];
    terminalPenaltyMatrix = [0 0 0 0 1];
    Ts = 1;
    [nx, nu] = size(Bd);

    % Define data for MPC controller
    N = 150; % Prediction horizon
    Q = diag([1e2 1e3]); % Weight for states in objective
    R = 1; % Weight for control input in objective
    W = 1e3; % Weight for terminal state

    % Robust tube-based MPC parameters
    disturbanceBound = 1; % Maximum expected disturbance magnitude for each state

    % Control gain K for robust control using dlqr
    Q_K = diag([1e2 0 1e3 0]); % Q for calculating control gain K
    R_K = R; % Same R for calculating control gain K
    K = dlqr(Ad, Bd, Q_K, R_K); % Control gain

    % Closed-loop dynamics
    A_cl = Ad + Bd * K;

    % Compute maximum deviation due to disturbances (tube size)
    disturbanceImpact = zeros(nx, 1);
    for i = 1:nx
        disturbanceImpact(i) = sum(abs(A_cl(:,i))) * disturbanceBound;
    end

    % Avoid explosion of internally defined variables in YALMIP
    yalmip('clear')

    % Setup the optimization problem
    u = sdpvar(repmat(nu,1,N),repmat(1,1,N));
    x_nom = sdpvar(repmat(nx+1,1,N+1),repmat(1,1,N+1)); % Nominal state, include SoE (fifth state)
    x_hat = sdpvar(repmat(nx+1,1,N+1),repmat(1,1,N+1)); % Deviation from nominal state
    r = sdpvar(2,1); % Reference for the system output
    constraints = [];
    objective = 0;

    for k = 1:N
        % Objective function (on nominal states)
        objective = objective + (r - Cd*x_nom{k})'*Q*(r - Cd*x_nom{k}) + u{k}'*R*u{k};

        % State transition constraints for nominal state
        vT_nom = (x_nom{k}(1).^(4:-1:0))*soc_ocv_coefficients - u{k}*R0 - x_nom{k}(2);
        soeEvolution_nom = x_nom{k}(5) - vT_nom*u{k}*deltaT/ENom;
        constraints = [constraints, x_nom{k+1}(1:4) == Ad*x_nom{k}(1:4)+Bd*u{k}; x_nom{k+1}(5) == soeEvolution_nom];

        % State transition constraints for deviation (tube)
        constraints = [constraints, x_hat{k+1}(1:4) == (Ad + Bd*K)*x_hat{k}(1:4); x_hat{k+1}(5) == x_nom{k+1}(5)];

        if k < N
            % Apply tube-based constraint tightening
            constraints = [constraints, -1 <= (u{k+1}-u{k}) <= 1];
        else
            % Terminal cost for the last nominal state
            objective = objective + (terminalPenaltyMatrix*x_nom{k})'*W*(terminalPenaltyMatrix*x_nom{k});
        end

        % Apply constraints on the control input considering tube
        constraints = [constraints, 0 <= u{k} <= 40];

        % Apply constraints on the third state (T_s) considering disturbance impact
        constraints = [constraints, x_nom{k+1}(4) + disturbanceImpact(4) <= 40];

        % Apply constraints on the SoE (fifth state)
        % constraints = [constraints, 0 <= x_nom{k+1}(5) + disturbanceImpact(5) <= 100]; % Example bounds for SoE
    end

    % Define an optimizer object which solves the problem for a particular
    % initial state and reference
    ops = sdpsettings('verbose', 1, 'solver', 'mosek');
    Controller = optimizer(constraints, objective, ops, {r, x_nom{1}}, u{1});

    % Solve the MPC problem for the initial state
    [uout,problem] = Controller({currentr, currentx});
    if problem
        % uout = -10; % Default output if problem occurs
    end

else    
    % Solve the MPC problem for subsequent time steps
    [uout,problem] = Controller({currentr, currentx});
    if problem
        % uout = -10; % Default output if problem occurs
    end 
end
