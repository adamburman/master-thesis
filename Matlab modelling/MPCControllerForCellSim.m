function uout = MPCControllerForCellSim(currentx,t)

persistent Controller

if t == 0
    % Compute discrete-time dynamics
    % Plant = ss(tf(1,[1 0 0]));
    % A = Plant.A;
    % B = Plant.B;
    % C = Plant.C;
    % D = Plant.D;
    % Ts = 0.1;
    % Gd = c2d(Plant,Ts);
    % Ad = Gd.A;
    % Bd = Gd.B;

    Ad = [1, 0, 0, 0; 
     0, 0.999976355162077, 0, 0; 
     0, 0.123485441953946, 0.772709770193292, 0.190091811880446; 
     0, 1.21727203076061, 0.0309729232429332, 0.968340552949701];
    Bd = [-3.56125356125356e-06; 0; 0.000759225545468568; 0.00748418218371487]; % Without correct + I/C1 term
    Ts = 1;
    [nx, nu] = size(Bd);
    
    % Define data for MPC controller
    N = 30;
    Q = diag([1 0 0 0]);
    
    % Avoid explosion of internally defined variables in YALMIP
    yalmip('clear')
    
    % Setup the optimization problem
    u = sdpvar(repmat(nu,1,N),repmat(1,1,N));
    x = sdpvar(repmat(nx,1,N+1),repmat(1,1,N+1));
    % pastu = sdpvar(1);
    % sdpvar r
    % Define simple standard MPC controller
    % Current state is known so we replace this
    % constraints = [-10 <= diff([pastu u{:}]) <= 10];
    constraints = [];
    objective = 0;
    for k = 1:N
        objective = objective + x{k}'*Q*x{k};
        constraints = [constraints, x{k+1} == Ad*x{k}+Bd*u{k}];
    constraints = [constraints, 0 <= u{k} <= 40];
    constraints = [constraints, 0 <= x{k+1}(3) <= 40]; % Constraint on the third state (T_s)
    constraints = [constraints, 0 <= x{k+1}(4) <= 40]; % Constraint on the fourth state (T_c)
    end
    
    % Define an optimizer object which solves the problem for a particular
    % initial state and reference
    % Force optimizer to turn on dispay until you know things work
    % To see log you have to use debug breaks in code and run manually
    ops = sdpsettings('verbose',2,'solver','mosek')
    Controller = optimizer(constraints,objective,ops,x{1},u{1});
    
    % And use it here 
    [uout,problem] = Controller(currentx);
    % uout = uout{1};
    if problem
       % Fix!
    end
    
else    
    % Almost no overhead
    [uout,problem] = Controller(currentx);
    % uout = uout{1};
    if problem
      % Debug, analyze, fix!
    end 
end