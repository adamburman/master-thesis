function uout = MPCControllerForModuleSim(currentr, currentx,t)

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

    % Ad = [1, 0, 0, 0; 
    %  0, 0.999976355162077, 0, 0; 
    %  0, 0.123485441953946, 0.772709770193292, 0.190091811880446; 
    %  0, 1.21727203076061, 0.0309729232429332, 0.968340552949701];
    % Bd = [-3.56125356125356e-06; 0; 0.000759225545468568; 0.00748418218371487]; % Without correct + I/C1 term

    % Corrected state-space using simulink linearization from diff. eq
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
    
    % ENom =  992623.375585428;
    % R0 = 0.0099131;
    % soc_ocv_coefficients = [-2.0713 4.5787 -2.7974 0.9751 3.4939]';
    deltaT = 5;
      Ad = [1 0 0 0;
                         0 9.15833994693067e-70 0 0;
                         0 0.000656952709364753 0.56552138106102 0.164795504405946;
                         0 0.00399305972173024 0.00255058338914636 0.996955204802277];
Bd = [-0.000178062678062678 0.00360795873336597 0.000219600401057448 0.00244140524681879]';
% 500s
Ad = [
    1,    0,    0,    0;
    0,    0,    0,    0;
    0, 0.00141965040460652, 0.0594380419361861, 0.354526999792495;
    0, 0.00395552542977103, 0.00548710767283533, 0.987587991726148
];

Bd = [
   -0.00089031339031339;
    0.00360795873336597;
    0.00310513296692454;
    0.0122021651401997
];

%50s
% Define the matrix 'Ad'
Ad = [
    1, 0, 0, 0;
    0, 6.2027022632556e-10, 0, 0;
    0, 0.00266683367332118, 0.701417227678209, 0.112829501824388;
    0, 0.0245823763965112, 0.0013907224677911, 0.998446753053842
];

% Define the vector 'Bd'
Bd = [
   -8.9031339031339e-05;
    0.00203854442302985;
    0.000115819611395861;
    0.00197730166261187
];

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

    % Lower sample rate
    % Ad = [1 0 0 0;...
    %     0 0 0 0;...
    %     0 0.003094 0.1491 0.6851;...
    %     0 0.003858 0.1116 0.8541];
    % Bd = [-3.561e-05;...
    %     0.0002938;...
    %     0.03789;...
    %     0.06888];


    Cd = [0 0 0 0 1;0 0 1 0 0];
    terminalPenaltyMatrix = [0 0 0 0 1];
    Ts = 1;
    [nx, nu] = size(Bd);

    % Define data for MPC controller
    N = 150;
    Q = diag([1e4 1e2]);
    R = 1e0;
    W = 1e4;

    % Avoid explosion of internally defined variables in YALMIP
    yalmip('clear')

    % Setup the optimization problem
    u = sdpvar(repmat(nu,1,N),repmat(1,1,N));
    x = sdpvar(repmat(nx+1,1,N+1),repmat(1,1,N+1));
    % pastu = sdpvar(1);
    r = sdpvar(2,1);
    % sdpvar r;
    % Define simple standard MPC controller
    % Current state is known so we replace this
    % constraints = [-10 <= diff([pastu u{:}]) <= 10];
    constraints = [];
    objective = 0;
    for k = 1:N
        objective = objective + (r - Cd*x{k})'*Q*(r - Cd*x{k}) + u{k}*R*u{k};
        vT = (x{k}(1).^(4:-1:0))*soc_ocv_coefficients - u{k}*R0 - x{k}(2);
        soeEvolution = x{k}(5) - vT*u{k}*deltaT/ENom;
        constraints = [constraints, x{k+1}(1:4) == Ad*x{k}(1:4)+Bd*u{k}; x{k+1}(5) == soeEvolution];
        
        % constraints = [constraints, -5 <= diff([u{:}]) <= 5];
        % constraints = [constraints, -1 <= (u{k+1} ]
            if k < N
            constraints = [constraints, -1 <= (u{k+1}-u{k}) <= 1];
            else
                objective = objective + (terminalPenaltyMatrix*x{k})'*W*(terminalPenaltyMatrix*x{k});
        end
    constraints = [constraints, 0 <= u{k} <= 40];
    constraints = [constraints, 0 <= x{k+1}(4) <= 40]; % Constraint on the third state (T_s)
    % constraints = [constraints, 0 <= x{k+1}(4) <= 50]; % Constraint on the fourth state (T_c)
    end

    % Define an optimizer object which solves the problem for a particular
    % initial state and reference
    % Force optimizer to turn on dispay until you know things work
    % To see log you have to use debug breaks in code and run manually
    ops = sdpsettings('verbose',1,'solver','mosek');
    Controller = optimizer(constraints,objective,ops,{r, x{1}},u{1});

    % And use it here 
    [uout,problem] = Controller({currentr, currentx});
    % uout = uout{1};
    if problem
        uout = 0;
       % Fix!
    end

else    
    % Almost no overhead
    [uout,problem] = Controller({currentr, currentx});
    % uout = uout{1};
    if problem
        uout = 0;
      % Debug, analyze, fix!
    end 
end