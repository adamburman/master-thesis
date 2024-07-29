function uout = FMINCONControllerForCellSim(currentr, currentx)
    persistent Ad Bd ENom soc_ocv_coefficients R0 deltaT

    if isempty(Ad)
        % Initialize parameters
        Ad = [1 0 0 0;...
              0 9.97688825315994e-120 0 0;...
              0 0.000855949360167465 0.772709770193294 0.190091811880446;...
              0 0.00437443381747106 0.0309729232429332 0.968340552949701];
        Bd = [-3.56125356125356e-06;...
              0.000293756563663899;...
              0.000758974104725855;...
              0.00748289716506867];
        ENom = 992623.375585428;
        soc_ocv_coefficients = [-2.0713 4.5787 -2.7974 0.9751 3.4939]';
        R0 = 0.00292724690915974; % Specify actual R0
        deltaT = 1; % Specify actual deltaT
    end

    % Optimization horizon
    N = 40;

    % Objective function and constraints setup
    x = currentx;
    u0 = 10.4*ones(N, 1);
    
    % Define cost function for fmincon
    costFunction = @(u) objectiveFunction(u, x, N, currentr, Ad, Bd, ENom, soc_ocv_coefficients, R0, deltaT);
    nonlcon = @(u) constraints(u, x, N, Ad, Bd, ENom, soc_ocv_coefficients, R0, deltaT);
    
    % Set optimization options
    options = optimoptions('fmincon', 'Display', 'final', 'Algorithm', 'sqp');
    
    % Define constraints
    lb = zeros(N, 1);   % Lower bound on input
    ub = 40 * ones(N, 1); % Upper bound on input
    A = zeros(2 * (N - 1), N);
    b = ones(2 * (N - 1), 1);
    
    % For I(k+1) - I(k) <= 1 and I(k) - I(k+1) <= 1
    for k = 1:(N - 1)
        A(k, k) = -1;
        A(k, k + 1) = 1;
    
        A(N - 1 + k, k) = 1;
        A(N - 1 + k, k + 1) = -1;
    end
    
    % Use fmincon to solve the optimization problem
    options = optimoptions('fmincon', 'Display', 'off', ...
    'MaxFunctionEvaluations', 10 * N, ...
    'MaxIterations', 1e4, ...
    'UseParallel', true);
    u_opt = fmincon(costFunction, u0, A, b, [], [], lb, ub, nonlcon, options);
    
    % if exitflag <= 0
    %     error('Optimization did not converge.');
    % end

    % Return the first control action
    uout = u_opt(1);
end

function J = objectiveFunction(u, x0, N, r, Ad, Bd, ENom, soc_ocv_coefficients, R0, deltaT)
    % Initialize variables
    Q = diag([10]);
    R = 1e-5;
    terminalPenalty = 1e2;
    Cd = [1 0 0 0 0];
    
    x = x0;
    J = 0;
    
    % Simulate the system over the horizon
    for k = 1:N
        % State space update
        x_next(1:4,1) = Ad*x(1:4) + Bd*u(k);
        vT = (x(1).^(4:-1:0)) * soc_ocv_coefficients - u(k)*R0 - x(2);
        x_next(5,1) = x(5) - vT*u(k)*deltaT/ENom;
        
        % Update objective
        J = J + (r - Cd*x_next)'*Q*(r - Cd*x_next) + u(k)'*R*u(k);
        if k == N
            J = J + terminalPenalty*x_next(5);
        end
        % Advance state
        x = x_next;
    end
end

function [c, ceq] = constraints(u, x0, N, Ad, Bd, ENom, soc_ocv_coefficients, R0, deltaT)
    % Initialize variables
    x = x0;
    
    % Constraints
    c = []; % Inequality constraints (c <= 0)
    ceq = []; % Equality constraints (ceq == 0)
    
    % Define temperature limits
    % Tmin = 0;  % Minimum allowable temperature
    Tmax = 40; % Maximum allowable temperature
    
    % Simulate the system over the horizon
    for k = 1:N
        % State space update
        x_next(1:4,1) = Ad * x(1:4) + Bd * u(k);
        vT = (x(1).^(4:-1:0)) * soc_ocv_coefficients - u(k)*R0 - x(2);
        x_next(5,1) = x(5) - vT*u(k)*deltaT/ENom;

        % Add inequality constraints for temperature
        c = [c; x_next(4) - Tmax];
        
        % Advance state
        x = x_next;
    end
end

% x = x0;
% N = 800;
% figure(4); clf; 
% titleStrings = {'SoC', 'V1', 'Ts', 'Tc', 'SoE'};
% MPCSolution = [];
% for k = 1:N
%     MPCSolution(k) = FMINCONControllerForCellSim(0, x(:,k));
%     x(:,k+1) = DPBatteryDynamics(x(:,k), MPCSolution(k), 1);
%     pause(0.01)
% 
%     for i = 1:5
%         subplot(6, 1, i);
%         plot(x(i, :));
%         title(titleStrings{i});
%     end
%     subplot(6, 1, 6);
%     plot(MPCSolution(:));
%     title('MPC Solution');
% end
% %%
% 
% figure(4); clf; titleStrings = {'SoC', 'V1', 'Ts', 'Tc', 'SoE'};
% for i = 1:5
%     subplot(6, 1, i);
%     plot(x(i, :));
%     title(titleStrings{i});
% end
% subplot(6, 1, 6);
% plot(MPCSolution);
% title('MPC Solution');
