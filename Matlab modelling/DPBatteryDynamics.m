function xNext = DPBatteryDynamics(x, u, deltaT)
    % Extract states
    SoC = x(1);
    SoE = x(2);
    V1 = x(3);
    Ts = x(4);
    Tc = x(5);
    x = x(:);
    % Extract parameters (if needed)
    Cn = 78; %2.3; % Ah
    Cc = 62.7; % J/K
    Cs = 4.5; % J/K
    Ru = 15; % K/W
    Rc = 1.94; % K/W
    Tf = 20; % Centrigrade
    Em = 4.1897;
    R0 = 0.0099131;
    R1 = 3e-4;
    C1 = 12.6138;
    ENom =  992623.375585428;

    soc_ocv_coefficients = [-2.0713 4.5787 -2.7974 0.9751 3.4939]';

    Ad = [1 0 0 0;...
        0 9.97688825315994e-120 0 0;...
        0 0.000855949360167465 0.772709770193294 0.190091811880446;...
        0 0.00437443381747106 0.0309729232429332 0.968340552949701];
    Bd = [-3.56125356125356e-06;...
            0.000293756563663899;...
            0.000758974104725855;...
            0.00748289716506867];
    Cd = [1 0 0 0;0 0 1 0];

    xNext = zeros(5,1);

    xNext(1:4) = Ad*x(1:4) + Bd*u;
    vT = (x(1).^(4:-1:0))*soc_ocv_coefficients - u*R0 - x(2);
    xNext(5) = x(5) - vT*u*deltaT/ENom;
    % % Define function handles for the differential equations
    % f = @(x, u) [
    %         -1/(3600*Cn)*u;...  % SoC
    %     -u*x(3)                 % SoE
    %     -x(3)/(R1*C1) + u/C1;...  % V1  
    %     (x(5)-x(4))/(Ru*Cs) - (x(4) - x(5))/(Rc*Cs);... % Ts
    %     (x(4)-x(5))/(Rc*Cc) + u*(x(3) + R0*u)/Cc   % Tc                   % Placeholder for dTc/dt
    % ];
end

% % For the state space and SoE development
% x0 = [1 1e-3 20 20 1]';
% x = x0;
% for k = 2:3600
%     x(:,k) = BatteryDynamics(x(:,k-1), IBar, 1);
% end
% 
% figure(1);clf;
% for i = 1:5
%     subplot(5,1,i)
%     plot(x(i,:))
% end


% 
% % Parameters
% deltaT = 1; % Time step
% x0 = [1; 0; 20; 20; 1]; % Initial state vector [SoC, V1, Ts, Tc, SoE]
% Tmax = 50; % Maximum temperature
% Imin = 0; % Minimum current
% Imax = 40; % Maximum current
% N0 = 1000; % Initial guess for number of steps
% 
% % Initial guess for optimization variables [I]
% I0 = zeros(N0, 1);
% 
% % Define bounds for optimization variables
% lb = Imin * ones(N0, 1);
% ub = Imax * ones(N0, 1);
% 
% % Objective function
% obj = @(I) DPCostFunction(I, deltaT, x0);
% 
% % Constraints
% nonlcon = @(I) DPConstraints(I, deltaT, x0, Tmax, Imin, Imax);
% 
% % Optimization options
% options = optimoptions('fmincon', 'Algorithm', 'sqp', 'Display', 'iter', 'StepTolerance', 1e-6, 'ConstraintTolerance', 1e-6);
% 
% % Run the optimization
% [I_opt, J_opt] = fmincon(obj, I0, [], [], [], [], lb, ub, nonlcon, options);
% 
% % Extract optimized current profile
% N_opt = J_opt; % The optimized number of steps is the cost function value
% I_opt = I_opt(1:N_opt); % Trim the current profile to the optimized number of steps
% 
% % Display results
% disp(['Optimized number of steps: ', num2str(N_opt)]);
% disp('Optimized current profile:');
% disp(I_opt);


% % x = x0;
% x = [0.978632478632473 0.011750262546556 31.5188321009912 37.8717190874606]';% 0.977261663416445]';
% N = 80;
% figure(4); clf; 
% titleStrings = {'SoC', 'V1', 'Ts', 'Tc', 'SoE'};
% MPCSolution = [];
% for k = 1:N
%     % r = exp(-.1*10^(-3)*(k+150))*100*0;
%     % r = 20-5*sin(1/(60)*(k-1));
%     r = [0;40];
%     MPCSolution(k) = MPCControllerForCellSim(r, x(:,k), k-1);
%     temp = DPBatteryDynamics([x(:,k);1], MPCSolution(k), 1);
%     x(:,k+1) = temp(1:4);
%     pause(0.01)
% 
%     for i = 1:4
%         subplot(6, 1, i);
%         plot(x(i, :));
%         title(titleStrings{i});
%     end
%     subplot(6, 1, 6);
%     plot(MPCSolution(:));
%     title('MPC Solution');
% end
% 
% figure(1); clf; hold on;
% I = repelem(initialGuess, holdTime);
% plot(repelem(initialGuess, holdTime), '--k', 'linewidth', 1);
% plot(DPModuleSolution, 'r', 'LineWidth', 1);
% legend('Initial guess', 'Solver output');
% 
% x0 = [1 0 20 20]';
% x = x0;
% 
% for k = 2:5000
%     x(:, k) = linsys5s.A*x(1:4, k - 1) + linsys5s.B*I(5*k);
% end
% 
% figure(2); clf; titleStrings = {'SoC', 'V1', 'Ts', 'Tc', 'SoE'};
% for i = 1:5
%     subplot(6, 1, i);
%     plot(x(i, :),'LineWidth',1);
%     title(titleStrings{i});
% end
% subplot(6, 1, 6);
% plot(DPModuleSolution);
% title('DP Solution');
