% test = simulate_model([62.7 4.5 15 1.94 4.1897 0.0099131 3e-4 12.6138], input_current);
% plot(test)

% Gradient descent optimization

opt_params = gradient_descent(input_current)
test = simulate_model(opt_params,input_current);
function optimized_parameters = gradient_descent(input_data)
    % Initialize parameters
    parameters = [62.7 4.5 15 1.94 4.1897 0.0099131 3e-4 12.6138];
    
    % Set hyperparameters
    learning_rate = .2;
    max_iterations = 100;
    tolerance = 1e-6;
    
    % Optimization loop
    for i = 1:max_iterations
        % Compute gradient
        gradient = compute_gradient_numerically(parameters, input_data);
        
        % Update parameters
        parameters = parameters - learning_rate * gradient;
        
        % Check convergence
        if norm(gradient) < tolerance
            print('converged')
            break;
        end
    end
    
    optimized_parameters = parameters;
end


%%%%%%%%%%%%%%%%
function output = simulate_model(parameters,input_current)
Cn = 78; % Ah
Cc = parameters(1); % J/K
Cs = parameters(2); % J/K
Ru = parameters(3); % K/W
Rc = parameters(4); % K/W
Tf = 20; % Centrigrade
Em = parameters(5);
R0 = parameters(6);



R1 = parameters(7);
C1 = parameters(8);

% Ac = [0 0 0 0;...
%     -R1/C1 0 0 0;...
%     ]


% Operating point, starting with TcBar = 40 Centigrade
TcBar = 40;
TsBar = (Ru*TcBar + Rc*Tf) / (Ru+Rc);
IBar = sqrt(Cc*(TcBar - TsBar) / (Rc*Cc*(R1+R0)));
V1Bar = R1*IBar;

% fprintf(sprintf('Operating points:\nV1 = %.4f\nTs = %.4f\nTc = %.4f\nI = %.4f\n', V1Bar, TsBar, TcBar, IBar))

syms SoC V1 Ts Tc I
x = [SoC V1 Ts Tc]';
u = I;

%function out = f(x, u)
    % x = soc, V1, Ts, Tc
    f = [-1/(3600*Cn)*I;...
        -R1/C1*V1;...
        (Tf-Ts)/(Ru*Cs) - (Ts - Tc)/(Rc*Cs);...
        (Ts-Tc)/(Rc*Cc) + I*(V1 + R0*I)];

    h = [SoC; Tc; I];
%end

Al = double(subs(jacobian(f, x'), I, IBar));
Bl = double(subs(jacobian(f, u),[I V1], [IBar V1Bar]));
Cl = double(jacobian(h, x'));
Dl = double(jacobian(h, u));

lSys = ss(Al,Bl,Cl,Dl);
ldSys = c2d(lSys, 1);
x0 = [1 0 Tf Tf]';    

Y = lsim(ldSys, input_current, 1:length(input_current), x0);
output = Y(:,2)';
end

function simulated_data = objective_function(parameters, experimental_data, input_data)
    % parameters: vector of parameters to be estimated
    % experimental_data: vector of experimental temperature data
    % input_data: input vector for pulse discharge
    
    % Simulate the state-space model using the current parameters and input data
    simulated_data = simulate_model(parameters, input_data);
    
    % Compute the error between simulated and experimental temperature data
    %error = norm(simulated_data - experimental_data);
end

function gradient = compute_gradient_numerically(parameters, input_data)
    num_params = numel(parameters);
    gradient = zeros(size(parameters));
    
    epsilon = 1e-2;
    for i = 1:num_params
        % Perturb the parameter value
        params_plus = parameters;
        params_plus(i) = params_plus(i) + epsilon;
        
        params_minus = parameters;
        params_minus(i) = params_minus(i) - epsilon;
        
        % Compute objective function values
        % obj_plus = objective_function(params_plus);
        % obj_minus = objective_function(params_minus);
        obj_plus = simulate_model(params_plus, input_data);
        obj_minus = simulate_model(params_minus, input_data);

        obj_plus = obj_plus(:,2)';
        obj_minus = obj_minus(:,2)';

        % Compute the numerical gradient approximation
        gradient(i) = (obj_plus - obj_minus) / (2 * epsilon);
    end
end
