yalmip('clear')

% Model data
Ad = [1 0 0 0;...
        0 9.97688825315994e-120 0 0;...
        0 0.000855949360167465 0.772709770193294 0.190091811880446;...
        0 0.00437443381747106 0.0309729232429332 0.968340552949701];
Bd = [-3.56125356125356e-06;...
        0.000293756563663899;...
        0.000758974104725855;...
        0.00748289716506867];
Cd = [1 0 0 0;0 0 1 0];
Ts = 1;
[nx, nu] = size(Bd);

% Prediction horizon
N = 40;
Q = diag([100 100]);
R = 1;

% States x(k), ..., x(k+N)
x = sdpvar(repmat(nx,1,N+1),repmat(1,1,N+1));
% Inputs u(k), ..., u(k+N) (last one not used)
u = sdpvar(repmat(nu,1,N),repmat(1,1,N));
% Binary for PWA selection
% d = binvar(repmat(2,1,N),repmat(1,1,N));

constraints = [];
objective = 0;
r = [0; 40];

for k = 1:N  

    % Feasible region
    constraints = [constraints , 0 <= u{k} <= 40,
                                 %-1 <= (u{k}-u{k+1}) <= 1,
                                 %0 <= x{k}(1) <= 1,
                                 0 <= x{k}(3) <= 50,
                                 %0 <= x{k+1}(1) <= 1,
                                 0 <= x{k+1}(3)   <= 50,
                                 x{k+1} == Ad*x{k} + Bd*u{k}];
    if k < N
        constraints = [constraints, -1 <= (u{k+1}-u{k}) <= 1];
    end
    % PWA Dynamics
    % constraints = [constraints ,implies(d{k}(1),x{k+1} == A*x{k}+B1*u{k}),
    %                             implies(d{k}(2),x{k+1} == A*x{k}+B2*u{k});
    %                             implies(d{k}(1),x{k}(1) >= 0),
    %                             implies(d{k}(2),x{k}(1) <= 0)];

    % It is EXTREMELY important to add as many
    % constraints as possible to the binary variables
    % constraints = [constraints, sum(d{k}) == 1];

    % Add stage cost to total cost
    objective = objective + (r - Cd*x{k})'*Q*(r - Cd*x{k}) + u{k}'*R*u{k}; %objective + 1000*norm([0;40] - Cd*x{k},1) + norm(u{k},1);
end

ops = sdpsettings('verbose',2);
[sol,diagn,Z,Valuefcn,Optimizer] = solvemp(constraints,objective ,ops,x{1},u{1});
% figure(1);plot(Valuefcn)

%%

explicit_states = x0;


for k = 1:1000
    assign(x{1}, explicit_states(:, k));
    explicit_inputs(k) = value(Optimizer);
    explicit_states(:, k+1) = Ad*explicit_states(:, k) + Bd*explicit_inputs(k);

end

figure(1);clf;
plot(explicit_states(3,:))
figure(2);clf;
plot(explicit_inputs)