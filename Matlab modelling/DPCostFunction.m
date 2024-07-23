function J = DPCostFunction(I)
    x(:,1) = [1 0 20 20 1]';
    N = length(I);
    deltaT = 1;
    for k = 2:N
        x(:, k) = DPBatteryDynamics(x(:, k-1), I(k-1), deltaT);
    end
    % Minimize steps to reach SoE <= 0.95 (or adjust threshold as needed)
    J = find(x(5,:) <= 0.95, 1, 'first');
    J = J.^2;
    if isempty(J)
        J = N.^2;  % No solution found, penalize with maximum steps
    end
end
