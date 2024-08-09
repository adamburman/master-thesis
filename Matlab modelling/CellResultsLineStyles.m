% Define the strategies and load the data
labels = {'CC-CV', 'CC-CT', 'PI', 'DP', 'TubeMPC'};
datasets = {CCCV, CCCT, PI, DP, TubeMPC};

% Load cubehelix colormap
colormap('cubehelix');
colorMap = colormap;

% Initialize line styles for black-and-white differentiation
lineStyles = {'-', '--', ':', '-.'};

% Define threshold temperature
thresholdTemp = 40;

% Initialize a figure with subplots for demonstration
numStrategies = length(labels);

%% Plotting State of Energy (SoE) vs. Time
figure;
hold on;

% Plot each strategy's SoE
for i = 1:numStrategies
    timeData = datasets{i}{2}.Time;
    SoE = datasets{i}{2}.Data(:, 1); % Extracting SoE

    plot(timeData, SoE, 'Color', colorMap(i,:), ...
         'LineStyle', lineStyles{mod(i-1, length(lineStyles))+1}, ...
         'DisplayName', labels{i}, 'LineWidth', 1.5);
end

xlabel('Time (s)');
ylabel('State of Energy (SoE)');
title('State of Energy vs. Time');
legend('show', 'Location', 'best');
grid on;
hold off;

%% Plotting Temperature (Surface and Core) vs. Time
figure;
subplot(2,1,1);
hold on;

% Plot each strategy's temperature
for i = 1:numStrategies
    timeData = datasets{i}{2}.Time;
    temperatures = datasets{i}{2}.Data(:, 2); % Extracting temperature
    
    aboveThreshold = temperatures > thresholdTemp;
    
    % Below threshold
    plot(timeData(~aboveThreshold), temperatures(~aboveThreshold), ...
         'Color', colorMap(i,:), ...
         'LineStyle', lineStyles{mod(i-1, length(lineStyles))+1}, ...
         'DisplayName', [labels{i} ' (Below 40°C)'], 'LineWidth', 1.5);
    
    % Above threshold
    plot(timeData(aboveThreshold), temperatures(aboveThreshold), ...
         'Color', [1, 0, 0], ...
         'LineStyle', lineStyles{mod(i-1, length(lineStyles))+1}, ...
         'DisplayName', [labels{i} ' (Above 40°C)'], 'LineWidth', 1.5);
end

xlabel('Time (s)');
ylabel('Temperature (°C)');
title('Surface Temperature vs. Time');
legend('show', 'Location', 'best');
grid on;
hold off;

subplot(2,1,2);
hold on;

% Plot each strategy's core temperature (same as surface here for example)
for i = 1:numStrategies
    timeData = datasets{i}{2}.Time;
    coreTemperatures = datasets{i}{2}.Data(:, 2); % Assuming same data

    aboveThreshold = coreTemperatures > thresholdTemp;
    
    % Below threshold
    plot(timeData(~aboveThreshold), coreTemperatures(~aboveThreshold), ...
         'Color', colorMap(i,:), ...
         'LineStyle', lineStyles{mod(i-1, length(lineStyles))+1}, ...
         'DisplayName', [labels{i} ' (Below 40°C)'], 'LineWidth', 1.5);
    
    % Above threshold
    plot(timeData(aboveThreshold), coreTemperatures(aboveThreshold), ...
         'Color', [1, 0, 0], ...
         'LineStyle', lineStyles{mod(i-1, length(lineStyles))+1}, ...
         'DisplayName', [labels{i} ' (Above 40°C)'], 'LineWidth', 1.5);
end

xlabel('Time (s)');
ylabel('Core Temperature (°C)');
title('Core Temperature vs. Time');
legend('show', 'Location', 'best');
grid on;
hold off;

%% Plotting Current vs. Time
figure;
hold on;

% Plot each strategy's current
for i = 1:numStrategies
    timeData = datasets{i}{2}.Time;
    current = datasets{i}{2}.Data(:, 3); % Extracting current

    plot(timeData, current, 'Color', colorMap(i,:), ...
         'LineStyle', lineStyles{mod(i-1, length(lineStyles))+1}, ...
         'DisplayName', labels{i}, 'LineWidth', 1.5);
end

xlabel('Time (s)');
ylabel('Current (A)');
title('Current vs. Time');
legend('show', 'Location', 'best');
grid on;
hold off;

%% Highlighting Temperature Exceedances for One Strategy
figure;
hold on;

% Load temperature data for Tube-based MPC
tubeTime = TubeMPC{2}.Time;
tubeTemp = TubeMPC{2}.Data(:, 2);

% Find indices where temperature exceeds threshold
exceedIdx = find(tubeTemp > thresholdTemp);

% Plot temperature
plot(tubeTime, tubeTemp, 'Color', colorMap(numStrategies,:), ...
     'LineStyle', '-', 'LineWidth', 1.5, 'DisplayName', 'Tube-based MPC Temperature');

% Add vertical lines at exceedance points
for idx = exceedIdx'
    xline(tubeTime(idx), 'r--', 'LineWidth', 1.2);
end

xlabel('Time (s)');
ylabel('Temperature (°C)');
title('Temperature Exceedance for Tube-based MPC');
legend('show', 'Location', 'best');
grid on;
hold off;

%% Subplots for SoE, Surface Temperature, Core Temperature, and Current
figure;

%% Subplot 1: State of Energy (SoE) vs. Time
subplot(2,2,1);
hold on;

% Plot each strategy's SoE
for i = 1:numStrategies
    timeData = datasets{i}{2}.Time;
    SoE = datasets{i}{2}.Data(:, 1); % Extracting SoE

    plot(timeData, SoE, 'Color', colorMap(i,:), ...
         'LineStyle', lineStyles{mod(i-1, length(lineStyles))+1}, ...
         'DisplayName', labels{i}, 'LineWidth', 1.5);
end

xlabel('Time (s)');
ylabel('State of Energy (SoE)');
title('SoE vs. Time');
legend('show', 'Location', 'best');
grid on;
hold off;

%% Subplot 2: Temperature vs. Time
subplot(2,2,2);
hold on;

% Plot each strategy's temperature
for i = 1:numStrategies
    timeData = datasets{i}{2}.Time;
    temperatures = datasets{i}{2}.Data(:, 2); % Extracting temperature

    aboveThreshold = temperatures > thresholdTemp;
    
    % Below threshold
    plot(timeData(~aboveThreshold), temperatures(~aboveThreshold), ...
         'Color', colorMap(i,:), ...
         'LineStyle', lineStyles{mod(i-1, length(lineStyles))+1}, ...
         'DisplayName', [labels{i} ' (Below 40°C)'], 'LineWidth', 1.5);
    
    % Above threshold
    plot(timeData(aboveThreshold), temperatures(aboveThreshold), ...
         'Color', [1, 0, 0], ...
         'LineStyle', lineStyles{mod(i-1, length(lineStyles))+1}, ...
         'DisplayName', [labels{i} ' (Above 40°C)'], 'LineWidth', 1.5);
end

xlabel('Time (s)');
ylabel('Temperature (°C)');
title('Temperature vs. Time');
legend('show', 'Location', 'best');
grid on;
hold off;

%% Subplot 3: Core Temperature vs. Time
subplot(2,2,3);
hold on;

% Plot each strategy's core temperature
for i = 1:numStrategies
    timeData = datasets{i}{2}.Time;
    coreTemperatures = datasets{i}{2}.Data(:, 2); % Assuming same data

    aboveThreshold = coreTemperatures > thresholdTemp;
    
    % Below threshold
    plot(timeData(~aboveThreshold), coreTemperatures(~aboveThreshold), ...
         'Color', colorMap(i,:), ...
         'LineStyle', lineStyles{mod(i-1, length(lineStyles))+1}, ...
         'DisplayName', [labels{i} ' (Below 40°C)'], 'LineWidth', 1.5);
    
    % Above threshold
    plot(timeData(aboveThreshold), coreTemperatures(aboveThreshold), ...
         'Color', [1, 0, 0], ...
         'LineStyle', lineStyles{mod(i-1, length(lineStyles))+1}, ...
         'DisplayName', [labels{i} ' (Above 40°C)'], 'LineWidth', 1.5);
end

xlabel('Time (s)');
ylabel('Core Temperature (°C)');
title('Core Temperature vs. Time');
legend('show', 'Location', 'best');
grid on;
hold off;

%% Subplot 4: Current vs. Time
subplot(2,2,4);
hold on;

% Plot each strategy's current
for i = 1:numStrategies
    timeData = datasets{i}{2}.Time;
    current = datasets{i}{2}.Data(:, 3); % Extracting current

    plot(timeData, current, ...
         'Color', colorMap(i,:), ...
         'LineStyle', lineStyles{mod(i-1, length(lineStyles))+1}, ...
         'DisplayName', labels{i}, 'LineWidth', 1.5);
end

xlabel('Time (s)');
ylabel('Current (A)');
title('Current vs. Time');
legend('show', 'Location', 'best');
grid on;
hold off;

% Adjust figure size
set(gcf, 'Position', [100, 100, 1200, 800]);

% Optimize subplot layout
sgtitle('Battery Discharge Analysis Across Different Strategies');
