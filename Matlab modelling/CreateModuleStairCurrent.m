% Initialize parameters
num_cycles = 1;    % Number of cycles
duration_39 = 3600*4; % Duration of 39 Amperes in seconds
rest_time = duration_39;   % Additional rest time after each cycle in seconds (can be changed)

% Initialize vectors
moduleParameterTime = [];
moduleParameterCurrent = [];

% Initial time
current_time = 0;

% Loop to generate the profile
for cycle = 1:num_cycles
    % Append 39 Amperes segment
    moduleParameterTime = [moduleParameterTime, current_time, current_time + duration_39];
    moduleParameterCurrent = [moduleParameterCurrent, 39, 39];
    
    % Increment the current time by 1440 seconds
    current_time = current_time + duration_39;
    
    % Calculate the rest period to the next nearest hour
    next_hour = ceil(current_time / 3600) * 3600;
    rest_duration = next_hour - current_time;
    
    % Append zero current segment with rest_time added
    moduleParameterTime = [moduleParameterTime, current_time + 1, current_time + rest_duration + rest_time];
    moduleParameterCurrent = [moduleParameterCurrent, 0, 0];
    
    % Update current time for the next cycle
    current_time = current_time + rest_duration + rest_time;
end

% Adjust to ensure no repeated time values
moduleParameterTime = moduleParameterTime + (0:length(moduleParameterTime)-1);

% Plot the current profile
figure(1);clf;
plot(moduleParameterTime, moduleParameterCurrent, 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Current (A)');
title('Current Profile');
grid on;

%%

clear exportVar
%%
tempTime = exportVar{1}.Values.Time;
temp1 = exportVar{1}.Values.Data(:,1);
temp2 = exportVar{1}.Values.Data(:,2);
temp3 = exportVar{1}.Values.Data(:,3);
temp4 = exportVar{1}.Values.Data(:,4);
figure(1);clf;plot(tempTime,temp4)