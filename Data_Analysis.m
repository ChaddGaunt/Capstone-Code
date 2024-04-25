results = readtable('Combined_results.xlsx');

pf_data = readtable('PF_Trial_1_var_0.75.csv');

Trial_number = 1;
variance = 0.75;

LiDAR_x = results.LiDAR_x;
LiDAR_y = results.LiDAR_y;
Trial_x = results.Trial1_x;
Trial_y = results.Trial1_y;

timestamps = results.timestamp;

%% Trajectory Plot 
hold on;

plot(LiDAR_x, LiDAR_y);
plot(Trial_x, Trial_y);

xlabel('X');
ylabel('Y');
title(['PF Trial ', num2str(Trial_number), ': UWB vs LiDAR Trajectories (var ', num2str(variance), ')']);
grid on;

legend('LiDAR Trajectory (Ground Truth)', 'UWB Trajectory (PF Result)');

%% Distance Error Plot
hold on;
n = size(results, 1);
distance_vector = zeros(n, 1);

for i = 1:n
    % Calculate Euclidean distance between each pair of points
    distance_vector(i) = sqrt((LiDAR_x(i) - Trial_x(i))^2 + (LiDAR_y(i) - Trial_y(i))^2);
end

mean_distanceError = mean(distance_vector);

% Given POSIX timestamp
posix_timestamp = timestamps;

% Convert POSIX timestamp to datetime object in UTC time zone
utc_time = datetime(posix_timestamp, 'ConvertFrom', 'posixtime', 'TimeZone', 'UTC');

% Convert UTC time to Australian Sydney time
sydney_time = datetime(utc_time, 'TimeZone', 'Australia/Sydney', 'Format', 'dd/MM/yy HH:mm:ss.SSS');

plot(sydney_time, distance_vector);
plot(sydney_time, ones(size(sydney_time)) * mean_distanceError, 'r', 'MarkerSize', 10);

xlim([min(sydney_time), max(sydney_time)]);

xlabel('Time');
ylabel('Distance Error (m)');
title(['PF Trial ', num2str(Trial_number), ': Distance Error Over Time']);
grid on;

legend('Distance Error', ['Mean Error (', num2str(mean_distanceError), ' m)']);

%% RMSE Plot
hold on

% Sample RMSE values
rmse = rmse([Trial_x,Trial_y],[LiDAR_x,LiDAR_y]);

% Plotting the vertical bar chart with custom colors
b = bar(["X", "Y"], [rmse]);

b.FaceColor = 'flat';
b.CData(1,:) = [0 0.4470 0.7410];
b.CData(2,:) = [0.8500 0.3250 0.0980];

% Adding labels and title
ylabel('RMSE');
title(['PF Trial ', num2str(Trial_number), ': Overal Root Mean Square Error (RMSE) of X and Y Values']);

%% Velocity Plot
hold on
p = size(pf_data, 1);
trial_velocity = zeros(p, 1);

for i = 1:p
    % Calculate Euclidean distance between each pair of points
    trial_velocity(i) = sqrt((pf_data.vx(i)^2)+(pf_data.vy(i)^2));
end

x_position = LiDAR_x; 
y_position = LiDAR_y;
timestamp = timestamps;

% Calculate displacement
dx = diff(x_position); % Change in x position
dy = diff(y_position); % Change in y position
dt = diff(timestamp); % Change in time

% Calculate velocity components
vx = dx ./ dt; % x velocity
vy = dy ./ dt; % y velocity

LiDAR_velocity = sqrt(vx.^2 + vy.^2);

% Given POSIX timestamp
posix_timestamp = timestamps;

% Convert POSIX timestamp to datetime object in UTC time zone
utc_time = datetime(posix_timestamp, 'ConvertFrom', 'posixtime', 'TimeZone', 'UTC');

% Convert UTC time to Australian Sydney time
sydney_time = datetime(utc_time, 'TimeZone', 'Australia/Sydney', 'Format', 'dd/MM/yy HH:mm:ss.SSS');

average_LiDAR_velocity = mean(LiDAR_velocity)
average_trial_velocity = mean(trial_velocity)

hold on;

subplot(1,2,1); % Create a subplot with 1 row and 2 columns, and select the first subplot
plot([sydney_time(2:end),sydney_time(2:end)], [LiDAR_velocity, ones(size(sydney_time(2:end))) * average_LiDAR_velocity]);
xlabel('Time');
ylabel('m/s');
subtitle('Ground Truth Velocity');
xlim([min(sydney_time), max(sydney_time)]);
legend('Ground Truth Velocity', ['Mean Velocity (', num2str(average_LiDAR_velocity), ' m/s)']);
grid on;

% Plot PF velocity
subplot(1,2,2); % Select the second subplot
plot([sydney_time, sydney_time], [trial_velocity, ones(size(sydney_time)) * average_trial_velocity]);
xlabel('Time');
ylabel('m/s');
subtitle('PF Velocity');
xlim([min(sydney_time), max(sydney_time)]);
legend('PF Velocity', ['Mean Velocity (', num2str(average_trial_velocity), ' m/s)']);
grid on;

% Set the same x-axis limits for both subplots
xlim([min(sydney_time), max(sydney_time)]);


hold off; % Release the plot



