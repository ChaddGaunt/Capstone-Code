% Load data from Excel file
data = readtable('sync_data.xlsx');

% Extract UWB and LiDAR data
uwb_timestamp = data.UWB_timestamp;
uwb_xyz = [data.UWB_x, data.UWB_y, data.UWB_z];
lidar_timestamp = data.LiDAR_timestamp;
lidar_xyz = [data.LiDAR_x, data.LiDAR_y, data.LiDAR_z];

% Remove NaN values from lidar_timestamp
lidar_timestamp = lidar_timestamp(~isnan(lidar_timestamp));

% Remove NaN values from lidar_xyz
lidar_xyz = lidar_xyz(~isnan(lidar_xyz(:,1)), :);

% Determine the time range covered by the LiDAR data
LiDAR_time_range = [min(lidar_timestamp), max(lidar_timestamp)];

% Filter UWB data within the time range of LiDAR data
valid_uwb_indices = uwb_timestamp >= LiDAR_time_range(1) & uwb_timestamp <= LiDAR_time_range(2);
uwb_timestamp = uwb_timestamp(valid_uwb_indices);
uwb_xyz = uwb_xyz(valid_uwb_indices, :);

% Interpolate UWB data to match timestamps of LiDAR data
interpolated_lidar_xyz = interp1(lidar_timestamp, lidar_xyz, uwb_timestamp, 'spline');

% Define column headings
column_headings = {'Timestamps', 'UWB_x', 'UWB_y', 'UWB_z', 'LiDAR_x', 'LiDAR_y', 'LiDAR_z'};

% Combine UWB and interpolated LiDAR data
combined_data = [uwb_timestamp, uwb_xyz, interpolated_lidar_xyz];

% Convert combined_data to a table
combined_table = array2table(combined_data, 'VariableNames', column_headings);

% Write the table to a CSV file
csv_filename = 'UPDATED_Interpolated_Data.xlsx';
writetable(combined_table, csv_filename);

disp(['Combined data has been saved to ', csv_filename]);

PF_table = readtable(csv_filename);

plot(PF_table.UWB_x, PF_table.UWB_y, '-o', 'MarkerSize', 2.5); % Plot with circles as markers
hold on; % Hold the current plot so that subsequent plots are added to it

% Plot the second line
% Assuming you have x and y data in 'table'
plot(PF_table.LiDAR_x, PF_table.LiDAR_y, 'o-', 'MarkerSize', 2.5); % Plot with circles as markers
