PF_table = readtable('Interpolated_data.xlsx');
anchorlocations = readtable('UWB Device Locations AdNav.csv');
table = readtable("PF_var_2.5_Associate_10.csv");


plot(table.x, table.y, '-o', 'MarkerSize', 2.5); 
hold on; 

plot(PF_table.LiDAR_x, PF_table.LiDAR_y, 'o-', 'MarkerSize', 2.5); % Plot with circles as markers

plot(anchorlocations.x, anchorlocations.y, 'o', 'MarkerSize', 5);

R = eul2rotm([anchorlocations.psi, anchorlocations.theta, anchorlocations.phi], 'ZYX');

% Adding arrows for anchor direction
for i = 1:length(anchorlocations.x)
    % Extract the rotation matrix for the current point
    R_current = R(:,:,i);
    
    % Define a fixed vector representing the initial direction (e.g., facing east)
    v = [1; 0; 0];
    
    % Transform the fixed vector to the local coordinate system
    local_v = R_current * v;
    
    % Extract the local x and y components of the transformed vector
    local_x = local_v(1);
    local_y = local_v(2);
    
    % Plot the arrow
    quiver(anchorlocations.x(i), anchorlocations.y(i), local_x, local_y, 'LineWidth', 1, 'MaxHeadSize', 1);
end


xlabel('X');
ylabel('Y');
title('Align PF UWB and LiDAR Trajectories');
grid on;

legend('UWB Trajectory (PF Results)', 'LiDar Trajectory (Ground Truth)');



