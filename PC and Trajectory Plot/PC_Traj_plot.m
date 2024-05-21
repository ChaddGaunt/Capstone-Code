hold on;
raw_data = readtable('Interpolated_data.xlsx');
anchorlocations = readtable('UWB Device Locations AdNav.csv');

PCdata = pcread('AdNav_Experiment3_PC_Level (No Floor).ply');

% Extract X, Y, and Z coordinates
X = PCdata.Location(:,1);
Y = PCdata.Location(:,2);
Z = PCdata.Location(:,3);


anchor1_indices = strcmp(raw_data.UWB_Anchor, '00280038:3136510B:34393732');
anchor1_x = raw_data.UWB_x(anchor1_indices);
anchor1_y = raw_data.UWB_y(anchor1_indices);
plot(anchor1_x, anchor1_y, 'o', 'MarkerSize', 4);


anchor2_indices = strcmp(raw_data.UWB_Anchor, '0047001D:3136510B:34393732');
anchor2_x = raw_data.UWB_x(anchor2_indices);
anchor2_y = raw_data.UWB_y(anchor2_indices);
plot(anchor2_x, anchor2_y, 'o', 'MarkerSize', 4);


anchor4_indices = strcmp(raw_data.UWB_Anchor, '002C0044:3136510B:34393732');
anchor4_x = raw_data.UWB_x(anchor4_indices);
anchor4_y = raw_data.UWB_y(anchor4_indices);
plot(anchor4_x, anchor4_y, 'o', 'MarkerSize', 4);


anchor5_indices = strcmp(raw_data.UWB_Anchor, '00440037:3136510B:34393732');
anchor5_x = raw_data.UWB_x(anchor5_indices);
anchor5_y = raw_data.UWB_y(anchor5_indices);
plot(anchor5_x, anchor5_y, 'o', 'MarkerSize', 4);


anchor7_indices = strcmp(raw_data.UWB_Anchor, '0040002D:3136510B:34393732');
anchor7_x = raw_data.UWB_x(anchor7_indices);
anchor7_y = raw_data.UWB_y(anchor7_indices);
plot(anchor7_x, anchor7_y, 'o', 'MarkerSize', 4);

%plot(raw_data.LiDAR_x, raw_data.LiDAR_y, 'k-', 'LineWidth', 2.5);

scatter(X, Y, 5, 'k', 'filled', 'MarkerFaceAlpha', 0.1); % Transparent grey color

R = eul2rotm([anchorlocations.psi, anchorlocations.theta, anchorlocations.phi], 'ZYX');


anchor_order = [1,5,2,7,4];
%color_order = ["#EDB120", "#0072BD", "#7E2F8E", "#A2142F", "#77AC30"];
color_order = ['y', 'b', 'm', 'r', 'g'];

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

    % Choose the color for the current triangle
    color_index = mod(i, length(color_order)) + 1; % Cycling through the color order
    color = color_order(color_index);
    %quiver(anchorlocations.x(i), anchorlocations.y(i), local_x, local_y, 'LineWidth', 1, 'MaxHeadSize', 1, 'Color', color);
    ang = atan2(local_y,local_x);
    ang = ang+pi;

    fprintf(1,'%d: Angle %4.2f\n',i,ang+pi)
    txt = sprintf("%d",anchor_order(i));
    Veh = vehicle([anchorlocations.x(i), anchorlocations.y(i), ang]);
    patch(Veh(1:3), Veh(4:6),  color,'FaceAlpha', 0.5);
    text(anchorlocations.x(i)+0.4, anchorlocations.y(i),txt);

end


xlabel('X');
ylabel('Y');
title('Raw UWB Trajectory');
grid on;

legend('Anchor 1', 'Anchor 2', 'Anchor 4', 'Anchor 5', 'Anchor 7');



