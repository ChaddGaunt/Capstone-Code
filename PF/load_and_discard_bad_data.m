close all;

data_table = readtable('Interpolated_data.xlsx');
anchorlocations = readtable('UWB Device Locations AdNav.csv');

T = data_table.timestamp';
X = data_table.UWB_x';
Y = data_table.UWB_y';
anchor_ID = data_table.UWB_Anchor;

process_data(X,Y,T, anchor_ID, anchorlocations)