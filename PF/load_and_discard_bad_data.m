close all;

data_table = readtable('Interpolated_data.xlsx');

T = data_table.timestamp';
X = data_table.UWB_x';
Y = data_table.UWB_y';

process_data(X,Y,T)