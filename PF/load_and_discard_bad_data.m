close all;

data_table = readtable('synced_data_output.xlsx');

T = data_table.UWB_timestamp';
X = data_table.UWB_x';
Y = data_table.UWB_y';

process_data(X,Y,T)