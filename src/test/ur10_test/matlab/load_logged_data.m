clc;
close all;
clear;

addpath('utils/');

binary = true;
filename = '../data/logged_data';
fid = fopen(filename);
if (fid < 0)
    error('Could not load %s\n', filename);
end

Time = read_mat(fid, binary);
q_data = read_mat(fid, binary);
dq_data = read_mat(fid, binary);
pos_data = read_mat(fid, binary);
Q_data = read_mat(fid, binary);
V_data = read_mat(fid, binary);
wrench_data = read_mat(fid, binary);
jTorques_data = read_mat(fid, binary);

save('logged_data.mat','Time','q_data','dq_data','pos_data','Q_data','V_data','wrench_data','jTorques_data');
