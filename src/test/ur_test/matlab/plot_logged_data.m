clc;
close all;
clear;

addpath('utils/');

filename = '../data/logged_data.bin';
fid = fopen(filename);
if (fid < 0), error('Faile to open %s\n', filename); end

Time = read_mat(fid);
Time = (Time - Time(1))/1000;
dq_data = read_mat(fid);
% dq_target_data = read_mat(fid);
dq_cmd_data = read_mat(fid);

for i=1:2

    figure;
    hold on;
    plot(dq_data(i,:), 'LineWidth',1.5, 'Color','blue');
%     plot(dq_target_data(i,:), 'LineWidth',1.5, 'Color','cyan');
    plot(dq_cmd_data(i,:), 'LineWidth',1.5, 'Color','magenta');
    hold off;
    
end

for i=1:2

    figure;
    hold on;
    plot(Time, dq_data(i,:), 'LineWidth',1.5, 'Color','blue');
%     plot(dq_target_data(i,:), 'LineWidth',1.5, 'Color','cyan');
    plot(Time, dq_cmd_data(i,:), 'LineWidth',1.5, 'Color','magenta');
    hold off;
    
end


Ts = diff(Time);
figure;
plot(Ts)
mean(Ts)
std(Ts)
