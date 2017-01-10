clear all; clc; close all;

data = importdata('data_log_ex4_32_cw.dat');

figure;
plot(data(:,5),data(:,6));
title('Exercise 4: Clockwise Square')
xlabel('x [m]'); ylabel('y [m]');

data = importdata('data_log_ex4_32_ccw.dat');

figure;
plot(data(:,5),data(:,6));
title('Exercise 4: Counter Clockwise Square')
xlabel('x [m]'); ylabel('y [m]');