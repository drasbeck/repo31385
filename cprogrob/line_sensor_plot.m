close all; clc; clear all;

data = importdata('data_log.dat');

figure;
hold on;
contourf(data(:,1:8)')
colorbar;
%plot((data(:,9)'+0.063)*8/0.13,'mx')
hold off;