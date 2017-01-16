clear all; clc; close all;

data = importdata('log.dat');
w = 0.267439;
t = 0:0.010:(length(data)-1)*0.010;

figure;
v = abs(data(:,3)-data(:,4))./2
plot(t,v)

figure;
plot(data(:,5),data(:,6));

figure
[X,Y] = meshgrid(data(:,5),data(:,6))
plot3(data(:,5),data(:,6),v)