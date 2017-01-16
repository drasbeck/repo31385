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
subplot(3,1,1)
plot(t,data(:,5))
subplot(3,1,2)
plot(t,data(:,6))
subplot(3,1,3)
plot(t,wrapToPi(data(:,7)))