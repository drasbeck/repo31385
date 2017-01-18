clear all; clc; close all;

data = importdata('logrlf.dat');
t = 0:0.010:(length(data)-1)*0.010;

figure;
plot(data(:,1),data(:,2));
xlabel('x [m]')
ylabel('y [m]')
axis([0 2.2 -0.6 0.6])
grid on;

figure
subplot(3,1,1)
plot(t,data(:,1))
xlabel('t [s]')
ylabel('x [m]')
grid on;
subplot(3,1,2)
plot(t,data(:,2))
xlabel('t [s]')
ylabel('y [m]')
grid on;
subplot(3,1,3)
plot(t,wrapToPi(data(:,3)))
xlabel('t [s]')
ylabel('\theta [rad]')
grid on;