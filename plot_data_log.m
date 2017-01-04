clear all; clc; close all;

data = importdata('data_log.dat');

figure;
subplot(2,1,1);
plot(data(:,3));
title('Right Motor Speed')
grid on;
xlabel('k'); ylabel('Speed [m/s]');
ax = axis; axis([ax(1) ax(2) -0.4 0.4])
subplot(2,1,2);
plot(data(:,4));
title('Left Motor Speed')
grid on;
xlabel('k'); ylabel('Speed [m/s]');
ax = axis; axis([ax(1) ax(2) -0.4 0.4])

figure;
subplot(2,1,1);
plot(data(:,5));
title('Odometry, x')
grid on;
xlabel('k'); ylabel('x [m]');
ax = axis; axis([ax(1) ax(2) -0.5 1.5])
subplot(2,1,2);
plot(data(:,6));
title('Odometry, y')
grid on;
xlabel('k'); ylabel('y [m]');
ax = axis; axis([ax(1) ax(2) -1.5 0.5])

figure;
subplot(2,1,1);
plot(data(:,7));
title('Odometry, \theta')
grid on;
xlabel('k'); ylabel('\theta [rad]');
ax = axis; axis([ax(1) ax(2) -2*2*pi-pi/2 0+pi/2])