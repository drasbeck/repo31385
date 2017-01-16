clear all; clc; close all;

data = importdata('log.dat');
w = 0.267439;
t = 0:0.010:(length(data)-1)*0.010;

figure;
v = abs(data(:,3)-data(:,4))./2
plot(t,v)

figure;
plot(data(:,5),data(:,6));
xlabel('x [m]')
ylabel('y [m]')

figure
subplot(3,1,1)
plot(t,data(:,5))
xlabel('t [s]')
ylabel('x [m]')
subplot(3,1,2)
plot(t,data(:,6))
xlabel('t [s]')
ylabel('y [m]')
subplot(3,1,3)
plot(t,wrapToPi(data(:,7)))
xlabel('t [s]')
ylabel('\theta [rad]')

figure;
contourf(data(:,17:24)')


minls = [47.8166, 47.8304, 49.2446, 48.5621, 48.6193, 48.6982, 48.8462, 49.5325];
maxls = [59.3748, 63.0730, 64.4773, 65.8383, 66.5010, 66.6174, 64.9901, 64.2327];
[m,n] = size(data(:,18:25))
for k = 1:m
   for l = 1:n
       ils(k,l) = (data(k,l+17) - minls(l)) / (maxls(l) - minls(l));
   end
end

figure;
surf(1-ils','EdgeColor','None');
view(2);
colormap(gray);