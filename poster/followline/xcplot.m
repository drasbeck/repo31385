clear all; clc; close all;

data = importdata('log.dat');

%minls = [47.8166, 47.8304, 49.2446, 48.5621, 48.6193, 48.6982, 48.8462, 49.5325];
%maxls = [59.3748, 63.0730, 64.4773, 65.8383, 66.5010, 66.6174, 64.9901, 64.2327];

x = data(:,1);
y = data(:,2);
xc = data(:,4);
lsi = data(:,5:12);
dist = sqrt(x.*x+y.*y);

[X,Y] = meshgrid(dist,(-3.5:1:3.5).*13./7);

figure;
hold on;
contourf(X,Y,1-lsi','EdgeColor','None');
view(2);
colormap(gray);
plot(dist,(xc.*13./7),'m-')
axis([-0.1 1.1 -8 8])
xlabel('Driven Distance [m]')
ylabel('Sensor Distance [cm]')
hold off;