clear all; clc; close all;

data = importdata('ucline.dat');
data = data(:,1:8);

figure;
contourf(data');

calib = [-5.2475   -4.6477   -3.0380   -4.8808   -5.0763   -5.0563   -4.5325   -1.7378;
    0.1058    0.0930    0.0605    0.0980    0.1025    0.1016    0.0911    0.0330]

data2 = [];
[m,n] = size(data)
for k = 1:m
    for l = 1:n
        data2(k,l) = calib(1,l) + calib(2,l) * data(k,l);
    end
end

figure;
contourf(data2');

% 
% Ii = [100 100 100 95 70 70 70 93];
% 
% Ii = (Ii-min(Ii))/(max(Ii)-min(Ii));
% 
% xi = 0.063:-0.018:-0.063;
% %xi = 1:8
% 
% xc = sum(xi.*(1-Ii))/sum(1-Ii)
% 
% figure;
% hold on;
% plot(xi,Ii,'b-');
% plot(xc,1,'rx');
% hold off;