clear all; clc; close all;

Ii = [100 100 100 95 70 70 70 93];

Ii = (Ii-min(Ii))/(max(Ii)-min(Ii));

xi = 0.063:-0.018:-0.063;
%xi = 1:8

xc = sum(xi.*(1-Ii))/sum(1-Ii)

figure;
hold on;
plot(xi,Ii,'b-');
plot(xc,1,'rx');
hold off;