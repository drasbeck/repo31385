clc;clear all;close all;

n=1;
Dn = 0.067;
Ce = 2000;
b = 0.268;
cm = pi * Dn / (n * Ce);
x_old = 0;
y_old = 0;
th_old = 0;

t = 5;
R = [-1000;1000;1000;1000;1000];
L = [1000;1000;1000;-1000;1000];

for i = 1:t
    NR = R(i);
    NL = L(i);
    dUR = cm*NR;
    dUL = cm*NL;

    dU = (dUR + dUL)/2;
    dth = (dUR - dUL)/b;

    th = th_old + dth;
    
    x = x_old + dU*cos(th);
    y = y_old + dU*sin(th);
    
    th_old = th;
    x_old = x;
    y_old = y;
end

x
y
th


