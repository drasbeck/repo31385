clear all; clc; close all;

lsb = importdata('lineb2.dat');
lsb = lsb(:,1:8);
lsf = importdata('linef2.dat');
lsw = importdata('linedw2.dat');
lsw = lsw(:,1:8);

mean_lsb = mean(lsb)
std_lsb = std(lsb)

mean_lsf = mean(lsf)
std_lsf = std(lsf)

mean_lsw = mean(lsw)
std_lsw = std(lsw)

dwlim = mean_lsf + (mean_lsw - mean_lsf) ./ 2

figure;
hold on;
plot(mean_lsb,'-b')
plot(mean_lsf-3.*std_lsf,':r')
plot(mean_lsf,'-r')
plot(mean_lsf+2.*std_lsf,':r')
plot(mean_lsw,'-g')
plot(dwlim,'-k')
hold off;

wlim = mean_lsf-3.*std_lsf
blim = mean_lsf+3.*std_lsf