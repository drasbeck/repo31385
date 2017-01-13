clear all; clc; close all;

lsb = importdata('log_calibline_black.dat');
lsb = lsb(:,1:8);
lsf = importdata('log_calibline_floor.dat');
lsw = importdata('log_calibline_white.dat');
lsw = lsw(:,1:8);

mean_lsb = mean(lsb)
std_lsb = std(lsb)

mean_lsf = mean(lsf)
std_lsf = std(lsf)

mean_lsw = mean(lsw)
std_lsw = std(lsw)

dwlim = mean_lsf + (mean_lsw - mean_lsf) ./ 2
dblim = mean_lsf - (mean_lsf - mean_lsb) ./ 2

figure;
hold on;
plot(mean_lsb,'-b')
%plot(mean_lsf-3.*std_lsf,':r')
plot(mean_lsf,'-r')
%plot(mean_lsf+3.*std_lsf,':r') % not in use
%plot(mean_lsb + 3. * std_lsb,':r')
plot(mean_lsw,'-g')
plot(dwlim,'-k')
plot(dblim,'-k')
hold off;

wlim = mean_lsf + (mean_lsw - mean_lsf) ./ 2
%wlim = mean_lsf-3.*std_lsf % not in use
blim = mean_lsf-3.*std_lsf