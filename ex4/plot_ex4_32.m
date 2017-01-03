clear all; clc; close all;

data = importdata('data_log.dat');

figure;
plot(data(:,5),data(:,6));