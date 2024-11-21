% Analyse Err fréquentiel
clc; clear all; close all;

load("BE_frequentiel.mat");

figure()
plot(tsim, x_balle)
hold on
plot(tsim, xd_balle)
%legend('Position', 'Position désirée')
title('position desirée & position')


% 1 s = 200 pts
t_0_3  = tsim(6401:10001);   % 0 à 3, 32 à 50
t_3_0  = tsim(13001: 15001); % 3 à 0, 65 à 75
t_0_6  = tsim(16001: 19001); % 0 à 6, 80 à 95
t_6_0  = tsim(20001: 23001); % 6 à 0, 100 à 115
t_0_m3 = tsim(25001: 27001); % 0 à -3, 125 à 135
t_m3_0 = tsim(30001: 32001); % -3 à 0, 150 à 160
t_0_m6 = tsim(35001: 37001); % 0 à -6, 175 à 185
t_m6_0 = tsim(39001: end);   % -6 à 0, 195 à 203 (end)