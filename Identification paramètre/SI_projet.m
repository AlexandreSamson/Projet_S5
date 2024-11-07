%% SI Conception de la boucle interne 
clear all; clc;close all;
load("data_1v_4-09_100hz.mat", "Vm", "servo", "omega_c", "tsimu");
%Variables
ms  = 0.064;
Js  = 4.129e-6;
rs  = 0.0127;
g   = 9.81;
km  = 0.0076776;
kt  = 0.0076830;
nm  = 0.69;
Jm  = 3.9001e-7;
Jeq = 0.0017728; % Jeq = Jm + N^2*Jc
ng  = 0.9;
Kg  = 70;
rarm = 0.0254;
L   = 0.4254;
N = ng*Kg;
% Valeur tirer de SC
Rm = 21.9604;
Beq = 0.0056; 

% FT de Gcm(s) theta_c/Vm;
numGcm = [Kg*ng*nm*kt]; % Kg*ng = N
denGcm = [Rm*Jeq (Rm*Beq +nm*kt*km) 0];
Gcm = tf(numGcm, denGcm);

% Lieu des racines SI-1 (à garder en commentaire si pas utiliser)
figure 
rlocus(Gcm) 
















