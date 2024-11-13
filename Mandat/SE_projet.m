%% SE - compensateur boucle externe
% Compensateur se fait sur la Gsm_int de SI (p.11 specs)
clear all; clc;close all;
load("data_1v_4-09_100hz.mat", "Vm", "servo", "omega_c", "tsimu");

% conception par les critères du domaine temporel (lieu des racines)
% critères de performance lors de la conception initiale
Mp_ini = 5;
ts_ini = 4;
tr_ini = 2;
tp_ini = 3;

% début de la conception 

phi = atan(-pi./(log(Mp_ini./100)));
zeta = cos(phi);

% calcul des omega n --> prendre la plus grande valeur
wn_ts = 4./(zeta*ts_ini);
wn_tr = (1+1.1*zeta+1.4*zeta.^2)./tr_ini;
wn_tp = pi./(tp_ini*sqrt(1-zeta.^2));
wn = max([wn_ts wn_tr wn_tp]); % omega maximum calculé ici

% pôles désirés 
s_des = -zeta*wn + 1i*(wn*sqrt(1-zeta.^2));

% fonction de transfert Gsm_int pris de SI_projet
numGsm = [0,0,0,0,0.00471961603060694];
denGsm = [1,4.11856536591719,0.0112805289604377,0,0];
Gsm = tf(numGsm, denGsm);

% Lieu des racines pour voir si un gain K peut ajuster (garder commenter)
figure
rlocus(Gsm)
hold on;
plot(real(s_des), imag(s_des),'p');
hold on;
plot(real(s_des), -imag(s_des),'s')
title('Lieu des racines de la G_s_m')
% UN GAIN K NE COMPENSE PAS 
























