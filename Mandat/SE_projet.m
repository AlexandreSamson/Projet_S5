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
plot(real(s_des), -imag(s_des),'p')
title('Lieu des racines de la G_s_m')
% CTRL + MAJ + R enleve les commentaires
% UN GAIN K NE COMPENSE PAS 

pol = polyval(denGsm, s_des);
ph = -rad2deg((pi+angle(pol))); 
dphi = -180 - ph; 

alpha = 180 - rad2deg(phi);
zphi = (alpha + dphi)./2;
pphi = (alpha - dphi)./2;

z = real(s_des) - imag(s_des)./tand(zphi);
p = real(s_des) - imag(s_des)./tand(pphi);

Ka = 1./abs(((s_des - z)./(s_des - p))*polyval(numGsm, s_des)./polyval(denGsm,s_des));

% Fonction de transfert Compensateur AvPh
numGa = Ka*[1 -z];
denGa = [1 -p];
ftGa = tf(numGa, denGa);

% Fonction de transfert Gsm(s) * Ga(s)
ftGext = Gsm * ftGa;

% Lieu des racines avant tune
figure
rlocus(ftGext)
hold on;
plot(real(s_des), imag(s_des),'p');
hold on;
plot(real(s_des), -imag(s_des),'p')
title('Lieu des racines de la G_s_m(s)*G_a(s)')





























