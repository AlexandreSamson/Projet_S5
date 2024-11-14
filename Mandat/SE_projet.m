clc; clear all; close all;
% Mandat SE
SM_Projet;
SI_projet;


flag_in = 0;

%t = [0:0.01:10];


out = sim('SE_1');
t = [out.x_sphere.Time];
u = ones(size(t));

%SE-1
%Fichier SimuLink SE_1.slx

%SE-2

y = lsim(Gsm_int,Time_Vm(:,2),Time_Vm(:,1));

figure()
plot(Time_Vm(:,1),y)
title('FT Gsm-int entre Vm - version linéaire')
ylabel('Position (m)')
xlabel('Time(s)')

%Entre en echelon
Gsm_int_BF = feedback(Gsm_int,1);
y_lin_ech = lsim(Gsm_int_BF, u, t);

figure()
plot(t, y_lin_ech)
plot(t, out.x_sphere.Data)
title('Réponse à l échelon du système')
xlabel('temps (s)')
ylabel('Position (m)')
legend('Linéaire', 'Non-Linéaire')
grid on
hold on
plot(t, 0.98*y_lin_ech(end)*[1:1], 'r--', 'linewidth', 2);
plot(t, 1.02*y_lin_ech(end)*[1:1], 'r--', 'linewidth', 2);
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





























