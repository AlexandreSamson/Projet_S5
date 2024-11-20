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
%SI_projet;
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
numGsm = [167.7210];
denGsm = [1 32.04 400.9 0 0]; % s^4 + s^3...
Gsm = tf(numGsm, denGsm);

% % Lieu des racines pour voir si un gain K peut ajuster (garder commenter)
% figure
% rlocus(Gsm)
% hold on;
% plot(real(s_des), imag(s_des),'p');
% hold on;
% plot(real(s_des), -imag(s_des),'p')
% title('Lieu des racines de la G_s_m')
% CTRL + MAJ + R enleve les commentaires
% UN GAIN K NE COMPENSE PAS 

pol = polyval(denGsm, s_des);
ph = -rad2deg((2*pi+angle(pol))); 
dphi = -180 - ph; % + 7,6

alpha = 180 - rad2deg(phi);
zphi = (alpha + dphi)./2;
pphi = (alpha - dphi)./2;

z = real(s_des) - imag(s_des)./tand(zphi);
p = real(s_des) - imag(s_des)./tand(pphi);

Ka = 1./abs(((s_des - z)./(s_des - p))*polyval(numGsm, s_des)./polyval(denGsm,s_des));

% Fonction de transfert Compensateur AvPh
numGa = Ka*[1 -z]; % fois 1.19
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


% critères de performances finales 
figure
margin(ftGext) % Première observation : pas assez de phase manque 3 deg

info = stepinfo(feedback(ftGext,1))

%%%%%%%%%%%%%%% Temporelle section en haut 

%% %% Frequentielle %%%%
% Définition de PM et BW
PM = deg2rad(45);  
BW = 2.3;          
err = 0.005;       

% Calcul du coefficient d'amortissement zeta basé sur la marge de phase
zeta = 0.5 * sqrt(tan(PM) * sin(PM))

% Calcul du numérateur et du dénominateur pour la fréquence à laquelle la marge est mesurée
wgNum = sqrt(sqrt(1 + 4 * zeta^4) - (2 * zeta^2));
wgDen = sqrt((1 - 2 * zeta^2) + sqrt(4 * zeta^4 - 4 * zeta^2 + 2));
wg = BW * (wgNum / wgDen) + 0.96;  % Fréquence de croisement de la bande passante


% Calcul du gain requis pour obtenir la bande passante désirée
[mag, phase] = bode(Gsm, wg);  
Kdes = 1 / mag               

% Création d'une nouvelle fonction de transfert avec le gain correctif
Gs2 = series(Kdes, Gsm)

% Récupération des marges de gain et de phase pour la fonction corrigée
[GM2, PM2, wp2, wg2] = margin(Gs2);

% Calcul de la marge de phase souhaitée en degrés
PMdes = rad2deg(PM);

% Calcul du déphasage nécessaire pour atteindre la marge de phase désirée
deltaPhi = PMdes - PM2 + 5 - 2.8;

% Calcul de alpha pour le correcteur d'avance de phase
alpha = (1 - sind(deltaPhi)) / (1 + sind(deltaPhi))

% Calcul du paramètre T du correcteur d'avance de phase
T = 1 / (wg * sqrt(alpha))
z = -1 / T
p = -1 / (alpha * T)  

% Calcul du gain du correcteur d'avance de phase
Ka = (Kdes / sqrt(alpha)) * 1.21;

% Définition des coefficients du numérateur et du dénominateur du correcteur
numAvPh = Ka * [1 1/T];
denAvPh = [1 1 /(alpha*T)];

% Création de la fonction de transfert du correcteur d'avance de phase
FTAvPh = tf(numAvPh, denAvPh)

% Création de la fonction de transfert totale (système corrigé)
FTAvPhtot = series(FTAvPh, Gsm)

figure;
margin(FTAvPhtot)

FTBF_freq = feedback(FTAvPhtot,1)


figure % Cas de la réponse à l’échelon 
t = [1:0.01:20]';
u = ones(size(t));  % Échelon unitaire 
% ou FTBF = feedback(FTBO,1) 
ybf = lsim(FTBF_freq,u,t); 
plot(t,ybf,'b', 'linewidth', 2) 
grid on 
hold on 
plot([t(1); t(end)], 0.98*ybf(end)*[1;1], 'r', 'linewidth', 2) 
plot([t(1); t(end)], 1.02*ybf(end)*[1;1], 'r', 'linewidth', 2) 


% Définir la valeur de stabilisation finale
y_final = ybf(end);

% Calcul des limites de 2%
lim_sup = 1.02 * y_final;
lim_inf = 0.98 * y_final;

% Déterminer le temps où la réponse reste dans les limites
idx_stable = find(ybf >= lim_inf & ybf <= lim_sup);

% Vérifier à partir de quel indice la réponse reste stable

for i = 1:length(idx_stable)
    if all(ybf(idx_stable(i):end) >= lim_inf & ybf(idx_stable(i):end) <= lim_sup)
        t_stab = t(idx_stable(i)); 
        break;
    end
end


% Affichage du temps de stabilisation
fprintf('Ts (2%%) : %.2f secondes\n', t_stab);

% Ajout d'une ligne verticale sur le graphique
if ~isnan(t_stab)
    hold on;
    plot([t_stab, t_stab], [0, y_final], '--g', 'LineWidth', 2);
end

BW = bandwidth(FTBF_freq)



























