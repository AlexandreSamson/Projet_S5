%% SC1 
clear all; clc;close all;load("data_1v_4-09_100hz.mat", "Vm", "servo", "omega_c", "tsimu");

% X_entree1 (X1) = u(t)
X1 = Vm;

% % X_sortie1 (X2) = dy(t)/dt donc la dérivée de la position -> omega_c
%X2  = omega_c;
% Sinon, on peut changer omega_c par la dérivée de servo
% Donc prendre : X2 = diff(servo)./diff(tsimu);
X2 = diff(servo)./diff(tsimu);

% Ordre 1, seulement une dérivée
X = [X1(1:end-1) X2];

% Y = y(t) donc les valeur d'angle directement -> theta_c
Y = servo(1:end-1); 
                                    %----Méthode des moindres carrés----%
R = X' * X;
P = X' * Y;

Rinv = inv(R);
A = Rinv*P;

K = A(1);
tau = A(2); % Verifier le signe de Tau
% Parti (d) 
Jeq = 0.0017728; % unité : kg m^2 
Rm = tau/Jeq; % unité : ohms 

nm = 0.69;
km = 0.0076776; % unité : Nm/A
kt = 0.007683; % unité : V / (rad/s)
Beq = (1-nm*km*kt)/Rm;

%% SC-2 R^2 et erreur RMS
% Fonction de transfert à partir de valeur de lissage par moindre carrés 
% Point (c) de SC-1. Valeur dans la section plus haut (tau et K);
num = K;
den = [-tau 1]; % tau ici est négatif voir Revue 2 section 3.1.4
FT = tf(num, den); % des moindres carrés 
[y_des, x_des] = lsim(FT, Vm,tsimu);

y_mes = servo; % yn : c'est theta_c. C'est la valeur mesurée (_mes)

E_squared = sum((y_des - y_mes).^2); 
N = length(tsimu);
RMSE = sqrt(1/N*E_squared);

y_moy = 1/N*sum(y_mes);
R_squared = sum((y_des - y_moy).^2)/(sum((y_mes - y_moy).^2));

figure
plot(tsimu, y_des)
hold on;
plot(tsimu, servo)
title("Moindre carrés vs Mesuré")
legend('Moindres carrés', 'Données expérimentales')
xlabel('Temps (secondes)')
ylabel('Angle (theta_c)')

%% SC-3 Reponse de la fct transfert Gcm(s) à l'échelon Vm

Gcm_num = [K]; % Le signe doit il être négatif ?
Gcm_den = [Rm*Jeq (Rm*Beq + nm*kt*km) 0];
Gcm = tf(Gcm_num, Gcm_den);

% Reponses a echelon 
[yGcm, xGcm] = lsim(Gcm, Vm, tsimu);

% Graphique des reponses a Vm 
figure
plot(xGcm, abs(yGcm)) % Ici malgré le fait que K soit négatif, comme on représente un angle -> Pertinent de l'avoir en Absolue (?)
hold on; 
plot(tsimu, servo)
title("Comparaison des systèmes")
legend('Gcm', 'Données expérimentales')
xlabel('Temps (secondes)')
ylabel('Angle (theta_c)')


%% SI-1 
figure
rlocus(Gcm) % Allure non stable 
figure
rlocus(FT)



