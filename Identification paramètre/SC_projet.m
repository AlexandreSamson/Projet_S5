%% SC1 
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

% Lissage par projection orthogonal 
A_moy = mean(deg2rad(omega_c(114:end))); % A partie regime permanent
y_ = abs(log(A_moy - deg2rad(omega_c(101:108)))./deg2rad(omega_c(101:108))); % y 

% Matrice coefficient C pour projection orthogonal
i = 1;  
for c = 101:108
    A_coef(i, :) = [1, tsimu(c)]; 
    i = i + 1;
end
A_lis = pinv(A_coef)*y_;
lambda = -A_lis(2); % Alpha = - Lambda     --> Lambda = - Alpha
t0 = A_lis(1) ./ lambda; % Beta = lambda * t0

omega_c_lisse = A_moy ./ (1 + exp(lambda*(tsimu-t0))); % repartir de la fonction sigmoide

% X_entree1 (X1) = u(t)
X1 = Vm;

% % X_sortie1 (X2) = dy(t)/dt donc la dérivée de la position -> omega_c
%X2  = omega_c;
% Sinon, on peut changer omega_c par la dérivée de servo
% Donc prendre : X2 = diff(servo)./diff(tsimu);
X2 = omega_c_lisse;

% Ordre 1, seulement une dérivée
X = [X1 X2];

% Y = y(t) donc les valeur d'angle directement -> theta_c
Y = deg2rad(servo); % Mettre en rad

%----Méthode des moindres carrés----%
R = X' * X;
P = X' * Y;

Rinv = inv(R);
A_moindres_carres = Rinv*P;

K = A_moindres_carres(1);
tau = -A_moindres_carres(2); % Verifier le signe de Tau --> Équivalent a  :1 - alpha_sortie1 et comme A(2) négatif alors tau positif 
% Calculer Rm et Beq à partir de la fonction de transfert standard
Beq = (1./tau)*(Jeq*((N*nm*kt)./K-nm*kt*km))./((N*nm*kt)./K);
Rm = (1./Beq)*(N*kt*nm./K-nm*kt*km);

%% SC-2 R^2 et erreur RMS
% Omega_c transformer en radian
E_squared = sum((omega_c_lisse(1:end-1) - deg2rad(omega_c)).^2); 
N = length(tsimu);
RMSE = sqrt(1/N*E_squared);

y_moy = 1/N*sum(deg2rad(omega_c));
R_squared = sum((omega_c_lisse(1:end-1) - y_moy).^2)/(sum((deg2rad(omega_c) - y_moy).^2));

%% SC-3 Reponse de la fct transfert Gcm(s) à l'échelon Vm et donnees experimentales 

num = [(Kg*ng*nm*kt)./(Rm*Beq + nm*kt*km)];
den = [(Rm*Jeq)./(Rm*Beq+nm*kt*km) 1];
FT = tf(num, den); % des moindres carrés 
[y_Gcm, x_Gcm] = lsim(FT, Vm,tsimu);

figure
plot(x_Gcm, y_Gcm);
hold on; 
plot(tsimu(1:end-1), deg2rad(omega_c));
title("Reponses à Vm de la FCT G_c_m(s) comparés aux données expérimentales")
xlabel('Temps (secondes)')
ylabel('Vitesse (rad/s)')
legend('FT G_C_M(s)', 'Données expérimentales')






