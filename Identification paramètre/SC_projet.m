%% SC1 
clear all; 
clc;
close all; 
load("data_1v_4-09_100hz.mat", "Vm", "servo", "tsimu", "omega_c")

% X_entree1 (X1) = u(t)
X1 = Vm;

% X_sortie1 (X2) = dy(t)/dt donc la dérivée de la position -> omega_c
X2  = omega_c;
% Sinon, on peut changer omega_c par la dérivée de servo


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
tau = A(2);
% Parti (d) 
Jeq = 0.0017728; % unité : kg m^2 
Rm = tau/Jeq;

nm = 0.69;
km = 0.0076776; % unité : Nm/A
kt = 0.007683; % unité : V / (rad/s)
Beq = (1-nm*km*kt)/Rm;

%% SC-2 R^2 et erreur RMS

% Fonction de transfert à partir de valeur de lissage par moindre carrés 
% Point (c) de SC-1. Valeur dans la section plus haut (tau et K);
num = K;
den = [tau 1];

FT = tf(num, den);







