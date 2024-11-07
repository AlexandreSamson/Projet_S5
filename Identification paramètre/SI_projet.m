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
denGcm = [Rm*Jeq (Rm*Beq +nm*kt*km) 0]
Gcm = tf(numGcm, denGcm)


% Lieu des racines SI-1 (à garder en commentaire si pas utiliser)
figure 
rlocus(Gcm) 



%SI-2 a)
Kcrit = 0.291;

%SI-2 b)
ts_b = 4/1.58 %Valeur 1.58 prix appartir du rlocus pour valeur Real pour ts constant

%SI-2 c) *****A REVOIR VALEUR DE ts_c *********
%Utiliser la regle de construction N5 pour avoir la valeur d'intersection
%de l'asympote avec axe Reel ou N7 point de separation
%N5
p = roots(denGcm);
ts_c = sum(p)/2

%N7
ts_c2 = -denGcm(2)/numGcm*2*denGcm(1)


%SI-2 d)
Gcm_BF = feedback(Gcm,Kcrit);
denGcm_BF = [Gcm_BF.Denominator{1, 1}(1) Gcm_BF.Denominator{1, 1}(2) Gcm_BF.Denominator{1, 1}(3)]/Gcm_BF.Denominator{1, 1}(1);
numGcm_BF = Gcm_BF.Numerator{1, 1}(3)/Gcm_BF.Denominator{1, 1}(1);

wn = sqrt(denGcm_BF(3));
zeta = denGcm_BF(2)/(2*wn);
ts_d = 4/wn*zeta








