
clc; close all; clear all;
load("data_1v_4-09_100hz.mat", "Vm", "servo", "omega_c", "tsimu");

% TO DO 
% Simplifier la manière de construire les FT

%Variables
ms  = 0.064;
Js  = 4.129e-6;
rs  = 0.0127;
g   = 9.81;
km  = 0.0076776;
kt  = 0.0076830;
nm  = 0.69;
Jm  = 3.9001e-7;
Jeq = 0.0017728; % Jeq = Jm*kg.^2*ng + Jc
ng  = 0.9;
Kg  = 70;
rarm= 0.0254;
L   = 0.4254;
Rm  = 3.6762;
Beq = 0.0073; % Beq = Bm*kg.^2*ng + Bc
N = ng*Kg;


%% SM-2
% Valeur pour Simulink - FTBO_Eq_diff_Vm_oc.slx - GOOD
kbb = 5*g*rarm/(L*7);
Eq_diff_coeff_oc = ((Beq*Rm)+nm*kt*km*Kg.^2*ng)/(Rm*Jeq);
Eq_diff_coeff_Vm = (nm*kt*Kg*ng)/(Jeq*Rm);

%% SM-3
%Valeur pour Simulink - Gcm.slx - Gsc.slx - GOOD
K_SM_3 = nm*kt*Kg*ng;
Gcm_den_s1_SM_3 = 1/(Rm*Beq + nm*kt*km*Kg.^2*ng);
Gcm_den_s2_SM_3 = 1/(Rm*Jeq);
Time_Vm = [tsimu Vm];
Time_servo = [tsimu servo];

%% SM-5
% Matrice 4x4 sans la variable d'etat im  - GOOD
coefA = -(Rm*Beq+nm*kt*km*Kg.^2*ng)/(Rm*Jeq);
kbb = 5*g*rarm/(L*7);
A1 = [0 1 0 0 ;
      0 0 kbb 0;
      0 0 0 1;
      0 0 0 coefA];
B1 = [0;
      0;
      0;
      (nm*kt*Kg*ng)/(Rm*Jeq)];

C1 = [1 0 0 0;
     0 0 1 0];

D1 = [0; 0];

% Valeur propre
Vp = eig(A1);

%% SM-6

%FTBO entre la tension en entrée (Vm) et l'angle du moteur (oc) - GOOD 
Gcm_num_SM_6 = nm*kt*Kg*ng;
Gcm_den_s2_SM_6 = Rm*Jeq;
Gcm_den_s1_SM_6 = (Rm*Beq + nm*kt*km*Kg.^2*ng);
Gcm_den_SM_6 = [Gcm_den_s2_SM_6 Gcm_den_s1_SM_6 0];
Gcm_SM_6 = tf(Gcm_num_SM_6, Gcm_den_SM_6);

p_Gcm_SM_6 = roots(Gcm_den_SM_6); %poles de la FTBO

%FTBO entre l'angle du moteur (oc) et la position de la charge (x) - GOOD
Gsc_num_SM_6 = 5*g*rarm;
Gsc_den_SM_6 = [L*7 0 0];
Gsc_SM_6 = tf(Gsc_num_SM_6, Gsc_den_SM_6);

p_Gsc_SM_6 = roots(Gsc_den_SM_6); %poles de la FTBO

%% SM-7
% FTBO de la tension en entrée (Vm) et de la position de la charge en
% sortie (x) et utilise les valeurs FT dans FTBO_Vm_oc - GOOD
Gsm_num_SM_7 = 5*g*rarm*nm*kt*Kg*ng;
Gsm_den_SM_7 = [Rm*Jeq*7*L (Rm*Beq+nm*kt*km*Kg.^2*ng)*7*L 0 0];
Gsm_SM_7 = tf(Gsm_num_SM_7, Gsm_den_SM_7);

p_Gsm_SM_7 = roots(Gsm_den_SM_7); %poles de la FTBO








