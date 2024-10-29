clc; close all; clear all;
load("data_1v_4-09_100hz.mat", "Vm", "servo", "tsimu", "omega_c")

%Variables
ms  = 0.064;
Js  = 4.129e-6;
rs  = 0.0127;
g   = 9.81;
km  = 0.0076776;
kt  = 0.0076830;
nm  = 0.69;
Jm  = 3.9001e-7;
Jeq = 0.0017728;
ng  = 0.9;
Kg  = 70;
rarm= 0.0254;
L   = 0.4254;
Rm  = 1852.55; %A etre identifié
Beq = 0.000539774; %A etre identifié

%Variables pas clair qu'on doit verifier
Jc = Jeq; %fr?
Bc = Beq; %fr?
Bm = 1; %??
N1 = 1; %??
N2 = 2; %??
N3 = 3; %??
N4 = 4; %??
N = N1*N3/N2*N4;

%% SM-2
%FTBO de la position (x) selon la tension en entree (Vm) - systeme complet
% x/Vm
Gsm_num_SM_2   = [5*N*nm*kt*g*rarm];
Gsm_den_s3_SM_2 = 7*L*(nm*kt*km+Bm*Rm+(N^2)*Bc*Rm);
Gsm_den_s4_SM_2 = 7*L*Rm*(Jm+(N^2)*Jc);

% Valeur pour SimuLink
kbb = (5*g*rarm)/(L*7);
Eq_diff_coeff_oc = (Bm*Rm+Bc*N^2*Rm+nm*kt*km)/(Rm*(Jm+Jc*N^2));
Eq_diff_coeff_Vm = (N*nm*kt/(Rm*Jm+Rm*Jc*N^2));

%% SM-3
K_SM_3 = N*nm*kt;
Gcm_den_s0_SM_3 = 1/(Rm*Bm + Bc*N.^2*Rm + nm*kt*km);
Gcm_den_s1_SM_3 = 1/(Rm*(Jm+Jc*N.^2));

%% SM-5
% B)
Coef_A = -(((Bm*Rm)+(Bc*N^2*Rm)+(nm*kt*km))/(Rm*(Jm+Jc*N^2)));
Lm = 0.1; %?????????????

% Non-linéaire
A = [0 1 0 0 0;
     0 0 kbb 0 0;
     0 0 0 1 0;
     0 0 0 Coef_A 0;
     0 0 0 -(km/(N*Lm)) -Rm/Lm];

Coef_B = (N*nm*kt/(Rm*Jm+Rm*Jc*N^2));
B = [0; 0; 0; Coef_B; -1/Lm];

C = [1 0 0 0 0;
     0 0 1 0 0];

D = [0; 0];

% C)
Vp = eig(A);

%% SM-6

%FTBO entre la tension en entrée (Vm) et l'angle du moteur (oc)
Gcm_num_SM_6 = N*nm*kt;
Gcm_den_s2_SM_6 = Rm*(Jm+Jc*N^2);
Gcm_den_s1_SM_6 = (Rm*Bm+Bc*N^2*Rm+nm*kt*km);
Gcm_den_SM_6 = [Gcm_den_s2_SM_6 Gcm_den_s1_SM_6 0];
Gcm_SM_6 = tf(Gcm_num_SM_6, Gcm_den_SM_6);

p_Gcm_SM_6 = roots(Gcm_den_SM_6); %poles de la FTBO

%FTBO entre l'angle du moteur (oc) et la position de la charge (x)M -
%linéariser
Gsc_num_SM_6 = 5*g*rarm;
Gsc_den_SM_6 = [L*7 0 0];
Gsc_SM_6 = tf(Gsc_num_SM_6, Gsc_den_SM_6);

p_Gsc_SM_6 = roots(Gsc_den_SM_6); %poles de la FTBO

%% SM-7
% FTBO de la tension en entrée (Vm) et de la position de la charge en
% sortie (x)
Gsm_num_SM_7 = Gsm_num_SM_2;
Gsm_den_SM_7 = [Gsm_den_s4_SM_2 Gsm_den_s3_SM_2 0 0 0];
Gsm_SM_7 = tf(Gsm_num_SM_7, Gsm_den_SM_7)

p_Gsm_SM_7 = roots(Gsm_den_SM_7); %poles de la FTBO

