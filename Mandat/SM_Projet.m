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
Jeq = 0.0017728; % Jeq = Jm + N^2*Jc
ng  = 0.9;
Kg  = 70;
rarm= 0.0254;
L   = 0.4254;
% Rm  = 1852.55; %A revoir  
% Beq = 0.000539774; %A revoir    Beq = Bm+N^2*Bc
N = ng*Kg;

% %FTBO de la position (x) selon la tension en entree (Vm)
% Gsm_num_SM_2   = [5*N*nm*kt*g*rarm];
% Gsm_den_s3_SM_2 = 7*L*(nm*kt*km+Rm*Beq);
% Gsm_den_s4_SM_2 = 7*L*Rm*Jeq;

%% SM-2
% Valeur pour Simulink - FTBO_Eq_diff_Vm_oc.slx
kbb = 5*g*rarm/(L*7);
Eq_diff_coeff_oc = (Beq*Rm+nm*kt*km)/(Rm*(Jeq));
Eq_diff_coeff_Vm = (N*nm*kt/(Rm*Jeq));

%% SM-3
%Valeur pour Simulink - Gcm.slx - Gsc.slx
K_SM_3 = N*nm*kt;
Gcm_den_s0_SM_3 = 1/(Rm*Beq + nm*kt*km);
Gcm_den_s1_SM_3 = 1/(Rm*Jeq);
Time_Vm = [tsimu Vm];
Time_servo = [tsimu servo];

%% SM-5
% Matrice 4x4 sans la variable d'etat im 
coefA = -(Rm*Beq+nm*kt*km)/(Rm*Jeq);
kbb = 5*g*rarm/(L*7);
A1 = [0 1 0 0 ;
      0 0 kbb 0;
      0 0 0 1;
      0 0 0 coefA];
B1 = [0;
      0;
      0;
      N*nm*kt/(Rm*Jeq)];

C1 = [1 0 0 0;
     0 0 1 0];

D1 = [0; 0];

% Valeur propre
Vp = eig(A1);

%% SM-6

%FTBO entre la tension en entrée (Vm) et l'angle du moteur (oc)
Gcm_num_SM_6 = N*nm*kt;
Gcm_den_s2_SM_6 = Rm*(Jeq);
Gcm_den_s1_SM_6 = (Rm*Beq+nm*kt*km);
Gcm_den_SM_6 = [Gcm_den_s2_SM_6 Gcm_den_s1_SM_6 0];
Gcm_SM_6 = tf(Gcm_num_SM_6, Gcm_den_SM_6);

p_Gcm_SM_6 = roots(Gcm_den_SM_6); %poles de la FTBO

%FTBO entre l'angle du moteur (oc) et la position de la charge (x)
Gsc_num_SM_6 = 5*g*rarm;
Gsc_den_SM_6 = [L*7 0 0];
Gsc_SM_6 = tf(Gsc_num_SM_6, Gsc_den_SM_6);

p_Gsc_SM_6 = roots(Gsc_den_SM_6); %poles de la FTBO

%% SM-7
% FTBO de la tension en entrée (Vm) et de la position de la charge en
% sortie (x) et utilise les valeurs FT dans FTBO_Vm_oc
Gsm_num_SM_7 = Gsm_num_SM_2;
Gsm_den_SM_7 = [Gsm_den_s4_SM_2 Gsm_den_s3_SM_2 0 0 0];
Gsm_SM_7 = tf(Gsm_num_SM_7, Gsm_den_SM_7);

p_Gsm_SM_7 = roots(Gsm_den_SM_7); %poles de la FTBO



tau = 0.3149;
K = 2.7013;
Beq = (1./tau)*(Jeq*((N*nm*kt)./K-nm*kt*km))./((N*nm*kt)./K);
Rm = (1./Beq)*(N*kt*nm./K-nm*kt*km);




