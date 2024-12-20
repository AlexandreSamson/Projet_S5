%% SI Conception de la boucle interne 
% clear all; clc;close all;
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
Rm = 3.6267;
Beq = 0.0073;

% FT de Gcm(s) theta_c/Vm;
numGcm = [Kg*ng*nm*kt]; % Kg*ng = N
denGcm = [Rm*Jeq (Rm*Beq + nm*kt*km*(Kg.^2)*ng) 0];
Gcm = tf(numGcm, denGcm);

% Ft de Gcm(s) standard
numGcm_st = [Kg*ng*nm*kt./(Rm*Jeq)];
denGcm_st = [1 (Rm*Beq + nm*kt*km*(Kg.^2)*ng)./(Rm*Jeq) 0./(Rm*Jeq)];
Gcm_st = tf(numGcm_st, denGcm_st);


% % Lieu des racines SI-1 (à garder en commentaire si pas utiliser)
figure(1) 
rlocus(Gcm) 
title('Lieu des racines de la fonction G_c_m')

%SI-2 a)
Kcrit = 4.94;
%SI-2 b)
ts_b = 4/16; %Valeur 16 = -zeta*wn pris appartir du rlocus 
%SI-2 c)
% À partir de la règle 7 Rlocus a la main
ts_c = 4./(16.019); % valeur calculé papier 


%SI-2 d)
Gcm_BF = feedback(Kcrit*Gcm,1);
denGcm_BF = [Gcm_BF.Denominator{1, 1}(1) Gcm_BF.Denominator{1, 1}(2) Gcm_BF.Denominator{1, 1}(3)]/Gcm_BF.Denominator{1, 1}(1);
numGcm_BF = Gcm_BF.Numerator{1, 1}(3)/Gcm_BF.Denominator{1, 1}(1);

tf_ = tf(numGcm_BF,denGcm_BF);

wn = sqrt(denGcm_BF(3));
zeta = denGcm_BF(2)/(2*wn);
ts_d = 4/wn*zeta;

%SI-2 F)
i = (Kcrit:0.1:100);
p_ts_cst = rlocus(Gcm,i);
phi = pi - angle(p_ts_cst(1,:));
Mp = 100*exp(-pi./tan(phi));

figure(2)
plot(phi,Mp)
title('MP-phi pour gain avec ts identique')
xlabel('phi (rad)')
ylabel('MP(%)')

%SI-3
Kint_Rac = 7.72; % trouver avec rlocus à damping = 0.8
zeta = 0.8;


Kint_Calc = ((Beq*Rm+nm*kt*Kg*Kg*ng*km)/(Rm*Jeq*2*zeta)).^2 * (Rm*Jeq)/(nm*kt*Kg*ng);
Gcm_BF = feedback(Gcm*Kint_Calc,1); %Visualiser la FT

% SI-4
poles_BF = roots(Gcm_BF.Denominator{1, 1});

figure(3) 
rlocus(Gcm)
hold on
plot(real(poles_BF),  imag(poles_BF),'Diamond','Color','r')
hold on
plot(real(poles_BF), -imag(poles_BF),'Diamond','Color','r')
title('Lieu des racines de la fonction G_c_m')
legend('G_c_m','Pôles désirés')

% SI-5
figure()
margin(Gcm)
title('Diagramme de Bode de G_c_m')

% SI-6
[GM,PM,wg,wp] = margin(Gcm*Kint_Calc);

zeta_Calc = 0.5*sqrt(tand(PM)*sind(PM));
PM_Calc = atand((2*zeta)/(sqrt(sqrt(1+4*zeta.^4)-2*zeta.^2)));

%SI-7 
kbb = 5*g*rarm/(L*7);
coefA =  -(Rm*Beq+nm*kt*km*Kg.^2*ng)/(Rm*Jeq);
A = [0 1 0 0;
    0 0 kbb 0;
    0 0 0 1;
    0 0 0 coefA];
B = [0 0 0 (nm*kt*Kg*ng)/(Rm*Jeq)]';
C = [1 0 0 0;
    0 0 1 0];
D = [0 0]';

Aint = (A - B*Kint_Calc*C(2,:));
Bint = B*Kint_Calc;
Cint = C;
Dint = D;

vp_Aint = eig(Aint);

%SI-8
[numInt, denInt] = ss2tf(Aint,Bint,Cint,Dint,1);
Gsm_int = tf(numInt(1,:),denInt);
Gcm_int = tf(numInt(2,:),denInt);

poles_FT_int = roots(denInt); % Valeur propre et poles des FT sont identiques

%SI-9 
%La FT Gsm_int est classe 2 et la FT Gsm etait classe 3

%SI-10
figure()
rlocus(Gsm_int)
title('Lieu des racines de la G_s_m de la boucle interne')
% 
% close all;

