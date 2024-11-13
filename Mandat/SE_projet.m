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

