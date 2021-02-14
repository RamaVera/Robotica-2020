clc;clear all;close all;set(0,'DefaultFigureWindowStyle','docked')
%%%%%%%%%%%%%%
%% TP 5
 
%% Parametros configurables

pctTrain = 0.4; % el 40% es de entrenamiento el resto es de test
graficar = false;

%% Parametros del problema
a = 0.2;
m = 2;
g = 9.8;

datos_pendulo;
nTrain = ceil(length(datos)*pctTrain);

t   = datos(:,1);
q   = datos(:,2);
qd  = datos(:,3);
qdd = datos(:,4);
tau = datos(:,5);

t_train   = datos(1:nTrain,1);
q_train   = datos(1:nTrain,2);
qd_train  = datos(1:nTrain,3);
qdd_train = datos(1:nTrain,4);
tau_train = datos(1:nTrain,5);

t_test   = datos(nTrain+1:end,1);
q_test   = datos(nTrain+1:end,2);
qd_test  = datos(nTrain+1:end,3);
qdd_test = datos(nTrain+1:end,4);
tau_test = datos(nTrain+1:end,5);

%% Grafico de t, tau, q, qd,qdd
if graficar,
    subplot(4,1,1)
    plot(t,tau)
    ylabel('tau[nm]')
    subplot(4,1,2)
    plot(t,q)
    ylabel('q[m]')
    subplot(4,1,3)
    plot(t,qd)
    ylabel('qd[m/s]')
    subplot(4,1,4)
    plot(t,qdd)
    ylabel('qdd[m/s^2]')
    xlabel('t[s]')
end
%% Regresion
Y   = tau_train - (a^2 *m*qdd_train+m*g*a*cos(q_train));
PHI = [qdd_train,  2*a*m*qdd_train+m*g*cos(q_train), -m*g*sin(q_train)];

% pseudoPHI = inv(PHI'*PHI)*PHI';
pseudoPHI = pinv(PHI);

condPHI = cond(PHI);
P = pseudoPHI*Y;

%% Calculo de error
Y   = tau - (a^2 *m*qdd+m*g*a*cos(q));
PHI = [qdd,  2*a*m*qdd+m*g*cos(q), -m*g*sin(q)];

E_tot = sum(abs(Y-PHI*P));

Y   = tau_test - (a^2 *m*qdd_test+m*g*a*cos(q_test));
PHI = [qdd_test,  2*a*m*qdd_test+m*g*cos(q_test), -m*g*sin(q_test)];

E_test = sum(abs(Y-PHI*P));

%% Resultados
pctTrain
nTrain

condPHI
E_test
E_tot

P


