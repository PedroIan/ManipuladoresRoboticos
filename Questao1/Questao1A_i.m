clc
clear
close all

% run('./rvctools/startup.m')

syms t;


%% Inicialização dos Parâmetros

L(1) = Revolute('d', .290, 'alpha', -pi/2, 'qlim', (11/12)*[-pi pi]);
L(2) = Revolute('a', .270, 'offset', -pi/2, 'qlim', (11/18)*[-pi pi]);
L(3) = Revolute('a', .070, 'alpha', -pi/2, 'qlim', [-(11/18)*pi (7/18)*pi]);
L(4) = Revolute('d', 0.302, 'alpha', pi/2, 'qlim', (8/9)*[-pi pi]);
L(5) = Revolute('alpha', -pi/2, 'qlim', (2/3)*[-pi pi]);
L(6) = Revolute('d', .072, 'offset', pi, 'qlim', (20/9)*[-pi pi]);

i120 = SerialLink(L, 'name', 'IRB 120')

q = [0 0 0];

qdot_lim = pi*[25/18 25/18 25/18 16/9 16/9 7/3];

%% Controle

desiredPosition = [0.38 .38 .5];

T = i120.fkine(desiredPosition);

J = i120.jacob0(q, 'rpy');

p = transl(T);

R = SO3(T);
R = R.R();

i120.plot(desiredPosition)
hold on
T.plot(desiredPosition)