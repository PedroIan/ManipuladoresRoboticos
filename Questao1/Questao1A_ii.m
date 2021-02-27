clc
clear
close all

% run('./rvctools/startup.m')

syms t;

%% Inicialização dos Parâmetros

L(1) = Revolute('d', .290, 'alpha', -pi / 2, 'qlim', (11/12) * [-pi pi]);
L(2) = Revolute('a', .270, 'offset', -pi / 2, 'qlim', (11/18) * [-pi pi]);
L(3) = Revolute('a', .070, 'alpha', -pi / 2, 'qlim', [-(11/18) * pi (7/18) * pi]);
L(4) = Revolute('d', 0.302, 'alpha', pi / 2, 'qlim', (8/9) * [-pi pi]);
L(5) = Revolute('alpha', -pi / 2, 'qlim', (2/3) * [-pi pi]);
L(6) = Revolute('d', .072, 'offset', pi, 'qlim', (20/9) * [-pi pi]);

i120 = SerialLink(L, 'name', 'IRB 120')

q = [0 0 0 0 -pi / 2 0];

qdot_lim = pi * [25/18 25/18 25/18 16/9 16/9 7/3];

ganho = 0.8;
epsilon = 25e-2;

e_ant = 1;
e = 1; 

%% Definições de Controle

posicaoInicial = [0 0 0 0 -pi / 2 0];

posicaoDesejada = [0.38 .38 .5 0 0 0];

T = i120.fkine(posicaoDesejada'); % Pega pose desejada do efetuador 
pd = transl(T); % Pega vetor de translação do efetuador
Rd = SO3(T); % Pega o objeto SO3 correspondente � rotação do efetuador
Rd = Rd.R; %Pega matriz de rotação do efetuador

rpyd = rotm2eul(Rd);

Td = SE3(Rd, pd);


%% Aplicação do Controle


figure(1)
i120.plot(posicaoInicial); % Plot robô na configuração inicial
hold on
T.plot('rgb') % Plot pose desejada
%%
i = 0

testeTic = tic;

while (norm(e) > epsilon | i < 20) % Critério de parada
    J = i120.jacob0(q, 'rpy'); % Jacobiana geométrica
    T = i120.fkine(q); % Cinemática direta para pegar a pose do efetuador 
    p = transl(T); % translação do efetuador
    R = SO3(T); 
    R = R.R; % Extrai rotação do efetuador
    i = i+1; % contador
    
    p_err = pd-p; % Erro de translação

    rpy = rotm2eul(R);
    
    rpy_til = rpyd - rpy;
    
    nphi = rotm2axang(Rd*R'); 
    nphi_err = nphi(1:3)*nphi(4); % Erro de rotação (n*phi)
    
    e_ant = e;
    e = [p_err'; rpy_til']; % Vetor de erro
    
    u = pinv(J)*ganho*e; % Lei de controle

    dt = toc(testeTic);
    testeTic = tic;

    q = q + 0.1*u'; % C�lculo de posicaoInicial (Regra do trapézio)
    
    i120.plot(q);
    control_sig(:,i) = u; % Sinal de controle
    err(i) = norm(e); % Norma do erro
    norm(e)
end
hold off

%% Plot sinal de controle e norma do erro

figure(2)
title('Sinais de Controle');

for(i = 1:6)
    subplot(3,2,i)
    plot(control_sig(i, :))
    title('Junta', i )
    xlabel('Iterações')
    ylabel('Sinal de controle: u(rad/s)')
    hold on
end

hold off

figure(3)
plot(err)
xlabel('Iterações')
ylabel('Norma do erro: |e|')
box off

%% Posição final Desejada

desiredPosition = [0.38 .38 .5 0 0 0];

T = i120.fkine(desiredPosition);

J = i120.jacob0(q, 'rpy');

p = transl(T);

R = SO3(T);
R = R.R();

i120.plot(desiredPosition)
hold on
T.plot(desiredPosition)
