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
epsilon = 10e-2;

e_ant = 1;
e = 1; 

%% Definições de Controle

posicaoInicial = [0 0 0 0 -pi / 2 0];

posicaoDesejada = [0.38 .38 .5 0 0 0];

T = i120.fkine(posicaoDesejada); % Pega pose desejada do efetuador 
pd = transl(T); % Pega vetor de transla��o do efetuador
Rd = SO3(T); % Pega o objeto SO3 correspondente � rota��o do efetuador
Rd = Rd.R; %Pega matriz de rota��o do efetuador

rpyd = rotm2eul(Rd);

Td = SE3(Rd, pd);
Td.plot('rgb')

%% Aplicação do Controle


figure(1)
i120.plot(q); % Plot rob� na configura��o inicial
hold on
T.plot('rgb') % Plot pose desejada
%%
i = 0

testeTic = tic;

while (norm(e) > epsilon | i < 100) % Crit�rio de parada
    J = i120.jacob0(q, 'rpy'); % Jacobiana geom�trica
    T = i120.fkine(q); % Cinem�tica direta para pegar a pose do efetuador 
    p = transl(T); % Transla��o do efetuador
    R = SO3(T); 
    R = R.R; % Extrai rota��o do efetuador
    i = i+1; % contador
    
    p_err = pd-p; % Erro de transla��o

    rpy = rotm2eul(R);
    
    rpy_til = rpyd - rpy;
    
    nphi = rotm2axang(Rd*R'); 
    nphi_err = nphi(1:3)*nphi(4); % Erro de rota��o (n*phi)
    
    e_ant = e;
    e = [p_err'; rpy_til']; % Vetor de erro
    
    u = pinv(J)*ganho*e; % Lei de controle

    dt = toc(testeTic);
    testeTic = tic;

    q = q + dt*u'; % C�lculo de posicaoInicial (Regra do trap�zio)
    
    i120.plot(q);
    control_sig(:,1) = u; % Sinal de controle
    err(i) = norm(e); % Norma do erro
    norm(e)
end
hold off

%% Plot sinal de controle e norma do erro

figure(2)
hold on
plot(control_sig(:, 1))

hold off
xlabel('Itera��es')
ylabel('Sinal de controle: u(rad/s)')

figure(3)
plot(err)
xlabel('Itera��es')
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
