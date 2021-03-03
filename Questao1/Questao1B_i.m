clc
clear
close all

syms t;

%% Confere se existe uma simulação ativa no CoppeliaSim

sim = remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
sim.simxFinish(-1); % just in case, close all opened connections
clientID = sim.simxStart('127.0.0.1', 19999, true, true, 5000, 5);

if (clientID >- 1)
    % sim=remApi('remoteApi','extApi.h'); % using the header (requires a compiler)
    disp('Connected to remote API server');

    h = [0 0 0 0 0 0 0];

    
    [r, h(1)] = sim.simxGetObjectHandle(clientID, 'joint_7', sim.simx_opmode_blocking);
    [r, h(2)] = sim.simxGetObjectHandle(clientID, 'joint_1', sim.simx_opmode_blocking);
    [r, h(3)] = sim.simxGetObjectHandle(clientID, 'joint_2', sim.simx_opmode_blocking);
    [r, h(4)] = sim.simxGetObjectHandle(clientID, 'joint_3', sim.simx_opmode_blocking);
    [r, h(5)] = sim.simxGetObjectHandle(clientID, 'joint_4', sim.simx_opmode_blocking);
    [r, h(6)] = sim.simxGetObjectHandle(clientID, 'joint_5', sim.simx_opmode_blocking);
    [r, h(7)] = sim.simxGetObjectHandle(clientID, 'joint_6', sim.simx_opmode_blocking);
    
    %%% Exemplo posição arbitrária
    
    % joint_position = [pi / 2 pi / 2 0 0 0 pi / 2 0];
    % for i = 1:7
    %     sim.simxSetJointTargetPosition(clientID, h(i), joint_position(i), sim.simx_opmode_streaming)
    % end

    pause(10);
    sim.delete(); % call the destructor!
else
    sim.delete();

    disp('CoppeliaSim não ativo, realizando simulações locais');
end


disp('Programa Finalizado');

%%

L(1) = Revolute('d', .290, 'alpha', -pi / 2, 'qlim', (11/12) * [-pi pi]);
L(2) = Revolute('a', .270, 'offset', -pi / 2, 'qlim', (11/18) * [-pi pi]);
L(3) = Revolute('a', .070, 'alpha', -pi / 2, 'qlim', [-(11/18) * pi (7/18) * pi]);
L(4) = Revolute('d', 0.302, 'alpha', pi / 2, 'qlim', (8/9) * [-pi pi]);
L(5) = Revolute('alpha', -pi / 2, 'qlim', (2/3) * [-pi pi]);
L(6) = Revolute('d', .072, 'offset', pi, 'qlim', (20/9) * [-pi pi]);

i120 = SerialLink(L, 'name', 'IRB 120')

q = [0 0 0 0 -pi / 2 0];

qdot_lim = pi * [25/18 25/18 25/18 16/9 16/9 7/3];

%%

wn = pi / 10;

pds(t) = [0.05 * sin(wn * t) + .428 .02 .05 * cos(wn * t) + .669 0 0 0];

pddots = diff(pds);

ganho = 1.8;

e = inf(6, 1);

cont0 = tic;

t0 = tic;

contf = toc(cont0);

double(pds(0))

T = i120.fkine(double(pds(0))); % Pega pose desejada do efetuador
pd = transl(T); % Pega vetor de translação do efetuador
Rd = SO3(T); % Pega o objeto SO3 correspondente � rotação do efetuador
Rd = Rd.R; %Pega matriz de rotação do efetuador

Td = SE3(Rd, pd);

rpyd = rotm2axang(Rd);

epsilon = 2e-2

%%

i = 0
minicount = 0

testeTic = tic;
inicio = tic;

while (toc(inicio) < 15)% Critério de parada
    JCompleta = i120.jacob0(q, 'rpy'); % Jacobiana geométrica
    J = JCompleta(1:3, :);
    T = i120.fkine(q); % Cinemática direta para pegar a pose do efetuador
    p = transl(T); % translação do efetuador
    R = SO3();
    R = R.R; % Extrai rotação do efetuador
    i = i + 1; % contador
    minicount = minicount + 0.1;

    posicaoMomentanea = double(pds(minicount));

    p_err = posicaoMomentanea(1:3) - p; % Erro de translação

    nphi = rotm2axang(Rd * R');
    nphi_err = nphi(1:3) * nphi(4); % Erro de rotação (n*phi)

    e_ant = e;
    e = [p_err'; nphi_err']; % Vetor de erro

    e = e(1:3, :);
    derivada = double(pddots(testeTic));
    derivadaDiminuida = derivada(1:3);

    u = pinv(J) * ganho * (derivadaDiminuida' + e); % Lei de controle

    dt = toc(testeTic);
    testeTic = tic;

    q = q + 0.1 * u'; % C�lculo de posicaoInicial (Regra do trapézio)

    i120.plot(q);
    control_sig(:, i) = u; % Sinal de controle
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