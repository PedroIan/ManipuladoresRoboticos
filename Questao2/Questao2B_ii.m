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

else
    disp('CoppeliaSim não ativo, realizando simulações locais');
end

%% Inicialização dos Parâmetros

L(1) = Link('prismatic', 'alpha', pi / 2, 'qlim', [0 .5]);
L(2) = Revolute('d', .290, 'alpha', -pi / 2, 'qlim', (11/12) * [-pi pi]);
L(3) = Revolute('a', .270, 'offset', -pi / 2, 'qlim', (11/18) * [-pi pi]);
L(4) = Revolute('a', .070, 'alpha', -pi / 2, 'qlim', [-(11/18) * pi (7/18) * pi]);
L(5) = Revolute('d', 0.302, 'alpha', pi / 2, 'qlim', (8/9) * [-pi pi]);
L(6) = Revolute('alpha', -pi / 2, 'qlim', (2/3) * [-pi pi]);
L(7) = Revolute('d', .072, 'offset', pi, 'qlim', (20/9) * [-pi pi]);

i120 = SerialLink(L, 'name', 'IRB 120');
i120.base = trotx(-pi / 2);

q = [0 0 0 0 0 -pi / 2 0];

qdot_lim = [0.5 pi * 25/18 pi * 25/18 pi * 25/18 pi * 16/9 pi * 16/9 pi * 7/3];

%%

wn = pi / 10;

%20(sin(ωnt) + sin(4ωnt)) + 428, 500, 20(cos(ωnt) + cos(4ωnt)) + 569,
pds(t) = [0.02 * (sin(wn * t) + sin(4 * wn * t)) + .428 .05 .02 * (cos(wn * t) + cos(4 * wn * t)) + .569];

pddots = diff(pds);

ganho = 1.8;

e = inf(6, 1);

posicaoDesejada = [0.428 0.0 0.569]

% old Code
%T = i120.fkine(posicaoDesejada); % Pega pose desejada do efetuador
%pd = transl(T); % Pega vetor de translação do efetuador
% end

Rd = SO3();
Rd = Rd.R;

%Tdesejado = SE3(Rd, posicaoDesejada),

rpyd = rotm2eul(Rd);

epsilon = 2e-2;

%%

i = 0;

testeTic = tic;
inicio = tic;

%restart
rpyd = [0 0 0];

% Chegar na posição inicial do círculo
while (norm(e) > epsilon)% Critério de parada
    i = i + 1; % contador

    T = i120.fkine(q); % Cinemática direta para pegar a pose do efetuador
    J = i120.jacob0(q, 'rpy'); % Jacobiana geométrica
    p = transl(T); % translação do efetuador
    R = SO3(T);
    R = R.R; % Extrai rotação do efetuador

    rpy = rotm2eul(R);

    rpy_til = rpyd - rpy;

    p_err = posicaoDesejada - p; % Erro de translação

    nphi = rotm2axang(Rd * R');
    nphi_err = nphi(1:3) * nphi(4); % Erro de rotação (n*phi)

    e = [p_err'; rpy_til']; % Vetor de erro

    % derivada = double(pddots(testeTic));

    u = pinv(J) * ganho * e; % Lei de controle

    dt = toc(testeTic);
    testeTic = tic;

    for junta = 1:6

        u(junta) = rem(u(junta), qdot_lim(junta));

        newU = q(junta) + u(junta) * dt;

        if newU < i120.qlim(junta, 1)
            u(junta) = (i120.qlim(junta, 1) - q(junta)) / dt;
        elseif newU > i120.qlim(junta, 2)
            u(junta) = (i120.qlim(junta, 2) - q(junta)) / dt;
        end

        deslocamentos(junta, i) = 180 * q(junta) / pi;

    end

    q = q + 0.1 * u'; % C�lculo de posicaoInicial (Regra do trapézio)

    if (clientID >- 1)

        for i = 1:7
            sim.simxSetJointTargetPosition(clientID, h(i), q(i), sim.simx_opmode_streaming)
        end

    end

    i120.plot(q);
    control_sig(:, i) = u; % Sinal de controle
    err(i) = norm(e); % Norma do erro
    trajetoria(:, i) = p;
    erroGeral(:, i) = e;
    vetorRPY(:, i) = 180 * rpy / pi;
end

hold off

inicioCirculo = tic;
testeTic = tic;

while (toc(inicioCirculo) < 90)
    i = i + 1; % contador

    T = i120.fkine(q); % Cinemática direta para pegar a pose do efetuador
    J = i120.jacob0(q, 'rpy'); % Jacobiana geométrica
    p = transl(T); % translação do efetuador
    R = SO3(T);
    R = R.R; % Extrai rotação do efetuador

    rpy = rotm2eul(R);

    rpy_til = rpyd - rpy;

    p_err = double(pds(toc(inicioCirculo))) - p; % Erro de translação

    nphi = rotm2axang(Rd * R');
    nphi_err = nphi(1:3) * nphi(4); % Erro de rotação (n*phi)

    e = [p_err'; rpy_til']; % Vetor de erro

    pdsdot = double(pddots(toc(inicioCirculo)));

    u = pinv(J) * ([pdsdot 0 0 0]' + ganho * e); % Lei de controle

    dt = toc(testeTic);
    testeTic = tic;

    for junta = 1:7

        u(junta) = rem(u(junta), qdot_lim(junta));

        newU = q(junta) + u(junta) * dt;

        if newU < i120.qlim(junta, 1)
            u(junta) = (i120.qlim(junta, 1) - q(junta)) / dt;
        elseif newU > i120.qlim(junta, 2)
            u(junta) = (i120.qlim(junta, 2) - q(junta)) / dt;
        end

        deslocamentos(junta, i) = 180 * q(junta) / pi;

    end

    q = q + 0.1 * u'; % C�lculo de posicaoInicial (Regra do trapézio)

    if (clientID >- 1)

        for i = 1:7
            sim.simxSetJointTargetPosition(clientID, h(i), q(i), sim.simx_opmode_streaming)
        end

    end

    i120.plot(q);
    control_sig(:, i) = u; % Sinal de controle
    err(i) = norm(e); % Norma do erro
    trajetoria(:, i) = p;
    erroGeral(:, i) = e;
    vetorRPY(:, i) = 180 * rpy / pi;
end

%% Plot sinal de controle e norma do erro

figure(2)
sgtitle('Sinais de Controle');

for (i = 1:7)
    subplot(3, 3, i)
    plot(control_sig(i, :))
    title('Junta', i)
    xlabel('Iterações')
    ylabel('u(rad/s)')
    hold on
end

hold off
xlabel('Iterações')
ylabel('u(rad/s)')

figure(3)
sgtitle('Norma do Erro')
plot(err)
xlabel('Iterações')
ylabel('Norma do erro: |e|')
box off

hold on
figure (4)
sgtitle('Trajetória do efetuador')
subplot(2, 2, 1)
hold on
grid on
plot3(trajetoria(1, :), trajetoria(2, :), trajetoria(3, :))
view(3)
hold off
legend('Caminho percorrido(m)', 'Location', 'Best');

subplot(2, 2, 2)
hold on
grid on
plot(vetorRPY(1, :))
hold off
xlabel('Iterações')
ylabel('Ângulo(Roll)')
legend('Roll', 'Location', 'Best');

subplot(2, 2, 3)
hold on
grid on
plot(vetorRPY(2, :))
hold off
xlabel('Iterações')
ylabel('Ângulo(Pitch)')
legend('Pitch', 'Location', 'Best');

subplot(2, 2, 4)
hold on
grid on
plot(vetorRPY(3, :))
hold off
xlabel('Iterações')
ylabel('Ângulo(Yaw)')
legend('Yaw', 'Location', 'Best');

hold on

figure(5)
sgtitle('Erros de posição')
subplot(1, 3, 1)
hold on
grid on
plot(erroGeral(1, :))
hold off
xlabel('Iterações')
ylabel('Erro(m)')
legend('Erro em x', 'Location', 'Best');

subplot(1, 3, 2)
hold on
grid on
plot(erroGeral(2, :))

hold off
xlabel('Iterações')
ylabel('Erro(m)')
legend('Erro em y', 'Location', 'Best');

subplot(1, 3, 3)
hold on
grid on
plot(erroGeral(3, :))
hold off
xlabel('Iterações')
ylabel('Erro(m)')
legend('Erro em z', 'Location', 'Best');

hold on

figure(6)
sgtitle('Deslocamento das juntas')
subplot(3, 3, 1)
hold on
grid on
title('Junta 1')
plot(deslocamentos(1, :))
hold off
xlabel('Iterações')
legend('q_1', 'Location', 'Best');

subplot(3, 3, 2)
hold on
grid on
title('Junta 2')
plot(deslocamentos(2, :))
hold off
xlabel('Iterações')
legend('q_2', 'Location', 'Best');

subplot(3, 3, 3)
hold on
grid on
title('Junta 3')
plot(deslocamentos(3, :))
hold off
xlabel('Iterações')
legend('q_3', 'Location', 'Best');

subplot(3, 3, 4)
hold on
grid on
title('Junta 4')
plot(deslocamentos(4, :))
hold off
xlabel('Iterações')
legend('q_4', 'Location', 'Best');

subplot(3, 3, 5)
hold on
grid on
title('Junta 5')
plot(deslocamentos(5, :))
hold off
xlabel('Iterações')
legend('q_5', 'Location', 'Best');

subplot(3, 3, 6)
hold on
grid on
title('Junta 6')
plot(deslocamentos(6, :))
hold off
xlabel('Iterações')
legend('q_6', 'Location', 'Best');

subplot(3, 3, 7)
hold on
grid on
title('Junta 7')
plot(deslocamentos(7, :))
hold off
xlabel('Iterações')
legend('q_7', 'Location', 'Best');

hold on

sim.delete();
disp('Programa Finalizado');
