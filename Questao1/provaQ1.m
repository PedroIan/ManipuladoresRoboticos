clc
clear
close all

syms t;

    %% Inicialização dos Parâmetros

L(1) = Revolute('d', 2 , 'alpha', pi/2);
L(2) = Revolute('a', 3);
L(3) = Revolute('a', 3, 'alpha', pi/2);

i120 = SerialLink(L, 'name', 'IRB 120');

q = [0 0 0];


%% Definições de Controle

posicaoInicial = [0 0 0];
i120.plot(posicaoInicial); % Plot robô na configuração inicial
posicaoDesejada = [-pi pi pi];
JCompleta = i120.jacob0(posicaoDesejada)

T = i120.fkine(posicaoDesejada); % Pega pose desejada do efetuador
pd = transl(T); % Pega vetor de translação do efetuador
Rd = SO3(); % Pega o objeto SO3 correspondente � rotação do efetuador
Rd = Rd.R; %Pega matriz de rotação do efetuador

Td = SE3(Rd, pd);
Td.plot('rgb')

ganho = 0.8;
epsilon = 2e-2;

e_ant = 1;
e = inf(6, 1);

%% Plot inicial

figure(1)
i120.plot(posicaoInicial); % Plot robô na configuração inicial
hold on
Td.plot('rgb')% Plot pose desejada
%%
i = 0;

testeTic = tic;

while (norm(e) > epsilon)% Critério de parada

    JCompleta = i120.jacob0(posicaoDesejada, 'rpy'); % Jacobiana geométrica
    J = JCompleta(1:3, :);
    T = i120.fkine(q); % Cinemática direta para pegar a pose do efetuador
    p = transl(T); % translação do efetuador
    R = SO3();
    R = R.R; % Extrai rotação do efetuador
    i = i + 1; % contador

    p_err = pd - p; % Erro de translação

    nphi = rotm2axang(Rd * R');
    nphi_err = nphi(1:3) * nphi(4); % Erro de rotação (n*phi)

    e_ant = e;
    e = [p_err'; nphi_err']; % Vetor de erro

    e = e(1:3, :);

    u = pinv(J) * ganho * e; % Lei de controle

    dt = toc(testeTic);
    testeTic = tic;

    q = q + 0.1 * u'; % C�lculo de posicaoInicial (Regra do trapézio)

    i120.plot(q);
    control_sig(:, i) = u; % Sinal de controle
    err(i) = norm(e); % Norma do erro
    norm(e);
end

hold off

%% Plot sinal de controle e norma do erro

figure(2)
title('Sinais de Controle');

for (i = 1:6)
    subplot(3, 2, i)
    plot(control_sig(i, :))
    title('Junta', i)
    xlabel('Iterações')
    ylabel('Sinal de controle: u(rad/s)')
    hold on
end

hold off
xlabel('Iterações')
ylabel('Sinal de controle: u(rad/s)')

figure(3)
plot(err)
xlabel('Iterações')
ylabel('Norma do erro: |e|')
box off

    

sim.delete();
disp('Programa Finalizado');
