clc
clear
close all

syms t;

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

pds(t) = [0.02 * (sin(wn * t) + sin(4 * wn * t)) + .428 .020 .02 * (cos(wn * t) + cos(4 * wn * t)) + .669 0 0 0];

pddots = diff(pds);

ganho = 1.8;

e = inf(6, 1);

cont0 = tic;

t0 = tic;

contf = toc(cont0);

double(pds(0))

T = i120.fkine(double(pds(0))); % Pega pose desejada do efetuador
pd = transl(T); % Pega vetor de transla��o do efetuador
Rd = SO3(T); % Pega o objeto SO3 correspondente � rota��o do efetuador
Rd = Rd.R; %Pega matriz de rota��o do efetuador

%Td = SE3(Rd, double(pds(contf)));

rpyd = rotm2eul(Rd);

epsilon = 2e-2

%%

i = 0

testeTic = tic;
inicio = tic;

while (toc(inicio) < 15)% Crit�rio de parada
    J = i120.jacob0(q, 'rpy'); % Jacobiana geom�trica
    T = i120.fkine(q); % Cinem�tica direta para pegar a pose do efetuador
    p = transl(T); % Transla��o do efetuador
    R = SO3(T);
    R = R.R; % Extrai rota��o do efetuador
    i = i + 1; % contador

    posicaoMomentanea = double(pds(testeTic));

    rpy = rotm2eul(R);

    rpy_til = rpyd - rpy;

    p_err = posicaoMomentanea(1:3) - p; % Erro de transla��o

    nphi = rotm2axang(Rd * R');
    nphi_err = nphi(1:3) * nphi(4); % Erro de rota��o (n*phi)

    e_ant = e;
    e = [p_err'; rpy_til']; % Vetor de erro

    derivada = double(pddots(testeTic));

    u = pinv(J) * ganho * (derivada' + e); % Lei de controle

    dt = toc(testeTic);
    testeTic = tic;

    q = q + dt * u'; % C�lculo de posicaoInicial (Regra do trap�zio)

    i120.plot(q);
    control_sig(:, 1) = u; % Sinal de controle
    err(i) = norm(e); % Norma do erro
    norm(e)
end

hold off
