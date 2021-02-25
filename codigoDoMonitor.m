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

ganho = 0.8;
epsilon = 2e-2;

e_ant = 1;
e = 0; 

%%

pd = [.38 .38 .5];
Rd = SO3();
Rd = Rd.R;
Td = SE3(Rd, pd);
rpyd = rotm2eul(Rd);

Td.plot('rgb')

%% Isso não tem nada a ver com o código do monitor. Daqui pra baixo é igual ao código da questão 1A_ii

i = 0

testeTic = tic;

while (norm(e - e_ant) > epsilon) % Crit�rio de parada
    J = i120.jacob0(q, 'rpy'); % Jacobiana geom�trica
    T = i120.fkine(q); % Cinem�tica direta para pegar a pose do efetuador 
    p = transl(T); % Transla��o do efetuador
    R = SO3(T); 
    R = R.R(); % Extrai rota��o do efetuador
    i = i+1; % contador
    
    p_err = pd-p; % Erro de transla��o
    
    nphi = rotm2axang(Rd*R'); 
    nphi_err = nphi(1:3)*nphi(4); % Erro de rota��o (n*phi)
    
    e_ant = e;
    e = [p_err'; nphi_err']; % Vetor de erro
    
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

