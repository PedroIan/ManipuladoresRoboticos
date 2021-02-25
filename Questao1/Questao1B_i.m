clc
clear
close all

syms t;

%%

L(1) = Revolute('d', .290, 'alpha', -pi/2, 'qlim', (11/12)*[-pi pi]);
L(2) = Revolute('a', .270, 'offset', -pi/2, 'qlim', (11/18)*[-pi pi]);
L(3) = Revolute('a', .070, 'alpha', -pi/2, 'qlim', [-(11/18)*pi (7/18)*pi]);
L(4) = Revolute('d', 0.302, 'alpha', pi/2, 'qlim', (8/9)*[-pi pi]);
L(5) = Revolute('alpha', -pi/2, 'qlim', (2/3)*[-pi pi]);
L(6) = Revolute('d', .072, 'offset', pi, 'qlim', (20/9)*[-pi pi]);

i120 = SerialLink(L, 'name', 'IRB 120')

q = [0 0 0 0 -pi/2 0];

qdot_lim = pi*[25/18 25/18 25/18 16/9 16/9 7/3];

%%

wn = pi / 10;

pds(t) = [0.05*sin(wn*t)+.428 .200 .05*cos(wn*t)+.669 0 0 0];


pddots = diff(pds);

lambda = 1.8;

e = inf(6 ,1);

cont0 = tic;

t0 = tic;

contf = toc(cont0);

Rd = SO3;

Rd = Rd.R();

%Td = SE3(Rd, double(pds(contf)));

rpyd = rotm2axang(Rd);

epsilon = 2e-2

%%

i = 0

testeTic = tic;

while (norm(e) > epsilon) % Crit�rio de parada
    J = i120.jacob0(q, 'rpy'); % Jacobiana geom�trica
    T = i120.fkine(q); % Cinem�tica direta para pegar a pose do efetuador 
    p = transl(T); % Transla��o do efetuador
    R = SO3(T); 
    R = R.R(); % Extrai rota��o do efetuador
    i = i+1; % contador
    
    p_err = pds(testeTic)-p; % Erro de transla��o
    
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