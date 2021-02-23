%% Comandos utilizados na explicação do Tutorial 2

% Manipuladores Roboticos 2020/1
% Rafael Fernandes Goncalves da Silva
% Universidade Federal de Minas Gerais

% Rodar este codigo linha por linha (F9)
% ou secao por secao (Ctrl+Enter)

% Para exemplificar o funcionamento das funcoes,
% algumas linhas deste codigo retornam erro

% Para ocultar os warnings
%#ok<*NOPTS,*NASGU,*ASGLU>

% Alterar para o caminho da toolbox
% run('rvctools/startup.m')

%% Links (Slide 4)

clc, clear, close all

L = Link('revolute', 'd', 1.2, 'a', 0.3, 'alpha', pi/2)

L = Revolute('d', 0, 'a', 0, 'alpha', pi/2)

L = Prismatic('theta', pi, 'a', 0, 'alpha', 0)

L = Revolute()

L = Prismatic('offset', 0.2)

L = Link('d', 1.2)
L = Link('theta', 0)

doc Revolute
doc Prismatic

%% Manipulador Planar (Slides 5, 6 e 7)

clc, clear, close all

L1 = Revolute('d', 0, 'a', .5, 'alpha', 0)
L2 = Revolute('d', 0, 'a', .5, 'alpha', 0)

robot1 = SerialLink([L1 L2], 'name', 'planar1')

robot2 = L1 + L2;
robot2.name = 'planar2'

q = [pi/4 -pi/4]

figure('position',[280 70 560 420])
robot1.plot(q)
view(0,90)

figure('position',[280 70 560 420])
robot2.plot(q)
view(0,90)

%% Manipulador Planar
% Animate

qf = [pi/3 -pi/1.5]
robot2.animate(qf)

% doc SerialLink/plot

%% Manipulador Cartesiano
% ## ERRO ## Tenta plotar uma junta sem conhecer o limite dela

clc, clear, close all

L1 = Prismatic('alpha', -pi/2)
L2 = Prismatic('alpha', -pi/2)
L3 = Prismatic('alpha', 0)

robot = SerialLink([L1 L2 L3], 'name', 'cartesiano')

q = [0.5 0.4 0.3];

figure('position',[280 70 560 420])
robot.plot(q)

%% Manipulador Cartesiano
% ## OK ## Especificando o qlim

L1.qlim = 0.5;
L2.qlim = 0.5;
L3.qlim = 0.5;

robot = SerialLink([L1 L2 L3], 'name', 'cartesiano');

figure('position',[280 70 560 420])
robot.plot(q)

%% Manipulador Cartesiano
% ## ERRO ## Tenta "somar" mais de duas juntas

clc, clear, close all

L1 = Prismatic('alpha', -pi/2, 'qlim', 0.5)
L2 = Prismatic('alpha', -pi/2, 'qlim', 0.5)
L3 = Prismatic('alpha', 0, 'qlim', 0.5)

robot = L1 + L2 + L3;
robot.name = 'cartesiano'

q = [0.5 0.4 0.3];

figure('position',[280 70 560 420])
robot.plot(q)

%% Manipulador SCARA (pag. 92 do Spong)
% ## OK ## Porem cria n variaveis

clc, clear, close all

L1 = Revolute('d', 0.4, 'a', 0.3);
L2 = Revolute('a', 0.3, 'alpha', pi);
L3 = Prismatic('qlim', 0.5);
L4 = Revolute('d', 0.3);

robot = SerialLink([L1 L2 L3 L4], 'name', 'SCARA')

q = [pi/4 -pi/4 0.4 pi/2];

figure('position',[280 70 560 420])
robot.plot(q)

%% Manipulador SCARA (pag. 92 do Spong)
% ## ERRO ## Tenta concatenar tipos de juntas diferentes

clc, clear, close all

L(1) = Revolute('d', 0.4, 'a', 0.3);
L(2) = Revolute('a', 0.3, 'alpha', pi);
L(3) = Prismatic('qlim', 0.5);
L(4) = Revolute('d', 0.3);

robot = SerialLink(L, 'name', 'SCARA')

q = [pi/4 -pi/4 0.4 pi/2];

figure('position',[280 70 560 420])
robot.plot(q)

%% Manipulador SCARA (pag. 92 do Spong)
% ## Ok ## Melhor forma (ou colocar tudo como Link)

clc, clear, close all

L(1) = Link('revolute', 'd', 0.4, 'a', 0.3);
L(2) = Revolute('a', 0.3, 'alpha', pi);
L(3) = Prismatic('qlim', 0.5);
L(4) = Revolute('d', 0.3);

robot = SerialLink(L, 'name', 'SCARA')

q = [pi/4 -pi/4 0.4 pi/2];

figure('position',[280 70 560 420])
robot.plot(q)

%% Unimate PUMA 560 (Slides 11 e 12)

clc, clear, close all

L(1) = Revolute('d', 0.4, 'alpha', pi/2, 'offset', pi/2);
L(2) = Revolute('a', 0.4318);
L(3) = Revolute('d', 0.15005, 'a', 0.0203, 'alpha', -pi/2);
L(4) = Revolute('d', 0.4318, 'alpha', pi/2);
L(5) = Revolute('alpha', -pi/2);
L(6) = Revolute();

p560 = SerialLink(L, 'name', 'Puma 560')

q = [0 0 0 0 0 0];

figure('position',[280 70 560 420])
p560.plot(q)
view([60 30])

%% Cinematica Direta (Slide 15)
% Com configuracao nula

clc, clear, close all

L(1) = Revolute('alpha', pi/2);
L(2) = Revolute('a', 0.4318);
L(3) = Revolute('d', 0.15005, 'a', 0.0203, 'alpha', -pi/2);
L(4) = Revolute('d', 0.4318, 'alpha', pi/2);
L(5) = Revolute('alpha', -pi/2);
L(6) = Revolute();

p560 = SerialLink(L, 'name', 'Puma 560');

q = [0 0 0 0 0 0];
figure('position',[280 70 560 420])
p560.plot(q)
T = p560.fkine(q)

%% Cinematica Direta (Slide 15)
% Com configuracao arbitraria

q = [-pi/4 pi/4 -3*pi/4 0 -pi/2 pi];
figure('position',[280 70 560 420])
p560.plot(q)
T = p560.fkine(q)

% doc SerialLink/fkine

%% Cinematica Inversa (Slide 18)
% Para configuracao nula

clc, clear, close all

L(1) = Revolute('alpha', pi/2);
L(2) = Revolute('a', 0.4318);
L(3) = Revolute('d', 0.15005, 'a', 0.0203, 'alpha', -pi/2);
L(4) = Revolute('d', 0.4318, 'alpha', pi/2);
L(5) = Revolute('alpha', -pi/2);
L(6) = Revolute();

p560 = SerialLink(L, 'name', 'Puma 560');

T = SE3([1 0 0 0.4521; 0 1 0 -0.15005; 0 0 1 0.4318; 0 0 0 1])
q = p560.ikine(T)
figure('position',[280 70 560 420])
p560.plot(q)

%% Cinematica Inversa (Slide 18)
% ## ERRO ## Com configuracao arbitraria

R = SO3.Rx(pi) * SO3.Rz(pi/6);
T = SE3(R);
T.t = [0.5; -0.5; 0]
q = p560.ikine(T)

%% Cinematica Inversa (Slide 18)
% ## OK ## Com configuracao inicial não nula

q0 = [0 0 -pi/2 0 0 pi]
figure('position',[280 70 560 420])
p560.plot(q0)

%% Cinematica Inversa (Slide 18)
% ## OK ## Com configuracao arbitraria

q = p560.ikine(T, 'q0', q0)
figure('position',[280 70 560 420])
p560.plot(q)

% doc SerialLink/ikine