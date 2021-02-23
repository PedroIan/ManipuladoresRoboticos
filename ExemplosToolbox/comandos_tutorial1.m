%% Comandos utilizados na explicação do Tutorial 1

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

%% Translacao (Slides 4 e 5)

clear, clc

x = 2, y = 3, z = 4
T = transl(x,y,z)

p = [2,3,4]
T = transl(p)

p = [2;3;4]
T = transl(p)

T = [0,-1,0,2;0,0,1,3;-1,0,0,4;0,0,0,1]
p = transl(T)

T = [0,-1,0,2;0,0,1,3;-1,0,0,4;0,0,0,1]
[x,y,z] = transl(T)

doc transl

%% Rotacao (Slides 6 e 7)

clear, clc

r = SO3()

R = [0,-1,0;0,0,1;-1,0,0]
r = SO3(R)

T = [0,-1,0,2;0,0,1,3;-1,0,0,4;0,0,0,1]
r = SO3(T)

q = pi
r = SO3.Rx(q)

q = pi/2
r = SO3.Ry(q)

q = -pi/4
r = SO3.Rz(q)

doc SO3

%% Exercicio Rotacao (Slides 8 e 9)

clear, clc

rx = SO3.Rx(pi/2)
ry = SO3.Ry(pi/2)
rx*ry
ry*rx

%% Transformacao Homogenea (Slides 10 e 11)

clear, clc

t = SE3()

x = 2, y = 3, z = 4
t = SE3(x,y,z)

p = [2,3,4]
t = SE3(p)

p = [2;3;4]
t = SE3(p)

R = [0,-1,0;0,0,1;-1,0,0]
r = SO3(R)
t = SE3(r)

t = SE3(R)  % Forma Incorreta

T = [0,-1,0,2;0,0,1,3;-1,0,0,4;0,0,0,1]
t = SE3(T)

t = SE3(R,p)
t = SE3(R,p')

t = SE3(r,p)     % Forma Incorreta
t = SE3(R,x,y,z) % Forma Incorreta

q = pi/4
t = SE3.Rx(q)

doc SE3

%% Exercicio Transformacao Homogenea (Slide 12)

clear, clc

tx = SE3.Rx(pi/2)
ty = SE3.Ry(pi/2)
tx*ty
ty*tx

%% Plot e Animate (Slides 13 e 14)

clear, clc, close all

T = [0,-1,0,2;0,0,1,3;-1,0,0,4;0,0,0,1]
t = SE3(T)

t.plot
t.plot('rgb')
t.animate('rgb')

%% Exercicio 9 (Lista 1)

clear, clc, close all

phi = pi/2
theta = pi
psi = 3*pi/2

r1 = SO3.Rx(phi)
r2 = SO3.Rz(theta)
r3 = SO3.Ry(psi)

r = r3*r1*r2

%% Animacao do Exercicio 9 (Lista 1)

clear, clc, close all

ra = SO3()
rb = SO3.Rx(pi/2) * ra
rc = rb * SO3.Rz(pi - 1e-6)  % Valores alterados para
rd = SO3.Ry(3*pi/4) * rc     % visualizar a rotacao
re = SO3.Ry(3*pi/4) * rd     % no sentido correto

ra.plot('rgb')
axis([-1.5 1.5 -1.5 1.5 -1.5 1.5])
drawnow, pause

ra.animate(rb,'rgb')
drawnow, pause

rb.animate(rc,'rgb')
drawnow, pause

rc.animate(rd,'rgb')
rd.animate(re,'rgb')

%% Exercicio 10 (Lista 1)

clear, clc, close all

r0 = SO3()
r0.plot('rgb')

h1 = SO3.Rx(-pi/2)
r1 = h1 * r0
r0.animate(r1,'rgb')

h2 = SO3.Rz(pi)
r2 = h2 * r1
r1.animate(r2,'rgb')

h3 = SO3.Rx(pi/2)
r3 = r2 * h3
r2.animate(r3,'rgb')

r3 = h2 * h1 * h3