%% Comandos utilizados na explicação do Tutorial 3

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
run('D:/Dropbox/UFMG/Docencia/rvctools/startup.m')

%% Matriz Antissimetrica (Slides 20, 21 e 22)

clc, clear, close all

s = [1 4 7]'
S = skew(s)

S = [0 8 -5; -8 0 2; 5 -2 0]
s = vex(S)

s = 1
S = skew(s)

S = [0 -3; 3 0]
s = vex(S)

s = [1 1]'
S = skew(s)

S = [0 -1 2; 1 0 3]
s = vex(S)

S = [1 2 4; 2 pi 7; -5 -7 0]
s = vex(S)

doc skew
doc vex

%% Propriedades de so(3) (linearidade)

clc, clear, close all

a = randn(3,1)
b = randn(3,1)
alpha = randn()
beta = randn()

S = skew(alpha*a + beta*b)
S = alpha*skew(a) + beta*skew(b)

%% Propriedades de so(3) (produto externo)

clc, clear, close all

a = randn(3,1)
p = randn(3,1)

s = skew(a)*p
s = cross(a,p)

%% Propriedades de so(3) (propriedade auxiliar)

clc, clear, close all

a = randn(3,1)
b = randn(3,1)
R = SO3.rand()
R = R.R()

s = R * cross(a,b)
s = cross(R*a,R*b)

%% Jacobiana Geometrica (Slides 25 e 26)

clc, clear, close all

mdl_puma560

q = [0 0 0 0 0 0];
p560.plot(q)
J = p560.jacob0(q)

q = [-pi/4 pi/4 -3*pi/4 0 -pi/2 pi];
p560.plot(q)
J = p560.jacob0(q)

doc SerialLink/jacob0
doc SerialLink/jacobe

%% Plotagem do PUMA com configuracao nula

clc, clear, close all

mdl_puma560

q = [0 0 0 0 0 0]';

figure
p560.plot(q')

%% Controle Cinematico

pd = [0.0 0.6 0.2];
Rd = SO3.Rx(-pi/2);% * SO3.Rz(pi);
Rd = Rd.R();
Td = SE3(Rd,pd)

lambda = 0.3;
epsilon = 2e-2;

e = inf(6,1);
t = tic;

hold on
Td.plot('rgb')

while (norm(e) > epsilon)
    
    T = p560.fkine(q);
    J = p560.jacob0(q);
    
    p = transl(T);
    p_til = pd - p;
    
    R = SO3(T);
    R = R.R();
    R_til = Rd * R';
    
    nphi_til = rotm2axang2(R_til);
    nphi_til = nphi_til(1:3) * nphi_til(4);
    
    e = [p_til'; nphi_til'];
    
    u = pinv(J) * lambda * e;
    
    dt = toc(t);
    t = tic;
    
    q = q + u*dt;
    
    p560.plot(q')
    
end

hold off