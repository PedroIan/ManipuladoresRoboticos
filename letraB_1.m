syms t;

%%%%

L(1) = Revolute('d', .290, 'alpha', -pi/2, 'qlim', (11/12)*[-pi pi]);
L(2) = Revolute('a', .270, 'offset', -pi/2, 'qlim', (11/18)*[-pi pi]);
L(3) = Revolute('a', .070, 'alpha', -pi/2, 'qlim', [-(11/18)*pi (7/18)*pi]);
L(4) = Revolute('d', 0.302, 'alpha', pi/2, 'qlim', (8/9)*[-pi pi]);
L(5) = Revolute('alpha', -pi/2, 'qlim', (2/3*[-pi pi]));
L(6) = Revolute('d', .072, 'offset', pi, 'qlim', (20/9)*[-pi pi]);

i120 = SerialLink(L, 'name', 'IRB 120')

q = [0 0 0 0 -pi/2 0];

qdot_lim = pi*[25/18 25/18 25/18 16/9 16/9 7/3];

%%%%

wn = pi / 10;

pds(t) = [0.05*sin(wn*t)+.428 .500 .05*cos(wn*t)+.569];


pddots = diff(pds);

lambda = 1.8;

e = inf(6 ,1);

cont0 = tic;

t0 = tic;

contf = toc(cont0);

Rd = SO3;

Rd = Rd.R();

Td = SE3(Rd, double(pds(contf)));

rpyd = rotm2eul(Rd);


%%%% Controle

T = i120.fkine(q);

J = i120.jacob0(q, 'rpy');

p = trans1(T);
pd = double(pds(contf));
p_til = pd - p;

R = SO3(T);
R = R.R();

rpy = rotm2eul(R);
rpy_til = rpyd - rpy;

e = [p_til'; rpy_til'];

pddot = [double(pddots(contf)) 0 0 0]

u = pinv(J)*pddot' + lambda*e);