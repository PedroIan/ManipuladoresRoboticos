%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Manipuladores rob�ticos 2018/1                                   %
%                                                                  %
% Exemplo Rampa                                                    %
% Rafael Palma de Brito - 11/04                                    %
%                                                                  %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

close all
clear
clc

%inicializa��o das vari�veis

t = 0;
raio = 2;

% Trajet�ria do rob�

thetatraj = 0:0.01:2*pi;
for j = 1:1:length(thetatraj)
    
    ybase(j) = raio*sin(thetatraj(j)); %#ok!
    xbase(j) = raio*cos(thetatraj(j)); %#ok!
    zbase(j) = 0; %#ok!
    
end

ptraj = [xbase', ybase',zbase'];

% --

% Rampa

d = raio + raio/4;
aux = -d:0.1:d;
n = length(aux);

x1 = aux; x3 = aux; y2 = aux; y4 = aux;
y1= -d*ones(1,n); y3 = d*ones(1,n);
x2 = d*ones(1,n); x4 = -d*ones(1,n);

% Defin��o das quatro retas do ret�ngulo
p1 = [x1', y1', zeros(n,1)];
p2 = [x2', y2', zeros(n,1)];
p3 = [x3', y3', zeros(n,1)];
p4 = [x4', y4', zeros(n,1)];

% --

% Matriz de transforma��o homog�nia -> Inercial at� centro trajet�ria

Ric = SO3.Rx(45);
% Ric=[1 0 0;
%     0 cos(pi/4) -sin(pi/4);
%     0 sin(pi/4)  cos(pi/4)]; %rota��o de pi/4 no eixo x.

Ric2 = SO3();

pi_ic = [0 -1 1]'; %transla��o de y=1 e z=1
Hic = SE3(Ric.R(), pi_ic);

%--

% Transla��o e rota��o dos pontos para plot da trajet�ria

Rtraj = SO3();

for j = 1:1:length(thetatraj)
    Htraj= Hic*SE3(Rtraj.R(), ptraj(j,:));
    ptraj(j,:) = transl(Htraj);
end

% --

% Transla��o e rota��o dos pontos para plot da trajet�ria
for i = 1:n
    
    Hrr= Hic*SE3(Rtraj.R(), p1(i,:)');
    p1r(i,:) = transl(Hrr);%#ok!
    
    Hrr= Hic*SE3(Rtraj.R(), p2(i,:)');
    p2r(i,:) = transl(Hrr);%#ok!
    
    Hrr= Hic*SE3(Rtraj.R(), p3(i,:)');
    p3r(i,:) = transl(Hrr); %#ok!
    
    Hrr= Hic*SE3(Rtraj.R(), p4(i,:)');
    p4r(i,:) = transl(Hrr); %#ok!
    
end
% --


% Matriz de transforma��o homog�nia -> centro traj at� o rob�

Rcr = eye(3); %rota��o
pc_cr = [raio 0 0]'; %transla��o

Hcr = SE3(Rcr, pc_cr);

% --


while t<5
    
    Rt = [cos(2*pi*t) -sin(2*pi*t) 0;
        sin(2*pi*t)  cos(2*pi*t) 0;
        0            0           1]; % 1 volta completa em 1 segundo (w=2pi).
    pt = zeros(3,1);
    
    Ht = SE3(Rt, pt);
    % --
    
    % Matriz de transforma��o homog�nea total
    
    H = Hic*Ht*Hcr;
    % --
    
    % Plot trajet�ria e rampa
    
    plot3(ptraj(:,1),ptraj(:,2),ptraj(:,3))
    hold on
    plot3(p1r(:,1),p1r(:,2),p1r(:,3))
    plot3(p2r(:,1),p2r(:,2),p2r(:,3))
    plot3(p3r(:,1),p3r(:,2),p3r(:,3))
    plot3(p4r(:,1),p4r(:,2),p4r(:,3))
    axis([-4 4 -4 4 -1 3])
    
    % Plot sistema de coordenadas do rob�
    H.plot('rgb');
    % Plot sistema de coordenadas centro da trajet�ria
    Hic.plot('rgb');
    
    
    xlabel('x'); ylabel('y'); zlabel('z');
    hold off
    pause(0.2);
    t = t+0.1;
end