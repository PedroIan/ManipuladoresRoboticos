%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Puma 560 control                                                    % 
%                                                                     %
% describes the kinematic control                                     %                                                                     %
% Adapted by Rafael Palma e Mariana Fonseca                           %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear
close all
clc

% Carrega modelo
mdl_puma560

% Controle

thetad = [57.6 18.9 -70.2 72.8 56 16]'; % Define configura��o desejada
T = p560.fkine(thetad); % Pega pose desejada do efetuador 
pd = transl(T); % Pega vetor de transla��o do efetuador
Rd = SO3(T); % Pega o objeto SO3 correspondente � rota��o do efetuador
Rd = Rd.R(); %Pega matriz de rota��o do efetuador

K = 1; % Define ganho
epsilon = 10e-5; % Define crit�rio de parada
e_ant = 1;
e = 0; 

i = 0;
theta = [0 0 0 0 0 0]'; % Define configura��o inicial do rob�

figure(1)
p560.plot(theta'); % Plot rob� na configura��o inicial
hold on
T.plot('rgb') % Plot pose desejada
%%
while (norm(e - e_ant) > epsilon) % Crit�rio de parada
    i = i+1; % contador
    J = p560.jacob0(theta); % Jacobiana geom�trica
    T = p560.fkine(theta); % Cinem�tica direta para pegar a pose do efetuador 
    p = transl(T); % Transla��o do efetuador
    R = SO3(T); 
    R = R.R(); % Extrai rota��o do efetuador
    
    p_err = pd-p; % Erro de transla��o
    
    nphi = rotm2axang2(Rd*R'); 
    nphi_err = nphi(1:3)*nphi(4); % Erro de rota��o (n*phi)
    
    e_ant = e;
    e = [p_err'; nphi_err']; % Vetor de erro
    
    u = pinv(J)*K*e; % Lei de controle

    theta = theta + 0.1*u; % C�lculo de theta (Regra do trap�zio)
    
    p560.plot(theta');
    control_sig(:,i) = u; % Sinal de controle
    err(i) = norm(e); % Norma do erro
    norm(e)
end
hold off

%% Plot sinal de controle e norma do erro

figure(2)
hold on
for i = 1:6
    plot(control_sig(i,:))
end
hold off
xlabel('Itera��es')
ylabel('Sinal de controle: u(rad/s)')

figure(3)
plot(err)
xlabel('Itera��es')
ylabel('Norma do erro: |e|')
box off

