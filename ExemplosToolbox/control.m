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

thetad = [57.6 18.9 -70.2 72.8 56 16]'; % Define configuração desejada
T = p560.fkine(thetad); % Pega pose desejada do efetuador 
pd = transl(T); % Pega vetor de translação do efetuador
Rd = SO3(T); % Pega o objeto SO3 correspondente � rotação do efetuador
Rd = Rd.R(); %Pega matriz de rotação do efetuador

K = 1; % Define ganho
epsilon = 10e-5; % Define Critério de parada
e_ant = 1;
e = 0; 

i = 0;
theta = [0 0 0 0 0 0]'; % Define configuração inicial do robô

figure(1)
p560.plot(theta'); % Plot robô na configuração inicial
hold on
T.plot('rgb') % Plot pose desejada
%%
while (norm(e - e_ant) > epsilon) % Critério de parada
    i = i+1; % contador
    J = p560.jacob0(theta); % Jacobiana geométrica
    T = p560.fkine(theta); % Cinemática direta para pegar a pose do efetuador 
    p = transl(T); % translação do efetuador
    R = SO3(T); 
    R = R.R(); % Extrai rotação do efetuador
    
    p_err = pd-p; % Erro de translação
    
    nphi = rotm2axang2(Rd*R'); 
    nphi_err = nphi(1:3)*nphi(4); % Erro de rotação (n*phi)
    
    e_ant = e;
    e = [p_err'; nphi_err']; % Vetor de erro
    
    u = pinv(J)*K*e; % Lei de controle

    theta = theta + 0.1*u; % C�lculo de theta (Regra do trapézio)
    
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
xlabel('Iterações')
ylabel('Sinal de controle: u(rad/s)')

figure(3)
plot(err)
xlabel('Iterações')
ylabel('Norma do erro: |e|')
box off

