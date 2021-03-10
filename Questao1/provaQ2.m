clc
clear
close all


J = [-5 0 0 0; -3 0 0 3; 0 -5 3 0; 0 -1 0 0; 0 0 1 0; 1 0 0 -1]

Jnovo = zeros(4);
Jnovo(1,:) = J(1,:);
Jnovo(2,:) = J(4,:);
Jnovo(3,:) = J(5,:);
Jnovo(4,:) = J(6,:);

vetorVelocidade = [3,-2,2,-1];

resultado = Jnovo\vetorVelocidade'