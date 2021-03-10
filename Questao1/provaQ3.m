%% limpeza de workspace
clc
clear
close all

 

%% calculo

 

R = [1 0 0; 0 0 -1; 0 -1 0];
v1 = [1; 3; 0];
w1 = [1; -2; 3];
p1 = [0;1; -3];

 

P0 = R*(v1 + skew(w1)*p1)