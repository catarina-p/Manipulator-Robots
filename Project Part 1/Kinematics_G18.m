% Este script define a a tabela Denavit-Hartenberg do manipulador,
% a partir da mesma define a matriz de rota��o do end-effector no 
% referencial 0, R06, e a posi��o final do end-effector no referencial 0,
% p6, e a matriz Jacobiana Geom�trica em fun��o dos �ngulos de rota��o nas 
% juntas. Tamb�m cria um bloco de simulink com os �ngulos nas juntas como 
% argumento de entrada, e estas matrizes como argumento de sa�da.

% Catarina Pires 90230
% Ricardo Henriques 90349
% �ltima altera��o a 23 de Abril 2021
%-------------------------------------------------------------------------
%% Denavit-Hartenberg
clear all
DH = Robot_G18();

%% Direct Kinematics
A06 = DKin_G18(DH);
R06 = A06(1:3,1:3);
p6 = A06(1:3,4);

%% Geometric Jacobian
[J] = GeoJacobian (DH,'RRRRRR');
J = simplify(J);

% %% Generate Matlab and Simulink Code for Robot_G18
% %  Create Simulink file RobotG18_Blocos
% new_system('RobotG18_Blocos','Library');
% open_system('RobotG18_Blocos');
% 
% %Generate optimized embeded Matlab function blocks for Simulink
% matlabFunctionBlock('RobotG18_Blocos/RobotG18_Direct_Kinematics',R06,p6,J);
% 
% % Save library in current Directory
% save_system('RobotG18_Blocos');
% close_system('RobotG18_Blocos');

