% Este script deve ser corrido antes de qualquer um dos ficheiros de
% Simulink. Este script define a dinâmica gravitacional, a
% matriz de massa e as velocidades ficticias do manipulador UR5. 
% Define também os ganhos dos controladores centralizado e descentralizado.
%
% Catarina Pires 90230
% Ricardo Henriques 90349
% Ultima alteração 28 Maio 2020
%% ----------------------------- Dynamics ---------------------------------
clear all

[DH,M] = Robot_G18();

syms dq1 dq2 dq3 dq4 dq5 dq6 ...
     ddq1 ddq2 ddq3 ddq4 ddq5 ddq6 real

acel_g = 9.81;
dq = [dq1 dq2 dq3 dq4 dq5 dq6]';
ddq = [ddq1 ddq2 ddq3 ddq4 ddq5 ddq6]';

w0_0 = [0;0;0];
dw0_0 = [0;0;0];
ddp0_0 = [0 0 acel_g]';

% Gravity Dynamics
g = Newton_Euler_G18(DH, M, zeros(size(DH,1),1), zeros(size(DH,1),1), zeros(3,1), zeros(3,1), ddp0_0);

% Acceleration Dynamics
tau_a = Newton_Euler_G18(DH, M, zeros(size(DH,1),1), ddq, zeros(3,1), dw0_0, zeros(3,1));
% Mass Matrix
B=jacobian(tau_a,ddq);

% Velocity Dynamics (Forças ficticias)
phi = Newton_Euler_G18(DH, M, dq, zeros(size(DH,1),1), w0_0, zeros(3,1), zeros(3,1));

%% Decentralized Control
B_WI = diag([4.818;4.696;0.9608;0.0121;0.004561;0.000275]);

% Low Gain
DK_pL = B_WI.*20^2;
DK_dL = 2*20*B_WI;

% Medium Gain
DK_pM = B_WI.*60^2;
DK_dM = 2*60*B_WI;

% High Gain
DK_pH = B_WI.*100^2;
DK_dH = 2*100*B_WI;

%% Centralized Control
% Low Gain
CK_pL = diag([400; 400; 400; 400; 400; 400]);
CK_dL = diag([40; 40; 40; 40; 40; 40]);

% Medium Gain
CK_pM = CK_pL.*9;
CK_dM = CK_dL.*3;

% High Gain
CK_pH = CK_pL.*25;
CK_dH = CK_dL.*5;

%% Generate Matlab and Simulink Code for Robot_G18
% %  Create Simulink file RobotG18_Blocos
% new_system('RobotG18_Blocos','Library');
% open_system('RobotG18_Blocos');
% 
% %Generate optimized embeded Matlab function blocks for Simulink
% matlabFunctionBlock('RobotG18_Blocos/Rigid_Body_Dynamics',B,phi,g);
% 
% % Save library in current Directory
% save_system('RobotG18_Blocos');
% close_system('RobotG18_Blocos');