function [DH,M] = Robot_G18()
%   Returns D-H table of parameters for Robotic Arm
%   Robot=[d v a alpha offset;
%          d v a alpha offset;
%          . . .   .   offset;
%          d v a alpha offset];
%   Use symbolic variables for each joint coordinate of the robot: in the d
%   column for a prismatic joint and in the v column for a rotational
%   joint. Name the variables from q1 to qn. In the last column, insert the
%   coordinate offset for the manipulator Home position.

% %% D-H table
syms q1 q2 q3 q4 q5 q6 real

       %   d |theta |  a  |alpha|offset|JointType
DH = [ 0.089    q1     0    pi/2    0     'R';
       0        q2   0.425    0     0     'R';
       0        q3   0.392    0     0     'R';
       0.109    q4     0   -pi/2    0     'R';
       0.095    q5     0    pi/2    0     'R';
       0.082    q6     0     0      0     'R'];

% Mass Parametres
% Centro de massa (na frame local)
M.rc{1} = [0 -0.005 0]' ;
M.rc{2} = [-0.2125 0 0.109]';
M.rc{3} = [-0.226 0 0.0135]'; 
M.rc{4} = [0 0 0]';
M.rc{5} = [0 0 0]';
M.rc{6} = [0 0 0]';

% Massas
M.m{1} = 1.388; %kg
M.m{2} = 1.388*2 + 6.0853; %kg
M.m{3} = 3.835 + 1.388 + 0.393; %kg
M.m{4} = 0.393; %kg
M.m{5} = 0.393; %kg
M.m{6} = 0.391; %kg

% Inércias
M.I{1} = diag([0.00306; 0.0028; 0.00306]); %kg.m^2

M.I{2} = diag([0.00306*2 + 0.00761; ...
    (0.00306 + M.m{1}*0.2125^2)*2 + 0.0488; ...
    (0.0028 + M.m{1}*0.2125^2)*2 + 0.0488]); %kg.m^2 ;

M.I{3} = diag([0.00306 + 0.000462 + 0.00263;...
    (0.00306 + M.m{1}*0.235^2) + (0.000462 + M.m{4}*0.2085^2) + (0.0389 + 3.835*0.0175^2); ...
    (0.0028 + M.m{1}*0.235^2) + (0.000269 + M.m{4}*0.2085^2) + (0.0389  + 3.835*0.0175^2)]);%kg.m^2 ;

M.I{4} = diag([0.000462; 0.000462; 0.000269]);%kg.m^2 ;

M.I{5} = diag([0.000462; 0.000462; 0.000269]);%kg.m^2 ;

M.I{6} = diag([0.00147;0.00147;0.000275]);%kg.m^2 ;
end