function Robot = Robot_G18()
%   Returns D-H table of parameters for Robotic Arm
%   Robot=[d v a alpha offset;
%          d v a alpha offset;
%          . . .   .   offset;
%          d v a alpha offset];
%   Use symbolic variables for each joint coordinate of the robot: in the d
%   column for a prismatic joint and in the v column for a rotational
%   joint. Name the variables from q1 to qn. In the last column, insert the
%   coordinate offset for the manipulator Home position.

syms q1 q2 q3 q4 q5 q6 real

       %   d   |theta |  a  |alpha|offset|JointType
Robot = [ 0.089    q1    0    pi/2    0; %    'R';
          0        q2  0.425    0     0; %    'R';
          0        q3  0.392    0     0; %    'R';
          0.109    q4    0   -pi/2    0; %    'R';
          0.095    q5    0    pi/2    0; %    'R';
          0.082    q6    0     0      0];%    'R'];

end