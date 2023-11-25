function [T] = DKin_G18( Robot )
%DKin Homogeneous Transformation Matrix for Robot Direct Kinematics and
%   T=[R p; 0 1]

n=size(Robot,1);      %get number of robot joints
q=symvar(Robot);      %get names of robot coordinates

%Direct kinematics for first link
T=DHTransf_G18(Robot(1,:));

for i=2:n
    %Direct kinematics for remaining links
    T=T*DHTransf_G18(Robot(i,:));
end

%Simplify
T=simplify(T);