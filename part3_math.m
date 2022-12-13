clc
clear

% Joint Velocities -> End Effector Velocities
% Joint 1 = Revolute
% Joint 2 = Revolute
% Joint 3 = Prismatic

qdot1 = 1;
qdot2 = 1;
qdot3 = 1;

jointVelocities = [qdot1 qdot2 qdot3];
endEffectorVelocity = Jacobian * jointVelocities




% End Effector Velocities -> Joint Velocities