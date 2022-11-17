% Group Project Part 1 Forward and Inverse Kinematics
% Last updated 11/14/2022

clc
clear

% Forward Kinematics

% Link lengths from Gazebo 
P1 = 2;
P2 = 1;
P3 = 1;

% This will come from ROS, current values are arbitrary for testing
q1 = 20;
q2 = 35;
q3 = 0.5;

% DH parameters from hand-written work
a = [P2 P3 0];
theta = [ q1 q2 0];
d = [P1 0 q3];
alpha = [ 0 0 0];

% A10, A21, and A32 transformation matrices
i = 1;
A1 = [cosd(theta(i)) (-sind(theta(i))*cosd(alpha(i))) (sind(theta(i))*sind(alpha(i))) (a(i)*cosd(theta(i)));
    sind(theta(i)) (cosd(theta(i))*cosd(alpha(i))) (-cosd(theta(i))*sind(alpha(i))) (a(i)*sind(theta(i)));
    0 sind(alpha(i)) cosd(alpha(i)) d(i);
    0 0 0 1];

i = 2;
A2 = [cosd(theta(i)) (-sind(theta(i))*cosd(alpha(i))) (sind(theta(i))*sind(alpha(i))) (a(i)*cosd(theta(i)));
    sind(theta(i)) (cosd(theta(i))*cosd(alpha(i))) (-cosd(theta(i))*sind(alpha(i))) (a(i)*sind(theta(i)));
    0 sind(alpha(i)) cosd(alpha(i)) d(i);
    0 0 0 1];

i = 3;
A3 = [cosd(theta(i)) (-sind(theta(i))*cosd(alpha(i))) (sind(theta(i))*sind(alpha(i))) (a(i)*cosd(theta(i)));
    sind(theta(i)) (cosd(theta(i))*cosd(alpha(i))) (-cosd(theta(i))*sind(alpha(i))) (a(i)*sind(theta(i)));
    0 sind(alpha(i)) cosd(alpha(i)) d(i);
    0 0 0 1];

% End effector pose
T30 = A1*A2*A3;
end_effector_pose = T30 % display whole end-effector T matrix
end_effector = T30(1:3,4); % use only position part of T

% Inverse Kinematics

%These will come from the forward kinematics
x30 = end_effector(1);
y30 = end_effector(2);
z30 = end_effector(3);

q3 = z30 - P1;

r = sqrt((y30^2) + (x30^2));

% Needed angles from law of cosines
alpha = acos(((r^2) + (P3^2) - (P2^2))/(2*r*P3));
gamma = acos(((P2^2)+(r^2)-(P3^2))/(2*P2*r));
beta = 180 - alpha - gamma;

q2 = 180 - beta;
q1 = atan(y30/x30)-gamma;

% Calculated joint values
joint_angles = [(q1/(pi/180)); (q2/(pi/180)); q3]



