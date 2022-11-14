%% Forward Kinematics
clc
clear

syms P1 P2 P3 q1 q2 q3
a = [P2 P3 0];
theta = [ q1 q2 0];
d = [P1 0 q3];
alpha = [ 0 0 0];

% NOTE: Please enter theta in degrees!

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

T30 = A1*A2*A3

