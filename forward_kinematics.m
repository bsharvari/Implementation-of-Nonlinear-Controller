% Function to compute the forward kinematics of the manipulator

function [Y Y_dot]= forward_kinematics(q,q_dot,d,a)

h1 = a(3)*cos(q(1))*cos(q(2) + q(3)) + d(4)*cos(q(1))*sin(q(2) + q(3)) + a(2)*cos(q(1))*cos(q(2)) - d(2)*sin(q(1));
h2 = a(3)*sin(q(1))*cos(q(2) + q(3)) + d(4)*sin(q(1))*sin(q(2) + q(3)) + a(2)*sin(q(1))*cos(q(2)) + d(2)*cos(q(1));
h3 = -1*a(3)*sin(q(2) + q(3)) + d(4)*cos(q(2) + q(3)) - a(2)*sin(q(2));

Y = [h1;h2;h3];

J = jacobian(q,d,a);
Y_dot = J*q_dot;
end