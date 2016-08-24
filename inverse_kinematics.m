% Function to determine the inverse kinematics for the manipulator

function [q q_dot]= inverse_kinematics(Y,Y_dot,a,d)

syms q1 q2 q3

Y1 = a(3)*cos(q1)*cos(q2 + q3) + d(4)*cos(q1)*sin(q2 + q3) + a(2)*cos(q1)*cos(q2) - d(2)*sin(q1) - Y(1);
Y2 = a(3)*sin(q1)*cos(q2 + q3) + d(4)*sin(q1)*sin(q2 + q3) + a(2)*sin(q1)*cos(q2) + d(2)*cos(q1) - Y(2);
Y3 = -1*a(3)*sin(q2 + q3) + d(4)*cos(q2 + q3) - a(2)*sin(q2) - Y(3);

[q_1,q_2,q_3] = solve(Y1,Y2,Y3,q1,q2,q3);
q_1 = double(q_1);
q_2 = double(q_2);
q_3 = double(q_3);

q = [q_1;q_2;q_3];

q_dot = [0;0;0];                                                           % q_dot is zero since y_dot is zero initially
end