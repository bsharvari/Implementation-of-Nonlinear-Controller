% Function to compute the Jacobian and its derivative

function [J J_dot] = jacobian(q,d,a)

syms q1 q2 q3
q_diff = [q1;q2;q3];
J = sym(zeros(3,3));
J_dot = sym(zeros(3,3));

h(1) = a(3)*cos(q1)*cos(q2 + q3) + d(4)*cos(q1)*sin(q2 + q3) + a(2)*cos(q1)*cos(q2) - d(2)*sin(q1);
h(2) = a(3)*sin(q1)*cos(q2 + q3) + d(4)*sin(q1)*sin(q2 + q3) + a(2)*sin(q1)*cos(q2) + d(2)*cos(q1);
h(3) = -1*a(3)*sin(q2 + q3) + d(4)*cos(q2 + q3) - a(2)*sin(q2);

% Calculation of Jacobian in symbolic form
for i = 1:3
    for j = 1:3
        J(i,j) = diff (h(i),q_diff(j));
    end
end

% Calculation of the derivative of Jacobian in symbolic form
for i = 1:3
    for j = 1:3
        J_dot(i,j) = diff (J(i,j),q_diff(j));
    end
end

% Substituting the values of q, to obtain the value of the Jacobian and its
% derivative
J = subs(J,[q1,q2,q3],[q(1),q(2),q(3)]);
J_dot = subs(J_dot,[q1,q2,q3],[q(1),q(2),q(3)]); 
end