% Function to design the task level controller

function tau = task_level_controller(U,q_dot,J,J_dot)

global D;
global C;
global g;

syms q1_dot q2_dot q3_dot
qdot = [q1_dot;q2_dot;q3_dot];

tau = D*inv(J)*(U - J_dot*q_dot) + C*qdot + g;

% Substituting the values of qdot to obtain the value of tau matrix
tau = subs(tau,qdot,q_dot);
end