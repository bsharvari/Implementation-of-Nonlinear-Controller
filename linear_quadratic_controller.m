% Function to design the Linear Quadratic Controller (LQ)

function U = linear_quadratic_controller(Yd,Yd_dot)

A = [0 1 0 0 0 0;0 0 0 0 0 0;0 0 0 1 0 0;0 0 0 0 0 0;0 0 0 0 0 1;0 0 0 0 0 0];
B = [0 0 0;1 0 0;0 0 0;0 1 0;0 0 0;0 0 1];
Q = 10*eye(6,6);
R = 13*eye(3,3);

[K,P,E] = lqr(A,B,Q,R);
x = vertcat(Yd,Yd_dot);
U = -1*K*x;
end