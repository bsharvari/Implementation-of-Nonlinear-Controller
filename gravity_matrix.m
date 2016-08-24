% Function to compute the values of the generalized gravitational forces

function g = gravity_matrix(q)

g = zeros(3,1);

g(1)= 0;
g(2) = -1*48.5564*cos(q(2)) + 1.0462*sin(q(2)) + 0.3683*cos(q(2) + q(3)) - 10.6528*sin(q(2) + q(3));
g(3) = 0.3683*cos(q(2) + q(3)) - 10.6528*sin(q(2) + q(3));
end