% Function to create the second order differential equatoins of the dynaics
% of the robotic manipulator

function dqdt = ode_dynamics_eqtn(t,q)
global tau; 
global D;
global c;
global g;
global r;

Dr = (1 + 0.01*r)*D;
cr = (1 + 0.01*r)*c;
gr = (1 + 0.01*r)*g;

% Dr = awgn(D,10);
% cr(:,:,1) = awgn(c(:,:,1),10);
% cr(:,:,2) = awgn(c(:,:,2),10);
% cr(:,:,3) = awgn(c(:,:,3),10);
% gr = awgn(g,10);

dqdt = zeros(6,1);

dqdt(1) = q(2);
dqdt(4) = (tau(1) - gr(1) - cr(1,1,1)*q(2)^2 - 2*cr(1,2,1)*q(2)*q(4) - 2*cr(1,3,1)*q(2)*q(6) - 2*cr(2,3,1)*q(4)*q(6) - cr(2,2,1)*q(4)^2 - cr(3,3,1)*q(6)^2 - Dr(1,2)*dqdt(4) - Dr(1,3)*dqdt(6))/Dr(1,1);
dqdt(2) = q(4);
dqdt(5) = (tau(2) - gr(2) - cr(1,1,2)*q(2)^2 - 2*cr(1,2,2)*q(2)*q(4) - 2*cr(1,3,2)*q(2)*q(6) - 2*cr(2,3,2)*q(4)*q(6) - cr(2,2,2)*q(4)^2 - cr(3,3,2)*q(6)^2 - Dr(2,1)*dqdt(2) - Dr(2,3)*dqdt(6))/Dr(2,2);
dqdt(3) = q(6);
dqdt(6) = (tau(3) - gr(3) - cr(1,1,3)*q(2)^2 - 2*cr(1,2,3)*q(2)*q(4) - 2*cr(1,3,3)*q(2)*q(6) - 2*cr(2,3,3)*q(4)*q(6) - cr(2,2,3)*q(4)^2 - cr(3,3,3)*q(6)^2 - Dr(3,1)*dqdt(2) - Dr(3,2)*dqdt(4))/Dr(3,3);

end