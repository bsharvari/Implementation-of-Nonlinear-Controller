% Function to calculate the control input

function [U e e_dot] = control_input(Y,Y_dot,Yd,Yd_dot,Yd_ddot,kd,kp)

e_ddot = Yd_ddot;
e_dot = Yd_dot - Y_dot;
e = Yd - Y;

U = e_ddot + kd*e_dot + kp*e;
end