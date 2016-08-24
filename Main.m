clc;
clear all;
close all;

global tau;
global D;
global c;
global C;
global g;
global r;

tau = zeros(3,1);
D = zeros(3,3);
c = zeros(3,3);
C = zeros(3,3);
g = zeros(3,1);

% DH Parametes
a = [0;0.4318;-0.02032;0];
d = [0;0.14909;0;0.43307];

% Initial conditions
Y = [-0.7765;0.0;0.045];
%Y = [-0.5;-0.1;0];
Y_dot = [0;0;0];

% Computing the matrix dynamic equations
[q q_dot] = inverse_kinematics(Y,Y_dot,a,d);

% Parameters for calculating the desired trajectory
R = 0.25;
w = pi/4;

% The velocity and position gain parameters
kd = 0.01*[1 0 0;0 1 0;0 0 1];
kp = 0.001*[1 0 0;0 1 0;0 0 1];

q_initial = q;
q_dot_initial = q_dot;

% Variation of the parameters of dynamics model, from their nominal values
r = 0;

for t = 0:0.01:6
    D = inertial_matrix(q);
    [c, C] = christoffel_symbols(q);
    g = gravity_matrix(q);
    
    % Computation of the desired trajectory, to calculate the error
    [Yd Yd_dot Yd_ddot] = desired_trajectory(w,t,R);    
    e = Yd - Y;
     
    % Computation of Jacobian
    [J J_dot] = jacobian(q,d,a);
    
    % Generating the control input
    [U e e_dot] = control_input(Y,Y_dot,Yd,Yd_dot,Yd_ddot,kd,kp);
    %U = linear_quadratic_controller(Yd,Yd_dot);
    %U = [0;0;1];

    % Plotting the errors
    figure(1);
    plot(t,e,'+');
    hold on;
    
    figure(2);
    plot(t,e_dot,'+');
    hold on;
    
    % Designing the task level controller
    tau = task_level_controller(U,q_dot,J,J_dot);

    % Solving the nonlinear dynamics differential equations
    t0 = t-0.01;
    tf = t+0.01;
    tspan = t0:0.005:tf;
    
    [time q_all] = ode45(@ode_dynamics_eqtn,tspan,[q q_dot]);
    
    q(1:3) = q_all(end,1:3);
    
    q_dot(1:3) = q_all(end,4:6);
    
    % Computation of forward kinematics
    [Y Y_dot] = forward_kinematics(q,q_dot,d,a);
%     absy = sqrt(Y(1)^2 + Y(2)^2 + Y(2)^2);
%     plot(t,absy,'+');
%     hold on;
end