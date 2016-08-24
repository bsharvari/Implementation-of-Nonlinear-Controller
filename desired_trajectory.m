% Function to compute the desired trajectory

function [Yd Yd_dot Yd_ddot] = desired_trajectory(w,t,R)
syms w1 t1

Yd1 = -1*0.866*R*cos(w1*t1) - 0.56;
Yd2 = R*sin(w1*t1);
Yd3 = 0.5*R*cos(w1*t1) - 0.08;

Yd = [Yd1;Yd2;Yd3];
Yd_dot = diff(Yd,t);
Yd_ddot = diff(Yd_dot,t);

% Substituting the values of w and t to compute the values of Yd
Yd = subs(Yd,[w1,t1],[w,t]);    
Yd_dot = subs(Yd_dot,[w1,t1],[w,t]);
Yd_ddot = subs(Yd_ddot,[w1,t1],[w,t]);
end