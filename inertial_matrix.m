% Function to calculate the values of the Inertial matrix

function D = inertial_matrix(q)
D = zeros(3,3);

D(1,1) = 2.4574 + 1.7181*cos(q(2))^2 + 0.4430*sin(q(2) + q(3))^2 - 0.0324*cos(q(2))*cos(q(2) + q(3)) - 0.0415*cos(q(2) + q(3))*sin(q(2) + q(3)) + 0.9378*cos(q(2))*sin(q(2) + q(3));
D(1,2) = 2.2312*sin(q(2)) - 0.0068*sin(q(2) + q(3)) - 0.1634*cos(q(2) + q(3));
D(1,3) = -1*0.0068*sin(q(2) + q(3)) - 0.1634*cos(q(2) + q(3));
D(2,1) = D(1,2);
D(2,2) = 5.1285 + 0.9378*sin(q(3)) - 0.0324*cos(q(3));
D(2,3) = 0.4424 + 0.4689*sin(q(3)) - 0.0162*cos(q(3));
D(3,1) = D(1,3);
D(3,2) = D(2,3);
D(3,3) = 1.0236;
end