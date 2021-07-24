%% ============================== Hexapod Forward Kinematics ============================================%%

% Hexapod Constructive Properties %========================================================================

Rground = 79.91;    % Ground Base of the Hexapod Radius;
Rbase = 49.659;     % Mobile Platform of the Hexapod Radius;

angleground = 23.47;    % Ground Base - Closest Angle between the joints;
anglebase = 50.03;      % Mobile Platform - Closest Angle between the joints;

% Calculation of the joints Angles for the definition of the base and platform vectors;
a_theta1 = 90 - (angleground/2);
a_theta2 = a_theta1 + angleground;
a_theta3 = a_theta2 + (360 - 3*angleground)/3;
a_theta4 = a_theta3 + angleground;
a_theta5 = a_theta4 + (360 - 3*angleground)/3;
a_theta6 = a_theta5 + angleground;

b_theta1 = 90 - (anglebase/2);
b_theta2 = b_theta1 + anglebase;
b_theta3 = b_theta2 + (360 - 3*anglebase)/3;
b_theta4 = b_theta3 + anglebase;
b_theta5 = b_theta4 + (360 - 3*anglebase)/3;
b_theta6 = b_theta5 + anglebase;

% Vector a1...a6 - Location of the ground joints in the reference coordinate system;

a1 = Rground*[cosd(a_theta1); sind(a_theta1); 0];
a2 = Rground*[cosd(a_theta2); sind(a_theta2); 0];
a3 = Rground*[cosd(a_theta3); sind(a_theta3); 0];
a4 = Rground*[cosd(a_theta4); sind(a_theta4); 0];
a5 = Rground*[cosd(a_theta5); sind(a_theta5); 0];
a6 = Rground*[cosd(a_theta6); sind(a_theta6); 0];

% Vector b1...b6 - Location of the platform joints in the platform coordinate system;
b1 = Rbase*[cosd(b_theta1); sind(b_theta1); 0];
b2 = Rbase*[cosd(b_theta2); sind(b_theta2); 0];
b3 = Rbase*[cosd(b_theta3); sind(b_theta3); 0];
b4 = Rbase*[cosd(b_theta4); sind(b_theta4); 0];
b5 = Rbase*[cosd(b_theta5); sind(b_theta5); 0];
b6 = Rbase*[cosd(b_theta6); sind(b_theta6); 0];

%===========================================================================

% Leg Length Inputs: L1,L2,L3,L4,L5,L6.

% Example:

L1 = 115; L2 = 115; L3 = 115;
L4 = 115; L5 = 115; L6 = 115;

% Legs Length Vector
L = [L1;L2;L3;L4;L5;L6];

% Variables=================================================================
syms x y z rx ry rz

%===========================================================================

P = [x; y; z]; % Translation point of control (Platform Center)
R = rotation2(rx,ry,rz); % Rotation (Platform Center)

% System of Non-linear equations from the Inverse Kinematics
eq1 = norm(P + R*b1 - a1) - L1;
eq2 = norm(P + R*b2 - a2) - L2;
eq3 = norm(P + R*b3 - a3) - L3;
eq4 = norm(P + R*b4 - a4) - L4;
eq5 = norm(P + R*b5 - a5) - L5;
eq6 = norm(P + R*b6 - a6) - L6;
%==========================================================================

% Newton Method %==========================================================

% Forward Kinematics Initial Guess Calculations %==========================

% ForwardX = ((-1.51694257e-9) + L1*(1.88740566) + L2*(-1.88740566) + L3*(-1.78630476e-1) + L4*(1.70265653) + L5*(-1.70265653) + L6*(1.7863047e-1));
% ForwardY = ((-0.00958264) + L1*(-0.87999987) + L2*(-0.87999987) + L3*(2.07628491) + L4*(-1.19622617) + L5*(-1.19622617) + L6*(2.07628491));
% ForwardZ = ((-11.31748409) + L1*(0.17310112) + L2*(0.17310112) + L3*(0.1730323) + L4*(0.17318892) + L5*(0.17318892) + L6*(0.1730323));

% ForwardRX = ((-0.0012806) + L1*(0.09379136) + L2*(0.09379136) + L3*(0.60417547) + L4*(-0.69795643) + L5*(-0.69795643) + L6*(0.60417547));
% ForwardRY = ((6.14411133e-10) + L1*(-7.47210598e-1) + L2*(7.47210598e-1) + L3*(4.53910209e-1) + L4*(-2.92504249e-1) + L5*(2.92504249e-1) + L6*(-4.53910209e-1));
% ForwardRZ = ((-3.11851055e-10) + L1*(-1.22550406) + L2*(1.22550406) + L3*(-1.22207852) + L4*(1.22116327) + L5*(-1.22116327) + L6*(1.22207852));

% Linear Regression Coeficient Matrix
RegCoef = [1.88740566 -1.88740566 -1.78630476e-1 1.70265653 -1.70265653 1.7863047e-1;
        -0.87999987 -0.87999987 2.07628491 -1.19622617 -1.19622617 2.07628491;
        0.17310112 0.17310112 0.1730323 0.17318892 0.17318892 0.1730323;
        0.09379136 0.09379136 0.60417547 -0.69795643 -0.69795643 0.60417547;
        -7.47210598e-1 7.47210598e-1 4.53910209e-1 -2.92504249e-1 2.92504249e-1 -4.53910209e-1;
        -1.22550406 1.22550406 -1.22207852 1.22116327 -1.22116327 1.22207852];

W0 = [-1.51694257e-9; -0.00958264; -11.31748409; -0.0012806; 6.14411133e-10; -3.11851055e-10];

% Initial Guess Vector X0 from Linear Regression
X0 = RegCoef*L + W0;

%===============================================================================

% Newton Method Auxiliars
Trial = 1;
index = 0;

% Desired Error Tolerance
Error = 1e-10;

% Define Equations Matrix
F = [eq1; eq2; eq3; eq4; eq5; eq6];

% Define the Jacobian Matrix
J = jacobian(F,[x y z rx ry rz]);

% Substitute the varibles with the linear regression calculated initial guess
f = vpa(subs(F,{x,y,z,rx,ry,rz},{X0.'}));

% Newton Method Iteration Loop:
while Trial > Error

   index = index + 1;

   SubJacob = vpa(subs(J,{x,y,z,rx,ry,rz},{X0.'})); % Substitute the Guess in the Jacobian Matrix

   InverseJacob = inv(SubJacob);

   delta = -InverseJacob*f;

   X0 = X0 + delta; % Update the guess with the delta calculated

   f = vpa(subs(F,{x,y,z,rx,ry,rz},{X0.'})); % Substitute the Guess in the system to check

   Trial = max(abs(f)); % Check the maximum error with the currently guess

end

disp(X0)
