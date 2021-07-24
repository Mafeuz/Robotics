
%%      - Newton Method Trial Solve Hexapod Forward Kinematics -         %%
%=========================================================================%

%Constructive Properties %========================================================================

Rground = 79.91;    % Ground Base of the Hexapod Radius;
Rbase = 49.659;     % Mobile Platform of the Hexapod Radius;

angleground = 23.47;    % Ground Base - Closest Angle between the joints;
anglebase = 50.03;      % Mobile Platform - Closest Angle between the joints;

%Calculation of the joints Angles for the definition of the base and platform vectors;
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

J = 0;

%Vector a1...a6 - Location of the ground joints in the reference coordinate system;

a1 = Rground*[cosd(a_theta1+J); sind(a_theta1+J); 0];
a2 = Rground*[cosd(a_theta2+J); sind(a_theta2+J); 0];
a3 = Rground*[cosd(a_theta3+J); sind(a_theta3+J); 0];
a4 = Rground*[cosd(a_theta4+J); sind(a_theta4+J); 0];
a5 = Rground*[cosd(a_theta5+J); sind(a_theta5+J); 0];
a6 = Rground*[cosd(a_theta6+J); sind(a_theta6+J); 0];

%Vector b1...b6 - Location of the platform joints in the platform coordinate system;
b1 = Rbase*[cosd(b_theta1+J); sind(b_theta1+J); 0];
b2 = Rbase*[cosd(b_theta2+J); sind(b_theta2+J); 0];
b3 = Rbase*[cosd(b_theta3+J); sind(b_theta3+J); 0];
b4 = Rbase*[cosd(b_theta4+J); sind(b_theta4+J); 0];
b5 = Rbase*[cosd(b_theta5+J); sind(b_theta5+J); 0];
b6 = Rbase*[cosd(b_theta6+J); sind(b_theta6+J); 0];


%Sectors===================================================================


X = Sampler(-20,20,1,1);
Y = Sampler(-20,20,1,1);
Z = Sampler(98,108,1,1);
RX = Sampler(-10,10,1,1);
RY = Sampler(-10,10,1,1);
RZ = Sampler(-15,15,1,1);

l1 = Leg(X,Y,Z,RX,RY,RZ,a1,b1);
l2 = Leg(X,Y,Z,RX,RY,RZ,a2,b2);
l3 = Leg(X,Y,Z,RX,RY,RZ,a3,b3);
l4 = Leg(X,Y,Z,RX,RY,RZ,a4,b4);
l5 = Leg(X,Y,Z,RX,RY,RZ,a5,b5);
l6 = Leg(X,Y,Z,RX,RY,RZ,a6,b6);

%==========================================================================

%Inputs: L1,L2,L3,L4,L5,L6.

%Variables=================================================================
syms x y z rx ry rz  
syms L1 L2 L3 L4 L5 L6

%==========================================================================

P = [x; y; z];
R = rotation2(rx,ry,rz);

%System====================================================================
eq1 = norm(P + R*b1 - a1) - L1;
eq2 = norm(P + R*b2 - a2) - L2;
eq3 = norm(P + R*b3 - a3) - L3;
eq4 = norm(P + R*b4 - a4) - L4;
eq5 = norm(P + R*b5 - a5) - L5;
eq6 = norm(P + R*b6 - a6) - L6;
%==========================================================================


%Newton Method=============================================================

% Forward Kinematics Initial Guess Calculations %==========================

ForwardX = ((-1.51694257e-9) + l1*(1.88740566) + l2*(-1.88740566) + l3*(-1.78630476e-1) + l4*(1.70265653) + l5*(-1.70265653) + l6*(1.7863047e-1));
ForwardY = ((-0.00958264) + l1*(-0.87999987) + l2*(-0.87999987) + l3*(2.07628491) + l4*(-1.19622617) + l5*(-1.19622617) + l6*(2.07628491));
ForwardZ = ((-11.31748409) + l1*(0.17310112) + l2*(0.17310112) + l3*(0.1730323) + l4*(0.17318892) + l5*(0.17318892) + l6*(0.1730323));

ForwardRX = ((-0.0012806) + l1*(0.09379136) + l2*(0.09379136) + l3*(0.60417547) + l4*(-0.69795643) + l5*(-0.69795643) + l6*(0.60417547));
ForwardRY = ((6.14411133e-10) + l1*(-7.47210598e-1) + l2*(7.47210598e-1) + l3*(4.53910209e-1) + l4*(-2.92504249e-1) + l5*(2.92504249e-1) + l6*(-4.53910209e-1));
ForwardRZ = ((-3.11851055e-10) + l1*(-1.22550406) + l2*(1.22550406) + l3*(-1.22207852) + l4*(1.22116327) + l5*(-1.22116327) + l6*(1.22207852));

% =========================================================================

%Aux:
del = 1;
index = 0;

%Desired Error:
Error = 1e-10;

eq1 = subs(eq1,{L1},{l1});
eq2 = subs(eq2,{L2},{l2});
eq3 = subs(eq3,{L3},{l3});
eq4 = subs(eq4,{L4},{l4});
eq5 = subs(eq5,{L5},{l5});
eq6 = subs(eq6,{L6},{l6});

F = [eq1; eq2; eq3; eq4; eq5; eq6];

J = jacobian(F,[x y z rx ry rz]);

X0 = [ForwardX; ForwardY; ForwardZ; ForwardRX; ForwardRY; ForwardRZ];
 
%Newton Iteration Loop:
while del > Error
    
   SubJacob = vpa(subs(J,{x,y,z,rx,ry,rz},{X0.'}));
     
   InverseJacob = inv(SubJacob);  
   
   f = vpa(subs(F,{x,y,z,rx,ry,rz},{X0.'}));
   
   del = max(abs(f));
   
   if del < Error
       disp('BREAK')
       break 
   end
   
   delx = -InverseJacob*f;
     
   X0 = X0 + delx;
   
   index = index + 1;
   
end

disp(index)
disp([X Y Z RX RY RZ])
disp(X0.')

errorX = X - X0(1); errorY = Y - X0(2); errorZ = Z - X0(3);
errorRX = RX - X0(4); errorRY = RY - X0(5); errorRZ = RZ - X0(6);

errorvector = [errorX errorY errorZ errorRX errorRY errorRZ];

maxerror = max(abs(errorvector));

disp(maxerror);

disp('END');




