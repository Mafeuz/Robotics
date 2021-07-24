function [R] = rotation2(thetaRX,thetaRY,thetaRZ)

thetaRX = thetaRX * (pi/180);
thetaRY = thetaRY * (pi/180);
thetaRZ = thetaRZ * (pi/180);

RX = [1 0 0; 0 cos(thetaRX) sin(thetaRX); 0 -sin(thetaRX) cos(thetaRX)];
RY = [cos(thetaRY) 0 -sin(thetaRY); 0 1 0; sin(thetaRY) 0 cos(thetaRY)];
RZ = [cos(thetaRZ) sin(thetaRZ) 0; -sin(thetaRZ) cos(thetaRZ) 0; 0 0 1];

%Transposition in order to keep the rotations related to the reference:
RX = RX.';
RY = RY.';
RZ = RZ.';

R = RZ*RY*RX;

end