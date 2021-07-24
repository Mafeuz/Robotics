function [R] = rotation(thetaRX,thetaRY,thetaRZ)

RX = [1 0 0; 0 cosd(thetaRX) sind(thetaRX); 0 -sind(thetaRX) cosd(thetaRX)];
RY = [cosd(thetaRY) 0 -sind(thetaRY); 0 1 0; sind(thetaRY) 0 cosd(thetaRY)];
RZ = [cosd(thetaRZ) sind(thetaRZ) 0; -sind(thetaRZ) cosd(thetaRZ) 0; 0 0 1];

%Transposition in order to keep the rotations related to the reference:
RX = RX.';
RY = RY.';
RZ = RZ.';

R = RZ*RY*RX;

end