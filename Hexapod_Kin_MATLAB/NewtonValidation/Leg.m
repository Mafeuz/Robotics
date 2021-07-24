function [L] = Leg(x,y,z,RX,RY,RZ,a,b)


P = [x; y; z];
R = rotation(RX,RY,RZ);
    
S = P + R*b - a;

L = norm(S);

end