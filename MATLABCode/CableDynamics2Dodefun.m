function [ Xdot ] = CableDynamics2Dodefun( t,X,stiffness )
% Calculates the length derivatives for the cable

x = X(1);
y = X(2);
theta = X(3);
P(:,1) = X(4:6);

xdot = cos(theta);
ydot = sin(theta);
thetadot = -(1/stiffness)*P(3)/2;
P1dot = 0;
P2dot = 9.8;
P3dot = -P(1)*sin(theta) + P(2)*cos(theta);

Xdot = [xdot;ydot;thetadot;P1dot;P2dot;P3dot];
end

