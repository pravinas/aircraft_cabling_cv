function [ output ] = CableDynamics2Dbcfun( Xa, Xb, Xa_ref )
% Defines the boundary conditions for the cable dynamics

output = [(Xa(1:3) - Xa_ref) ; (Xb(4:6))];


end

