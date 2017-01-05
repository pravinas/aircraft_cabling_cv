function [ output ] = CableDynamics2Dbcfun( Xa, Xb, Xa_ref, Xb_ref )
% Defines the boundary conditions for the cable dynamics

output = [(Xa(1:3) - Xa_ref) ; (Xb(1:3) - Xb_ref)];


end

