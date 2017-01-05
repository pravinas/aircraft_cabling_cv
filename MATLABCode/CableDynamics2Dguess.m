function [ result ] = CableDynamics2Dguess( t,X0,Xf )
% Returns the result 



result = t*Xf + (1-t)*X0;


end

