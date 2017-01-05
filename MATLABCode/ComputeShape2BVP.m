function [ solTrue ] = ComputeShape2BVP( X_init, X_final, Length, stiffness, YGround )
% Computes the shape of the cable given the end point boundary conditions

x_initial = X_init(1);
y_initial = X_init(2);
theta_initial = X_init(3);

x_final = X_final(1);
y_final = X_final(2);
theta_final = X_final(3);
stiffness = stiffness/Length;

% Transform into the baseline configuration of unit length

%First translate
x1_initial = 0;
y1_initial = 0;

x1_final = x_final - x_initial;
y1_final = y_final - y_initial;

% Next scale the cable lengths
xSolve_initial = x1_initial/Length;
ySolve_initial = y1_initial/Length;

xSolve_final = x1_final/Length;
ySolve_final = y1_final/Length;

% Solve for the cable configuration in the normalised coordinates
tGuess = linspace(0,1,150);
i=1;

% for stiffness = [ 0.01 0.1 0.5];
% Generate guess and boundary condition vectors
Xa_ref = [xSolve_initial;ySolve_initial;theta_initial];
Xb_ref = [xSolve_final;ySolve_final;theta_final];

Xa_guess = [xSolve_initial;ySolve_initial;-1;1;-5;2];
Xb_guess = [xSolve_final;ySolve_final;1;1;0;0];

%Generate function handles
bvpbcfun = @(Xa,Xb)CableDynamics2Dbcfun(Xa,Xb,Xa_ref,Xb_ref);
guessfun = @(t)CableDynamics2Dguess(t,Xa_guess,Xb_guess);
odefun = @(t,X)CableDynamics2Dodefun(t,X,stiffness);

%solinit = bvpinit(tGuess,guessfun);
load guessNormal;
solinit = sol;
clear sol;

sol = bvp4c(odefun, bvpbcfun, solinit);

% Transform back to the original coordinate frame
% Scale to the original length
sol1 = sol;
sol1.x = sol.x*Length;
sol1.y(1,:) = sol1.y(1,:)*Length;
sol1.y(2,:) = sol1.y(2,:)*Length;


%Translate to original position
solTrue = sol1;
solTrue.y(1,:) = solTrue.y(1,:) + x_initial;
solTrue.y(2,:) = solTrue.y(2,:) + y_initial;

SolLength = size(solTrue.y(2,:),2);

 for i = 1:SolLength
     if solTrue.y(2,i) <= YGround;
         solTrue.y(2,i) = YGround;
         solTrue.y(3,i) = 0;
     end
end

