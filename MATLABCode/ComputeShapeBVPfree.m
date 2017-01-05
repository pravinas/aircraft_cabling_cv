function [ solTrue ] = ComputeShapeBVPfree( X_init, Length, stiffness, mode, yGround )
% Computes the cable shape with one of the ends free

x_initial = 0;
y_initial = 0;
theta_initial = X_init(3);
tGuess = linspace(0,1,15);
stiffness = stiffness/Length;

if strcmpi(mode , 'right')
    theta_initial = theta_initial;
elseif strcmpi(mode, 'left')
    theta_initial = -theta_initial;
end



%Generate guess and boundary condition vectors
Xa_ref = [x_initial;y_initial;theta_initial];

Xa_guess = [x_initial;y_initial;-1;0;-9.8*Length;8];
Xb_guess = [0.7;0;-1;0;0;0];

%Generate function handles
bvpbcfun = @(Xa,Xb)CableDynamics2Dbcfun_free(Xa,Xb,Xa_ref);
guessfun = @(t)CableDynamics2Dguess(t,Xa_guess,Xb_guess);
odefun = @(t,X)CableDynamics2Dodefun(t,X,stiffness);


solinit = bvpinit(tGuess,guessfun);

sol = bvp4c(odefun, bvpbcfun, solinit);

if strcmpi(mode , 'right')
% Scale and translate to the original position

% sol1 is the scaled solution
    sol1 = sol;
    sol1.x = sol1.x*Length;
    sol1.y(1,:) = sol1.y(1,:)*Length;
    sol1.y(2,:) = sol1.y(2,:)*Length;

% solTrue is the final solution

    solTrue = sol1;
    solTrue.y(1,:) = solTrue.y(1,:) + X_init(1);
    solTrue.y(2,:) = solTrue.y(2,:) + X_init(2);
    
% Flatten out the portion on the floor
    
    crossingLength = interp1(solTrue.y(2,:) , solTrue.x , yGround);
    if ~isnan(crossingLength)
    crossingX = interp1(solTrue.x , solTrue.y(1,:) , crossingLength);
    for i = 1:length(solTrue.x)
        if solTrue.x(i) > crossingLength
            solTrue.y(2,i) = yGround;
            solTrue.y(1,i) = crossingX + (solTrue.x(i)-crossingLength);
            solTrue.y(3,i) = 0;
        end
    end
    %solTrue.y(1,:) = smooth(solTrue.x(1,:));
    %solTrue.y(2,:) = smooth(solTrue.y(2,:));
    %solTrue.y(3,:) = smooth(solTrue.y(3,:));
    end

elseif strcmpi(mode, 'left')
    % Scale 
    sol1 = sol;
    sol1.x = sol1.x*Length;
    sol1.x = Length - flip(sol1.x);
    sol1.y(1,:) = -sol1.y(1,:)*Length;
    sol1.y(2,:) = sol1.y(2,:)*Length;
    sol1.y(3,:) =  - sol.y(3,:);
    sol1.y = flip(sol1.y, 2);
    
    % Translate
    
    solTrue = sol1;
    solTrue.y(1,:) = solTrue.y(1,:) + X_init(1);
    solTrue.y(2,:) = solTrue.y(2,:) + X_init(2);
    
    crossingLength = interp1(solTrue.y(2,:) , solTrue.x , yGround);
    if ~isnan(crossingLength)
    crossingX = interp1(solTrue.x , solTrue.y(1,:) , crossingLength);
    for i = 1:length(solTrue.x)
        if solTrue.x(i) < crossingLength
            solTrue.y(2,i) = yGround;
            solTrue.y(1,i) = crossingX + (solTrue.x(i)-crossingLength);
            solTrue.y(3,i) = 0;
        end
    end
    %solTrue.y(1,:) = smooth(solTrue.x(1,:));
    %solTrue.y(2,:) = smooth(solTrue.y(2,:));
    %solTrue.y(3,:) = smooth(solTrue.y(3,:));
    end
end



    


end

