function [ Cable errorFlag ] = ComputeCableShape( Cable, Manipulator )


% Create the module to solve for the shape of the cable

% Define the ground level
yGround = Cable.Ground;

% Retrieve the length parameters
if ~isempty(Cable.clamped)
    clampedLength = Cable.clamped(:,1);
else
    clampedLength = [];
end
    
if ~isempty(Cable.gripped)
    gripPointID = Cable.gripped(:,1);
    grippedLength = Cable.gripPointsPos(gripPointID);
else
    grippedLength = [];
end

BC.lengths = [clampedLength;grippedLength];

% Retrieve state configurations at each clamp
if ~isempty(Cable.clamped)
    clampID = Cable.clamped(:,2);
    clampedState = Cable.clampPos(clampID,:);
else
    clampedState = [];
end

if ~isempty(Cable.gripped)
    manipID = Cable.gripped(:,2);
    n_manipUsed = length(manipID);
else
    manipID = [];
    n_manipUsed = 0;
end
 
if n_manipUsed > 0
for i = 1:n_manipUsed
    grippedState(i,:) = Manipulator(manipID(i)).position;
end
else
    grippedState = [];
end

BC.states = [clampedState; grippedState];

% Sort the boundary conditions in ascending order

[sortedBC.lengths, index] = sort(BC.lengths);
sortedBC.states = BC.states(index,:);
clear BC;

% Evaluate that the wire is not stretched in the current configuration,
% else the state is inadmissible

n_BC = length(sortedBC.lengths);
stretch_flag = 0;
for i = 1:n_BC-1
    if norm(sortedBC.states(i,1:2) - sortedBC.states(i+1,1:2)) > abs(sortedBC.lengths(i)-sortedBC.lengths(i+1))
        stretch_flag = 1;
    end
end

if stretch_flag == 1 % Cable has been stretched. Inadmissible and non-computable state
    warning('Cable Stretched');
    errorFlag = 0;
    return
else % Non stretched cable. The shape can be computed
    errorFlag = 1;
    if isempty(sortedBC.lengths) % Cable on the floor
        Cable.configuration.length = [0 Cable.length]';
        Cable.configuration.state = [Cable.configuration.state(1,1) yGround 0;
                                    Cable.configuration.state(1,1)+Cable.length yGround 0];
        
    
    elseif length(sortedBC.lengths) == 1
    
        sol = ComputeShapeBVPfree(sortedBC.states(1,:) ,  sortedBC.lengths(1) , Cable.stiffness , 'left', yGround);
        Cable.configuration.length = [sol.x'];
        Cable.configuration.state = sol.y(1:3,:)';
        
        sol = ComputeShapeBVPfree(sortedBC.states(1,:), Cable.length - sortedBC.lengths(1), Cable.stiffness, 'right', yGround);
        Cable.configuration.length = [Cable.configuration.length; (sol.x(2:length(sol.x))' + sortedBC.lengths(1))];
        Cable.configuration.state = [Cable.configuration.state; sol.y(1:3,2:length(sol.x))'];
        
    elseif length(sortedBC.lengths) > 1
        
        nBC = length(sortedBC.lengths);
        sol = ComputeShapeBVPfree(sortedBC.states(1,:) , sortedBC.lengths(1) , Cable.stiffness , 'left', yGround);
        Cable.configuration.length = [sol.x'];
        Cable.configuration.state = sol.y(1:3,:)';
        
        for i = 1:nBC-1
            % Compute the cable shape
            Length = sortedBC.lengths(i+1) - sortedBC.lengths(i);
            X_init = sortedBC.states(i,:);
            X_final = sortedBC.states(i+1,:);
            sol = ComputeShape2BVP(X_init, X_final, Length, Cable.stiffness, yGround);
            
            % Add the shape to the Cable configuration
            lengthAdd = sol.x(2:length(sol.x))' + sortedBC.lengths(i);
            Cable.configuration.length = [Cable.configuration.length; lengthAdd];
            stateAdd = sol.y(1:3 , 2:length(sol.x))';
            Cable.configuration.state = [Cable.configuration.state; stateAdd]; 
        end
        
        %Add the last hanging part to the system
        
        sol = ComputeShapeBVPfree(sortedBC.states(nBC,:) , (Cable.length - sortedBC.lengths(nBC)) , Cable.stiffness , 'right', yGround);
        lengthAdd = sol.x(2:length(sol.x))' + sortedBC.lengths(nBC);
        Cable.configuration.length = [Cable.configuration.length; lengthAdd];
        stateAdd = sol.y(1:3, 2:length(sol.x))';
        Cable.configuration.state = [Cable.configuration.state; stateAdd];
    end    
end


end

