function [ GripPointPos ] = getGrippingPosition( Cable, cableID, gripID )
% out = getGrippingPosition(Cable, cableID, gripID);

% Cable: A structure array that defines the cable in the simulation. At
% this point the shape of the cable must have been calculated and saved in
% Cable.configuration structure

%cableID: An integer index to specify the cable from the Cable array that
%is being referenced

%gripID: An array integer index indicating the index of the gripping point whose
%state must be accessed

%Note that the cable configuration must have been computed and updated when
%this function is called

selectedCable = Cable(cableID);

if nargin == 3
    
    %Provide the position of the specific gripping point only
    
    %Get the length parameter of the gripping point selected
    gripLengthPos(:,1) = selectedCable.gripPointsPos(gripID);
    
    %Interpolate the states from the configuration structure
    gripX(:,1) = interp1(selectedCable.configuration.length , selectedCable.configuration.state(:,1) , gripLengthPos);
    gripY(:,1) = interp1(selectedCable.configuration.length , selectedCable.configuration.state(:,2) , gripLengthPos);
    gripTheta(:,1) = interp1(selectedCable.configuration.length , selectedCable.configuration.state(:,3) , gripLengthPos);
    
    GripPointPos.gripID = gripID;
    GripPointPos.length = gripLengthPos;
    GripPointPos.state = [gripX, gripY, gripTheta];
else
    % Access the state of all the gripping points
    
    gripID(:,1) = 1:length(selectedCable.gripPointsPos);
    gripLengthPos(:,1) = selectedCable.gripPointsPos;
    
    gripX(:,1) = interp1(selectedCable.configuration.length , selectedCable.configuration.state(:,1) , gripLengthPos);
    gripY(:,1) = interp1(selectedCable.configuration.length , selectedCable.configuration.state(:,2) , gripLengthPos);
    gripTheta(:,1) = interp1(selectedCable.configuration.length , selectedCable.configuration.state(:,3) , gripLengthPos);
    
    GripPointPos.gripID = gripID;
    GripPointPos.length = gripLengthPos;
    GripPointPos.state = [gripX, gripY, gripTheta];
end


end

