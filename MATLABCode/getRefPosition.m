function [ refPointPos ] = getRefPosition( Cable, cableID, refID )
% out = getRefPosition(Cable, cableID, refID);

% Cable: A structure array that defines the cable in the simulation. At
% this point the shape of the cable must have been calculated and saved in
% Cable.configuration structure

%cableID: An integer index to specify the cable from the Cable array that
%is being referenced

%refID: An array integer index indicating the index of the reference point whose
%state must be accessed

%Note that the cable configuration must have been computed and updated when
%this function is called

selectedCable = Cable(cableID);

if nargin == 3
    
    %Provide the position of the specific gripping point only
    
    %Get the length parameter of the gripping point selected
    refLengthPos(:,1) = selectedCable.refPointPos(refID);
    
    %Interpolate the states from the configuration structure
    refX(:,1) = interp1(selectedCable.configuration.length , selectedCable.configuration.state(:,1) , refLengthPos);
    refY(:,1) = interp1(selectedCable.configuration.length , selectedCable.configuration.state(:,2) , refLengthPos);
    refTheta(:,1) = interp1(selectedCable.configuration.length , selectedCable.configuration.state(:,3) , refLengthPos);
    
    refPointPos.refID = refID;
    refPointPos.length = refLengthPos;
    refPointPos.state = [refX, refY, refTheta];
else
    % Access the state of all the refping points
    
    refID(:,1) = 1:length(selectedCable.refPointPos);
    refLengthPos(:,1) = selectedCable.refPointPos;
    
    refX(:,1) = interp1(selectedCable.configuration.length , selectedCable.configuration.state(:,1) , refLengthPos);
    refY(:,1) = interp1(selectedCable.configuration.length , selectedCable.configuration.state(:,2) , refLengthPos);
    refTheta(:,1) = interp1(selectedCable.configuration.length , selectedCable.configuration.state(:,3) , refLengthPos);
    
    refPointPos.refID = refID;
    refPointPos.length = refLengthPos;
    refPointPos.state = [refX, refY, refTheta];
end



end

