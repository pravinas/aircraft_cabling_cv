function [ position ] = GetPosition(Cable, cableID, l)
% [ position ] = GetPosition(Cable, cableID, l)

% This function acceses the position of the point that is at the length "l"
% along the length of the cable.

% Cable: Is a structure array of the type Cable.
% cableID: Index of the cable along whose length the position is requested
% l: Length parameter along the cable where the position is requested. Can
% also be a vector

% Note that before calling this function, the cable shape must already have
% been computed

n = length(l);
selectedCable = Cable(cableID);

for i=1:n
    if l(i) > selectedCable.length
        position(i,:) = [nan nan nan];
    else
        l_x = interp1(selectedCable.configuration.length, selectedCable.configuration.state(:,1), l(i));
        l_y = interp1(selectedCable.configuration.length, selectedCable.configuration.state(:,2), l(i));
        l_theta = interp1(selectedCable.configuration.length, selectedCable.configuration.state(:,3), l(i));
        position(i,:) = [l_x l_y l_theta];
    end
end


end

