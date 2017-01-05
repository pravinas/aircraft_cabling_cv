function [ MarkerPos ] = getMarkerPos( SystemNode, CableID )
% Return the vector of cartesian positions of the markers on the identified
% Cable

Cable = SystemNode.State.Cable;
MarkerPos = GetPosition(Cable, CableID, Cable(CableID).markers);
MarkerPos = MarkerPos(:,[1 2]);
end

