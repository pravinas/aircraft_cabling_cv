function [ CurrentPosition, DesiredPosition ] = getCurrentAndDesiredPos( SystemNode, ControlPoint, ReferenceLocation )
%

CurrentPosition = [];
DesiredPosition = [];

nPoints = size(ControlPoint,1);

for i = 1:nPoints
    cable = ControlPoint(i).cable;
    length = ControlPoint(i).length;
    pos = GetPosition(SystemNode.State.Cable, cable, length)';
    CurrentPosition = [CurrentPosition; pos(1:2)];
    
    cable = ReferenceLocation(i).cable;
    length = ReferenceLocation(i).length;
    pos = GetPosition(SystemNode.State.Cable, cable, length)';
    DesiredPosition = [DesiredPosition; pos(1:2)];
end

