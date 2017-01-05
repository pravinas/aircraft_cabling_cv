function [ AlignFlag ] = IsRefPointAligned(SystemNode, RefPoint, Tolerance)
%

CableID = RefPoint.CableID;
RefID = RefPoint.RefID;

% The Alignment needs to be checked when all the manipulators are released,
% not when a cable is potentially held in place by a manipulator

%Vacating all manipulators
nManipulators = max(size(SystemNode.State.Manipulator));

for i = 1:nManipulators
    [SystemNode, ~, ~] = ReleaseManipulator(SystemNode, i);
end



RefPosition = getRefPosition(SystemNode.State.Cable, CableID, RefID);
RefPosition = RefPosition.state;
RefPosition = RefPosition(1:2);

ClampPosition = SystemNode.State.Cable(CableID).clampPos(RefID,:);
ClampPosition = ClampPosition(1:2);

distance = norm(RefPosition - ClampPosition);
if distance > Tolerance
    AlignFlag = 0;
else
    AlignFlag = 1;
end



end

