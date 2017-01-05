function [ GraspNode, ParentNode, flag ] = GraspMultipleRefPoint( InNode, CableID, ManipID, RefID, Tolerance )
%CableID, ManipId and RefID are arrays of same length. This function only
%produces the graspnode with all the cables grasped by the manipulators

%Determine the Reference points that actually need alignment
nRefPoints = max(size(RefID));
ParentNode = InNode;
GraspNode = InNode;
for i=1:nRefPoints

GripPointPos = GraspNode.State.Cable(CableID(i)).gripPointsPos;
SelectedRefPos = GraspNode.State.Cable(CableID(i)).refPointPos(RefID(i));

AbsDifference = abs(GripPointPos - SelectedRefPos);
[~, GripPointIndex] = min(AbsDifference);

[GraspNode tempParentNode flag1] = GraspCableParallel(GraspNode, ManipID(i), CableID(i), GripPointIndex);
if flag1 == 0
    ParentNode = InNode; 
    flag = 0;
    return;
end


end
flag = 1;


end

