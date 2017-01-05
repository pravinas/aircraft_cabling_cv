function [ OutNode GraspNode ParentNode TransitionHandle flag ] = AlignRefPoint( SystemNode, CableID, ManipID, RefID, Tolerance )
%

%Get the closest gripping point
selectedReferencePoint.CableID = CableID;
selectedReferencePoint.RefID = RefID;

AlignFlag = IsRefPointAligned(SystemNode, selectedReferencePoint, Tolerance);

if AlignFlag == 1

    flag = 1;
    OutNode = SystemNode;
    GraspNode = SystemNode;
    ParentNode = SystemNode;
    TransitionHandle = [];

else

ParentNode = SystemNode;

GripPointPos = SystemNode.State.Cable(CableID).gripPointsPos;
SelectedRefPos = SystemNode.State.Cable(CableID).refPointPos(RefID);

AbsDifference = abs(GripPointPos - SelectedRefPos);
[~, GripPointIndex] = min(AbsDifference);

[GraspNode newParentNode flag1] = GraspCable(SystemNode, ManipID, CableID, GripPointIndex);
if flag1 == 0
    GraspNode = SystemNode;
    OutNode = SystemNode;
    flag = 0;
    TransitionHandle = [];
    return;
end
[OutNode newParentNode flag2] = ClampCable(GraspNode, ManipID, RefID);

if flag2 == 0
    
    OutNode = GraspNode;
    flag = 0;
    TransitionHandle = [];
    return;
    
end

TransitionHandle = @(SystemNode)ClampCable(SystemNode, ManipID, RefID);

flag = 1;
end

end

