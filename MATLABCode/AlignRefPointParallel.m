function [ OutNode GraspNode ParentNode TransitionHandle flag ] = AlignRefPointParallel( PrevGraspNode, PrevTransitionHandle, CableID, ManipID, RefID, Tolerance )
%

%Get the closest gripping point
selectedReferencePoint.CableID = CableID;
selectedReferencePoint.RefID = RefID;

AlignFlag = IsRefPointAligned(PrevGraspNode, selectedReferencePoint, Tolerance);

if AlignFlag == 1

    flag = 1;
    OutNode = SystemNode;
    GraspNode = SystemNode;
    ParentNode = ParentNode
    TransitionHandle = [];

else

ParentNode = PrevGraspNode;

GripPointPos = PrevGraspNode.State.Cable(CableID).gripPointsPos;
SelectedRefPos = PrevGraspNode.State.Cable(CableID).refPointPos(RefID);

AbsDifference = abs(GripPointPos - SelectedRefPos);
[~, GripPointIndex] = min(AbsDifference);

[GraspNode ParentNode flag1] = GraspCableParallel(PrevGraspNode, ManipID, CableID, GripPointIndex);
if flag1 == 0
    GraspNode = SystemNode;
    OutNode = SystemNode;
    flag = 0;
    return;
end

GraspNode = PrevTransitionHandle(GraspNode);

[OutNode ParentNode flag2] = ClampCableParallel(GraspNode, ManipID, RefID);

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

