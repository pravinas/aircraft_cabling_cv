function [ SystemNode ] = ResolveInterlinkControlProblem( SystemNode, UsedManipulator )


% It is assumed that the resolving manipulator has already been assigned a
% gripping point to move the resolving cable

ViolatedLinkList = ViolatedInterlinks(SystemNode);
ControlPointLinkList = ViolatedLinkList;
[ControlPoint ReferenceLocation] = ClassifyInterlinks(SystemNode, ViolatedLinkList, UsedManipulator);

decayFactor = 0.1;
tempNode = SystemNode;

CurrentViolatedLinkList = ViolatedInterlinks(tempNode);
if isempty(CurrentViolatedLinkList)
    return
end
nIter = 100;
ManipulatorPositions = zeros(100,3);
iterCount = 1;
resolveFlag = 0;


while (~isempty(CurrentViolatedLinkList) && resolveFlag == 0)
    
    %Reconfigure the local control problem to add any new interlinks that
    %may have been stretched
    
    [ControlPointLinkList ControlPoint ReferenceLocation] = ReconfigureControlProblem(tempNode, CurrentViolatedLinkList, ControlPointLinkList, ControlPoint, ReferenceLocation, UsedManipulator);
    
    %Generate the current and the desired positions;
    
    [CurrentPosition DesiredPosition] = getCurrentAndDesiredPos(tempNode, ControlPoint, ReferenceLocation);
    
    %Generate the workspace drive command
    
    DesiredRate = DesiredPosition - CurrentPosition;
    
    %Compute the Local Jacobian
    LocalJacobian = CreateJacobian(tempNode, ControlPoint, UsedManipulator, decayFactor);
    
    %Evaluate the movement command
    MotionCommand = pinv(LocalJacobian)*DesiredRate;
    targetManipulatorPosition = tempNode.State.Manipulator(UsedManipulator).position + 0.1*MotionCommand';
    [tempNode ParentNode flag] = RepositionManipulator(SystemNode, UsedManipulator, targetManipulatorPosition);
     PlotState(tempNode)
     pause(1);
    ManipulatorPositions(iterCount,:) = targetManipulatorPosition;
    
    if iterCount >= nIter
        resolveFlag = 1;
    end
    iterCount = iterCount+1;
    CurrentViolatedLinkList = ViolatedInterlinks(tempNode);
end

[SystemNode ParentNode flag] = RepositionManipulatorParallel(SystemNode, UsedManipulator, targetManipulatorPosition);

end

