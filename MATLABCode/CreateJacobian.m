function [ LocalJacobian ] = CreateJacobian( SystemNode, ControlPoint, UsedManipulator, decayFactor )
%

ManipPosition = SystemNode.State.Manipulator(UsedManipulator).position;
ManipPosition = ManipPosition(1:2)';
ManipCable = SystemNode.State.Manipulator(UsedManipulator).cable;
ManipGripID = SystemNode.State.Manipulator(UsedManipulator).grip;
ManipLength = getGrippingPosition(SystemNode.State.Cable, ManipCable, ManipGripID);
ManipLength = ManipLength.length;

%Now evaluate the manipulator cable configuration jacobian


LocalJacobian = [];
for i = 1:length(ControlPoint)
    
    %Evaluate Weight
    currentLength = (ControlPoint(i).length);
    currentWeight = exp(-decayFactor*abs(ManipLength - currentLength));
    currentWeightMatrix = diag([currentWeight currentWeight]);
    
    %Evaluate the translational Jacobian
    JTrans = eye(2);
    
    %Evaluate the rotational Jacobian
    ControlPointPos = GetPosition(SystemNode.State.Cable, ControlPoint(i).cable, ControlPoint(i).length);
    ControlPointPos = ControlPointPos(1:2)';
    
    deltaPos = ControlPointPos - ManipPosition;
    theta = atan2(deltaPos(2), deltaPos(1));
    
    R = norm(deltaPos);
    
    JRot = [-R*sin(theta); R*cos(theta)];
    
    CurrentJacobian = currentWeightMatrix*[JTrans JRot];
    
    LocalJacobian = [LocalJacobian; CurrentJacobian];
end




end

