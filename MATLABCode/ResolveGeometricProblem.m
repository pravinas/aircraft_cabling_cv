function [ OutNode, ParentNode, SuccessFlags] = ResolveGeometricProblem( InNode, CorrespondingCablesStruct )
%

nCorrespondingCables = max(size(CorrespondingCablesStruct));

[InterlinkByCableStructure] = ClassifyInterlinkByCables(InNode);

%Set up the initial problem and the variables
for i = 1:nCorrespondingCables
[CableParameters(i).ControlPoint CableParameters(i).ReferenceLocation] = ClassifyInterlinks(InNode, CorrespondingCablesStruct(i).ViolatedInterlink, CorrespondingCablesStruct(i).AssignedManipulator);
CableParameters(i).ControlPointLinkList = CorrespondingCablesStruct.ViolatedInterlink;
end

decayFactor = 0.5;
tempNode = InNode;
maxIter = 100;
nIter = 1;
checksum = CheckCorrespondingCableInterlinks(tempNode, CorrespondingCablesStruct);

while checksum~=0 && nIter<=maxIter
    
    for i = 1:nCorrespondingCables
        UsedManipulator = CorrespondingCablesStruct.AssignedManipulator;
        if ~isempty(InterlinkViolatedOnCable(tempNode, CorrespondingCablesStruct.CableIndex)); %Thus take reconfiguring action only if the interlink on the current cable are violated
            
            currentViolatedLinkList = InterlinkViolatedOnCable(tempNode, CorrespondingCablesStruct.CableIndex);
            %Reconfigure the local control problem to add any new interlinks that
            %may have been stretched
            [CableParameters(i).ControlPointLinkList CableParameters(i).ControlPoint CableParameters(i).ReferenceLocation] = ReconfigureControlProblem(tempNode, currentViolatedLinkList, CableParameters(i).ControlPointLinkList, CableParameters(i).ControlPoint, CableParameters(i).ReferenceLocation, UsedManipulator );
            
            %Generate the current and the desired positions;
            [CurrentPosition, DesiredPosition] = getCurrentAndDesiredPos(tempNode, CableParameters(i).ControlPoint, CableParameters(i).ReferenceLocation);
            
            %Generate the workspace drive command    
            DesiredRate = DesiredPosition - CurrentPosition;
            
            %Compute the Local Jacobian
            LocalJacobian = CreateJacobian(tempNode, CableParameters(i).ControlPoint, UsedManipulator, decayFactor);       
            
            %Evaluate the movement command
            MotionCommand = pinv(LocalJacobian)*DesiredRate;
            targetManipulatorPosition(i,:) = tempNode.State.Manipulator(UsedManipulator).position + 0.1*MotionCommand';
            [tempNode ParentNode flag] = RepositionManipulator(tempNode, UsedManipulator, targetManipulatorPosition);
            %PlotStateLargeGoal(tempNode);
        end
    end
    
    nIter = nIter+1;
    checksum = CheckAllInterlinks(tempNode);
    
end


for i = 1:nCorrespondingCables
    SuccessFlags(i) = ~max(size(InterlinkViolatedOnCable(tempNode, CorrespondingCablesStruct(i).CableIndex)));
    [OutNode ParentNode flag] = RepositionManipulatorParallel(InNode, CorrespondingCablesStruct(i).AssignedManipulator, targetManipulatorPosition(i,:));
end



end

