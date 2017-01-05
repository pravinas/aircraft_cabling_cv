function [ ControlPointLinkList, ControlPoint, ReferenceLocation ] = ReconfigureControlProblem(SystemNode,  currentViolatedLinkList, ControlPointLinkList, ControlPoint, ReferenceLocation, UsedManipulator  )

%




% For each link in the current violated links, check if it is already a
% part of the control problem

nViolatedLinks = size(currentViolatedLinkList,1);

if nViolatedLinks ~= 0
    Interlink = SystemNode.State.Interlink;
    for i = 1:nViolatedLinks
        if ~isContain(ControlPointLinkList, currentViolatedLinkList(i))
            
            ControlPointLinkList = [ControlPointLinkList; currentViolatedLinkList(i)];
            GraspedCable = SystemNode.State.Manipulator(UsedManipulator).cable;
            
            cable1 = Interlink(currentViolatedLinkList(i)).cable1;
            cable2 = Interlink(currentViolatedLinkList(i)).cable2;
            length1 = Interlink(currentViolatedLinkList(i)).length1;
            length2 = Interlink(currentViolatedLinkList(i)).length2;
            
            if cable1 == GraspedCable
                ControlPointadd.cable = cable1;
                ControlPointadd.length = length1;
                ControlPoint = [ControlPoint; ControlPointadd];
                        
                ReferenceLocationadd.cable = cable2;
                ReferenceLocationadd.length = length2;
                ReferenceLocation = [ReferenceLocation; ReferenceLocationadd];
                
            end
            
            if cable2 == GraspedCable
                ControlPointadd.cable = cable2;
                ControlPointadd.length = length2;
                ControlPoint = [ControlPoint; ControlPointadd];
        
                ReferenceLocationadd.cable = cable1;
                ReferenceLocationadd.length = length2;
                ReferenceLocation = [ReferenceLocation; ReferenceLocationadd];
            end
        end
    end
end






end

