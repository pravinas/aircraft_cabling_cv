function [ ControlPoint, ReferenceLocation ] = ClassifyInterlinks( SystemNode, ViolatedLinkList, UsedManipulator )

% [ ControlPoint, ReferenceLocations ] = ClassifyInterlinkPoints( SystemNode, ViolatedLinkList, UsedManipulator )

% Classifies the endpoints of the violated interlinks into 

GraspedCable = SystemNode.State.Manipulator(UsedManipulator).cable;
Interlink = SystemNode.State.Interlink;

nViolatedLinks = size(ViolatedLinkList,1);
DesiredPos = [];
CurrentPos = [];
ControlPoint = [];
ReferenceLocation = [];

for i = 1:nViolatedLinks
    
    cable1 = Interlink(ViolatedLinkList(i)).cable1;
    cable2 = Interlink(ViolatedLinkList(i)).cable2;
    length1 = Interlink(ViolatedLinkList(i)).length1;
    length2 = Interlink(ViolatedLinkList(i)).length2;
    
    if cable1 == GraspedCable
        ControlPoint(i,1).cable = cable1;
        ControlPoint(i,1).length = length1;
        pos = GetPosition(SystemNode.State.Cable, cable1, length1)';
        CurrentPos = [CurrentPos; pos(1:2)];
        
        
        ReferenceLocation(i,1).cable = cable2;
        ReferenceLocation(i,1).length = length2;
        pos = GetPosition(SystemNode.State.Cable, cable2, length2)';
        DesiredPos = [DesiredPos; pos(1:2)];
    end
    
    if cable2 == GraspedCable
        ControlPoint(i,1).cable = cable2;
        ControlPoint(i,1).length = length2;
        pos = GetPosition(SystemNode.State.Cable, cable2, length2)';
        CurrentPos = [CurrentPos; pos(1:2)];
        
        ReferenceLocation(i,1).cable = cable1;
        ReferenceLocation(i,1).length = length2;
        pos = GetPosition(SystemNode.State.Cable, cable1, length1)';
        DesiredPos = [DesiredPos; pos(1:2)];
    end
end
end

