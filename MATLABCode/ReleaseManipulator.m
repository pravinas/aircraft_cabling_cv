function [ OutNode, ParentNode, flag ] = ReleaseManipulator( InNode, manipID )
% [Cable Manipulator] = ReleaseManip(Cable, Manipulator, manipID)

% Cable: An array of structures of the type Cable
% Manipulator: An array of structures of the type Manipulator
% manipID: Index of the manipulator that must be released

ParentNode = InNode;
InState = InNode.State;
Cable = InState.Cable;
Manipulator = InState.Manipulator;
Interlink = InState.Interlink;

selectedManipulator = Manipulator(manipID);
if ~isempty(selectedManipulator.cable);
    cableID = selectedManipulator.cable;
    selectedCable = Cable(cableID);

    [flag index] = isContain(selectedCable.gripped(:,2), manipID);
    selectedCable.gripped = DeleteElement(selectedCable.gripped, index); % Remove the corresponding gripped row from the list
    [selectedCable] = ComputeCableShape(selectedCable, Manipulator);
    Cable(cableID) = selectedCable; %Re-insert the cable
end

% Delete the entry from the Manipulator field
selectedManipulator.cable = [];
selectedManipulator.grip = [];

Manipulator(manipID) = selectedManipulator;

FunctionNode = InNode;
FunctionNode.State.Cable = Cable;
FunctionNode.State.Manipulator = Manipulator;
FunctionNode.State.Interlink = Interlink;
[FunctionNode] = InterlinkChecker(FunctionNode);
Cable = FunctionNode.State.Cable;
Manipulator = FunctionNode.State.Manipulator;
Interlink = FunctionNode.State.Interlink;

flag = 1;

%Create the OutNode
OutNode.StartState = InNode.StartState;

% Add the action to the ActionList
ActionList = InNode.ActionList;
AddRow{1,1} = @(InputNode)ReleaseManipulator(InputNode, manipID);
ActionList = AddRowtoCellArray(ActionList, AddRow);
OutNode.ActionList = ActionList;

OutState.Cable = Cable;
OutState.Manipulator = Manipulator;
OutState.Interlink = Interlink;

OutNode.State = OutState;

end

