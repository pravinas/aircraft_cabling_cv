function [ OutNode, ParentNode, flag ] = GraspCableParallel(InNode, manipID, cableID, gripID )

% [ Cable Manipulator ] = GraspCable( Cable, Manipulator, manipID, CableID, gripID )

% Cable: an array of structures of the type Cable.

% Manipulator: an array of the type Manipulator

% cableID: Index of the cable that must be grasped

% manipID: Index of the manipulator that must be used for grasping
% operation



% The manipulator is repositioned to grasp the selected Cable. An error
% will be generated if any of the following conditions is true

% Selected Manipulator is already occupied
% The selected gripping point is already gripped
% gripID more than number of gripping points on a cable

ParentNode = InNode;
InState = InNode.State;

%Reassign the Structures
Cable = InState.Cable;
Manipulator = InState.Manipulator;
Interlink = InState.Interlink;

selectedManipulator = Manipulator(manipID);
selectedCable = Cable(cableID);

if ~isempty(selectedManipulator.cable) % Check the manipulator occupancy
    warning('Manipulator Occupied');
    flag = 0;
    OutNode = InNode;
    return;
end

if ~isempty(selectedCable.gripped)
if isContain(selectedCable.gripped(:,1), gripID)
    warning('Grip point gripped using another manipulator');
    flag = 0;
    OutNode = InNode;
    return;
end
end

if ~isempty(selectedCable.clamped)
    gripLength = selectedCable.gripPointsPos(gripID);
    if isContain(selectedCable.clamped, gripLength)
        warning('Grip Point already clamped')
        flag = 0;
        OutNode = InNode;
        return;
    end
end

if length(selectedCable.gripPointsPos) < gripID
    warning('Grip point non existent');
    flag = 0;
    OutNode = InNode;
    return;
end

% Reposition the requested gripper to the gripping point position
GripPos = getGrippingPosition(Cable, cableID, gripID);
gripTarget = GripPos.state;
gripTarget(3) = 0; %All the gripping actions will result in the cable being locally horizontal
% Apply the Reposition action on the temporary node

FunctionNode = InNode;
FunctionNode.State.Cable = Cable;
FunctionNode.State.Manipulator = Manipulator;
FunctionNode.State.Interlink = Interlink;
[FunctionNode, ~, flag] = RepositionManipulator(FunctionNode, manipID, gripTarget);
% ADD ERROR CHECKS HERE
Cable = FunctionNode.State.Cable;
Manipulator = FunctionNode.State.Manipulator;
Interlink = FunctionNode.State.Interlink;


selectedManipulator = Manipulator(manipID);
selectedCable = Cable(cableID);

% Add the manipulator gripping point pair to the Cable structure
selectedManipulator.cable = cableID;
selectedManipulator.grip = gripID;

selectedCable.gripped = push(selectedCable.gripped, [gripID manipID]);

Manipulator(manipID) = selectedManipulator; %Re-insert the manipulator in the array
[selectedCable] = ComputeCableShape(selectedCable, Manipulator); %Recompute the shape of the cable to reflect the gripping action
Cable(cableID) = selectedCable; %Re-insert the cable into the array

FunctionNode = InNode;
FunctionNode.State.Cable = Cable;
FunctionNode.State.Manipulator = Manipulator;
FunctionNode.State.Interlink = Interlink;
%[FunctionNode] = InterlinkChecker(FunctionNode);
Cable = FunctionNode.State.Cable;
Manipulator = FunctionNode.State.Manipulator;
Interlink = FunctionNode.State.Interlink;

% The action is now successful. the ActionList can be updated and OutNode
% can be created

flag = 1; %Declare the action as a success

%Start creating the OutNode
OutNode.StartState = InNode.StartState;

%Add the action to the ActionList
ActionList = InNode.ActionList;
AddRow{1,1} = @(InputNode)GraspCable(InputNode, manipID, cableID, gripID);
ActionList = AddRowtoCellArray(ActionList, AddRow);
OutNode.ActionList = ActionList;

%Assign the output structures
OutState.Cable = Cable;
OutState.Manipulator = Manipulator;
OutState.Interlink = Interlink;

%Assign the output Node
OutNode.State = OutState;

end

