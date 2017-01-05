function [ OutNode ParentNode flag ] = ClampCable( InNode, manipID, clampID)

% [ Cable, Manipulator ] = ClampCable( Cable, Manipulator, manipID, gripID)

% This function attaches the cable held by the selected manipulator to the
% clamping point specific to the grasped cable

%Error conditions are
% 1) Vacant manipulator
% 2) Occupied clamping point
% 3) Stretched Cable: handled by shape computation
% 4) Clamping point not available for the cable

%An edit must be made to the functionality to ensure that if any other
%manipulators are also holding the cable, they must move in formation and
%maintain the separation between the manipulators that existed before the
%repositioning for clamping


% Reassign the input structures to the structures used in the function
ParentNode = InNode;
InState = InNode.State;
% FunctionNode = InNode;

% Extract the Cable, Manipulator and Interlink structures
Cable = InState.Cable;
Manipulator = InState.Manipulator;
Interlink = InState.Interlink;

if isempty(Manipulator(manipID).cable) % Detect an unoccupied manipulator and return error
    warning('Vacant manipulator')
    flag = 0;
    OutNode = InNode;
    return;
end

cableID = Manipulator(manipID).cable;
selectedCable = Cable(cableID);

if ~isempty(selectedCable.clamped)
if isContain(selectedCable.clamped(:,2), clampID); %Check occupancy of the clamping point
    warning('Clamping point occupied');
    flag = 0;
    OutNode = InNode;
end
end

if clampID > size(selectedCable.clampPos,1)
    warning('Clamp point unavailable');
    flag = 0;
    OutNode = InNode;
end



% Determine the clamping target
clampTarget = selectedCable.clampPos(clampID, :);
FunctionNode = InNode;
FunctionNode.State.Cable = Cable;
FunctionNode.State.Manipulator = Manipulator;
FunctionNode.State.Interlink = Interlink;


% Determine if any other manipulators are operating on the same cable
nManipulators = max(size(Manipulator));
CoManipulators = [];
CoManipulatorPosition = [];
CoManipulatorGripLength = [];
for i = 1:nManipulators
    if ~isempty(Manipulator(i).cable)
        if Manipulator(i).cable == cableID && i ~= manipID;
            CoManipulators = [CoManipulators; i];
            CoManipulatorGripPoint = Manipulator(i).grip;
            CoManipulatorGripLength = [CoManipulatorGripLength Cable(cableID).gripPointsPos(CoManipulatorGripPoint)];
            CoManipulatorPosition = [CoManipulatorPosition; Manipulator(i).position];
        end
    end
end

% Determine the positions of the other manipulators assuming that the
% formation is held

if ~isempty(CoManipulators)
    
    LeadManipulatorPosition = Manipulator(manipID).position;
    LeadManipulatorGripID = Manipulator(manipID).grip;
    LeadManipulatorGripLength = Cable(cableID).gripPointsPos(LeadManipulatorGripID);
    nCoManipulators = max(size(CoManipulators));
    for i = 1:nCoManipulators
        initialTargetDistance = norm(CoManipulatorPosition(i,:) - clampTarget);
        slack = abs(CoManipulatorGripLength(i) - LeadManipulatorGripLength);
        if (initialTargetDistance >= slack)
            initialDistance = norm(LeadManipulatorPosition - CoManipulatorPosition(i,:));
            Direction = (CoManipulatorPosition(i,:) - clampTarget)./norm(CoManipulatorPosition(i,:) - clampTarget);
            CoManipulatorTarget = clampTarget + initialDistance*Direction;
            [FunctionNode ParentNode flag] = RepositionManipulator(FunctionNode, CoManipulators(i), CoManipulatorTarget);
        end
    end
    
end

%Reposition the clamping manipulator to the clamp position
%Create a function node here to parse structures to an internal action


[FunctionNode FunctionParentNode repositionFlag] = RepositionManipulator(FunctionNode, manipID, clampTarget);

%Success Checker for reposition
if repositionFlag == 0
    OutNode = InNode;
    flag = 0;
    return;
end

%Reassign State Structures
Cable = FunctionNode.State.Cable;
Manipulator = FunctionNode.State.Manipulator;
Interlink = FunctionNode.State.Interlink;

selectedCable = Cable(cableID);
selectedManipulator = Manipulator(manipID);

% Add the clamp information to the clamp array
gripID = selectedManipulator.grip;
clampLength = selectedCable.gripPointsPos(gripID);
addToClamped = [clampLength, clampID];
selectedCable.clamped = push(selectedCable.clamped, addToClamped);

%Delete the manipulator grip from the gripped point list
[~, index] = isContain(selectedCable.gripped(:,2), manipID);
selectedCable.gripped = DeleteElement(selectedCable.gripped, index); % Remove the corresponding gripped row from the list
% Delete the cable information from the manipulator
selectedManipulator.cable = [];
selectedManipulator.grip = [];

% Re-insert the manipulator into the structure array
Manipulator(manipID) = selectedManipulator;

% Recompute the cable configuration
[selectedCable errorFlag] = ComputeCableShape(selectedCable, Manipulator);
if errorFlag == 0
        OutNode = InNode;
        flag = 0; %Declare the action as unsuccessful
        return;
end
    
% Reinsert the cable into the structure
Cable(cableID) = selectedCable;

FunctionNode = InNode;
FunctionNode.State.Cable = Cable;
FunctionNode.State.Manipulator = Manipulator;
FunctionNode.State.Interlink = Interlink;
[FunctionNode] = InterlinkChecker(FunctionNode);
Cable = FunctionNode.State.Cable;
Manipulator = FunctionNode.State.Manipulator;
Interlink = FunctionNode.State.Interlink;

flag = 1; % Declare the actions succesful

% Start Constructing the OutNode
OutNode.StartState = InNode.StartState;

%Add the action to the ActionList
ActionList = InNode.ActionList;
AddRow{1,1} = @(InputNode)ClampCable(InputNode, manipID, clampID);
ActionList = AddRowtoCellArray(ActionList, AddRow);
OutNode.ActionList = ActionList;

% Reassign the output structure
OutState.Cable = Cable;
OutState.Manipulator = Manipulator;
OutState.Interlink = Interlink;

OutNode.State = OutState;


end

