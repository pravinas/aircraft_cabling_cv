function [ OutNode ParentNode flag ] = RepositionManipulator( InNode, manipID, targetPos )
% [Cable Manipulator] = RepositionManipulator(Cable, Manipulator, manipID,
% targetPos);

% This function repositions the specified manipulator to the target
% position. If the cable is stretched then it returns an error message
% stating it

% Cable: an array of Cable structures that specify the boundary condition
% and the input configuration

% Manipulator: an array of structures of the type Manipulator

% manipID: Index of the manipulator that must be repositioned

% targetPos: A row vector specifying the target state of the manipulator

% Create the Parent node and the action variable that will transfer to the
% new state

% Assign the states and the parent Node
ParentNode = InNode;
InState = InNode.State;

% Assign the State variables as used in the function
Cable = InState.Cable;
Manipulator = InState.Manipulator;
Interlink = InState.Interlink;

selectedManipulator = Manipulator(manipID); % Isolate the required manipulator
selectedManipulator.position = targetPos; % Set the position
Manipulator(manipID) = selectedManipulator; % Re-insert the modified Manipulator in the array


% Reevaluate the configuration of the cables if the manipulator is grasping
% any cable

if ~isempty(Manipulator(manipID).cable)
    cableID = Manipulator(manipID).cable;
    selectedCable = Cable(cableID);
    [selectedCable errorFlag] = ComputeCableShape(selectedCable, Manipulator); %ADD ERROR CHECKER HERE%
    
    %Error checker for stretching the cable
    if errorFlag == 0
        OutNode = InNode;
        flag = 0; %Declare the action as unsuccessful
        return;
    end
    
    Cable(cableID) = selectedCable; %Reinsert the selected Cable in the structure
end

% Check for stretching the Interlinks
FunctionNode = InNode; % Copy the StartState and the Action Variables but only need to update the state.
FunctionNode.State.Cable = Cable;
FunctionNode.State.Manipulator = Manipulator;
FunctionNode.State.Interlink = Interlink;
[FunctionNode] = InterlinkChecker(FunctionNode);
Cable = FunctionNode.State.Cable;
Manipulator = FunctionNode.State.Manipulator;
Interlink = FunctionNode.State.Interlink;

% Action is now sucessful start creating the OutNode

flag = 1; %Declare action successful

%Start Creating the OutNode
OutNode.StartState = InNode.StartState;

%Add the action to the ActionList
ActionList = InNode.ActionList; %Extract the ActionList
n = size(ActionList,1);
LastActionSet = ActionList{n,1};
AddColumn = @(InputNode)RepositionManipulator(InputNode, manipID, targetPos);
LastActionSet = AddColumntoCellArray(LastActionSet, AddColumn);
ActionList{n,1} = LastActionSet;
OutNode.ActionList = ActionList;

% Create Output Structure
OutState.Cable = Cable;
OutState.Manipulator = Manipulator;
OutState.Interlink = Interlink;

OutNode.State = OutState;

end

