function [ OutNode, ParentNode, funflag ] = UnclampCable( InNode, cableID, clampID )
% [ Cable, Manipulator ] = Unclamp( Cable, Manipulator, cableID, clampID )

% This function releases the cable held at the selected Clamp point. It
% must be noted that the robotic system is not involved in this operation

% The error modes are:
% 1) The clamping point to be released is not defined for the given cable
% 2) The clamping point to be released is not occupied. (Not an error hence
% does not return the message

ParentNode = InNode;
InState = InNode.State;

Cable = InState.Cable;
Manipulator = InState.Manipulator;
Interlink = InState.Interlink;

selectedCable = Cable(cableID);
nclamps = size(selectedCable.clampPos);
if clampID > nclamps
    warning('No such clamping point exists')
    funflag = 0;
    OutNode = InNode;
    return;
end

% Check if the selected point is on the list of used point
if ~isempty(selectedCable.clamped)
[flag, index] = isContain(selectedCable.clamped(:,2), clampID);
else
    flag = 0;
    index = 0;
end

if flag == 0 % Clamp not occupied
    funflag = 1;
    OutState.Cable = Cable;
    OutState.Manipulator = Manipulator;
    OutState.Interlink = Interlink;

    OutNode.State = OutState;
    return;
else 
    
    selectedCable.clamped = DeleteElement(selectedCable.clamped, index); % Remove the clamp point from the list
    selectedCable = ComputeCableShape(selectedCable, Manipulator);
    Cable(cableID) = selectedCable; %Reinsert the cable into the array
    
    FunctionNode = InNode;
    FunctionNode.State.Cable = Cable;
    FunctionNode.State.Manipulator = Manipulator;
    FunctionNode.State.Interlink = Interlink;
    [FunctionNode] = InterlinkChecker(FunctionNode);
    Cable = FunctionNode.State.Cable;
    Manipulator = FunctionNode.State.Manipulator;
    Interlink = FunctionNode.State.Interlink;
end

funflag = 1;

%Create the OutNode
OutNode.StartState = InNode.StartState;

%Add to the ActionList
ActionList = InNode.ActionList;
AddRow{1,1} = @(InputNode)UnclampCable(InputNode, cableID, clampID);
ActionList = AddRowtoCellArray(ActionList, AddRow);
OutNode.ActionList = ActionList;

OutState.Cable = Cable;
OutState.Manipulator = Manipulator;
OutState.Interlink = Interlink;

OutNode.State = OutState;

end

