function [ OccupiedManipulators ] = DetermineOccupiedManipulators(SystemNode, ClampingManipulator)
%

nManipulators = max(size(SystemNode.State.Manipulator));
OccupiedManipulators = [];
for i = 1:nManipulators
    
    if i~= ClampingManipulator
    
    if ~isempty(SystemNode.State.Manipulator(i).cable)
        
        newOccupiedManipulators.ManipID = i;
        newOccupiedManipulators.CableID = SystemNode.State.Manipulator(i).cable;
        newOccupiedManipulators.GripPointID = SystemNode.State.Manipulator(i).grip;
        OccupiedManipulators = [OccupiedManipulators newOccupiedManipulators];
    end
    end
    
end


end

