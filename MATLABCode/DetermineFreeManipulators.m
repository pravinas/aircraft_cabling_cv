function [ FreeManipulators ] = DetermineFreeManipulators( SystemNode, ClampingManipulator )
%

nManipulators = max(size(SystemNode.State.Manipulator));
FreeManipulators = [];
for i = 1:nManipulators
    
    if i~=ClampingManipulator
        
        if isempty(SystemNode.State.Manipulator(i).cable);
            FreeManipulators = [FreeManipulators i];
        end
        
    end
    
end


end

