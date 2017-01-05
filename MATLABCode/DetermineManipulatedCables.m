function [ CablesManipulatedList ] = DetermineManipulatedCables( OccupiedManipulators )
%

nOccupiedManipulators = max(size(OccupiedManipulators));

if nOccupiedManipulators > 0
    CablesManipulatedList = zeros(1,nOccupiedManipulators);
    for i = 1:nOccupiedManipulators
        
        CablesManipulatedList(i) = OccupiedManipulators.CableID;
        
    end
else
    CablesManipulatedList = [];
end

end

