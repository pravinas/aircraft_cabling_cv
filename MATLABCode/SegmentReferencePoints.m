function [ CorrespondingCablesStruct ] = SegmentReferencePoints( SystemNode , CorrespondingCablesStruct )
%


nCorrespondingCables = max(size(CorrespondingCablesStruct));

for i = 1:nCorrespondingCables
    
    currentCorrespondingCable = CorrespondingCablesStruct(i);
    currentCorrespondingCableID = currentCorrespondingCable.CableIndex;
    ReferencePointLocations = SystemNode.State.Cable(currentCorrespondingCableID).refPointPos;
    nReferencePoints = size(ReferencePointLocations,2);
    
    %Determine the left reference point
    LeftmostInterlinkLength = currentCorrespondingCable.ViolatedInterlinkLength(1);
    
    for j=1:nReferencePoints
        
        if ReferencePointLocations(j) >= LeftmostInterlinkLength;
            LeftmostReferencePoint = j-1;
            LeftmostReferenceLength = ReferencePointLocations(j-1);
            break;        
        end
    
    end
    
    nViolatedInterlinks = max(size(currentCorrespondingCable.ViolatedInterlink));
    RightmostInterlinkLength = currentCorrespondingCable.ViolatedInterlinkLength(nViolatedInterlinks);
    for j = nReferencePoints:-1:1
        
        if ReferencePointLocations(j) <= RightmostInterlinkLength;
            RightmostReferencePoint = j+1;
            RightmostReferenceLength = ReferencePointLocations(j+1);
            break
        end
    end
    
    
    currentCorrespondingCable.AffectedReferencePoints = [LeftmostReferencePoint:1:RightmostReferencePoint];
    CorrespondingCablesStruct(i) = currentCorrespondingCable;
    
    
end


end

