function [ CorrespondingCablesStruct ] = IdentifyRelevantGripPoints( SystemNode, CorrespondingCablesStruct, InterlinkByCableStructure )
%

nCorrespondingCables = max(size(CorrespondingCablesStruct));
CorrespondingCablesStruct(1).GripPointsConsidered = [];
for i = 1:nCorrespondingCables
    
    currentCorrespondingCable = CorrespondingCablesStruct(i);
    currentCorrespondingCableID = currentCorrespondingCable.CableIndex;
    LeftmostStretchedInterlink = currentCorrespondingCable.ViolatedInterlink(1);
    nViolatedLinks = max(size(currentCorrespondingCable.ViolatedInterlink));
    RightmostStretchedInterlink = currentCorrespondingCable.ViolatedInterlink(nViolatedLinks);
    nAffectedReferencePoints = max(size(currentCorrespondingCable.AffectedReferencePoints));
    LeftmostReferencePoint = currentCorrespondingCable.AffectedReferencePoints(1);
    LeftmostConsideredLength = SystemNode.State.Cable(currentCorrespondingCableID).refPointPos(LeftmostReferencePoint);
    RightmostReferencePoint = currentCorrespondingCable.AffectedReferencePoints(nAffectedReferencePoints);
    RightmostConsideredLength = SystemNode.State.Cable(currentCorrespondingCableID).refPointPos(RightmostReferencePoint);
    
    selectedInterlinkByCableStructure = InterlinkByCableStructure(currentCorrespondingCableID);
    [containFlag, index] = isContain(selectedInterlinkByCableStructure.InterlinkID, LeftmostStretchedInterlink);    
    if containFlag == 1
        while index > 1
            LeftmostConsideredInterlinkIndex = index-1;
            LeftmostConsideredLength2 = selectedInterlinkByCableStructure.InterlinkLength(LeftmostConsideredInterlinkIndex);
            if LeftmostConsideredLength2 <= LeftmostConsideredLength
                LeftmostConsideredLength = LeftmostConsideredLength2;
                break;
            end
            index = index-1;
        end
%         if index > 1
%             LeftmostConsideredInterlinkIndex = index-1;
%             LeftmostConsideredLength = selectedInterlinkByCableStructure.InterlinkLength(LeftmostConsideredInterlinkIndex);
%         else
%             %LeftmostConsideredInterlinkIndex = 1;
%             LeftmostReferencePoint = currentCorrespondingCable.AffectedReferencePoints(1);
%             LeftmostConsideredLength = SystemNode.State.Cable(currentCorrespondingCableID).refPointPos(LeftmostReferencePoint);
%         end
    else
        LeftmostConsideredInterlinkIndex = 1;
        LeftmostConsideredLength = selectedInterlinkByCableStructure.InterlinkLength(LeftmostConsideredInterlinkIndex);
    end
    
    
    
    
    [containFlag, index] = isContain(selectedInterlinkByCableStructure.InterlinkID, RightmostStretchedInterlink);
    nInterlinksOnCable = max(size(selectedInterlinkByCableStructure.InterlinkID));    
    if containFlag == 1
        while index < nInterlinksOnCable
            RightmostConsideredInterlinkIndex = index + 1;
            RightmostConsideredLength2 = selectedInterlinkByCableStructure.InterlinkLength(RightmostConsideredInterlinkIndex);
            if RightmostConsideredLength2 >= RightmostConsideredLength
                RightmostConsideredLength = RightmostConsideredLength2;
                break
            end
            index = index+1;
        end
            
%         if index < nInterlinksOnCable
%             RightmostConsideredInterlinkIndex = index + 1;
%             RightmostConsideredLength = selectedInterlinkByCableStructure.InterlinkLength(RightmostConsideredInterlinkIndex);
%         else
%             %RightmostConsideredInterlinkIndex = nInterlinksOnCable;
%             nAffectedReferencePoints = max(size(currentCorrespondingCable.AffectedReferencePoints));
%             RightmostReferencePoint = currentCorrespondingCable.AffectedReferencePoints(nAffectedReferencePoints);
%             RightmostConsideredLength = SystemNode.State.Cable(currentCorrespondingCableID).refPointPos(RightmostReferencePoint);
%         end
    else
        RightmostConsideredInterlinkIndex = nInterlinksonCable;
        RightmostConsideredLength = selectedInterlinkByCableStructure.InterlinkLength(RightmostConsideredInterlinkIndex);
    end
    
    
    
    FullGripPointList = SystemNode.State.Cable(currentCorrespondingCableID).gripPointsPos;
    GripPointsConsidered = [];
    nGripPoints = max(size(FullGripPointList));
    
    for j = 1:nGripPoints
        
        if FullGripPointList(j) >= LeftmostConsideredLength && FullGripPointList(j) <= RightmostConsideredLength
            GripPointsConsidered = [GripPointsConsidered j];
        end
        
    end
    
    currentCorrespondingCable.GripPointsConsidered = GripPointsConsidered;
    CorrespondingCablesStruct(i) = currentCorrespondingCable;
    
    
    
end


end

