function [ FinalAlignmentList idealNode ] = DetermineReferencePointsToAlign( SystemNode, Tolerance, CorrespondingCablesStruct, FreeManipulators, CurrentAlignmentList )
% FinalAlignmentList is the CurrentAlignmentList and the appended list of
% additional nodes that need to be added to the SystemNode Alignment

nCorrespondingCables = max(size(CorrespondingCablesStruct));
tempNode = SystemNode;
UseManip = FreeManipulators(1);
FinalAlignmentList = CurrentAlignmentList;


    
    
    
    for i = 1:nCorrespondingCables
        selectedCorrespondingCable = CorrespondingCablesStruct(i);
        CableID = selectedCorrespondingCable.CableIndex;
        RelevantInterlinks = selectedCorrespondingCable.ViolatedInterlink;
        SortedRefPointList = ArrangeReferencePointsByPreference(tempNode, selectedCorrespondingCable);
        
        exitFlag = 0;
        j=1;
        
        while exitFlag == 0
            
        [tempNode, GraspNode, ParentNode, flag] = AlignRefPoint(tempNode, CableID, UseManip, SortedRefPointList(j),Tolerance);
        newElement.CableID = CableID;
        newElement.RefID = SortedRefPointList(j);
        FinalAlignmentList = [FinalAlignmentList newElement];
        LinkFlags = IsInterlinkViolated(tempNode, RelevantInterlinks);
        sumTotal = sum(LinkFlags);
        if sumTotal > 0
            exitFlag = 0;
        else
            exitFlag = 1;
            idealNode = tempNode;
        end
        j=j+1;    
        end
        
        newViolatedInterlinks = ViolatedInterlinks(tempNode);
        if isempty(newViolatedInterlinks)
            continue
        else
            [newCorrespondingCables newCorrespondingCablesStruct] = DetermineCorrespondingCables(tempNode, newViolatedInterlinks, CableID);
            [newCorrespondingCablesStruct] = SegmentReferencePoints(tempNode, newCorrespondingCablesStruct);
            [InterlinkByCableStructure] = ClassifyInterlinkByCables(SystemNode);
            [newCorrespondingCablesStruct] = IdentifyRelevantGripPoints(SystemNode, newCorrespondingCablesStruct, InterlinkByCableStructure);
            [AddedAlignmentList idealNode] = DetermineReferencePointsToAlign(tempNode, Tolerance, newCorrespondingCablesStruct, FreeManipulators, FinalAlignmentList);
            FinalAlignmentList = AddedAlignmentList;
        end
        
        
        
    end
    
end




