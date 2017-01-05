function [ OutNode, SuccessFlag ] = AlignReferencePointList( SystemNode, Tolerance,  ReferencePointsList, FreeManipulators, OccupiedManipulators )
%


RemainingReferencePointsList = ReferencePointsList';

exitFlag = 0;
SuccessFlag = 0;
TempNode = SystemNode;
ClampingManipulator = FreeManipulators(1);
AttemptsSinceLastSuccess = 0;

while exitFlag == 0
    
    if isempty(RemainingReferencePointsList)
        
        SuccessFlag = 1;
        exitFlag = 1;
        break;
    else
        
        currentListSize = max(size(RemainingReferencePointsList));
        selectedReferencePoint = RemainingReferencePointsList(1)
        [TempNode TempGraspNode ParentNode TransitionHandle flag] = AlignRefPoint(TempNode, selectedReferencePoint.CableID, ClampingManipulator, selectedReferencePoint.RefID , Tolerance);
        ViolatedInterlinksList = ViolatedInterlinks(TempNode);
        
        if flag == 0
            RemainingReferencePointsList = DeleteElement(RemainingReferencePointsList,1);
            RemainingReferencePointsList = [RemainingReferencePointsList; selectedReferencePoint];
            AttemptsSinceLastSuccess = AttemptsSinceLastSuccess+1;
            TempNode = ParentNode;
        end
            
        if flag == 1
            if isempty(ViolatedInterlinksList)

                RemainingReferencePointsList = DeleteElement(RemainingReferencePointsList, 1);
                AttemptsSinceLastSuccess = 0;
            else

                NewFreeManipulators = DetermineFreeManipulators(TempNode, ClampingManipulator);
                [TempNode, ResolveSuccessFlag] = ResolveInterlinkConflict(TempGraspNode, TransitionHandle, Tolerance, OccupiedManipulators, NewFreeManipulators, ClampingManipulator);
                if ResolveSuccessFlag == 1

                    RemainingReferencePointsList = DeleteElement(RemainingReferencePointsList, 1);
                    AttemptsSinceLastSuccess = 0;

                else

                    RemainingReferencePointsList = DeleteElement(RemainingReferencePointsList,1);
                    RemainingReferencePointsList = [RemainingReferencePointsList; selectedReferencePoint];
                    AttemptsSinceLastSuccess = AttemptsSinceLastSuccess+1;
                    TempNode = ParentNode; %Backtrack to last successful clamping

                end
            end
        end
        if AttemptsSinceLastSuccess >= currentListSize; %Declare failure to align all the points in the list
                
            exitFlag = 1;
            SuccessFlag = 0;
        end
    end
    
    OutNode = TempNode;
    
    
end

end

