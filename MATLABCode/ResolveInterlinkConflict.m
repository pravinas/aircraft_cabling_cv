function [ OutNode InterlinkResolutionSuccessFlag ] = ResolveInterlinkConflict( GraspNode, TransitionHandle, Tolerance, OccupiedManipulators, FreeManipulators, ClampingManipulator )

%The end state for this function involves the interlink conflicts satisfied
%and the FreeManipulators not being occupied at the end.

%The grasping points of the OccupiedManipulators remain unchanged at the
%end of the function

% Create the Corresponding Cables Structure to identify the cables
% corresponding to the violated interlinks
SystemNode = TransitionHandle(GraspNode);
ClampedCable = GraspNode.State.Manipulator(ClampingManipulator).cable;

CurrentViolatedInterlinks = ViolatedInterlinks(SystemNode);
[CorrespondingCables CorrespondingCablesStruct] = DetermineCorrespondingCables(SystemNode, CurrentViolatedInterlinks, ClampedCable);
[InterlinkByCableStructure] = ClassifyInterlinkByCables(SystemNode);
[CorrespondingCablesStruct] = SegmentReferencePoints(SystemNode, CorrespondingCablesStruct);
[CorrespondingCablesStruct] = IdentifyRelevantGripPoints(SystemNode, CorrespondingCablesStruct, InterlinkByCableStructure);
nCorrespondingCables = max(size(CorrespondingCablesStruct));
CorrespondingCablesStruct(1).AssignedManipulator = []; %These will be filled in later

%Determine the Corresponding cables that are not already occupied
CablesOriginallyManipulated = DetermineManipulatedCables(OccupiedManipulators);
CablesUnassigned = [];
for i = 1:nCorrespondingCables
    
    if ~isContain(CablesOriginallyManipulated, CorrespondingCablesStruct(i).CableIndex)
        
        CablesUnassigned = [CablesUnassigned i]; %CablesUnassigned elements refer to the index of the cable in the CorrespondingCables structure
    else
        
        [~, index] = isContain(CablesOriginallyManipulated, CorrespondingCablesStruct(i).CableIndex);
        CorrespondingCablesStruct(i).AssignedManipulator = OccupiedManipulators(index).ManipID;
        
    end
    
end

%Now if the number of unassigned cables is greater than the number of
%FreeManipulators, the interlink resolution is infeasible

nCablesUnassigned = max(size(CablesUnassigned));
nFreeManipulators = max(size(FreeManipulators));

if nCablesUnassigned > nFreeManipulators %Interlink resolution is not possible as the number of cables affected is more than the manipulators available
    
    [OutNode ParentNode flag] = TransitionHandle(GraspNode);
    InterlinkResolutionSuccessFlag = 0;
    return; %Exit the function after declaring failure to resolve the interlinks constraint
    
else %Carry out the rest of the function here
    
    %First assign the unassigned cables to the free manipulators
    if nCablesUnassigned > 0 %This is required only if there are any unassigned cables
        for i = 1:nCablesUnassigned
            
            %Assign the manipulator to the correspondingCable indicated by
            %the unassigned cables array
            selectedCorrespondingCable = CorrespondingCablesStruct(CablesUnassigned(i));
            selectedCorrespondingCable.AssignedManipulator = FreeManipulators(i);
            CorrespondingCablesStruct(CablesUnassigned(i)) = selectedCorrespondingCable;
            
        end
    end
    
    SingleStepAlignmentCableID = [];
    SingleStepAlignmentRefID = [];
    SingleStepAlignmentManipID = [];
    RepositionCableID = CablesOriginallyManipulated;
    
    if nCablesUnassigned > 0 %else single step alignment is not possible
        for i = 1:nCablesUnassigned

            selectedCorrespondingCable = CorrespondingCablesStruct(CablesUnassigned(i));
            CableID = selectedCorrespondingCable.CableIndex;
            ManipID = selectedCorrespondingCable.AssignedManipulator;
            SortedReferencePointList = ArrangeReferencePointsByPreference(GraspNode, selectedCorrespondingCable);
            nReferencePoints = max(size(SortedReferencePointList));
            addFlag = 0;
            for j = 1:nReferencePoints

                RefID = SortedReferencePointList(j);
                [tempNode tempGraspNode ParentNode tempTransitionHandle] = AlignRefPoint(SystemNode, CableID, ManipID, RefID, Tolerance);
                FlagList = IsInterlinkViolated(tempNode, selectedCorrespondingCable.ViolatedInterlink);
                if sum(FlagList == 0) %The alignment resolves all interlinks between the current cable and the clamped cable
                    tempCurrentViolatedInterlinks = ViolatedInterlinks(tempNode);
                    if ~isempty(tempCurrentViolatedInterlinks) %The resolution does not violate any new interlink
                        [tempCorrespondingCables tempCorrespondingCablesStruct] = DetermineCorrespondingCables(tempNode, CurrentViolatedInterlinks, ClampedCable);
                        ntempCorrespondingCables = max(size(tempCorrespondingCables));
                        for k = 1:ntempCorrespondingCables
                            Containflag(k) = ~isContain(CablesOriginallyManipulated,tempCorrespondingCables(k));
                        end
                        if sum(Containflag) == 0 %All the interlinks violated by aligning the new cable have cables that are already being manipulated
                            SingleStepAlignmentCableID = [SingleStepAlignmentCableID CableID];
                            SingleStepAlignmentRefID = [SingleStepAlignmentRefID RefID];
                            SingleStepAlignmentManipID = [SingleStepAlignmentManipID ManipID];
                            addFlag = 1;
                            break;
                            
                        end
                    else
                        SingleStepAlignmentCableID = [SingleStepAlignmentCableID CableID];
                        SingleStepAlignmentRefID = [SingleStepAlignmentRefID RefID];
                        SingleStepAlignmentManipID = [SingleStepAlignmentManipID ManipID];
                        addFlag = 1;
                        break;
                    end
                end

            end

            if addFlag == 0
                RepositionCableID = [RepositionCableID CableID];
            end

        end
    
        if isempty(RepositionCableID) %Only single step resolution has resolved the interlink problem. No need for positioning the cables

            %Perform the single step resolution
            [GraspNode, ParentNode, flag] = GraspMultipleRefPoint(GraspNode, SingleStepAlignmentCableID, SingleStepAlignmentManipID, SingleStepAlignmentRefID, Tolerance);
            %%PlotStateLargeGoal(GraspNode)
            [GraspNode] = TransitionHandle(GraspNode);
            %PlotStateLargeGoal(GraspNode)
            nSingleStepAlignments = max(size(SingleStepAlignmentCableID));
            for i = 1:nSingleStepAlignments

                [GraspNode ParentNode flag] = ClampCableParallel(GraspNode, SingleStepAlignmentManipID(i), SingleStepAlignmentRefID(i));

            end
           %Declare Success
           InterlinkResolutionSuccessFlag = 1; %successful exit point
           OutNode = GraspNode;
           return;
        end
    end
    
    
    % Now look into respositioning for resolving the interlink constraint
    % through repositioning the other cables that cannot conduct a one step
    % resolution
    
    % First transfer the system node to the state which grasps the clamping
    % cable and the single step resolution cables
    
    if ~isempty(SingleStepAlignmentCableID) % There are possible single step alignments
    [GraspNode ParentNode flag] = GraspMultipleRefPoint(GraspNode, SingleStepAlignmentCableID, SingleStepAlignmentManipID, SingleStepAlignmentRefID, Tolerance);
    
    [SystemNode] = TransitionHandle(GraspNode);
    nSingleStepAlignments = max(size(SingleStepAlignmentCableID));
        for i = 1:nSingleStepAlignments
            
            [SystemNode ParentNode flag] = ClampCableParallel(SystemNode, SingleStepAlignmentManipID(i), SingleStepAlignmentRefID(i));
            
        end
    end
    
    
    %At this stage the last Node is the GraspNode which has grasped all the
    %cables for single step alignment. Grasping for reposition must be done
    %parallel to this stage. SystemNode has the clamping cable clamped and
    %all the single stage resolutions completed as well
    
    %Create a new structure of CorrespondingCables that deals only with
    %repositionsing
    
    nRepositionCables = max(size(RepositionCableID));
    RepositionCorrespondingCablesStruct = [];
    for i=1:nCorrespondingCables
        if isContain(RepositionCableID, CorrespondingCablesStruct(i).CableIndex)
            newCorrespondingCablesStruct = CorrespondingCablesStruct(i);
            RepositionCorrespondingCablesStruct = [RepositionCorrespondingCablesStruct newCorrespondingCablesStruct];
        end
    end
    
    %Adding the cables in the originally manipulated list but not in the
    %CorrespondingCablesStruct
    nCablesOriginallyManipulated = max(size(CablesOriginallyManipulated));
    if ~isempty(CablesOriginallyManipulated)
        RepositionCablesIncludedList = CablesIncluded(RepositionCorrespondingCablesStruct);
        for i = 1:nCablesOriginallyManipulated
            selectedOccupiedManipulator = OccupiedManipulators(i);
            selectedOccupiedCable = selectedOccupiedManipulator.CableID;
            if ~isContain(RepositionCablesIncludedList, selectedOccupiedCable)
                newCorrespondingCablesStruct.CableIndex = selectedOccupiedCable;
                newCorrespondingCablesStruct.AssignedManipulator = OccupiedManipulators(i).ManipID;                
                newCorrespondingCablesStruct.ViolatedInterlink = InterlinkViolatedOnCable(SystemNode, selectedOccupiedCable);
                newCorrespondingCablesStruct = DetermineViolatedInterlinkLength(GraspNode, newCorrespondingCablesStruct);
                newCorrespondingCablesStruct.AffectedReferencePoints = [];
                newCorrespondingCablesStruct.GripPointsConsidered = [];
                RepositionCorrespondingCablesStruct = [RespositionCorrespondingCablesStruct newCorrespondingCablesStruct]; 
            end
        end
    end
    
    
    
    %Now the reference points that must be aligned to free up the
    %manipulators must be determined here.
    %This is to be done only if the number of repositioning manipulators is
    %more than the originally manipulated cables
    ReferencePointsToAlignList = [];
    nCablesOriginallyManipulated = max(size(CablesOriginallyManipulated));
    if nCablesOriginallyManipulated < nRepositionCables
    CurrentAlignmentList = [];
    tempFreeManipulators = DetermineFreeManipulators(SystemNode, 0);
    ReferencePointsToAlignList = DetermineReferencePointsToAlign(SystemNode,Tolerance, RepositionCorrespondingCablesStruct, tempFreeManipulators, CurrentAlignmentList);
    end
    
    
    GeometricResolutionExitFlag = 0;
    GeometricResolutionSuccessFlag = 0;
    nRepositionCorrespondingCables = max(size(RepositionCorrespondingCablesStruct));
    for i = 1:nRepositionCorrespondingCables
        SortedGripPointList(i).List = ArrangeGripPointsByPreference(GraspNode, RepositionCorrespondingCablesStruct(i))';
    end
    CableResolveFlag = zeros(nRepositionCorrespondingCables,1);
    
    while GeometricResolutionExitFlag == 0
        tempGraspNode = GraspNode;
        %select from available gripping point for each correspondingcable
        
        %Begin Sampling GripPoints
        for i = 1:nRepositionCorrespondingCables
            if ~isContain(CablesOriginallyManipulated, RepositionCorrespondingCablesStruct(i).CableIndex)
            if ~isempty(SortedGripPointList(i).List)&& CableResolveFlag(i)==0;
                selectedGripPoint(i) = SortedGripPointList(i).List(1);
                SortedGripPointList(i).List = DeleteElement(SortedGripPointList(i).List,1);
                size(SortedGripPointList(i).List)
            end
            end
        end
        % End Sampling Grip Points
        
        %Act to Grasp Cables at selected Grip Points. Begin with GraspNode
        %and add parallel
        for i = 1:nRepositionCorrespondingCables
            if ~isContain(CablesOriginallyManipulated, RepositionCorrespondingCablesStruct(i).CableIndex)
                [tempGraspNode ParentNode flag] = GraspCableParallel(tempGraspNode, RepositionCorrespondingCablesStruct(i).AssignedManipulator, RepositionCorrespondingCablesStruct(i).CableIndex, selectedGripPoint(i));
            end
        end
        [tempSystemNode ParentNode flag] = TransitionHandle(tempGraspNode);
        %PlotStateLargeGoal(tempSystemNode)
        
        %Determine if all the interlinks are within the sphere of influence
        %of the gripped Manipulators
        SphereOfInfluenceFlags = CheckSphereOfInfluence(tempSystemNode, RepositionCorrespondingCablesStruct);
        %Compute the geometric resolution only if all the interlinks are
        %within the sphere of the influence of the respective grips on the
        %corresponding cables
        if sum(SphereOfInfluenceFlags < nRepositionCorrespondingCables)
            CableResolveFlag = zeros(nRepositionCorrespondingCables,0);
        else
            %Compute the repositioning for the manipulators
            [tempSystemNode tempParentNode CableResolveFlag] = ResolveGeometricProblem(tempSystemNode, RepositionCorrespondingCablesStruct)
        end
        
        checkSum = sum(CableResolveFlag);
        
        if checkSum < nRepositionCables
            GeometricResolutionExitFlag = 0; %resample the grip points and start again
        end
        
        if checkSum == nRepositionCables %geometric resolution was successful, now need to align the reference point list and release manipulators
            newOccupiedManipulators = DetermineOccupiedManipulators(tempSystemNode, 0);
            newFreeManipulators = DetermineFreeManipulators(tempSystemNode, 0);
            nestedAlignSuccessFlag = 1;
            if ~isempty(ReferencePointsToAlignList)
                [ tempSystemNode, nestedAlignSuccessFlag ] = AlignReferencePointList( tempSystemNode, Tolerance,  ReferencePointsToAlignList, newFreeManipulators, newOccupiedManipulators );
            end
            if nestedAlignSuccessFlag == 1
                OutNode = tempSystemNode;
                GeometricResolutionExitFlag = 1;
                GeometricResolutionSuccessFlag = 1;
                InterlinkResolutionSuccessFlag = 1;
                %Now need to release the non occupied Manipulators
                for j = 1:nRepositionCorrespondingCables
                    if ~isContain(CablesOriginallyManipulated, RepositionCorrespondingCablesStruct(j).CableIndex)
                        [OutNode ParentNode ReleaseFlag] = ReleaseManipulator(OutNode, RepositionCorrespondingCablesStruct(j).AssignedManipulator);
                    end
                end
                break;                         
            else
                GeometricResolutionExitFlag = 0;
                CableResolveFlag = zeros(nRepositionCorrespondingCables,1);
            end
        end
        
        %Check for grip point list empty
        if checkSum < nRepositionCables
            checksum = 0;
            for i=1:nRepositionCorrespondingCables
                if ~isContain(CablesOriginallyManipulated, RepositionCorrespondingCablesStruct(i).CableIndex)
                    checksum = checksum + isempty(SortedGripPointList(i).List);
                end
            end
            if checksum == nRepositionCorrespondingCables-nCablesOriginallyManipulated
                GeometricResolutionExitFlag = 1;
                GeometricResolutionSuccessFlag = 0;
            end
        end
        % End check for grip point  list empty
    end
    
    if GeometricResolutionSuccessFlag == 0
        InterlinkResolutionSuccessFlag = 0;
        OutNode = GraspNode;
    end
    

end
end

