function SphereOfInfluenceFlags = CheckSphereOfInfluence(tempSystemNode, RepositionCorrespondingCablesStruct)

nRepositionCorrespondingCables = max(size(RepositionCorrespondingCablesStruct));
SphereOfInfluenceFlags = zeros(nRepositionCorrespondingCables,1);


for i = 1:nRepositionCorrespondingCables
    selectedCorrespondingCable = RepositionCorrespondingCablesStruct(i);
    ViolatedInterlinkLength = selectedCorrespondingCable.ViolatedInterlinkLength;
    CableID = selectedCorrespondingCable.CableIndex;
    ManipID = selectedCorrespondingCable.AssignedManipulator;
    GripID = tempSystemNode.State.Manipulator(ManipID).grip;
    GripLength = tempSystemNode.State.Cable(CableID).gripPointsPos(GripID);
    %Determine the leftmost length of sphere of influence
    
    %Determine the leftmost clamping point
    clamped = tempSystemNode.State.Cable(CableID).clamped;
    [values, indices] = sort(clamped(:,1));
    clamped = clamped(indices,:);
    LeftmostClampedLength = 0;
    if ~isempty(clamped)
        nClamped = size(clamped,1);
        for j = 1:nClamped
            if clamped(j,1) < GripLength
                LeftmostClampedLength = clamped(j,1);
            end
            if clamped(j,1) >= GripLength
                break;
            end
        end
    else
        LeftmostClampedLength = 0;
    end
    
    %Determine the Leftmost grippingPoint
    LeftmostGripLength = 0;
    gripped = tempSystemNode.State.Cable(CableID).gripped;
    [values, indices] = sort(gripped(:,1));
    gripped = gripped(indices,:);
    if ~isempty(gripped)
        nGripped = size(gripped,1);
        for j = 1:nGripped
            if gripped(j,2) ~= ManipID
                grippedLength = tempSystemNode.State.Cable(CableID).gripPointsPos(gripped(j,1));
                if grippedLength < GripLength
                    LeftmostGripLength = grippedLength;
                end
                if grippedLength >= GripLength
                    LeftmostGripLength = 0;
                end
            end
        end
    else
        LeftmostGripLength = 0;
    end
    
    LeftmostInfluenceLength = max([LeftmostGripLength, LeftmostClampedLength]);

    
    %Determine the rightmost length of sphere of influence
    
    %Determine the rightmost clamping point
    clamped = tempSystemNode.State.Cable(CableID).clamped;
    [values, indices] = sort(clamped(:,1));
    clamped = clamped(indices,:);
    RightmostClampedLength = tempSystemNode.State.Cable(CableID).length;
    if ~isempty(clamped)
        nClamped = size(clamped,1);
        for j = nClamped:-1:1
            if clamped(j,1) > GripLength
                RightmostClampedLength = clamped(j,1);
            end
            if clamped(j,1) <= GripLength
                break;
            end
        end
    else
        RightmostClampedLength = tempSystemNode.State.Cable(CableID).length;
    end
    
    %%Determine the Rightmost grippingPoint
    RightmostGripLength = tempSystemNode.State.Cable(CableID).length;
    gripped = tempSystemNode.State.Cable(CableID).gripped;
    if ~isempty(gripped)
        nGripped = size(gripped,1);
        for j = nGripped:-1:1
            if gripped(j,2) ~= ManipID
                grippedLength = tempSystemNode.State.Cable(CableID).gripPointsPos(gripped(j,1));
                if grippedLength > GripLength
                    RightmostGripLength = grippedLength;
                end
                if grippedLength <= GripLength
                    RightmostGripLength = tempSystemNode.State.Cable(CableID).gripped;
                end
            end
        end
    else
        RightmostGripLength = tempSystemNode.State.Cable(CableID).gripped;
    end
    
    RightmostInfluenceLength = max([RightmostGripLength, RightmostClampedLength]);
    


    %Check if all the interlinks are within the sphere of influence
    nViolatedInterlinks = max(size(selectedCorrespondingCable.ViolatedInterlinkLength));
    LinkFlags = zeros(nViolatedInterlinks,1) + 1;
    for j = 1:nViolatedInterlinks
        if ViolatedInterlinkLength(j)<LeftmostInfluenceLength || ViolatedInterlinkLength(j) > RightmostInfluenceLength
            LinkFlags(j) = 0;
        end
    end
    if sum(LinkFlags)<nViolatedInterlinks
        SphereOfInfluenceFlags(i) = 0;
    else 
        SphereOfInfluenceFlags(i) = 1;
    end
end
end

