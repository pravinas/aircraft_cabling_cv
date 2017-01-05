function [ CorrespondingCables CorrespondingCablesStruct ] = DetermineCorrespondingCables( SystemNode, CurrentViolatedInterlinks, ClampedCable )
%


nViolatedLinks = max(size(CurrentViolatedInterlinks));

CorrespondingCables = [];
CorrespondingCablesStruct = [];

for i = 1:nViolatedLinks
    
    CurrentInterlink = CurrentViolatedInterlinks(i);
    cable1 = SystemNode.State.Interlink(CurrentInterlink).cable1;
    cable2 = SystemNode.State.Interlink(CurrentInterlink).cable2;
    length1 = SystemNode.State.Interlink(CurrentInterlink).length1;
    length2 = SystemNode.State.Interlink(CurrentInterlink).length2;
    
    if cable1 == ClampedCable
        
        [containFlag index] = isContain(CorrespondingCables , cable2);
        if containFlag == 0;            
            CorrespondingCables = [CorrespondingCables; cable2];
            newCorrespondingCablesStruct.CableIndex = cable2;
            newCorrespondingCablesStruct.ViolatedInterlink = CurrentInterlink;
            newCorrespondingCablesStruct.ViolatedInterlinkLength = length2;
            newCorrespondingCablesStruct.AffectedReferencePoints = [];
            CorrespondingCablesStruct = [CorrespondingCablesStruct; newCorrespondingCablesStruct];
        else
            
            selectedCorrespondingCable = CorrespondingCablesStruct(index);
            selectedCorrespondingCable.ViolatedInterlink = [selectedCorrespondingCable.ViolatedInterlink; CurrentInterlink];
            selectedCorrespondingCable.ViolatedInterlinkLength = [selectedCorrespondingCable.ViolatedInterlinkLength; length2];
            CorrespondingCablesStruct(index) = selectedCorrespondingCable;
            
        end        
    end
    
    if cable2 == ClampedCable
        
        [containFlag index] = isContain(CorrespondingCables, cable1);
        if containFlag == 0
            CorrespondingCables = [CorrespondingCables, cable1];
            newCorrespondingCablesStruct.CableIndex = cable1;
            newCorrespondingCablesStruct.ViolatedInterlink = CurrentInterlink;
            newCorrespondingCablesStruct.ViolatedInterlinkLength = length1;
            newCorrespondingCablesStruct.AffectedReferencePoints = [];
            CorrespondingCablesStruct = [CorrespondingCablesStruct; newCorrespondingCablesStruct];
        else
            selectedCorrespondingCable = CorrespondingCablesStruct(index);
            selectedCorrespondingCable.ViolatedInterlink = [selectedCorrespondingCable.ViolatedInterlink; CurrentInterlink];
            selectedCorrespondingCable.ViolatedInterlinkLength = [selectedCorrespondingCable.ViolatedInterlinkLength; length1];
            CorrespondingCablesStruct(index) = selectedCorrespondingCable;
        end
    end    
end

nCorrespondingCables = max(size(CorrespondingCablesStruct));
for i = 1:nCorrespondingCables
    
   [sortedLengths index] = sort(CorrespondingCablesStruct(i).ViolatedInterlinkLength);
   CorrespondingCablesStruct(i).ViolatedInterlinkLength = sortedLengths;
   CorrespondingCablesStruct(i).ViolatedInterlink = CorrespondingCablesStruct(i).ViolatedInterlink(index);
    
end


end

