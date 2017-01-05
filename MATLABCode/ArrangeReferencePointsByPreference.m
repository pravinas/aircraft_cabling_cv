function [ SortedReferencePointList ] = ArrangeReferencePointsByPreference( SystemNode, selectedCorrespondingCable )
%

AffectedReferencePoints = selectedCorrespondingCable.AffectedReferencePoints;
nAffectedReferencePoints = max(size(selectedCorrespondingCable.AffectedReferencePoints));
nViolatedInterlinks = max(size(selectedCorrespondingCable.ViolatedInterlink));
Interlink = SystemNode.State.Interlink;
Cable = SystemNode.State.Cable;
CableID = selectedCorrespondingCable.CableIndex;

for i = 1:nAffectedReferencePoints
    
    RefLength(i) = Cable(CableID).refPointPos(AffectedReferencePoints(i));
    sum = 0;
    
    for j = 1:nViolatedInterlinks
        
        linklength = selectedCorrespondingCable.ViolatedInterlinkLength(j);
        diff = abs(RefLength(i) - linklength);
        sum = sum+diff;
        
    end
    
    RefPriority(i) = sum;
    
end

[sortedPriorityVable indices] = sort(RefPriority, 'ascend');

SortedReferencePointList = AffectedReferencePoints(indices);

end

