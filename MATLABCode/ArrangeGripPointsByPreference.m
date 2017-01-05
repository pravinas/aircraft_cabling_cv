function [ SortedGripPointList ] = ArrangeGripPointsByPreference( SystemNode, selectedCorrespondingCable )
%

GripPointsConsidered = selectedCorrespondingCable.GripPointsConsidered;
nGripPointsConsidered = max(size(selectedCorrespondingCable.GripPointsConsidered));
nViolatedInterlinks = max(size(selectedCorrespondingCable.ViolatedInterlink));
Interlink = SystemNode.State.Interlink;
Cable = SystemNode.State.Cable;
CableID = selectedCorrespondingCable.CableIndex;

for i = 1:nGripPointsConsidered
    
    RefLength(i) = Cable(CableID).gripPointsPos(GripPointsConsidered(i));
    sum = 0;
    
    for j = 1:nViolatedInterlinks
        
        linklength = selectedCorrespondingCable.ViolatedInterlinkLength(j);
        diff = abs(RefLength(i) - linklength);
        sum = sum+diff;
        
    end
    
    RefPriority(i) = sum;
    
end

[sortedPriorityVable indices] = sort(RefPriority, 'ascend');

SortedGripPointList = GripPointsConsidered(indices);

end

