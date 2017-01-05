function [ selectedCorrespondingCable ] = DetermineViolatedInterlinkLength( InNode, selectedCorrespondingCable )

ViolatedInterlink = selectedCorrespondingCable.ViolatedInterlink;
CableID = selectedCorrespondingCable.CableIndex;
nViolatedInterlinks = max(size(ViolatedInterlink));
ViolatedInterlinkLength = [];

for i = 1:nViolatedInterlinks
    
    cable1 = InNode.State.Interlink(ViolatedInterlink(i).cable1;
    cable2 = InNode.State.Interlink(ViolatedInterlink(i).cable2;
    length1 = InNode.State.Interlink(ViolatedInterlink(i).length1;
    length2 = InNode.State.Interlink(ViolatedInterlink(i).length2;
    
    if cable1 == CableID;
        ViolatedInterlinkLength = [ViolatedInterlinkLength length1];
    end
    
    if cable2 == CableID
        ViolatedInterlinkLength = [ViolatedInterlinkLength length2];
    end
end



end

