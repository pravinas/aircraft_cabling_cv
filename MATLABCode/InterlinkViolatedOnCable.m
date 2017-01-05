function [ ViolatedInterlinkList ] = InterlinkViolatedOnCable( InNode, CableID )
%

[InterlinkByCableStructure] = ClassifyInterlinkByCables(InNode);
ViolatedInterlinkList = [];
flagList = IsInterlinkViolated(InNode, InterlinkByCableStructure(CableID).InterlinkID);

nLinks = max(size(flagList));

if nLinks>0
    for i = 1:nLinks
        if flagList(i) == 1
            ViolatedInterlinkList = [ViolatedInterlinkList InterlinkByCableStructure(CableID).InterlinkID(i)];
        end
    end
end


end

