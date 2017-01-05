function [ InterlinkByCableStructure ] = ClassifyInterlinkByCables( SystemNode )
%

Interlink = SystemNode.State.Interlink;
nInterlinks = max(size(Interlink));

InterlinkByCableElement.cableID = 0;
InterlinkByCableElement.InterlinkID = [];
InterlinkByCableElement.InterlinkLength = [];

nCables = max(size(SystemNode.State.Cable));

InterlinkByCableStructure(nCables,1) = InterlinkByCableElement;

for i = 1:nCables
    InterlinkByCableStructure(i).cableID = i;
end

for i = 1:nInterlinks
    
    cable1 = Interlink(i).cable1;
    cable2 = Interlink(i).cable2;
    length1 = Interlink(i).length1;
    length2 = Interlink(i).length2;
    
    InterlinkByCableStructure(cable1).InterlinkID = [InterlinkByCableStructure(cable1).InterlinkID i];
    InterlinkByCableStructure(cable1).InterlinkLength =[InterlinkByCableStructure(cable1).InterlinkLength length1];
    
    InterlinkByCableStructure(cable2).InterlinkID = [InterlinkByCableStructure(cable2).InterlinkID i];
    InterlinkByCableStructure(cable2).InterlinkLength = [InterlinkByCableStructure(cable2).InterlinkLength length2];
    
end

for i = 1:nCables
    
    if ~isempty(InterlinkByCableStructure(i).InterlinkLength)
        
        [sortedLength index] = sort(InterlinkByCableStructure(i).InterlinkLength);
        InterlinkByCableStructure(i).InterlinkLength = sortedLength;
        InterlinkByCableStructure(i).InterlinkID = InterlinkByCableStructure(i).InterlinkID(index);
        
    end

end

