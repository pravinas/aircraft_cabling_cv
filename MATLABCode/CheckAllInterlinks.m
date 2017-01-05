function [ checksum ] = CheckAllInterlinks( InNode )
%

nInterlinks = max(size(InNode.State.Interlink));
checksum = 0;

for i = 1:nInterlinks;
    checksum = checksum + InNode.State.Interlink(i).flag;
end


end

