function [ checksum ] = CheckCorrespondingCableInterlinks( InNode, CorrespondingCablesStruct )
% Returns a non zero value if any of the listed interlink is violated

checksum = 0;
nCorrespondingCables = max(size(CorrespondingCablesStruct));

for i = 1:nCorrespondingCables
    [FlagList] = IsInterlinkViolated(InNode, CorrespondingCablesStruct(i).ViolatedInterlink);
    checksum = checksum+sum(FlagList);
end



end

