function [ CablesIncludedList ] = CablesIncluded( CorrespondingCablesStruct )


nCorrespondingCables = max(size(CorrespondingCablesStruct));
CablesIncludedList = zeros(nCorrespondingCables, 1);
for i = 1:nCorrespondingCables;
    CablesIncludedList(i) = CorrespondingCablesStruct(i).CableIndex;
end


end

