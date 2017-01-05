function [ ViolatedFlagList ] = IsInterlinkViolated( SystemNode, QueryInterlinkList )
% QueryInterlinkList is the list of the indices of the interlinks whose
% integrity needs to be checked

nQueries = max(size(QueryInterlinkList));

for i = 1:nQueries
    
    ViolatedFlagList(i) = SystemNode.State.Interlink(QueryInterlinkList(i)).flag;
    
end


end

