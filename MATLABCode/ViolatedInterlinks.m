function [ ViolatedLinkList ] = ViolatedInterlinks( SystemNode )
    %[ ViolatedLinkList ] = ViolatedInterlinks( SystemNode )
    
    % This function creates an array of indices of the links that have been currently
    % violated. It is assumed that the interlink checker has already been
    % run and the state has been updated.
    
    %SystemNode: The search node that has been created by an action
    % ViolatedLinkList: A column array that lists the index of the
    % interlinks that are being violated
    

Interlink = SystemNode.State.Interlink;
nLinks = size(Interlink,1);

ViolatedLinks = [];

for i = 1:nLinks
    
    if Interlink(i).flag == 1
        ViolatedLinks = [ViolatedLinks; i];
    end
end

ViolatedLinkList = ViolatedLinks;
end

