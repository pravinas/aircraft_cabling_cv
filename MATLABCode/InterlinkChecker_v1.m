function [ OutNode ] = InterlinkChecker( InNode )

% [Cable, Manipulator, Interlink ] = InterlinkChecker( Cable, Manipulator, Interlink )
% This function evaluates the integrity of the interlink constraints and
% sets the flags that represent whether the particular interlink is
% stretched or not

% Cable: This is an array of the structures of the type Cable
% Manipulator: This is an array of structures of type Manipulator
% Interlink: This is an array of structures of the type Interlinks

% Before calling this function, it must be ensured that the shape of the
% cable has been computed.

% This fork modifies the checker to ignore the stretches that occur on the
% ground as the behaviour of the cables lying on the ground is not
% accurately modeled

Cable = InNode.State.Cable;
Manipulator = InNode.State.Manipulator;
Interlink = InNode.State.Interlink;

nLinks = size(Interlink, 1);

if nLinks > 0 % The problem contains interlink constraints
    for i = 1:nLinks
        pos1 = GetPosition(Cable, Interlink(i).cable1, Interlink(i).length1);
        pos2 = GetPosition(Cable, Interlink(i).cable2, Interlink(i).length2);
        pos1 = pos1(1:2);
        pos2 = pos2(1:2);
        
               
        if norm(pos1 - pos2) >= Interlink(i).linkLength %Link has been stretched
            Interlink(i).flag = 1;
        else
            Interlink(i).flag = 0;
        end
        
    end        
end

OutNode.StartState = InNode.StartState;
OutNode.ActionList = InNode.ActionList;

OutNode.State.Cable = Cable;
OutNode.State.Manipulator = Manipulator;
OutNode.State.Interlink = Interlink;


end

