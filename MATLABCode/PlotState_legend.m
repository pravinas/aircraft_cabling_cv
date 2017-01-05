function [ out ] = PlotState( InNode )
% out = PlotState(Cable, Manipulator)

% This function plots a visualisation of the entire state of the cable
% including the clamping positions, reference points, gripping points,
% manipulator position and cable configurations

% Cable: A structure array containing structures that describe the entire state of each cable. Note
% that the configuration of the cable must be updated at this step

% Manipulator: A structure array containing the information. The gripping
% information on the Manipulator and the Cable structure must match.

clf;

Cable = InNode.State.Cable;
Manipulator = InNode.State.Manipulator;
Interlink = InNode.State.Interlink;
stretchflag = 0;

for i=1:length(Cable)
    
    % Plot the Cable configuration first
    
    figure(1);
    hold on;
    
    hline1 = plot(Cable(i).configuration.state(:,1), Cable(i).configuration.state(:,2), 'LineWidth' , 1.5);
    %Plot Reference Points
    refPointPos = getRefPosition(Cable, i);
    hline7 = plot(refPointPos.state(:,1), refPointPos.state(:,2), 'k*', 'MarkerSize', 11, 'LineWidth', 1.5 );
    
    %Plot the gripping points
    GripPointPos = getGrippingPosition(Cable, i);
    hline2 = plot(GripPointPos.state(:,1) , GripPointPos.state(:,2) , 'r*');
    
    %Plot Clamping Points.
    hline3 = plot(Cable(i).clampPos(:,1), Cable(i).clampPos(:,2), 'ko', 'MarkerSize', 11, 'LineWidth', 1.5);
    
    
end

for i = 1:length(Manipulator)
    
    %Plot the position of each manipulator
    hline4 = plot(Manipulator(i).position(1), Manipulator(i).position(2), 'mx', 'MarkerSize', 11, 'LineWidth', 1.5);
end

% Plot the Interlinks

if ~isempty(Interlink)
for i = 1:length(Interlink)
    
    if Interlink(i).flag == 0 % Link not stretched
        pos1 = GetPosition(Cable, Interlink(i).cable1, Interlink(i).length1);
        pos2 = GetPosition(Cable, Interlink(i).cable2, Interlink(i).length2);
        
        hline5 = plot([pos1(1) pos2(1)], [pos1(2) pos2(2)] , 'go-', 'LineWidth' , 1.5);
    else
        pos1 = GetPosition(Cable, Interlink(i).cable1, Interlink(i).length1);
        pos2 = GetPosition(Cable, Interlink(i).cable2, Interlink(i).length2);
        
        hline6 = plot([pos1(1) pos2(1)], [pos1(2) pos2(2)] , 'ro-', 'LineWidth' , 1.5);
        stretchflag = 1;
    end
end
end
if isempty(Interlink)
if stretchflag == 0
    %legend([hline1 hline3 hline7 hline2 hline4 hline5 hline6], 'Cable', 'Clamp Positions', 'Reference Points', 'Grip points', 'Manipulators', 'Interlink Unstretched', 'Interlink Stretched');
    legend([hline4], 'Manipulators');
end
end

axis equal; 
hold off;

end

