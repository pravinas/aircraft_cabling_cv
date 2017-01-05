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

for i=1:length(Cable)
    
    % Plot the Cable configuration first
    
    figure(1);
    hold on;
    
    hl1 = plot(Cable(i).configuration.state(:,1), Cable(i).configuration.state(:,2), 'b' ,'LineWidth' , 3);
    %Plot Reference Points
    refPointPos = getRefPosition(Cable, i);
    hl2 = plot(refPointPos.state(:,1), refPointPos.state(:,2), 'k*', 'MarkerSize', 20, 'LineWidth', 3 );
    
    %Plot the gripping points
    GripPointPos = getGrippingPosition(Cable, i);
    hl3 = plot(GripPointPos.state(:,1) , GripPointPos.state(:,2) , 'r*' , 'MarkerSize' , 10, 'LineWidth' , 2);
    
    %Plot Clamping Points.
    hl4 = plot(Cable(i).clampPos(:,1), Cable(i).clampPos(:,2), 'ko', 'MarkerSize', 20, 'LineWidth', 3);
    
    
end

for i = 1:length(Manipulator)
    
    %Plot the position of each manipulator
    hl5 = plot(Manipulator(i).position(1), Manipulator(i).position(2), 'mx', 'MarkerSize', 30, 'LineWidth', 3)
end

% Plot the Interlinks

if ~isempty(Interlink)
for i = 1:length(Interlink)
    
    if Interlink(i).flag == 0 % Link not stretched
        pos1 = GetPosition(Cable, Interlink(i).cable1, Interlink(i).length1);
        pos2 = GetPosition(Cable, Interlink(i).cable2, Interlink(i).length2);
        
        hl6 = plot([pos1(1) pos2(1)], [pos1(2) pos2(2)] , 'go-', 'LineWidth' , 2.5);
    else
        pos1 = GetPosition(Cable, Interlink(i).cable1, Interlink(i).length1);
        pos2 = GetPosition(Cable, Interlink(i).cable2, Interlink(i).length2);
        
        hl7 = plot([pos1(1) pos2(1)], [pos1(2) pos2(2)] , 'ro-', 'LineWidth' , 2.5);
    end
end
end
%legend([hl1 hl4 hl2 hl3 hl5 hl6 hl7] , 'Cable' , 'Clamp Locations' , 'Reference Points' , 'Gripping Points' , 'Manipulator' , 'Interlink not Stretched' , 'Interlink Stretched')
axis equal; 
set(gca, 'XTickLabel' , '')
set(gca, 'YTickLabel' , '')
set(gca,'FontSize',14)
hold off;
grid on
box on

end

