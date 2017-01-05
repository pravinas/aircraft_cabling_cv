function [ out ] = PlotStateLargeGoal2( InNode )
% out = PlotState(Cable, Manipulator)

% This function plots a visualisation of the entire state of the cable
% including the clamping positions, reference points, gripping points,
% manipulator position and cable configurations

% Cable: A structure array containing structures that describe the entire state of each cable. Note
% that the configuration of the cable must be updated at this step

% Manipulator: A structure array containing the information. The gripping
% information on the Manipulator and the Cable structure must match.

%figure;
clf;
% imshow('fuselagem.jpg');
% axes('Position' , [0.1 0.1 0.8 0.8] , 'color' , 'black');
% I = imread('fuselagem.jpg');
% imshow(I, 'InitialMagnification' , 70)

Cable = InNode.State.Cable;
Manipulator = InNode.State.Manipulator;
Interlink = InNode.State.Interlink;
%axes('Position' , [0.1 0.1 0.8 0.8], 'Color' , 'none');
for i=1:length(Cable)
    
    % Plot the Cable configuration first
    
    figure(1);
    hold on;
    
    lin1 = plot(Cable(i).configuration.state(:,1), Cable(i).configuration.state(:,2), 'b', 'LineWidth' , 3  );
    %Plot Reference Points
    refPointPos = getRefPosition(Cable, i);
    lin2 = plot(refPointPos.state(:,1), refPointPos.state(:,2), 'k*', 'MarkerSize', 20, 'LineWidth', 3 );
    
%     Plot the gripping points
%     GripPointPos = getGrippingPosition(Cable, i);
%     plot(GripPointPos.state(:,1) , GripPointPos.state(:,2) , 'r*' , 'MarkerSize' , 15, 'LineWidth' , 2);
%     
    %Plot Clamping Points.
    lin3 = plot(Cable(i).clampPos(:,1), Cable(i).clampPos(:,2), 'ko', 'MarkerSize', 20, 'LineWidth', 3);
    
%     %Plot the alignment tolerance
%     tolerance = 0.15;
%     ClampPos = Cable(i).clampPos(:,1:2);
%     for i = 1:size(ClampPos,1)
%         theta = linspace(0,2*pi,100);
%         r = ones(1,length(theta));
%         [X,Y] = pol2cart(theta, tolerance*r);
%         X = X + ClampPos(i,1);
%         Y = Y + ClampPos(i,2);
%         lin5 = plot(X,Y,'m' , 'LineWidth' , 3);
%     end
    
    
end

for i = 1:length(Manipulator)
    
    %Plot the position of each manipulator
    lin6 = plot(Manipulator(i).position(1), Manipulator(i).position(2), 'rx', 'MarkerSize', 30, 'LineWidth', 3);
end

% Plot the Interlinks

if ~isempty(Interlink)
for i = 1:length(Interlink)
    
    if Interlink(i).flag == 0 % Link not stretched
        pos1 = GetPosition(Cable, Interlink(i).cable1, Interlink(i).length1);
        pos2 = GetPosition(Cable, Interlink(i).cable2, Interlink(i).length2);
        
        lin4 = plot([pos1(1) pos2(1)], [pos1(2) pos2(2)] , 'go-', 'LineWidth' , 2.5);
    else
        pos1 = GetPosition(Cable, Interlink(i).cable1, Interlink(i).length1);
        pos2 = GetPosition(Cable, Interlink(i).cable2, Interlink(i).length2);
        
        plot([pos1(1) pos2(1)], [pos1(2) pos2(2)] , 'ro-', 'LineWidth' , 2.5);
    end
end
end
axis equal; 
legend([lin1 lin2 lin3 lin6] , 'Cables' , 'Reference Points' , 'Clamp Positions'  , 'Manipulator')
set(gca, 'XTickLabel' , '')
set(gca, 'YTickLabel' , '')
set(gca , 'FontSize' , 18)
title('System State')
hold off;
grid on
box on

end

