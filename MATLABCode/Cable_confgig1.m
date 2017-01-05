clear all;

% Cable 1 Definition
Cable.Ground = 0;
Cable.length = 2.2;
Cable.clampPos = [0 0.3 0;
                0.4 0.3 0;
                0.8 0.4 0;
                1.2 0.4 0];
Cable.refPointPos = [0.2;0.8;1.4;2.0];
Cable.gripPointsPos = [0:0.05: Cable.length]';
Cable.clamped = []; 
Cable.gripped = [];
Cable.configuration.length = [0 Cable.length]';
Cable.configuration.state = [-0.3 0 0;
                            1.5 0 0];
Cable.stiffness = 0.1;
Cable.markers = linspace(0,Cable.length,15);


% % Cable(2) definition
% Cable(2).Ground = -0.05;
% Cable(2).length = 2.2;
% Cable(2).clampPos = [0 0.4 0;
%                     0.5 0.4 0;
%                     1.0 0.4 0;
%                     1.5 0.4 0];
% Cable(2).refPointPos = [0.2 0.8 1.4 2.0]';
% Cable(2).gripPointsPos = [0:0.05:Cable(2).length]';
% Cable(2).clamped = [];
% Cable(2).gripped = [];
% Cable(2).configuration.length = [0 Cable(2).length]';
% Cable(2).configuration.state = [-0.3 0 0;
%                             Cable(2).length 0 0];
% Cable(2).stiffness = 0.1;

%Define Manipulators

Manipulator(1).position = [0.9 0.2 0];
Manipulator(1).cable = [];
Manipulator(1).grip = [];

% Manipulator(2).position = [0.1 0.3 0];
% Manipulator(2).cable = [];
% Manipulator(2).grip = [];

% % Define the interlinking constraints
% Interlink(1).cable1 = 1;
% Interlink(1).cable2 = 2;
% Interlink(1).length1 = 1.1;
% Interlink(1).length2 = 1.0;
% Interlink(1).linkLength = 0.3;
% Interlink(1).flag = 0;
% 
% Interlink(2).cable1 = 1;
% Interlink(2).cable2 = 2;
% Interlink(2).length1 = 1.6;
% Interlink(2).length2 = 1.5;
% Interlink(2).linkLength = 0.5;
% Interlink(2).flag = 0;
% 

%Compute the shape of all cables and plot the state
for i = 1:length(Cable)
    selectedCable = Cable(i);
    selectedCable = ComputeCableShape(selectedCable, Manipulator);
    Cable(i) = selectedCable;
end

% Define the StartState and the Action list
StartState.Cable = Cable;
StartState.Manipulator = Manipulator;
StartState.Interlink = [];
ActionList{1,1} = 0;

% Create the StartNode
StartNode.StartState = StartState;
StartNode.ActionList = ActionList;
StartNode.State = StartState;



SystemNode = StartNode;

[SystemNode] = InterlinkChecker(SystemNode);

[SystemNode ParentNode flag] = AlignRefPoint(SystemNode, 1, 1,1,0.05);
[SystemNode ParentNode flag] = AlignRefPoint(SystemNode, 1, 1,2,0.05);
[SystemNode ParentNode flag] = AlignRefPoint(SystemNode, 1, 1,3,0.05);
[SystemNode ParentNode flag] = AlignRefPoint(SystemNode, 1, 1,4,0.05);
PlotStateLarge(SystemNode)

% fig = gcf;
% a = get(fig,'Children')
% lines = get(a,'Children')
% 
% refHandle = lines(3);
% gripHandle = lines(4);
% manipHandle = lines(1);
% 
% set(refHandle,'visible','off');
% set(gripHandle,'visible','off');
% set(manipHandle,'visible', 'off');
% xlim([-0.2 1.5]);
% ylim([0.05 0.5]);
% % Initialte the input structure
% 
% 
% [SystemNode ParentNode flag] = RepositionManipulator(SystemNode, 1, [1.6 0.2 0]);
% PlotState_cable(SystemNode)
% title('Cables, Reference Points and Clamping Positions' , 'FontSize' , 12)
% grid on
% 
% %pause(3);
% [SystemNode, ParentNode, flag] = GraspCable(SystemNode, 1, 2, 5);
% PlotState(SystemNode)
% %pause(3);
% [SystemNode, ParentNode, flag] = RepositionManipulator(SystemNode, 1, [0.2 0.4 0]);
% PlotState(SystemNode)
% %pause(3)
% [SystemNode, ParentNode, flag] = ClampCable(SystemNode, 1, 1);
% [SystemNode, ParentNode, flag] = GraspCable(SystemNode, 1, 2, 17);
% PlotState(SystemNode)
% [SystemNode, ParentNode, flag] = ClampCable(SystemNode, 1, 2);
% PlotState_cable(SystemNode)
% title('Cables and Clamping Positions' , 'FontSize' , 12)
% grid on
% PlotState(SystemNode)
% [SystemNode, ParentNode, flag] = GraspCable(SystemNode, 2, 1, 17);
% target = SystemNode.State.Manipulator(2).position + [-0.4 0.3 0];
% [SystemNode, ParentNode, flag] = RepositionManipulator(SystemNode, 2, target);
% PlotState(SystemNode)
% [SystemNode, ParentNode, flag] = UnclampCable(SystemNode, 2, 1);
% PlotState(SystemNode)
% [SystemNode, ParentNode, flag] = ClampCable(SystemNode, 2, 2);
% PlotState(SystemNode)
% [SystemNode, ParentNode, flag] = GraspCable(SystemNode, 2, 2, 5);
% PlotState(SystemNode);
% [SystemNode, ParentNode, flag] = ClampCable(SystemNode, 2, 1);
% PlotState(SystemNode)
% [SystemNode, ParentNode, flag] = GraspCable(SystemNode, 1, 1, 5);
% PlotState_legend(SystemNode);
% % [SystemNode, ParentNode, flag] = ClampCable(SystemNode, 1, 1);
% % PlotState(SystemNode)
% % [SystemNode, ParentNode, flag] = GraspCable(SystemNode, 2, 1, 29);
% % PlotState(SystemNode)
% % [SystemNode, ParentNode, flag] = GraspCable(SystemNode, 1, 2, 29);
% % PlotState(SystemNode)
% % [SystemNode, ParentNode, flag] = ClampCable(SystemNode, 1, 3);
% % [SystemNode, ParentNode, flag] = ClampCable(SystemNode, 2, 3);
% % PlotState(SystemNode)
% % [SystemNode, ParentNode, flag] = GraspCable(SystemNode, 1, 1, 41);
% % [SystemNode, ParentNode, flag] = GraspCable(SystemNode, 2, 2, 41);
% % PlotState(SystemNode)
% % tic
% % [SystemNode, ParentNode, flag] = ClampCable(SystemNode, 1, 4);
% % tstop = toc;
% % [SystemNode, ParentNode, flag] = ClampCable(SystemNode, 2, 4);
% % PlotState(SystemNode)
% 
% % [SystemNode parentNode flag] = ReleaseManipulator(SystemNode, 2)
% % PlotState(SystemNode)
% save SystemNode.mat SystemNode;
% % [SystemNode ParentNode flag] = ReleaseManipulator(SystemNode, 2);
% % PlotState(SystemNode)
% % [SystemNode ParentNode flag] = ClampCable(SystemNode, 1, 1);
% % target = SystemNode.State.Manipulator(1).position - [0 0.1 0];
% % [SystemNode ParentNode flag] = RepositionManipulator(SystemNode, 1, target);
% % PlotState(SystemNode);
% % [SystemNode ParentNode flag] = UnclampCable(SystemNode, 1, 6)
% % PlotState(SystemNode);
% % [SystemNode ParentNode flag] = RepositionManipulator(SystemNode, 1, [0.2 0.3 0]); 
% % PlotState(SystemNode);
% % [SystemNode ParentNode flag] = ReleaseManipulator(SystemNode, 1);
save Sample_CableState.mat SystemNode
