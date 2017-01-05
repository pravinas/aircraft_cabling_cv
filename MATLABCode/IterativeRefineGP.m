function [ FinalGP minDistNorm ] = IterativeRefineGP( InitialGP, CablePointCloud, minDistThresh, SystemNode )

GP = InitialGP;
% % Plots for debugging
% lpred = linspace(0,SystemNode.State.Cable(1).length,300);
% yPred = predict(GP.Ymdl,lpred');
% xPred = predict(GP.Xmdl,lpred');
% 
% PlotStateLargeGoal2(SystemNode)
% legend off
% hold on
% h1 = plot(xPred,yPred,'x-');
% h2 = plot(GP.observations(:,1),GP.observations(:,2),'ro','MarkerSize',12,'LineWidth',1.5);

%% Initiate Refinement
GP = InitialGP;
[ newGP minDist minDistNorm ] = RefineGP( GP, CablePointCloud, minDistThresh );
oldGP = GP;
GP = newGP;

%Plots for debugging
lpred = linspace(0,SystemNode.State.Cable(1).length,300);
yPred = predict(GP.Ymdl,lpred');
xPred = predict(GP.Xmdl,lpred');

PlotStateLargeGoal2(SystemNode)
legend off
hold on
h1 = plot(xPred,yPred,'x-');
h2 = plot(GP.observations(:,1),GP.observations(:,2),'ro','MarkerSize',12,'LineWidth',1.5);

while minDistNorm >= minDistThresh
    [ newGP minDist minDistNorm ] = RefineGP( GP, CablePointCloud, minDistThresh );
    oldGP = GP;
    GP = newGP;
%     
%     %Plots for debugging
%     lpred = linspace(0,SystemNode.State.Cable(1).length,300);
%     yPred = predict(GP.Ymdl,lpred');
%     xPred = predict(GP.Xmdl,lpred');
%     
%     PlotStateLargeGoal2(SystemNode)
%     legend off
%     hold on
%     h1 = plot(xPred,yPred,'x-');
%     h2 = plot(GP.observations(:,1),GP.observations(:,2),'ro','MarkerSize',12,'LineWidth',1.5);
end

FinalGP = GP;


end

