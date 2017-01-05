load Sample_CableState.mat
clf
%% Define Marker Positions
MarkerPos = getMarkerPos(SystemNode,1);
length = SystemNode.State.Cable(1).markers';
nMarkers = size(MarkerPos,1);
noise = randn(nMarkers,2)*5e-3;
%MarkerPos = MarkerPos+noise;

%% Generate PointCloud

CablePointCloudDensity = 5;
PointsPerMarker = 20;
[ CablePointCloud, MarkerPointCloud ] = GeneratePointCloud( SystemNode, CablePointCloudDensity, PointsPerMarker);

% Plot the point cloud
PlotStateLargeGoal2(SystemNode);
legend off
hold on
f11 = plot(CablePointCloud(:,1), CablePointCloud(:,2),'.');
f12 = plot(MarkerPointCloud(:,1),MarkerPointCloud(:,2),'.');
xlim([-0.2 1.4])
ylim([-0.1 0.5]);
legend([f11 f12],{'Cable point cloud','Marker point cloud'},'Location','SouthEast');

Cablelines = findobj('Type','line','color','b');
set(Cablelines,'visible','off')
line = findobj('Type','line','DisplayName','Cable point cloud');
set(line,'visible','off');


%% Obtain Marker position estimates

[idx, C] = kmeans(MarkerPointCloud, nMarkers);
[sortedC, I] = sort(C(:,1),'ascend');
estMarkerPos = C(I,:);

% Markerplot for debugging
PlotStateLargeGoal2(SystemNode);
hold on
plot(estMarkerPos(:,1),estMarkerPos(:,2),'ro');

%% Generate a GP model

% A GP model can be generated from the estimated marker positions and the
% known length parameters for the markers

GP.length = length;
GP.observations = estMarkerPos;

Xtbl = table(GP.length, GP.observations(:,1), 'VariableNames',{'l', 'X'});
Ytbl = table(GP.length, GP.observations(:,2), 'VariableNames',{'l', 'Y'});
GP.Xmdl = fitrgp(Xtbl,'X' , 'KernelFunction','matern32' , 'KernelParameters',[0.05; 0.09]);
GP.Ymdl = fitrgp(Ytbl,'Y' , 'KernelFunction','matern32' , 'KernelParameters',[0.05; 0.04]);

%Test GP fit
lpred = SystemNode.State.Cable.configuration.length;
yPred = predict(GP.Ymdl,lpred);
xPred = predict(GP.Xmdl,lpred);

clf;
PlotStateLargeGoal2(SystemNode)
legend off
hold on
h1 = plot(xPred,yPred,'.-');
h2 = plot(estMarkerPos(:,1),estMarkerPos(:,2),'ro','MarkerSize',12,'LineWidth',1.5);
%h3 = plot(MarkerPos(:,1), MarkerPos(:,2),'r*','MarkerSize',12,'LineWidth',1.5);
legend([h1 h2],{'Predicted Cable Position','Estimated Marker Position'})


% [ newGP minDist minDistNorm ] = RefineGP( GP, CablePointCloud, 0.02 );
% % oldGP = GP;
% GP = newGP;
% 
% [ newGP minDist minDistNorm ] = RefineGP( GP, CablePointCloud, 0.02 );
% oldGP = GP;
% GP = newGP;
% % 
% [ newGP minDist minDistNorm ] = RefineGP( GP, CablePointCloud, 0.02 );
% oldGP = GP;
% GP = newGP;

[FinalGP minDistNorm] = IterativeRefineGP(GP, CablePointCloud, 0.008, SystemNode);
oldGP = GP;
GP = FinalGP;
[OutputCurve] = NormalizeCurve(GP,CablePointCloud,SystemNode,1,1000);
%% check the accuracy of new GP


newGP2.length = ResampleGPLength(GP.length);
xPred = predict(GP.Xmdl,newGP2.length);
yPred = predict(GP.Ymdl,newGP2.length);
estObservations = [xPred yPred];
% 
clf
PlotStateLargeGoal2(SystemNode)
legend off
hold on
plot(GP.observations(:,1),GP.observations(:,2),'o')
plot(OutputCurve.position(:,1), OutputCurve.position(:,2),'.-')

lengthID = [100 200 300 400 500 600 700 800 900 1000];
ActualPos = GetPosition(SystemNode.State.Cable, 1, OutputCurve.length(lengthID));
plot(ActualPos(:,1),ActualPos(:,2),'o','MarkerSize',14);
plot(OutputCurve.position(lengthID,1),OutputCurve.position(lengthID,2),'*','MarkerSize',14)


lpred = OutputCurve.length;
xPred = predict(GP.Xmdl,lpred);
yPred = predict(GP.Ymdl,lpred);


ActualPos = GetPosition(SystemNode.State.Cable, 1, OutputCurve.length);
ActualPos = ActualPos(:,[1 2]);
NormPredictedPos = OutputCurve.position;
NormError = NormPredictedPos - ActualPos;
for i = 1:size(NormError,1)
    RegnormError(i) = norm(NormError(i,:));
end
PredictedPos = [xPred yPred];
Error = PredictedPos - ActualPos;
for i = 1:size(Error,1)
    normError(i) = norm(Error(i,:));
end
% plot(ActualPos(1),ActualPos(2),'o');
% plot(OutputCurve.position(lengthID,1),OutputCurve.position(lengthID,2),'*')

figure(2)
clf;
f1 = plot(OutputCurve.length, RegnormError);
hold on
f2 = plot(lpred, normError);
legend([f1 f2],{'Normalized','Non-Normalized'});




