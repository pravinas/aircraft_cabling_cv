load Sample_CableState.mat
close all
MarkerPos = getMarkerPos(SystemNode,1);
length = SystemNode.State.Cable(1).markers';

nMarkers = size(MarkerPos,1);
noise = randn(nMarkers,2)*5e-3;
MarkerPos = MarkerPos+noise;


%Create a cloud of 100 points around each marker within a 2cm

PointCloud = [];

for i = 1:nMarkers
    
    mu = MarkerPos(i,:);
    sigma = [1e-5 0; 0 1e-5];
    PointCloudMarker = mvnrnd(mu, sigma, 20);
    PointCloud = [PointCloud; PointCloudMarker];
end

plot(MarkerPos(:,1), MarkerPos(:,2),'ro');
hold on
plot(PointCloud(:,1),PointCloud(:,2),'x');

[idx C] = kmeans(PointCloud, nMarkers);

plot(C(:,1), C(:,2), 'k*');
[sortedC I] = sort(C(:,1),'ascend');
C = C(I,:);
estMarkerPos = C;



Xtbl = table(length, estMarkerPos(:,1), 'VariableNames',{'l', 'X'});
Ytbl = table(length, estMarkerPos(:,2), 'VariableNames',{'l', 'Y'});



lpred = SystemNode.State.Cable.configuration.length;

Xmdl = fitrgp(Xtbl,'X' , 'KernelFunction','matern32' , 'KernelParameters',[0.1; 0.09]);
Ymdl = fitrgp(Ytbl,'Y' , 'KernelFunction','matern32' , 'KernelParameters',[0.1; 0.04]);

% Xmdl = fitrgp(Xtbl,'X'  , 'KernelParameters',[0.04; 0.09]);
% Ymdl = fitrgp(Ytbl,'Y' , 'KernelParameters',[0.04; 0.04]);


%PlotStateLargeGoal2(SystemNode)
yPred = predict(Ymdl,lpred);
xPred = predict(Xmdl,lpred);


% plot(Xtbl.l, Xtbl.X,'o');
% hold on
% 
% plot(lpred,xPred)
% %clf


PlotStateLargeGoal2(SystemNode)
legend off
hold on
h1 = plot(xPred,yPred,'x-');
h2 = plot(estMarkerPos(:,1),estMarkerPos(:,2),'ro','MarkerSize',12,'LineWidth',1.5);
h3 = plot(MarkerPos(:,1), MarkerPos(:,2),'r*','MarkerSize',12,'LineWidth',1.5);
legend([h1 h2 h3],'Predicted Shape','Estimated Marker Position','True Marker Position')

%Compute the errors in the predicted and acual positions at the given
%lengths

ActualPos = SystemNode.State.Cable(1).configuration.state;
ActualPos = ActualPos(:,[1 2]);
PredictedPos = [xPred yPred];
Error = PredictedPos - ActualPos;
for i = 1:size(Error,1)
    normError(i) = norm(Error(i,:));
end

figure(2)
plot(lpred, normError);
%%
figure(3);
plot(MarkerPos(:,1), MarkerPos(:,2),'ro');
hold on
plot(PointCloud(:,1),PointCloud(:,2),'x');

plot(C(:,1), C(:,2), 'k*');

CablePointCloudDensity = 5;
PointsPerMarker = 20;
[ CablePointCloud MarkerPointCloud ] = GeneratePointCloud( SystemNode, CablePointCloudDensity, PointsPerMarker);


figure(1)
plot(CablePointCloud(:,1), CablePointCloud(:,2),'.')

