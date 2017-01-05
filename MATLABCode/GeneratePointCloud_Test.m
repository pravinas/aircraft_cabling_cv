load Sample_CableState.mat
% 
% [CablePointCloud] = CreateCablePC(SystemNode,4);

CablePointCloudDensity = 5;
PointsPerMarker = 20;
[ CablePointCloud MarkerPointCloud ] = GeneratePointCloud( SystemNode, CablePointCloudDensity, PointsPerMarker);

%PlotStateLargeGoal2(SystemNode);
%plot(MarkerPointCloud(:,1),MarkerPointCloud(:,2),'.')

figure(2);
[sorted idx] = sort(CablePointCloud(:,1));
CablePointCloud = CablePointCloud(idx,:);
PlotStateLargeGoal2(SystemNode);
hold on
plot(CablePointCloud(:,1),CablePointCloud(:,2),'.')
plot(MarkerPointCloud(:,1),MarkerPointCloud(:,2),'.')

