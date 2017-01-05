% This script compares the performance of k means clusetering and GMM based
% clustering algorithms

load Sample_CableState.mat
close all

%% Generate Point Cloud data

CableID = 1;
MarkerPos = getMarkerPos(SystemNode,CableID);
nMarkers = size(MarkerPos,1);

CablePointCloudDensity = 5;
PointsPerMarker = 20;
[ CablePointCloud MarkerPointCloud ] = GeneratePointCloud( SystemNode, CablePointCloudDensity, PointsPerMarker);

figure(1)
h1 = plot(MarkerPointCloud(:,1), MarkerPointCloud(:,2), '.');
axis equal
hold on

%% K means clustering

[idx KMeansClusterCenters] = kmeans(MarkerPointCloud, nMarkers);
h2 = plot(KMeansClusterCenters(:,1), KMeansClusterCenters(:,2), 'x', 'MarkerSize',10,'LineWidth',1.5);


legend([h1 h2],{'PC points','K-Means'});

%% GMM clustering
sigma = [1e-5 0; 0 1e-5];
init = gmdistribution(KMeansClusterCenters, sigma); 
S.mu = init.mu;
S.Sigma = init.Sigma
S.PComponents = init.ComponentProportion;
mdl = gmdistribution.fit(MarkerPointCloud,nMarkers,'start',S,'regularize',0.0001,'SharedCov',true);
plot(mdl.mu(:,1), mdl.mu(:,2),'o');


