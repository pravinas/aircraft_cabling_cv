load Sample_CableState.mat
clf
MarkerPos = getMarkerPos(SystemNode,1);
length = SystemNode.State.Cable(1).markers';
nMarker = size(MarkerPos,1);

%Create a cloud of 100 points around each marker within a 2cm

PointCloud = [];

for i = 1:nMarker
    
    mu = MarkerPos(i,:);
    sigma = [1e-5 0; 0 1e-5];
    PointCloudMarker = mvnrnd(mu, sigma, 100);
    PointCloud = [PointCloud; PointCloudMarker];
end

plot(MarkerPos(:,1), MarkerPos(:,2),'ro');
hold on
plot(PointCloud(:,1),PointCloud(:,2),'x');

[idx C] = kmeans(PointCloud, nMarker);

plot(C(:,1), C(:,2), 'k*');



