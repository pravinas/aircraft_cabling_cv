function [ MarkerPointCloud ] = CreateMarkerPC( SystemNode, CableID, nPointPerMarker )
%

MarkerPos = getMarkerPos(SystemNode,CableID);
%length = SystemNode.State.Cable(CableID).markers';

nMarkers = size(MarkerPos,1);
noise = randn(nMarkers,2)*1e-3;
MarkerPos = MarkerPos+noise;
MarkerPointCloud = [];

for i = 1:nMarkers
    
    mu = MarkerPos(i,:);
    sigma = [1e-5 0; 0 1e-5];
    PointCloudMarker = mvnrnd(mu, sigma, nPointPerMarker);
    MarkerPointCloud = [MarkerPointCloud; PointCloudMarker];
end





end

