function [ CablePointCloud, MarkerPointCloud ] = GeneratePointCloud( SystemNode, CablePointCloudDensity, PointsPerMarker )
%

CablePointCloud = CreateCablePC(SystemNode, 1, CablePointCloudDensity);
MarkerPointCloud = CreateMarkerPC(SystemNode, 1, PointsPerMarker);
end

