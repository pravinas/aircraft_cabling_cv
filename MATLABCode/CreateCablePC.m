function [ CablePointCloud ] = CreateCablePC( SystemNode, CableID, nPointsPerSlice )

%configuration = SystemNode.State.Cable(CableID).configuration;

l = [0:0.005:SystemNode.State.Cable(CableID).length];
positions = GetPosition(SystemNode.State.Cable,CableID,l);


npoints = max(size(l));
thicknessvar = 0.005;
CablePointCloud = [];


for i = 1:npoints
    
    theta = positions(i,3);
    axes = [-sin(theta) cos(theta)];
    dist = thicknessvar*linspace(-1,1,nPointsPerSlice)';
    Offsets = kron(dist,axes);
    initialPos = kron(ones(nPointsPerSlice,1),positions(i,[1 2]));
    PointCloudLocal = initialPos + Offsets;
    CablePointCloud = [CablePointCloud; PointCloudLocal];
end


end

