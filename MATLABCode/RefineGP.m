function [ newGP minDist minDistNorm ] = RefineGP( GP, CablePointCloud, minDistThresh )


%% Resample the lengths for the new GP

newGP.length = ResampleGPLength(GP.length);

%% Determine observations for the resampled points
xPred = predict(GP.Xmdl,newGP.length);
yPred = predict(GP.Ymdl,newGP.length);
estObservations = [xPred yPred];

% Enter the observations of the old GP points
newGPsize = max(size(newGP.length));
newGP.observations = zeros(newGPsize,2);
minDist = zeros(newGPsize,1);

for i = 1:(newGPsize+1)/2
    newGP.observations(2*i-1,:) = GP.observations(i,:);
    currEstObservation = estObservations(2*i-1,:);
    sizePointCloud = size(CablePointCloud,1);
    dist = zeros(sizePointCloud,1);
    for j = 1:sizePointCloud;
        relPos = CablePointCloud(j,:) - currEstObservation;
        dist(j) = norm(relPos);
    end
    
    [minDist(2*i-1) minID] = min(dist);
    
end

%Determine the closest point for the refinement points

for i = 1:(newGPsize-1)/2
    
    %Choose the point cloud point nearest to the estimated positions
    currEstObservation = estObservations(2*i,:);
    sizePointCloud = size(CablePointCloud,1);
    dist = zeros(sizePointCloud,1);
    for j = 1:sizePointCloud;
        relPos = CablePointCloud(j,:) - currEstObservation;
        dist(j) = norm(relPos);
    end
    
    [minDist(2*i) minID] = min(dist);
    ClosestPoint = CablePointCloud(minID,:);
    newGP.observations(2*i,:) = ClosestPoint;
end

%% Construct the new GP

% The new GP is only constructed if the minimum distance is larger than a
% given threshhold

minDistNorm = norm(minDist,'inf');

if minDistNorm >= minDistThresh
    
    Xtbl = table(newGP.length, newGP.observations(:,1), 'VariableNames',{'l', 'X'});
    Ytbl = table(newGP.length, newGP.observations(:,2), 'VariableNames',{'l', 'Y'});
    newGP.Xmdl = fitrgp(Xtbl,'X' , 'KernelFunction','matern32' , 'KernelParameters',[0.1; 0.09]);
    newGP.Ymdl = fitrgp(Ytbl,'Y' , 'KernelFunction','matern32' , 'KernelParameters',[0.1; 0.04]);
else
    newGP = GP;
end

end

