function [ OutputCurve ] = NormalizeCurve( GP, PointCloud, SystemNode, CableID, nReconstPoints )
% OutputCurve.length
% OutputCurve.position

oldParamSamples = linspace(0,max(GP.length),nReconstPoints);

xPred = predict(GP.Xmdl,oldParamSamples');
yPred = predict(GP.Ymdl,oldParamSamples');

Positions = [xPred yPred];

LengthParam = zeros(nReconstPoints, 1);

for i = 2:nReconstPoints
    deltaX = xPred(i) - xPred(i-1);
    deltaY = yPred(i) - yPred(i-1);
    LengthParam(i) = LengthParam(i-1) + norm([deltaX deltaY]);
end

LengthParam = (SystemNode.State.Cable(CableID).length/max(LengthParam))*LengthParam;

OutputCurve.length = LengthParam;
OutputCurve.position = Positions;
end

