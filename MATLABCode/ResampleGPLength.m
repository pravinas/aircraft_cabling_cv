function [ NewLengths ] = ResampleGPLength( InitialLengths )
%

nlengths = length(InitialLengths);

NewLengths = zeros(2*nlengths-1,1);

for i = 1:nlengths
    NewLengths(2*i-1) = InitialLengths(i);
end

for i = 1:(nlengths-1)
    NewLengths(2*i) = (NewLengths(2*i-1) + NewLengths(2*i+1))/2;
end


