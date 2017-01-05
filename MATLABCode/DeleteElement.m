function [ outMatrix ] = DeleteElement( matrix, index )
% Delete the row with index from the matrix.

matSize = size(matrix, 1);

if index > matSize
    error('Array size insufficient');
end

if matSize == 1
    outMatrix = []; % Result is an empty matrix
end

if index == 1 % index to delete is the first element
    outMatrix = matrix(2:matSize,:);
elseif index == matSize % Index to be deleted is the last element
    outMatrix = matrix(1:(index-1),:);
else
    outMatrix1 = matrix(1:(index-1),:);
    outMatrix2 = matrix((index+1):matSize,:);
    
    outMatrix = [outMatrix1; outMatrix2];
end

