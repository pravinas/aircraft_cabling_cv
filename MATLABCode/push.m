function [ outMatrix ] = push( matrix, row )
% Adds the row to the top of the matrix

outMatrix = [row; matrix];
end

