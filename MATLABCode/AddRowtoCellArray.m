function [OutputCellArray] = AddRowtoCellArray( CellArray, CellRow )
% [OutputCellArray] = AddRowtoCellArray( CellArray, CellRow )

% This function adds the given cell array row to a larger cell array
% CellArray: The larger cell array datat type to which an addition needs to
% be made
% CellRow: The element to be added

n = size(CellArray, 1); % Get the height of the cell array

OutputCellArray = CellArray;
OutputCellArray{n+1,1} = CellRow;
end

