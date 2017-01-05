function [OutputCellArray] = AddColumntoCellArray( CellArray, CellColumn )
% [OutputCellArray] = AddRowtoCellArray( CellArray, CellRow )

% This function adds the given cell array row to a larger cell array
% CellArray: The larger cell array datat type to which an addition needs to
% be made
% CellColumn: The element to be added

n = size(CellArray, 2); % Get the width of the cell array

OutputCellArray = CellArray;
OutputCellArray{1,n+1} = CellColumn;
end

