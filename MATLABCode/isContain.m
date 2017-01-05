function [ flag index ] = isContain( array, element )
% array: 1D vector that needs to be tested for containing the test element

% element: the test element
% index returns the array index of the first instant of the occurence of
% the element

flag = 0;
index = 0;

for i = 1:length(array)
    
    if array(i) == element
        flag = 1;
        index = i;
    end
end
end

