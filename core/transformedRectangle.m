% Copyright 2016 Bassam Alrifaee
% 
% This program is free software: you can redistribute it and/or modify
% it under the terms of the GNU General Public License version 3 as 
% published by the Free Software Foundation.
% 
% This program is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU General Public License for more details.
% 
% You should have received a copy of the GNU General Public License
% along with this program. If not, see <http://www.gnu.org/licenses/>.

% Creates a rectangle with the given position and orientation.
function [ polygon_XY ] = transformedRectangle( x,y,angle, Length, Width )

    unitSquare = [ 0 0 0 1; 1 0 0 1; 1 1 0 1; 0 1 0 1]';
    
    % Read this bottom-up
    polygon_XY =  makehgtform('translate',[x y 0]) ...
                * makehgtform('zrotate',angle) ...
                * makehgtform('scale',[Length Width 1]) ...
                * makehgtform('translate',[-.5 -.5 0]) ...
                * unitSquare;
            
    polygon_XY = polygon_XY(1:2,:);
end

