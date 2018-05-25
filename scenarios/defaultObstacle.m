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

function [ obstacle ] = defaultObstacle()

    % obstacles are modeled as rotated rectangles that can move with
    % constant speed and direction.
    
    obstacle = struct;
    
    % obstacle's center position [m]
    obstacle.x = 0; 
    obstacle.y = 0;
        
    obstacle.heading = 0; % obstacle's rotation AND directon of movement [radians]
    obstacle.speed = 0; % [m/s]
    obstacle.length = 2; % obstacle's size measured along its direction of movement [m]
    obstacle.width = 2; % obstacle's size measured at right angels to its direction of movement[m]
end