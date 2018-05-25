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

% Computes equidistant points along a piecewise linear curve. The first
% point is the point on the curve closest to the given point
% (vehicle_x,vehicle_y). All following points are on the curve with a
% distance of 'stepSize' to their predecessor.
%
% Arguments: 
%       nSamples: number of points created
%       referenceTrajectory: piecewise linear curve [x1 y1; x2 y2; ...]
%       vehicle_x,vehicle_y: start point
%       stepSize: Distance between points
% Returns: points [x1 y1; x2 y2; ...]
function [ ReferencePoints ] = sampleReferenceTrajectory(nSamples, referenceTrajectory, vehicle_x,vehicle_y, stepSize )

    ReferencePoints = zeros(nSamples,2);
    
    [~, ~, x, y, TrajectoryIndex ] = getShortestDistance(referenceTrajectory(:,1),referenceTrajectory(:,2),vehicle_x,vehicle_y);
    
    nLinePieces = size(referenceTrajectory,1);
    currentPoint = [x y];
    
    % All line-segments are assumed to be longer than stepSize. Should it
    % become necessary to have short line-segments this algorithm needs to
    % be changed.
    for i=1:nLinePieces-1
        assert(norm(referenceTrajectory(i+1,:)-referenceTrajectory(i,:),2)>stepSize);
    end
    
    for i=1:nSamples
        % make a step
        remainingLength = norm(currentPoint-referenceTrajectory(TrajectoryIndex,:),2);
        if remainingLength > stepSize || TrajectoryIndex == nLinePieces
            currentPoint = currentPoint + stepSize*normalize(referenceTrajectory(TrajectoryIndex,:)-referenceTrajectory(TrajectoryIndex-1,:));
        else
            currentPoint = referenceTrajectory(TrajectoryIndex,:);            
            TrajectoryIndex = min(TrajectoryIndex+1, nLinePieces);            
            currentPoint = currentPoint + (stepSize-remainingLength)*normalize(referenceTrajectory(TrajectoryIndex,:)-referenceTrajectory(TrajectoryIndex-1,:));
        end
        
        % record step
        ReferencePoints(i,:) = currentPoint;
    end
end


function y=normalize(x)
    y = x/norm(x,2);
end