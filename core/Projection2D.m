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

function [xp, yp, projection_distance, lambda, line_segment_len] = Projection2D(x1,y1,x2,y2,x3,y3)

    % Takes a line and a point, determines the projection (point with shortest distance), distance and
    % 'parameter' of the projection.
    % Params:
    %      x1,y1,x2,y2: Points that make a line.
    %      x3,y3:       The point to be projected.
    % Returns:
    %      xp,yp:       The projected point.
    %      projection_distance: Signed distance of (xp,yp) and (x3,y3)
    %      lambda:      The line 'parameter'. Is zero if (x1,y1)==(xp,yp)
    %                   and one if (x2,y2)==(xp,yp).
    %      line_segment_len: Distance between (x1,y1) and (x2,y2)

    b = sqrt((x2-x1)^2+(y2-y1)^2);
    line_segment_len = b;
    if ( b ~= 0 )        
        % normalized direction p1 to p2        
        xn = (x2-x1)/b; yn = (y2-y1)/b;
        
        % vector p1 to p3
        x31 = x3 - x1;
        y31 = y3 - y1;

        % dot product to project p3 on the line from p1 to p2
        projection_dotproduct = xn * x31 + yn * y31;
        
        % cross product to determine the distance from the line to p3
        projection_distance = xn * y31 - yn * x31;
        
        % calculate the projected point
        xp = x1 + projection_dotproduct*xn;
        yp = y1 + projection_dotproduct*yn;

        lambda = projection_dotproduct/b;        
        
    else
       projection_distance = sqrt((x3-x1)^2+(y3-y1)^2);
       lambda = 0;
       xp = x1;
       yp = y1;
    end


end


