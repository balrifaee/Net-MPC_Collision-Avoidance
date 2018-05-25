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

function [signed_distance_min, arclength_min, x_min, y_min, index_min ] = getShortestDistance(curve_x,curve_y,x,y)

    % Finds the point on a piecewise linear curve that is closest to a
    % given point.
    % Params:
    %     curve_x, curve_y:
    %         A polygonal chain (a.k.a. piecewise linear curve)
    %     x,y:
    %         The point to be projected onto the curve
    % Returns:
    %     x_min, y_min:
    %         The projected point on the curve.
    %     arclength_min:
    %         Arc length on the curve between (curve_x(1),curve_y(1)) and 
    %         (x_min,y_min).
    %     signed_distance_min:
    %         Signed distance between (x_min,y_min) and (x,y).
    %         Left ~ positive, right ~ negative.

    assert(length(x)==1);
    assert(length(y)==1);
    assert(length(curve_x) == length(curve_y));
    assert(length(curve_x) >= 2);
    arclength_sum = 0;

    
    % Guess first point as minimum
    x_min = curve_x(1);
    y_min = curve_y(1);
    arclength_min=0;
    signed_distance_min = sqrt((x-curve_x(1))^2 +(y-curve_y(1))^2);
    index_min = 2;
    
    for j = 2:(length(curve_x));
        [xp, yp, signed_distance, lambda, piecelength] =  Projection2D(curve_x(j-1),curve_y(j-1),curve_x(j),curve_y(j),x,y);

        % Projected point is between the end points.
        if (0 < lambda || j==2) && (lambda < 1 || j==length(curve_x))
            if abs(signed_distance) < abs(signed_distance_min)
                x_min = xp;
                y_min = yp;
                signed_distance_min = signed_distance;
                arclength_min= arclength_sum + lambda * piecelength;
                index_min = j;
            end
        else
            d_end = sqrt((x-curve_x(j))^2 +(y-curve_y(j))^2);
            if  abs(d_end) < abs(signed_distance_min)
                x_min = curve_x(j);
                y_min = curve_y(j);
                signed_distance_min = sign(signed_distance)*d_end;
                arclength_min= arclength_sum + piecelength;
                index_min = j;
            end
        end
        arclength_sum = arclength_sum+piecelength;
    end
    
end