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

function [ scenario ] = circleScenario( angles )
    radius = 30;
    scenario = defaultScenario();
    for angle=angles
        s = sin(angle);
        c = cos(angle);
        veh = defaultVehicle();
        veh.labelOffset = [3 -3]*[c s;-s c]+[-2 0];
        veh.x_start = -c*radius;
        veh.y_start = -s*radius;
        veh.heading = angle;
        veh.referenceTrajectory = [-c*radius -s*radius;c*radius s*radius]; 
        scenario = addVehicle( scenario, veh );
    end
    
    scenario.plotLimits = 1.1*radius*[-1 1;-1 1];    
    if length(angles)==2 && max(abs(sin(angles))) < 0.1
        scenario.plotLimits(2,:) = [-6 6];
    end
end