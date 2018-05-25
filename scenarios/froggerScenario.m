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

function scenario = froggerScenario

    scenario = defaultScenario();
    veh1 = defaultVehicle();
    veh1.x_start = -18;
    veh1.referenceTrajectory = [-100 0; 100 0]; 
    scenario = addVehicle( scenario, veh1 );    
    
    for o=-2:3
        ob = defaultObstacle();
        ob.x = 18;
        ob.y = 7*o-15;
        ob.speed = 2;
        ob.heading = pi/2;
        ob.length = 2;
        ob.width = 2;
        scenario = addObstacle(scenario, ob);
        ob.x = 14;
        scenario = addObstacle(scenario, ob);
    end
        
    scenario.plotLimits = 35*[-1 1;-1 1];
end