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

function [ scenario ] = parallelScenario(nVeh)
    scenario = defaultScenario();
    positions = (1:nVeh)-ceil(nVeh/2);
    
    order = 1:nVeh;
    order = [flip(order(1:2:end)) order(2:2:end)];
    
    positions(order) = positions;
    
    for i = 1:nVeh
        y = 2.2*positions(i);
        veh = defaultVehicle();
        veh.x_start = -40;
        veh.y_start = y;
        veh.labelOffset=[-6.1-4.5*mod(positions(i)-1,2) 0];
        veh.referenceTrajectory = [-30 y; 30 y]; 
        scenario = addVehicle( scenario, veh );
    end
    
    ob1 = defaultObstacle();
    ob1.length = 2;
    ob1.width = 4;
    ob1.y = -0.05;
    scenario = addObstacle(scenario, ob1);
        
    
    
    if nVeh == 2
        scenario.CouplingAdjacencyMatrixPB = logical([0 1;0 0]);
    elseif nVeh > 2
        scenario.CouplingAdjacencyMatrixPB = logical(diag(1:(nVeh-2),2));
        scenario.CouplingAdjacencyMatrixPB(1,2) = true;
    end
    
    scenario.plotLimits = [-50 30; -12 12];
end