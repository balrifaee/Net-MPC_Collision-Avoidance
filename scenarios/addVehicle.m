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

function [ scenario ] = addVehicle( scenario, vehicle )

    scenario.x0 = [scenario.x0; ...
        scenario.model.makeInitStateVector(vehicle)];
    
    scenario.nVeh = scenario.nVeh + 1;    
    scenario.Q = [scenario.Q; vehicle.Q];
    scenario.Q_final = [scenario.Q_final; vehicle.Q_final];
    scenario.R = [scenario.R; vehicle.R];
    scenario.RVeh = [scenario.RVeh; norm([vehicle.Length vehicle.Width],2)/2];
    scenario.Lf = [scenario.Lf; vehicle.Lf];
    scenario.Lr = [scenario.Lr; vehicle.Lr];
    scenario.Width = [scenario.Width; vehicle.Width];
    scenario.Length = [scenario.Length; vehicle.Length];    
    scenario.u0 = [scenario.u0; vehicle.u0];
    
    scenario.referenceTrajectories{end+1} = vehicle.referenceTrajectory;
end

