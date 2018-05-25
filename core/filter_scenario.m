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

% Reduce scenario structure to selected vehicles
function scenario = filter_scenario(scenario, vehicle_filter)
properties = {'Q' 'Q_final' 'R' 'Lf' 'Lr' 'Length' 'Width' 'RVeh' 'x0' 'u0' 'dsafeObstacles'};
for i=1:length(properties)
    property = properties{i};
    value = scenario.(property);
    scenario.(property) = value(vehicle_filter,:);
end
scenario.referenceTrajectories = scenario.referenceTrajectories(vehicle_filter);
scenario.dsafeVehicles = scenario.dsafeVehicles(vehicle_filter,vehicle_filter);
scenario.CooperationCoefficientMatrix = scenario.CooperationCoefficientMatrix(vehicle_filter,vehicle_filter);
scenario.nVeh = sum(vehicle_filter);
end