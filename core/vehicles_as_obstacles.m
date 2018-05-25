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

% This function converts a scenario representation based on the scenario
% and iter structs into a different scenario, in which the selected
% vehicles are represented as obstacles with fixed trajectories.
function [scenario_v, iter_v] = vehicles_as_obstacles(scenario, iter, vehicle_filter, vehicle_trajectories)

assert(all( size(vehicle_trajectories) == [scenario.nVeh scenario.model.ny scenario.Hp] ))
assert( length(vehicle_filter) == scenario.nVeh );
assert( islogical( vehicle_filter ));
idx = indices;

scenario_v = filter_scenario(scenario, ~vehicle_filter);
iter_v = filter_iter(iter, ~vehicle_filter);

new_obstacles = nan(sum(vehicle_filter),6);

new_obstacles(:) = [nan(sum(vehicle_filter),2) iter.x0(vehicle_filter,idx.heading) iter.x0(vehicle_filter,idx.speed) scenario.Length(vehicle_filter) scenario.Width(vehicle_filter)];

scenario_v.obstacles = [scenario_v.obstacles;new_obstacles];

new_obstacle_trajectories = vehicle_trajectories(vehicle_filter,:,:);
iter_v.obstacleFutureTrajectories = cat(1, iter_v.obstacleFutureTrajectories, new_obstacle_trajectories);

scenario_v.nObst = size(scenario_v.obstacles,1);

[scenario_v.dsafeVehicles,scenario_v.dsafeObstacles] = calculateAllSafetyDistances(scenario_v);

end