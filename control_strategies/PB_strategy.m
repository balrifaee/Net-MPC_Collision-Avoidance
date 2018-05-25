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

% This function implements priority-based non-cooperative decentralized MPC 
% by calling some other centralized controller for each vehicle with a 
% reduced input data set, that does not contain other vehicles with lower 
% or equal priority, and treats vehicles with higher priority as 
% uncontrolled obstacles with fixed trajectories.
function [U,trajectoryPrediction,controllerOutput] = PB_strategy( scenario,iter,previousControllerOutput,sub_controller ) 

[isDAG, topo_groups] = kahn(scenario.CouplingAdjacencyMatrixPB);

assert( isDAG, 'Coupling matrix is not a DAG' );

% add 'self-connections'.
scenario.CouplingAdjacencyMatrixPB = scenario.CouplingAdjacencyMatrixPB + eye(scenario.nVeh) > 0;

trajectoryPrediction = zeros(scenario.Hp,scenario.model.ny,scenario.nVeh);
U = zeros(scenario.Hp,scenario.nVeh);


controllerOutput=struct;
optimizerTimer = tic;
groups = PB_predecessor_groups(topo_groups);
for grp_idx = 1:length(groups)
    group = groups(grp_idx);
    for grp_member_idx = 1:length(group.members) 
        vehicle_idx = group.members(grp_member_idx);
        
        
        % Filter out vehicles with lower or same priority.
        priority_filter = false(1,scenario.nVeh);
        priority_filter(group.predecessors) = true; % keep all with higher priority
        priority_filter(vehicle_idx) = true; % keep self
        scenario_filtered = filter_scenario(scenario, priority_filter);
        iter_filtered = filter_iter(iter, priority_filter);
        
        self_index = sum(priority_filter(1:vehicle_idx));        
        v2o_filter = true(1,scenario_filtered.nVeh);
        v2o_filter(self_index) = false;
        
        [scenario_v, iter_v] = vehicles_as_obstacles(scenario_filtered, iter_filtered, v2o_filter, permute(trajectoryPrediction(:,:,priority_filter),[3 2 1]));

        
        sub_controller_prev_output = struct;
        try
            sub_controller_prev_output = previousControllerOutput.sub_controller_output{vehicle_idx};
        end
        [U_v,trajectoryPrediction_v,sub_controllerOutput] = sub_controller(scenario_v, iter_v, sub_controller_prev_output);
        controllerOutput.sub_controller_output{vehicle_idx} = sub_controllerOutput;

        trajectoryPrediction(:,:,vehicle_idx) = trajectoryPrediction_v;
        U(:,vehicle_idx) = U_v;
        
        groups(grp_idx).computationTime(grp_member_idx) = sub_controllerOutput.optimizerTime;
    end
end
controllerOutput.optimizerTime = toc(optimizerTimer);
controllerOutput.trajectoryPrediction = trajectoryPrediction;





controllerOutput.PB_Timers = struct;
controllerOutput.PB_Timers.full_parallel = 0;
controllerOutput.PB_Timers.level_parallel = 0;
controllerOutput.PB_Timers.serial = 0;
for i=1:length(groups)
    group=groups(i);
    for time=group.computationTime
        controllerOutput.PB_Timers.serial = controllerOutput.PB_Timers.serial + time;
        controllerOutput.PB_Timers.full_parallel = max(controllerOutput.PB_Timers.full_parallel, time);        
    end
    if( scenario.nObst > 0 || i > 1)
        controllerOutput.PB_Timers.level_parallel = controllerOutput.PB_Timers.level_parallel + ...
            max(group.computationTime);
    end
end


end

