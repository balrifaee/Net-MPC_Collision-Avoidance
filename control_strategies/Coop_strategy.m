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

% This function implements cooperative decentralized MPC by calling some other
% centralized controller for each vehicle with a reduced input data set, that does not
% contain other non-coupled vehicles.
function [U,trajectoryPrediction,controllerOutput] = Coop_strategy( scenario,iter,prevOutput,sub_controller ) 

% Make coupling matrix symmetric and add 'self-connections'.
scenario.CouplingAdjacencyMatrixCoop = scenario.CouplingAdjacencyMatrixCoop+scenario.CouplingAdjacencyMatrixCoop'+eye(scenario.nVeh)>0;

assert( norm(scenario.CooperationCoefficientMatrix-scenario.CooperationCoefficientMatrix') < 1e-12, 'Cooperation matrix is not symmetric' );

trajectoryPrediction = zeros(scenario.Hp,scenario.model.ny,scenario.nVeh);
U = zeros(scenario.Hp,scenario.nVeh);

controllerOutput=struct;
controllerOutput.optimizerTime = 0;
controllerOutput.sub_controller_output = {};
for v=1:scenario.nVeh    
    % Filter out non coupled vehicles.
    scenario_filtered = filter_scenario(scenario, scenario.CouplingAdjacencyMatrixCoop(v,:));
    iter_filtered = filter_iter(iter, scenario.CouplingAdjacencyMatrixCoop(v,:));    
    self_index = sum(scenario.CouplingAdjacencyMatrixCoop(v,1:v));
    
    % Pass the cooperation coefficients through.
    % This is a bit 'tacked on', but with the current architecture, there is
    % no other simple way.
    scenario_filtered.CooperationCoefficients = scenario_filtered.CooperationCoefficientMatrix(self_index,:);
    
    % Call sub controller
    sub_controller_prev_output = struct;
    try
        sub_controller_prev_output = prevOutput.sub_controller_output{v};
    end
    [U_v,trajectoryPrediction_v,controllerOutput_v] = sub_controller(scenario_filtered, iter_filtered, sub_controller_prev_output);
    controllerOutput.sub_controller_output{v} = controllerOutput_v;
    
    controllerOutput.optimizerTime = max(controllerOutput.optimizerTime, controllerOutput_v.optimizerTime);
    
    % Compose results
    trajectoryPrediction(:,:,v) = trajectoryPrediction_v(:,:,self_index);
    U(:,v) = U_v(:,self_index);
end
end