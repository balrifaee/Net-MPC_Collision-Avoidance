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

function scenario = completeScenario(scenario)

% Calculate tick-rates and round time constants to multiples of a tick.    
scenario.ticks_per_sim = round(scenario.dt / scenario.tick_length); % Ticks per simulation step.
scenario.dt = scenario.ticks_per_sim * scenario.tick_length; % Make dt a multiple of the tick.
scenario.Nsim = round(scenario.T_end / scenario.dt); % Number of simulation steps
scenario.T_end = scenario.Nsim * scenario.dt; % Make total time a multiple of dt.
scenario.ticks_total = round(scenario.T_end / scenario.tick_length); % Total number of ticks.    
scenario.ticks_delay_x = round(scenario.delay_x / scenario.tick_length); % Ticks of the measurement delay.
scenario.delay_x = scenario.ticks_delay_x * scenario.tick_length; % Make measurement delay a multiple of the tick.
scenario.ticks_delay_u = round(scenario.delay_u / scenario.tick_length); % Ticks of the control delay.
scenario.delay_u = scenario.ticks_delay_u * scenario.tick_length; % Make control delay a multiple of the tick.
scenario.nObst = size(scenario.obstacles, 1);

T = linspace(0, scenario.T_end, scenario.ticks_total+1);

[scenario.dsafeVehicles,scenario.dsafeObstacles] = calculateAllSafetyDistances(scenario);

% Fill in coop matrices if they are missing. Priorities are given based on
% the vehicle index
if ~isfield(scenario,'CooperationCoefficientMatrix')
alpha = 1;
scenario.CooperationCoefficientMatrix = alpha*ones(scenario.nVeh,scenario.nVeh)+ (1-alpha)*eye(scenario.nVeh,scenario.nVeh);
end
if ~isfield(scenario,'CouplingAdjacencyMatrixCoop')
scenario.CouplingAdjacencyMatrixCoop = logical(triu(ones(scenario.nVeh),1));
end
if ~isfield(scenario,'CouplingAdjacencyMatrixPB')
scenario.CouplingAdjacencyMatrixPB = logical(triu(ones(scenario.nVeh),1));
end

end