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

% Preprocessing step for any MPC controller. Compensates for input, computation and output delays with numerical integration using past controller outputs.
function [iter, MPC_delay_compensation_trajectory] = MPC_init(scenario, x_measured, u_path, obstacleState, uMax)

    idx = indices;
    iter = struct;
    
    MPC_delay_compensation_trajectory = zeros(10,scenario.model.nx,scenario.nVeh);
    
    assert(size(u_path,2) * scenario.tick_length - (scenario.delay_x + scenario.dt + scenario.delay_u) < 1e-10);
    for v=1:scenario.nVeh
        
        [~,Y] = ode45(@(t,x) scenario.model.ode(x, u_path(v,min(size(u_path,2),floor(t/scenario.tick_length)+1)),scenario.Lf(v),scenario.Lr(v)),...
            linspace(0, scenario.delay_x + scenario.dt + scenario.delay_u,10), ...
            x_measured(v,:)', odeset('RelTol',1e-8,'AbsTol',1e-8));        
        
        iter.x0(v,:) = Y(end,:);
        iter.u0(v,:) = u_path(v,end);
        MPC_delay_compensation_trajectory(:,:,v) = Y;
    end
    
    iter.ReferenceTrajectoryPoints = zeros(scenario.Hp,2,scenario.nVeh);
    for v=1:scenario.nVeh
        % Find equidistant points on the reference trajectory.
        iter.ReferenceTrajectoryPoints(:,:,v) = sampleReferenceTrajectory(...
            scenario.Hp, ... % number of prediction steps
            scenario.referenceTrajectories{v}, ...
            iter.x0(v,idx.x), ... % vehicle position x
            iter.x0(v,idx.y), ... % vehicle position y
            iter.x0(v,idx.speed)*scenario.dt);  % distance traveled in one timestep
    end
    
   
    % Determine Obstacle positions (x = x0 + v*t)
    iter.obstacleFutureTrajectories = zeros(scenario.nObst,2,scenario.Hp);
    for k=1:scenario.Hp
        step = (k*scenario.dt+scenario.delay_x + scenario.dt + scenario.delay_u)*scenario.obstacles(:,idx.speed);
        iter.obstacleFutureTrajectories(:,idx.x,k) = step.*cos( scenario.obstacles(:,idx.heading) ) + obstacleState(:,idx.x);
        iter.obstacleFutureTrajectories(:,idx.y,k) = step.*sin( scenario.obstacles(:,idx.heading) ) + obstacleState(:,idx.y);
    end
    
    iter.uMax = uMax;

end