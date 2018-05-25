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

function [ scenario ] = defaultScenario()


    % A tick is the shortest timespan considered during simulation.
    % All other time constants must be integral multiples. [s]
    scenario.tick_length = 0.01;
    scenario.T_end = 20; % Duration of simulation. [s]
    scenario.delay_x = 0; % Measurement transport delay [s].
    scenario.delay_u = .03; % Control transport delay [s].
    scenario.dt   = 0.4; % MPC sample time [s]
    scenario.Hp   = 14; % Prediction horizon
    scenario.Hu   = 3; % Control horizon
    scenario.lateralAccelerationLimit = 9.81/2; % [m/s^2]
    scenario.mechanicalSteeringLimit = pi/180 * 3; % [radians]
    scenario.duLim = scenario.mechanicalSteeringLimit * 2; % Steering limit per timestep [radians]
    scenario.model = bicycleModelRajamani2();
    scenario.nVeh = 0; % Number of vehicles
    
    
    % The following variables are vectors or matrices,
    % where each row corresponds to a vehicle.
    % For more info see defaultVehicle()
    scenario.Q = []; % Trajectory deviation weights for MPC
    scenario.Q_final = []; % Trajectory deviation weight for the final prediction step    
    scenario.R = []; % Steering rate weights for MPC
    scenario.Lf = []; % Distance between vehicle center and front axle center [m]
    scenario.Lr = []; % Distance between vehicle center and rear axle center [m]
    scenario.Length = []; % Vehicle length (bumper to bumper)[m]
    scenario.Width = []; % Vehicle width [m]
    scenario.RVeh = []; % Vehicle radius, distance between center and a corner [m]
    scenario.x0 = []; % Vehicle state: row=[x y heading speed acceleration]
                      %         units:     [m m radians m/s   m/s^2       ]
    scenario.u0 = []; % Initial steering angle [radians]
    
    % The reference trajectory for each vehicle. [m]
    % referenceTrajectories{v}(i,1) corresponds to the x-coordiante of the
    % i-th point in the piecewise linear reference-curve of vehicle v.
    % Likewise referenceTrajectories{v}(i,2) corresponds to the
    % y-coordiante.
    scenario.referenceTrajectories = cell(0);

    % Obstacle data: Each row corresponds to an obstacle and has the
    % strucure: [x y heading speed length width]
    % For more info see defaultObstacle().
    scenario.obstacles = double.empty(0,6);
    
    % Limits for the plot axes [m]. Format:[xmin xmax; ymin ymax]
    scenario.plotLimits = 5*[-10 10;-10 10];
end

