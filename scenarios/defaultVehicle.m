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

function [ vehicle ] = defaultVehicle()

    vehicle = struct;
    vehicle.u0 = 0; % initial steering angle [radians]    
    
    vehicle.x_start = 0; % [m]
    vehicle.y_start = 0; % [m]
    vehicle.heading = 0; % [radians]
    
    % A list of (x,y) points that make up a piecewise linear curve. The curve is
    % the vehicles desired trajectory. [m]
    vehicle.referenceTrajectory = [0 0; 1 0; 3 1]; 
    
    vehicle.speed = 4; % [m/s]
    vehicle.acceleration = 0; % [m/s^2]
    vehicle.Length = .98; % Vehicle length (bumper to bumper)[m]
    vehicle.Width = .88; % Vehicle width [m]
    vehicle.Lf = .34; % Distance between vehicle center and front axle center [m]
    vehicle.Lr = .34; % Distance between vehicle center and rear axle center [m]
    vehicle.Q = 1; % Trajectory deviation weights for MPC
    vehicle.Q_final = 20; % Trajectory deviation weight for the final prediction step
    vehicle.R = 4000; % Steering rate weights for MPC
    
    vehicle.labelOffset = [0 0];

end


