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

function [ model ] = bicycleModelRajamani2()

    model = struct;
    model.ode = @ode;
    model.nx = 6;
    model.nu = 1;
    model.ny = 2;
    model.jacobian = auto_diff(model);
    model.makeInitStateVector = @(vehicle) [vehicle.x_start vehicle.y_start vehicle.heading vehicle.speed vehicle.acceleration 0];
end


function dx = ode(x,u_ref,Lf,Lr)
    % From Vehicle Dynamics and Control, Rajesh Rajamani, p. 24
    % With modifications:
    % * Added steering dynamic: U/U_ref = 1/(1+Ts), T = 0.1 sec.
    % * Added correction for velocity: The measured speed on the buggy
    % is that of the rear axle. In this model we need the speed of the
    % center.
    % Note: There is a time delay between the controller output and the
    % actuator's reaction. It is not modeled here, but taken care of in the
    % simulation setup.

    L = Lf+Lr;
    R = Lr/L;
    phi = x(3);
    a = x(5);
    u = x(6);
    v_rear = x(4);
    v_center = v_rear * sqrt(  1 + (R*tan(u))^2  );
    
    
    dx = x;
    dx(1) = v_center*cos(phi+atan(R*tan(u)));
    dx(2) = v_center*sin(phi+atan(R*tan(u)));
    dx(3) = v_center/L*tan(u)*cos(atan(R*tan(u)));
    dx(4) = a;
    dx(5) = 0;
    dx(6) = (u_ref-u)/0.1;
end