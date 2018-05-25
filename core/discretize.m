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

function [ Ad,Bd,Cd,Ed ] = discretize( x0,u0,Lf,Lr, dt, model )

    % Compute the linearization and discretization of a non-linear
    % continous model with a known jacobian around a given point.
    %
    % Model form: dx/dt = f(x,u)
    %
    % Discretization form: x(k+1) = Ad*x(k) + Bd*u(k) + Ed
    
    if size(x0, 1)==1
        x0=x0';
    end
    
    [Ac,Bc,Cc,Ec] = model.jacobian(x0,u0,Lf,Lr);

    % These two versions below seem to be equivalent.
    % The expm version has the advantage that it does not rely on the
    % Control System Toolbox.
    
    %% c2d version
%     m = ss( Ac, Bc, Cc, zeros(model.ny,model.nu) );
%     md = c2d( m,dt );
%     Ad = md.a;
%     Bd = md.b;
%     Cd = md.c;
% 
%     maff  = ss( Ac,Ec,Cc,zeros(model.ny,1) );
%     mdaff = c2d(maff,dt);
%     Ed = mdaff.b;
    
    %% expm version
    % Formula from wikipedia
    % https://en.wikipedia.org/wiki/Discretization#cite_ref-1
    % expm([A B; 0 0] * dt) == [Ad Bd; 0 eye]
    % Cited from
    % Raymond DeCarlo: Linear Systems: A State Variable Approach with Numerical Implementation, Prentice Hall, NJ, 1989
    % page 215
    tmp = expm(dt*[Ac Bc; zeros(size(Ac,2)+size(Bc,2)-size(Ac,1),size(Ac,2)+size(Bc,2))]);
    Ad = tmp(1:size(Ac,1),1:size(Ac,2));
    Bd = tmp(1:size(Bc,1),[1:size(Bc,2)]+size(Ac,2));
    Cd = Cc;
    
    tmp = expm(dt*[Ac Ec; zeros(size(Ac,2)+size(Ec,2)-size(Ac,1),size(Ac,2)+size(Ec,2))]);
    Ed = tmp(1:size(Ec,1),[1:size(Ec,2)]+size(Ac,2));

end