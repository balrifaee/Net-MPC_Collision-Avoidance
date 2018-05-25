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

function df = auto_diff(model)

    u = sym('u');
    Lf = sym('Lf');
    Lr = sym('Lr');
    x = sym('x',[model.nx,1]);
    
    Ac = jacobian(model.ode(x,u,Lf,Lr),x);
    Bc = jacobian(model.ode(x,u,Lf,Lr),u);
    Cc = eye(model.ny,model.nx);
    Ec = model.ode(x,u,Lf,Lr) - Ac * x - Bc * u;
    
    df = matlabFunction(Ac,Bc,Cc,Ec,'Vars',{x,u,Lf,Lr});    
end