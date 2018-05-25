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

function du=clipDeltaUtoSteeringLimit(du,iter,scenario)
    du = min(scenario.duLim,max(-scenario.duLim,du));
    du = reshape(du, scenario.Hu, scenario.nVeh);
    for v=1:scenario.nVeh
        u_sum = iter.u0(v);
        for j=1:scenario.Hu
            if u_sum + du(j,v) > iter.uMax(v)
                 du(j,v) = iter.uMax(v) - u_sum;
            elseif u_sum + du(j,v) < -iter.uMax(v)
                du(j,v) = -iter.uMax(v) - u_sum;
            end
            u_sum = u_sum + du(j,v);
        end
    end
    du = reshape(du, scenario.Hu*scenario.nVeh,1);
end