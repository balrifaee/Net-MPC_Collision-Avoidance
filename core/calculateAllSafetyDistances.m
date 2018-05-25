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

function [dsafeVehicles,dsafeObstacles] = calculateAllSafetyDistances(scenario)
    nVeh = scenario.nVeh;    
    nObst = size(scenario.obstacles,1);
    idx = indices;
    dsafeVehicles = zeros(nVeh,nVeh);
    dsafeObstacles = zeros(nVeh,nObst);
    for v=1:nVeh
        for v2 = 1:nVeh            
            max_chord_length = (scenario.x0(v,idx.speed) + scenario.x0(v2,idx.speed))*scenario.dt;
            W1 = scenario.Width(v)/2;
            W2 = scenario.Width(v2)/2;
            L1 = scenario.Length(v)/2; 
            L2 = scenario.Length(v2)/2;
            R = sqrt(L1^2 + W1^2) + sqrt(L2^2 + W2^2);
            dsafe = sqrt((max_chord_length/2)^2 + R^2);
            dsafeVehicles(v,v2) = dsafe;
        end
        for o=1:nObst
            max_chord_length = (scenario.x0(v,idx.speed) + scenario.obstacles(o,idx.speed))*scenario.dt;
            W1 = scenario.Width(v)/2;
            W2 = scenario.obstacles(o,idx.width)/2;
            L1 = scenario.Length(v)/2;
            L2 = scenario.obstacles(o,idx.length)/2;
            R = sqrt(L1^2 + W1^2) + sqrt(L2^2 + W2^2);
            dsafeObstacles(v,o) = sqrt((max_chord_length/2)^2 + R^2);
        end
    end
end