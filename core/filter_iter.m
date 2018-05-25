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

% Reduce iter structure to selected vehicles
function iter = filter_iter(iter, vehicle_filter)
iter.x0 = iter.x0(vehicle_filter,:);
iter.u0 = iter.u0(vehicle_filter,:);
iter.ReferenceTrajectoryPoints = iter.ReferenceTrajectoryPoints(:,:,vehicle_filter);
iter.uMax = iter.uMax(vehicle_filter);
end