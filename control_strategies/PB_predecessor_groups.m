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

function [ groups ] = PB_predecessor_groups( topological_grouping_matrix )

groups = struct;

for group_idx = 1:size(topological_grouping_matrix,1)
    groups(group_idx).members = find(topological_grouping_matrix(group_idx,:));
    if group_idx == 1
        groups(group_idx).predecessors = [];
    else
        groups(group_idx).predecessors = [groups(group_idx-1).predecessors groups(group_idx-1).members];
    end
end

end

