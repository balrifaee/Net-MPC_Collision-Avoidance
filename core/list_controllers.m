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

% Returns a list of avalible controllers.
function [ list ] = list_controllers

    list = {...   
        'SCP' @SCP_controller; ...   
        'MILP' @MILP_CPLEX_controller; ...
        'MIQP' @MIQP_CPLEX_controller; ...
    };

end