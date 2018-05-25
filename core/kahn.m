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

% Kahn's topological sorting algorithm
function [valid, L] = kahn(A)
% init
a = length(A);
l = 1; % nr. of levels
L = zeros(l,a);
in_D = sum(A,1);
old_zeros_in_D = [];
while ~all(in_D == 1)
    zero_in_D = find(in_D==0);
    if isempty(zero_in_D)
        valid = false;
        return
    end
    L(l,zero_in_D) = 1;
    l = l+1;
    A(zero_in_D,:) = 0;
    in_D = sum(A,1);
    old_zeros_in_D = [old_zeros_in_D zero_in_D];
    in_D(old_zeros_in_D) = 1;
end
% check results
valid = isequal(sum(sum(L,1)),a);