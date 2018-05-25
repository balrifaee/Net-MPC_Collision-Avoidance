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

function startup
    CPLEX_PATH = 'C:\Program Files\IBM\ILOG\CPLEX_Studio1263\cplex\matlab\x64_win64';
    CPLEX_PATH = '..\..\..\trunk\CPLEX\matlab64'; % <- edit as necessary
    
    addpath(CPLEX_PATH);
    
    dirs = strsplit(genpath(pwd),';');
    for d=dirs
        if(isempty(strfind(d{1},'output')) && isempty(strfind(d{1},'slprj')))
            addpath(d{1});
        end
    end    
end

