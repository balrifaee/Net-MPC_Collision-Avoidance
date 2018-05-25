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

function replay(result)

    close all;    
    currentStep = 1;
    
    function replot()
        currentStep = max(1,min(result.scenario.Nsim,currentStep));
        plotOnline(result,currentStep);
    end
    
    
    function keyPressCallback(~,eventdata)
        if strcmp(eventdata.Key, 'rightarrow')
            currentStep = currentStep + 1;
            replot();
        elseif strcmp(eventdata.Key, 'leftarrow')
            currentStep = currentStep - 1;
            replot();
        elseif strcmp(eventdata.Key, 'escape')
            close all;
        end
    end
    
    f=figure('units','normalized','outerposition',[.05 .05 .9 .9]);
    set(f,'WindowKeyPressFcn',@keyPressCallback);
    replot();
    
end