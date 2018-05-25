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

% Let the user choose a scenario and controller.
function [ scenarioConstructor, controller, controllerName ] = startOptionsUI()

    % ====== load previous choice ======
    try
        load([tempdir 'scenarioControllerSelection']);
    end
    if 1 ~= exist('scenarioSelection','var') || ~isinteger(scenarioSelection)
        scenarioSelection = int32(1);
    end
    if 1 ~= exist('controllerSelection','var') || ~isinteger(controllerSelection)
        controllerSelection = int32(1);
    end
    if 1 ~= exist('strategySelection','var') || ~isinteger(strategySelection)
        strategySelection = int32(1);
    end

    % ============= scenario ============
    scenarios = list_scenarios();
    scenarioSelection = max(1,min(size(scenarios,1),scenarioSelection));    

    [scenarioSelection,ok] = listdlg(...
        'ListString',{scenarios{:,1}}, ...
        'SelectionMode', 'single', ...
        'InitialValue', scenarioSelection, ...
        'ListSize', [300,400], ...
        'PromptString', 'Choose a scenario');
    
    if ~ok 
        error('Canceled');
    end
    
    scenarioConstructor = scenarios{scenarioSelection,2};
    
    
    % ============= controller ============
    controllers = list_controllers();
    controllerSelection = max(1,min(size(controllers,1),controllerSelection));
    
    [controllerSelection,ok] = listdlg(...
        'ListString',{controllers{:,1}}, ...
        'SelectionMode', 'single', ...
        'InitialValue', controllerSelection, ...
        'ListSize', [300,400], ...
        'PromptString', 'Choose a controller');
    
    if ~ok 
        error('Canceled');
    end
    
    controller = controllers{controllerSelection,2};
    controllerName = controllers{controllerSelection,1};    
    
    
    
    % ============= strategy ============
    
    strategies = list_strategies;
    strategySelection = max(1,min(size(strategies,1),strategySelection));
    
    [strategySelection,ok] = listdlg(...
        'ListString',{strategies{:,1}}, ...
        'SelectionMode', 'single', ...
        'InitialValue', strategySelection, ...
        'ListSize', [300,400], ...
        'PromptString', 'Choose a strategy');
    
    if ~ok 
        error('Canceled');
    end
    
    controllerModifier = strategies{strategySelection,2};
    controller = controllerModifier(controller);
    controllerName = [controllerName ' ' strategies{strategySelection,1}];
    
    % ============= save choice ============    
    scenarioSelection = int32(scenarioSelection);
    controllerSelection = int32(controllerSelection);
    strategySelection = int32(strategySelection);
    save([tempdir 'scenarioControllerSelection'], 'scenarioSelection', 'controllerSelection', 'strategySelection');
end
