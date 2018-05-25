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
% 
% This project is implemented by Bassam Alrifaee, from 
% the RWTH Aachen University, during his PhD thesis 
% titled "Networked Model Predictive Control for Vehicle Collision 
% Avoidance". Helpful contributions were made by the following students:
% Janis Maczijewski, Marwan Chawa, Mohamed Hetaba, Mostafa Nabil, 
% Kevin Kostyszyn, Mark Azer, Masoumeh G. Mamaghani
% 
% Also thanks to Arthur Richards' work which provided a starting point 
% for the initial implementation.: A. G. Richards and J. P. How, 
% "Aircraft Trajectory Planning with Collision Avoidance using Mixed 
% Integer Linear Programming" in Proceedings of the American Control 
% Conference, 2002. 
% 
% Users are requested to cite the following in any work 
% utilizing this software: 
% [1] B. Alrifaee. Networked Model Predictive Control for Vehicle 
% Collision Avoidance. PhD thesis, RWTH Aachen University, 2017. 
% [2] B. Alrifaee, F. J. He\sseler, and D. Abel. Coordinated 
% Non-Cooperative Distributed Model Predictive Control for Decoupled 
% Systems Using Graphs. In 6th IFAC Workshop on Distributed Estimation 
% and Control in Networked Systems NecSys 2016, Tokyo, Japan, September 2016. 
% [3]	B. Alrifaee, J. Maczijewski, and D. Abel. Sequential Convex 
% Programming MPC for Dynamic Vehicle Collision Avoidance. In 2017 IEEE 
% Conference on Control Technology and Applications (CCTA), pages 2202–2207, 
% Aug 2017.
% [4] B. Alrifaee, M. G. Mamaghani, and D. Abel. Centralized Non-Convex 
% Model Predictive Control for Cooperative Collision Avoidance of 
% Networked Vehicles. In Intelligent Control (ISIC), 2014 IEEE 
% International Symposium on, pages 1583-1588, Oct 2014. 
% 
% 
% Video of Experimental Results:
% https://youtu.be/X2syxG5GI6g
% 
% Video of the Simulation Results:
% https://youtu.be/zS3UBx09O6M
% 
% RWTH Aachen University:
% http://www.rwth-aachen.de/

function main
    close all;
    clc
    
    % Let user select the scenario and controller.
    [ scenarioConstructor, controller, controllerName ] = startOptionsUI();
    
    runSimulation(scenarioConstructor, controller, controllerName, true);
end