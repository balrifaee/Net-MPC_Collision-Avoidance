This project is implemented by Bassam Alrifaee, from 
the RWTH Aachen University, during his PhD thesis 
titled "Networked Model Predictive Control for Vehicle Collision 
Avoidance". Helpful contributions were made by the following students:
Janis Maczijewski, Marwan Chawa, Mohamed Hetaba, Mostafa Nabil, 
Kevin Kostyszyn, Mark Azer, Masoumeh G. Mamaghani

Also thanks to Arthur Richards' work which provided a starting point 
for the initial implementation.: A. G. Richards and J. P. How, 
"Aircraft Trajectory Planning with Collision Avoidance using Mixed 
Integer Linear Programming" in Proceedings of the American Control 
Conference, 2002.

Users are requested to cite the following in any work 
utilizing this software:
# This MATLAB simulation
[1] B. Alrifaee. MATLAB Simulation of Networked Model Predictive Control 
for Vehicle Collision Avoidance, May 2017. https://doi.org/10.5281/zenodo.1252992
# PhD thesis
[2] B. Alrifaee. Networked Model Predictive Control for Vehicle 
Collision Avoidance. PhD thesis, RWTH Aachen University, 2017. 
# Distributed MPC
[3] B. Alrifaee, F. J. He\sseler, and D. Abel. Coordinated 
Non-Cooperative Distributed Model Predictive Control for Decoupled 
Systems Using Graphs. In 6th IFAC Workshop on Distributed Estimation 
and Control in Networked Systems NecSys 2016, Tokyo, Japan, September 2016. 
# Optimization
[4] B. Alrifaee, J. Maczijewski, and D. Abel. Sequential Convex 
Programming MPC for Dynamic Vehicle Collision Avoidance. In 2017 IEEE 
Conference on Control Technology and Applications (CCTA), pages 2202–2207, 
Aug 2017.
[5] B. Alrifaee, M. G. Mamaghani, and D. Abel. Centralized Non-Convex 
Model Predictive Control for Cooperative Collision Avoidance of 
Networked Vehicles. In Intelligent Control (ISIC), 2014 IEEE 
International Symposium on, pages 1583-1588, Oct 2014. 


Video of Experimental Results:
https://youtu.be/X2syxG5GI6g
 
Video of the Simulation Results:
https://youtu.be/zS3UBx09O6M
 
RWTH Aachen University:
http://www.rwth-aachen.de/


# MATLAB code (Tested with version R2016a)

# Dependencies
    # CPLEX (Tested with version 12.6.3)
        * Download and install IBM ILOG CPLEX Optimization Studio.
		* CPLEX is free for academics, search for "IBM Academic Initiative"
        * Set the path in 'startup.m'.

# Usage
    # startup.m
        Run the startup to setup libraries and paths.

    # main.m
        If you run 'main.m' you will be prompted to select a scenario and controller.
        The simulation will start and show a live plot at each simulation step.
        The simulation result will be saved in 'output/<scenarioName>.<controllerName>/'.
        You can press <space> to pause and continue the simulation or <escape> to abort.







# Controller Interface
	All controllers have a common interface:
    [U,trajectoryPrediction,controllerOutput] = controller(scenario,iter,previousOutput)
    
    * U
        The predicted steering angles for all prediction steps and vehicles, matrix(Hp, nVeh).
    * trajectoryPrediction
        The predicted trejectory of all vehicles for all prediction steps, matrix(Hp,ny,nVeh).
    * controllerOutput
        A struct with optional outputs.
    
    * scenario
        A struct describing the scenario. For more see 'defaultScenario()'.
    * iter
        Time-variant controller inputs (current vehicle/obstacle states, reference trajectories, etc).
    * previousOutput
        The 'controllerOutput' from the previous iteration. Useful for initializing
        an optimizer with the last result. Empty on the first iteration.
