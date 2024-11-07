# TendonForces
This repository is a validated forcing module that can be used in the open-source simulation software PyElastica. This forcing module applies the external forcing effects of tendon actuation on a rod. The validation of this forcing module was done through physical experimentation, results which can be seen here: (INCLUDE RESULTS)

To use this forcing module, you must first be sure to have installed PyElastica properly (follow the procedure in their repo https://github.com/GazzolaLab/PyElastica)

TendonForces is a forcing module that must be imported to the simulation script in order to be used. This can be done by either importing TendonForces into the external_forces.py script that is installed by default when installing PyElastica, editing external_forces.py by copying and pasting the whole TendonForces class in it, or directly importing TendonForces into the simulation script (though for sake of order, it is preferred to keep forcing modules in the external_forces.py script).

**NOTE: the examples provided in this repository assume that the TendonForces class is defined inside external_forces.py.**

Now, when building the simulator, the TendonForces forcing module must be called and supplied with the tendon configuration parameters. This can be seen in the example provided.

The tendon configuration parameters (arguments for the initialization of TendonForces) are the following:

**vertebra_height: (float)**
        Height at which the tendon contacts the vertebra. It should be the highest point on the tendon-vertebra space.
        
**num_vertebrae: (int)**
        Amount of vertebrae to be used in the system.
        
**first_vertebra_node: (int)**
        The first node to have a vertebra, from the base of the rod to the tip.
        
**final_vertebra_node: (int)**
        The last node to have a vertebra, from the base of the rod to the tip.
        
**vertebra_mass: (float)**
        Total mass of a single vertebra.
        
**tension: (float)**
        Tension applied to the tendon in the system.
        
**vertebra_height_orientation: (numpy.ndarray)**
        1D (dim) numpy array. Describes the orientatation of the vertebrae in the system.
        
**n_elements: (int)**
        Total amount of nodes in the rod system. This value is set in the simulator and is copied to this class for later use.

The structure of TendonForces is such that it is allowed to use several different tendon configurations simultaneously in one simulation, and because tension is one of the arguments provided, the application of the tendon actuation is always done in an open-loop.

