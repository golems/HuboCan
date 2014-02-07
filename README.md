HuboCan
=======

Low-level CAN interface for real-time Hubo operation on Linux


This package consists of five libraries

HuboCan: Handles interfacing between software and CAN hardware

HuboState: Handles the collection and distribution of state information

HuboCmd: Provides the API for sending control commands

HuboAgg: Provides an interface for aggregating commands which are sent by users

HuboRT: Provides an interface for daemonizing and managing a process, as well as handling real-timeness


The executable delivered with this package is called hubo_interface which makes use of the HuboCan, HuboState, and HuboAgg libraries in order to facilitate a closed control loop with the CAN hardware.

For anyone interested in simulation, simply make use of the HuboAgg library and the HuboState library in order to create a closed simulation loop which operates off of the same inputs and outputs as the standard HuboCan.
