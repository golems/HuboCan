HuboCan
=======

Complete framework for real-time operation of Hubo on Linux

This source code compiles into two libraries:

libHuboCan -- This is the central library to the HuboCan framework. Linking this library grants
access to all the key features available.

libHuboQt -- This library provides some Qt-widget and window interfaces for human interaction
with the HuboCan framework. These plugins are designed to be either run in their own windows
or made into plugins for any Qt-based framework.

There are five namespaces used in the HuboCan library:

| Namespace | Description | Class list |
|-----------|-------------|------------|
| HuboCan | This is the foundational namespace of the library which deals with low-level CAN protocol and hardware interface | CanPump, CanDevice, HuboDescription, HuboJmc, HuboJoint, HuboSensor |
| HuboCmd | This namespace deals with sending and receiving real-time control commands (i.e. commands which must be updated every control cycle) to the lowest layer of the framework | Commander, Aggregator, AuxSender, AuxReceiver |
| HuboState | This namespace is used to collect state data which gets published by the lowest level of the HuboCan framework, and can be used to synchronize a process with the low-level control cycle | State |
| HuboPath | This namespace provides tools for creating, interpolating, and running trajectories on Hubo (as well as interrupting and reversing trajectories while they are running) | Operator, Player, Trajectory |
| HuboRT (Real Time) | This namespace contains the tools which are used for creating real-time processes and managing those processes, as well as their Ach channels | Daemonizer, Manager, ManagerReq |
