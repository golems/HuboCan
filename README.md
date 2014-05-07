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
| HuboState | This namespace is used to collect state data which gets published by the hardware interface level of the HuboCan framework, and can be used to synchronize a process with the low-level control cycle | State |
| HuboPath | This namespace provides tools for creating, interpolating, and running trajectories on Hubo (as well as interrupting and reversing trajectories while they are running) | Operator, Player, Trajectory |
| HuboRT (Real Time) | This namespace contains the tools which are used for creating real-time processes and managing those processes, as well as their Ach channels | Daemonizer, Manager, ManagerReq |

### HuboQt Namespace

#### ManagerWidget

This widget is designed to expose the most basic features of the HuboRT::ManagerReq class. In other
words, it provides an interface to the Manager process that runs on Hubo, allowing you to
create/destroy Ach channels and launch/stop/kill processes remotely without needing to SSH. This
also provides a convenient interface for managing Ach Networking Daemons by simply clicking
Reconnect/Disconnect. This widget is intended to be the "front page" of any Qt-based operating
software for Hubo.

### HuboPath Namespace

#### Operator

The Operator class has three major roles: (1) Helping to construct a HuboPath::Trajectory, (2)
Sending that Trajectory to the HuboPath::Player in order to execute it on the robot, and (3) Sending
instructions to the Player to start, pause, or reverse its current trajectory.

#### Player

The Player class is designed to provide a framework for receiving, sanity-checking, and executing
trajectories. In the future this class will be upgraded to accept a real-time controller class
which will be able to modify trajectories on the fly based on sensor feedback.

#### Trajectory

The Trajectory class is a container class for handling trajectory data. Currently this only includes
jointspace waypoints and limit (position, velocity, and acceleration) checking, but in the future
will (hopefully) include control schemes, end effector waypoints, and self-collision checks.

Note: The Trajectory class has a very nice C++ stream operator (std::cout << ) which will print out
its contents in a clean and easy-to-read format.

### HuboCan Namespace

#### CanPump

The purpose of the CanPump class is to provide a skeleton for a real-time CAN-based hardware
interface. The CanPump class alone is not able to interface with hardware; rather, that is what the
SocketCanPump class (which inherits CanPump) is made for. But SocketCanPump is an implementation
specifically made for SocketCan. It is conceivable that not all Hubos (or any other robots which
wish to use this framework) will use SocketCan, so the CanPump class exists to provide all the key
timing functionality and connection to the rest of the HuboCan framework with no effort. Simply
inherit the CanPump class overload the virtual functions _send_frame() and _wait_on_frame() with
your hardware implementation.

#### CanDevice

The CanDevice class gives a blueprint for CanDevices to add to the CanPump. The idea is that a CAN
bus will have any number of devices attached to it (in our case, mostly joints and sensors). Each
of these devices needs a software class to send frames to (and decode frames from) those physical
devices. CanDevice is the blueprint for these software classes. At each control cycle, the CanPump
will trigger the update() function on each CanDevice which has been given to it. And every time a
CAN frame is received by the CanPump, it will call the decode() function on each CanDevice to give
them all the opportunity to read the incoming frames.

#### HuboJmc

The physical Hubo robot has JMCs (Joint Motor Controllers) which are connected to the CAN bus.
Each JMC is considered one device on the CAN bus (whether that JMC controls one, two, or more
joints). Different JMC types will have very different implementations, so the HuboJmc class simply
serves as a minimalist blueprint for these devices. The many different types of JMCs can be seen in
HuboCan/HuboJmc.hpp. This class may serve as a good example of a CanDevice implementation.

#### HuboSensor (Not yet fully implemented)

In addition to JMCs, the physical robot has sensor devices connected to CAN. HuboSensor represents
a minimalist blueprint for various sensor classes (i.e. HuboImu and HuboFt)

#### HuboDescription

This class is an [abstract factory](http://sourcemaking.com/design_patterns/abstract_factory/cpp/2).
It reads in a .dd (device description) file and churns out the appropriate CanDevices. When
implementing a new CanDevice, it is also important to update the HuboDescription class to be able
to correctly read (and transmit, if necessary) your device description (.dd).

It is acceptable to create your own class which inherits HuboDescription, but it would be preferable
to make changes directly to HuboDescription so that everyone can benefit from any new device
implementations.
