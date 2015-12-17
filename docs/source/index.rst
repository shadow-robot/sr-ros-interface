
.. toctree::
   :maxdepth: 3
   :hidden:

   self

.. toctree::
   :maxdepth: 3
   :hidden:

    Install Instructions <generated/shadow_robot/INSTALL>
    Starting the robots <generated/sr_interface/sr_robot_launch/README>
    High Level Interface <generated/sr_interface/sr_robot_commander/README>
    Examples <generated/sr_interface/sr_example/README>

Shadow Robot's Documentation
========================================

.. raw:: html

    <div style="position: relative; padding-bottom: 40px; height: 0; overflow: hidden; max-width: 100%; height: auto;">
        <iframe width="640" height="480" src="https://www.youtube.com/embed/3WAp_DHwg1c" frameborder="0" allowfullscreen></iframe>
    </div>

Overview
--------

This is the starting point for our core software packages built around
`our robots <http://www.shadowrobot.com>`__. We'll focus more on the
simulated side of things here (as you'll get a full training when
receiving one of our robots), but all the code that runs on our simulated
robot also runs on the real hardware.

Our code is split into different repositories:

 - `sr\_common <https://github.com/shadow-robot/sr_common>`__: This repository contains the bare minimum for communicating with the Shadow Hand from a remote computer (urdf models and messages).
 - `sr\_core <https://github.com/shadow-robot/sr_core>`__: These are the core packages for the Shadow Robot hardware and simulation.
 - `sr\_interface <https://github.com/shadow-robot/sr_interface>`__: This repository contains the high level interface and its dependencies for interacting simply with our robots.
 - `sr\_tools <https://github.com/shadow-robot/sr_tools>`__: This repository contains more advanced tools that might be needed in specific use cases.
 - `sr\_visualization <https://github.com/shadow-robot/sr-visualization>`__: This repository contains the various rqt_gui plugins we developed.
 - `sr\_config <https://github.com/shadow-robot/sr_config>`__: This repository contains the customer specific configuration for the Shadow Robot Hand.

Getting Started
---------------

Installation Instructions
~~~~~~~~~~~~~~~~~~~~~~~~~

To install our software please take a look at these `install steps <generated/shadow_robot/INSTALL.html>`__.

Running the software
~~~~~~~~~~~~~~~~~~~~

You can start our different robots (one hand - full hand, Lite, Extra Lite, Left, Right, ... -, two hands, hand and UR10, etc...) in both simulation and with the real hardware using the launch files in `sr_robot_launch <generated/sr_interface/sr_robot_launch/README.html>`__

Connecting to the robots
~~~~~~~~~~~~~~~~~~~~~~~~

Depending on your level of expertise with ROS and our software stack, as well as the use case there are two main ways to interface with our hand:

 - for a high level, low expertise access to our hand, please refer to the `robot commander interface <generated/sr_interface/sr_robot_commander/README.html>`__.
 - for power users, you can obviously access the different ROS interfaces directly.
