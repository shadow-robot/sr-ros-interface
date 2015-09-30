.. Shadow Robot documentation master file, created by
   sphinx-quickstart on Wed Sep 30 09:25:50 2015.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

Shadow Robot's Documentation
========================================

Contents:

.. toctree::
   :maxdepth: 2

   Install Instructions <generated/shadow_robot/INSTALL.rst>

Overview
--------

This is the starting point for our core software packages built around
`our robots <http://www.shadowrobot.com>`__. We'll focus more on the
simulated side of things here (as you'll get a full training when
receiving one of our robot), but all the code that runs on our simulated
robot also runs on the real hardware.

Our code is split into different repositories:
 - `sr\_common <https://github.com/shadow-robot/sr_common>`__: This repository contains the bare minimum for communicating with the Shadow Hand from a remote computer (urdf models and messages).
 - `sr\_core <https://github.com/shadow-robot/sr_core>`__: These are the core packages for the Shadow Robot hardware and simulation.
 - `sr\_interface <https://github.com/shadow-robot/sr_interface>`__: This repository contains the high level interface and its dependencies for interacting simply with our robots.
 - `sr\_tools <https://github.com/shadow-robot/sr_tools>`__: This repository contains more advanced tools that might be needed in specific
use cases.
 - `sr\_visualization <https://github.com/shadow-robot/sr-visualization>`__: This repository contains the various rqt_gui plugins we developed.

Getting Started
---------------

Installation Instructions
~~~~~~~~~~~~~~~~~~~~~~~~~

We'll assume that you've already installed ROS Indigo from `those
instructions <http://wiki.ros.org/indigo/Installation/Ubuntu>`__ using
the recommended proposed solution.

To install our software please take a look at those `install steps <INSTALL.md>`__.

Running the software
~~~~~~~~~~~~~~~~~~~~

See sr_robot_launch