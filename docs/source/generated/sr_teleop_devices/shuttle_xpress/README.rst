Shuttle Xpress
==============

This is a driver for the Contour Shuttle Xpress controller.

I took some inspiration from this
`link <http://www.orangecoat.com/how-to/read-and-decode-data-from-your-mouse-using-this-pyusb-hack>`__

ROS communication
-----------------

This node simply publishes a **~state** topic of the type
**shuttle\_xpress/ShuttleExpress.msg**.

Install
-------

Proper pyusb version
~~~~~~~~~~~~~~~~~~~~

*TODO use rosdep instead* You'll need pyusb 1.0+ for this driver to
work. To install it:

::

    sudo pip install --upgrade --pre pyusb

Accessing without sudo
~~~~~~~~~~~~~~~~~~~~~~

I found those informations in the following
`wiki <http://www.tincantools.com/wiki/Accessing_Devices_without_Sudo>`__
to access the Shuttle Xpress without sudo.

-  first make sure you're in the plugdev group.
-  then create a udev file:

::

    sudo nano /etc/udev/rules.d/89-shuttle-xpress.rules

-  Add the following udev rule to that file:

::

    ATTRS{idProduct}=="0020", ATTRS{idVendor}=="0b33", MODE="666", GROUP="plugdev"

-  Reload the rules without rebooting:

   ::

       sudo udevadm trigger


