Logitech R400
=============

This is a driver for the Logitech R400 pointer.

I took some inspiration from this
`link <http://www.orangecoat.com/how-to/read-and-decode-data-from-your-mouse-using-this-pyusb-hack>`__

ROS communication
-----------------

This node simply publishes a **~state** topic of the type
**logitech\_r400/LogitechR400.msg**.

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
to access the Logitech R400 without sudo.

-  first make sure you're in the plugdev group.
-  then create a udev file:

::

    sudo nano /etc/udev/rules.d/89-logitech-r400.rules

-  Add the following udev rule to that file:

::

    ATTRS{idProduct}=="0111", ATTRS{idVendor}=="0e00", MODE="666", GROUP="plugdev"

or

::

    ATTRS{idProduct}=="c538", ATTRS{idVendor}=="046d", MODE="666", GROUP="plugdev"

(try both - which is succesfull depends on exactly what device you have)

-  Reload the rules without rebooting:

   ::

       sudo udevadm trigger


