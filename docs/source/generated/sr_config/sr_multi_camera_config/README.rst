Extrinsic Calibration of Multiple Cameras
=========================================

This package contains the bare-bones of what is necessary to use the
industrial\_extrinsic\_calibration stack to "automatically" calibrate a
scene involving multiple cameras. ## Initial Configuration Principally,
there are three things that need to be configured.

Target
~~~~~~

Frame
^^^^^

The location of the target is set via the link *target\_frame* and the
joint which connects it to the *world\_frame* in
**camera\_scene.xacro**. This file is found in the **urdf** directory of
this package. #### Setup **camera\_scene\_targets.yaml** in the **yaml**
directory of this package defines the target(s) we will use, and their
location within the *target\_frame*. The best place to look for these
are in
`industrial\_extrinsic\_cal/targets <https://github.com/ros-industrial/industrial_calibration/tree/indigo-devel/industrial_extrinsic_cal/targets>`__,
where you can find both the images from which to make the physical
targets, and their corresponding yaml descriptions.

Camera setup
~~~~~~~~~~~~

**camera\_scene\_cameras.yaml** in the **yaml** directory of this
package is where we define the cameras whose positions we will be
calibrating. The main thing is to make sure the images topics correspond
to the setup of the camera driver, and the frames correspond to those
defined in **camera\_scene.xacro**. At the moment, they're all called
"asus" as that's how they were named in the example.

Calibration Job
~~~~~~~~~~~~~~~

**camera\_scene\_caljob.yaml** is where we associate each camera with
the target with which it will be calibrated. If a camera can see
multiple targets, you can limit the range of interest (in pixels) so
that only the desired target is selected.

Calibrating
-----------

First launch you cameras.

``roslaunch sr_multi_camera_config cameras.launch``

will in its curernt form launch two of the standard Logitech web-cams we
use at shadow. Remeber to set the camera serial numbers to reflect the
particular cameras you have.

Then launch the calibration job -

``roslaunch sr_multi_camera_config camera_scene_cal.launch``

Finally, call the service with

``rosservice call /calibration_service "{allowable_cost_per_observation: 0.25}"``

The *allowable\_cost\_per\_observation* sets how accurate the
calibration must be to be accepted. 0.25 is the default, and it seems to
work well enough. Maybe we should experiment with this a bit.

The output of the calibration is a new yaml file called
**mutable\_joint\_states.yamlnew**. Rename this to just ".yaml" to use
the new calibration.

Using the Calibration
---------------------

``roslaunch sr_multi_camera_config publish_tf.launch``

will look at the xacro and relevant yaml files and publish the tfs
needed to use the new calibration.

Running ``gen_urdf.sh`` in the urdf directory will generate a fixed urdf
of the camera setup for using later.
