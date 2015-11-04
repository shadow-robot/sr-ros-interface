**sr\_remappers** is used to remap data coming from different hardware
to the Shadow Hardware. It also provides apps to remap data coming from
the Shadow Hardware to different hardware.

Generic principle for the Remapper
----------------------------------

The generic principle for those remappers is as follow:

-  subscribe to a given topic, get the vectors of incoming data.
-  multiply these vectors as they come by a mapping matrix => remap one
   (or more) input data to one (or more) output data.
-  publish the generated vector to a new topic.

Cyberglove Remapper
-------------------

This is a remapper used to remap coming from a cyberglove node to a
shadowhand node. It allows the user to control the Shadow Robot Dextrous
Hand with a Cyberglove from Immersion. The cyberglove node can be found
in the shadow\_robot stack as well. There's a tool in sr\_control\_gui
to generate an optimal mapping matrix for a given user in a few steps.

You can specify different parameters in the launch file
remapper\_glove.launch:

-  cyberglove\_prefix: set the prefix from which the data are coming.
-  sendupdate\_prefix: set the prefix to which the remapped data will be
   published.
-  cyberglove\_mapping\_path: the path to the mapping matrix.

Code API
--------

-  The CalibrationParser class is taking care of parsing the calibration
   matrices and multiplying the input vector to compute the remapped
   vectors.
-  shadowhand\_to\_cyberglove\_remapper::ShadowhandToCybergloveRemapper
   is where the subscribe / publish are done for the Cyberglove.

