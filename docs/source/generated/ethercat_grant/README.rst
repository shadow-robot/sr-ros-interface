ethercat\_grant
===============

This is a repo for the pr2-grant utility. It makes it possible to run
the *ros\_ethercat\_loop* without requiring root privileges.

Installation
------------

If you installed this package from *apt-get* then you're good to go.
(This is definitely the recommended way)

If you compiled this package from source, then you'll need to copy the
resulting ``ethercat_grant`` executable to ``/usr/local/bin`` and add
the sticky bit to it: ``sudo chmod +s /usr/local/bin/ethercat_grant``.

Use
---

Using ethercat\_grant makes it possible to not use *sudo* anymore for
running the ethercat main loop. Just use
``launch-prefix="ethercat_grant"`` in your launch files for the
*ros\_ethercat\_loop*.

When releasing this package (developers only)
---------------------------------------------

If we change the ``scripts/postinst`` post installation script, it has
to be manually copied, commited and pushed to the branch with name
``debian/ROS_DISTRO/ethercat_grant`` inside the debian directory (where
ROS\_DISTRO is one of hydro, indigo...)

e.g.
https://github.com/shadow-robot/ethercat\_grant-release/tree/debian/indigo/ethercat\_grant/debian

See
http://answers.ros.org/question/191779/add-postinstall-rule-for-deb-package-creation/
