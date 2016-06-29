`Bootstrap <http://getbootstrap.com>`__
=======================================

|Bower version| |npm version| |Build Status| |devDependency Status|
|Selenium Test Status|

Bootstrap is a sleek, intuitive, and powerful front-end framework for
faster and easier web development, created by `Mark
Otto <https://twitter.com/mdo>`__ and `Jacob
Thornton <https://twitter.com/fat>`__, and maintained by the `core
team <https://github.com/orgs/twbs/people>`__ with the massive support
and involvement of the community.

To get started, check out http://getbootstrap.com!

Table of contents
-----------------

-  `Quick start <#quick-start>`__
-  `Bugs and feature requests <#bugs-and-feature-requests>`__
-  `Documentation <#documentation>`__
-  `Contributing <#contributing>`__
-  `Community <#community>`__
-  `Versioning <#versioning>`__
-  `Creators <#creators>`__
-  `Copyright and license <#copyright-and-license>`__

Quick start
-----------

Five quick start options are available:

-  `Download the latest
   release <https://github.com/twbs/bootstrap/archive/v3.3.4.zip>`__.
-  Clone the repo: ``git clone https://github.com/twbs/bootstrap.git``.
-  Install with `Bower <http://bower.io>`__:
   ``bower install bootstrap``.
-  Install with `npm <https://www.npmjs.com>`__:
   ``npm install bootstrap``.
-  Install with `Meteor <https://www.meteor.com/>`__:
   ``meteor add twbs:bootstrap``.

Read the `Getting started
page <http://getbootstrap.com/getting-started/>`__ for information on
the framework contents, templates and examples, and more.

What's included
~~~~~~~~~~~~~~~

Within the download you'll find the following directories and files,
logically grouping common assets and providing both compiled and
minified variations. You'll see something like this:

::

    bootstrap/
    ├── css/
    │   ├── bootstrap.css
    │   ├── bootstrap.css.map
    │   ├── bootstrap.min.css
    │   ├── bootstrap-theme.css
    │   ├── bootstrap-theme.css.map
    │   └── bootstrap-theme.min.css
    ├── js/
    │   ├── bootstrap.js
    │   └── bootstrap.min.js
    └── fonts/
        ├── glyphicons-halflings-regular.eot
        ├── glyphicons-halflings-regular.svg
        ├── glyphicons-halflings-regular.ttf
        ├── glyphicons-halflings-regular.woff
        └── glyphicons-halflings-regular.woff2

We provide compiled CSS and JS (``bootstrap.*``), as well as compiled
and minified CSS and JS (``bootstrap.min.*``). CSS `source
maps <https://developers.google.com/chrome-developer-tools/docs/css-preprocessors>`__
(``bootstrap.*.map``) are available for use with certain browsers'
developer tools. Fonts from Glyphicons are included, as is the optional
Bootstrap theme.

Bugs and feature requests
-------------------------

Have a bug or a feature request? Please first read the `issue
guidelines <https://github.com/twbs/bootstrap/blob/master/CONTRIBUTING.md#using-the-issue-tracker>`__
and search for existing and closed issues. If your problem or idea is
not addressed yet, `please open a new
issue <https://github.com/twbs/bootstrap/issues/new>`__.

Documentation
-------------

Bootstrap's documentation, included in this repo in the root directory,
is built with `Jekyll <http://jekyllrb.com>`__ and publicly hosted on
GitHub Pages at http://getbootstrap.com. The docs may also be run
locally.

Running documentation locally
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

1. If necessary, `install
   Jekyll <http://jekyllrb.com/docs/installation>`__ (requires v2.5.x).

-  **Windows users:** Read `this unofficial
   guide <http://jekyll-windows.juthilo.com/>`__ to get Jekyll up and
   running without problems.

2. Install the Ruby-based syntax highlighter,
   `Rouge <https://github.com/jneen/rouge>`__, with
   ``gem install rouge``.
3. From the root ``/bootstrap`` directory, run ``jekyll serve`` in the
   command line.
4. Open http://localhost:9001 in your browser, and voilà.

Learn more about using Jekyll by reading its
`documentation <http://jekyllrb.com/docs/home/>`__.

Documentation for previous releases
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Documentation for v2.3.2 has been made available for the time being at
http://getbootstrap.com/2.3.2/ while folks transition to Bootstrap 3.

`Previous releases <https://github.com/twbs/bootstrap/releases>`__ and
their documentation are also available for download.

Contributing
------------

Please read through our `contributing
guidelines <https://github.com/twbs/bootstrap/blob/master/CONTRIBUTING.md>`__.
Included are directions for opening issues, coding standards, and notes
on development.

Moreover, if your pull request contains JavaScript patches or features,
you must include `relevant unit
tests <https://github.com/twbs/bootstrap/tree/master/js/tests>`__. All
HTML and CSS should conform to the `Code
Guide <https://github.com/mdo/code-guide>`__, maintained by `Mark
Otto <https://github.com/mdo>`__.

Editor preferences are available in the `editor
config <https://github.com/twbs/bootstrap/blob/master/.editorconfig>`__
for easy use in common text editors. Read more and download plugins at
http://editorconfig.org.

Community
---------

Keep track of development and community news.

-  Follow [@getbootstrap on Twitter](https://twitter.com/getbootstrap).
-  Read and subscribe to `The Official Bootstrap
   Blog <http://blog.getbootstrap.com>`__.
-  Chat with fellow Bootstrappers in IRC. On the ``irc.freenode.net``
   server, in the ``##bootstrap`` channel.
-  Implementation help may be found at Stack Overflow (tagged
   ```twitter-bootstrap-3`` <http://stackoverflow.com/questions/tagged/twitter-bootstrap-3>`__).
-  Developers should use the keyword ``bootstrap`` on packages which
   modify or add to the functionality of Bootstrap when distributing
   through `npm <https://www.npmjs.com/browse/keyword/bootstrap>`__ or
   similar delivery mechanisms for maximum discoverability.

Versioning
----------

For transparency into our release cycle and in striving to maintain
backward compatibility, Bootstrap is maintained under `the Semantic
Versioning guidelines <http://semver.org/>`__. Sometimes we screw up,
but we'll adhere to those rules whenever possible.

Creators
--------

**Mark Otto**

-  https://twitter.com/mdo
-  https://github.com/mdo

**Jacob Thornton**

-  https://twitter.com/fat
-  https://github.com/fat

Copyright and license
---------------------

Code and documentation copyright 2011-2015 Twitter, Inc. Code released
under `the MIT
license <https://github.com/twbs/bootstrap/blob/master/LICENSE>`__. Docs
released under `Creative
Commons <https://github.com/twbs/bootstrap/blob/master/docs/LICENSE>`__.

.. |Bower version| image:: https://img.shields.io/bower/v/bootstrap.svg?style=flat
.. |npm version| image:: https://img.shields.io/npm/v/bootstrap.svg?style=flat
   :target: https://www.npmjs.com/package/bootstrap
.. |Build Status| image:: https://img.shields.io/travis/twbs/bootstrap/master.svg?style=flat
   :target: https://travis-ci.org/twbs/bootstrap
.. |devDependency Status| image:: https://img.shields.io/david/dev/twbs/bootstrap.svg?style=flat
   :target: https://david-dm.org/twbs/bootstrap#info=devDependencies
.. |Selenium Test Status| image:: https://saucelabs.com/browser-matrix/bootstrap.svg
   :target: https://saucelabs.com/u/bootstrap
