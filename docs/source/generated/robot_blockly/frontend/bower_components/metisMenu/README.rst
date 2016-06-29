metisMenu |Build Status|
========================

    Easy menu jQuery plugin for Twitter Bootstrap 3

    Now support cdnjs & jsdelivr

Installation
------------

-  `npm <http://npmjs.org/>`__

.. code:: bash

    npm install metismenu

-  `Bower <http://bower.io>`__

.. code:: bash

    bower install metisMenu

-  `Download <https://github.com/onokumus/metisMenu/archive/master.zip>`__

Usage
-----

1. Include Twitter Bootstrap StyleSheet

   .. code:: html

       <link rel="stylesheet" href="//cdnjs.cloudflare.com/ajax/libs/twitter-bootstrap/3.3.0/css/bootstrap.min.css">

2. Include metisMenu StyleSheet

   .. code:: html

       <link rel="stylesheet" href="//cdn.jsdelivr.net/bootstrap.metismenu/1.1.2/css/metismenu.min.css">

3. Include jQuery

   .. code:: html

       <script src="//cdnjs.cloudflare.com/ajax/libs/jquery/2.1.1/jquery.min.js"></script>

4. Include Twitter Bootstrap Script

   .. code:: html

       <script src="//cdnjs.cloudflare.com/ajax/libs/twitter-bootstrap/3.3.0/js/bootstrap.min.js"></script>

5. Include metisMenu plugin's code

   .. code:: html

       <script src="//cdn.jsdelivr.net/bootstrap.metismenu/1.1.2/js/metismenu.min.js"></script>

6. Call the plugin:

   .. code:: javascript

       $("#menu").metisMenu();

Options
~~~~~~~

toggle
^^^^^^

Type: ``Boolean`` Default: ``true``

For auto collapse support.

.. code:: javascript

      $("#menu").metisMenu({
        toggle: false
      });

doubleTapToGo
^^^^^^^^^^^^^

Type: ``Boolean`` Default: ``false``

For double tap support.

.. code:: javascript

      $("#menu").metisMenu({
        doubleTapToGo: true
      });

`DEMO <http://demo.onokumus.com/metisMenu/>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Contains a simple HTML file to demonstrate metisMenu plugin.

Release History
~~~~~~~~~~~~~~~

**DATE** **VERSION** **CHANGES** \* 2014-11-01 v1.1.3 Bootstrap 3.3.0 \*
2014-07-07 v1.1.0 Add double tap functionality \* 2014-06-24 v1.0.3
cdnjs support & rename plugin \* 2014-06-18 v1.0.3 Create grunt task \*
2014-06-10 v1.0.2 Fixed for IE8 & IE9

Author
------

metisMenu was made with love by these guys and a bunch of awesome
`contributors <https://github.com/onokumus/metisMenu/graphs/contributors>`__.

|Osman Nuri Okumuş| \| --- \| --- \| --- \| --- \| --- \| --- \| ---
`Osman Nuri Okumuş <http://onokumus.com>`__ \|

License
-------

`MIT
License <https://github.com/onokumus/metisMenu/blob/master/LICENSE>`__

.. |Build Status| image:: https://secure.travis-ci.org/onokumus/metisMenu.png?branch=master
   :target: https://travis-ci.org/onokumus/metisMenu
.. |Osman Nuri Okumuş| image:: https://0.gravatar.com/avatar/4fa374411129d6f574c33e4753ec402e?s=70
   :target: http://onokumus.com
