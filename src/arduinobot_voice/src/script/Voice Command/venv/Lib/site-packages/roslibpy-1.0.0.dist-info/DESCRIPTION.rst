============================
roslibpy: ROS Bridge library
============================



**Python ROS Bridge library** allows to use Python and IronPython to interact
with `ROS <http://www.ros.org>`_, the open-source robotic middleware.
It uses WebSockets to connect to
`rosbridge 2.0 <http://wiki.ros.org/rosbridge_suite>`_ and provides publishing,
subscribing, service calls, actionlib, TF, and other essential ROS functionality.

Unlike the `rospy <http://wiki.ros.org/rospy>`_ library, this does not require a
local ROS environment, allowing usage from platforms other than Linux.

The API of **roslibpy** is modeled to closely match that of `roslibjs`_.


Main features
-------------

* Topic publishing and subscribing.
* Service calls (client).
* Service advertisement (server).
* ROS parameter management (get/set/delete).
* ROS API services for getting ROS meta-information.
* Actionlib support for interfacing with preemptable tasks.
* TF Client via the ``tf2_web_republisher``.

**Roslibpy** runs on Python 2.7 and 3.x and IronPython 2.7.


Installation
------------

To install **roslibpy**, simply use ``pip``::

    pip install roslibpy

For IronPython, the ``pip`` command is slightly different::

    ipy -X:Frames -m pip install --user roslibpy

Remember that you will need a working ROS setup including the
**rosbridge server** and **TF2 web republisher** accessible within your network.


Documentation
-------------

The full documentation, including examples and API reference
is available on `readthedocs <https://roslibpy.readthedocs.io/>`_.


Contributing
------------

Make sure you setup your local development environment correctly:

* Clone the `roslibpy <https://github.com/gramaziokohler/roslibpy>`_ repository.
* Create a virtual environment.
* Install development dependencies:

::

    pip install -r requirements-dev.txt

**You're ready to start coding!**

During development, use `pyinvoke <http://docs.pyinvoke.org/>`_ tasks on the
command prompt to ease recurring operations:

* ``invoke clean``: Clean all generated artifacts.
* ``invoke check``: Run various code and documentation style checks.
* ``invoke docs``: Generate documentation.
* ``invoke test``: Run all tests and checks in one swift command.
* ``invoke``: Show available tasks.

For more details, check the *Contributor's Guide* available as part of `the documentation <https://roslibpy.readthedocs.io/>`_.


Releasing this project
----------------------

Ready to release a new version **roslibpy**? Here's how to do it:

* We use `semver <http://semver.org/>`_, i.e. we bump versions as follows:

  * ``patch``: bugfixes.
  * ``minor``: backwards-compatible features added.
  * ``major``: backwards-incompatible changes.

* Update the ``CHANGELOG.rst`` with all novelty!
* Ready? Release everything in one command:

::

    invoke release [patch|minor|major]

* Profit!


Credits
-------

This library is based on `roslibjs`_ and to a
large extent, it is a line-by-line port to Python, changing only where a more
idiomatic form makes sense, so a huge part of the credit goes to the
`roslibjs authors <https://github.com/RobotWebTools/roslibjs/blob/develop/AUTHORS.md>`_.

.. _roslibjs: http://wiki.ros.org/roslibjs


Changelog
=========

All notable changes to this project will be documented in this file.

The format is based on `Keep a Changelog <http://keepachangelog.com/en/1.0.0/>`_
and this project adheres to `Semantic Versioning <http://semver.org/spec/v2.0.0.html>`_.

1.0.0
----------

**Changed**

* Changed behavior: Topics automatically reconnect when websockets is reconnected

**Added**

* Added blocking behavior to more ROS API methods: ``ros.get_nodes`` and ``ros.get_node_details``.
* Added reconnection support to IronPython implementation of websockets
* Added automatic topic reconnection support for both subscribers and publishers

**Fixed**

* Fixed reconnection issues on the Twisted/Autobahn-based implementation of websockets

0.7.1
----------

**Fixed**

* Fixed blocking service calls for Mac OS

0.7.0
----------

**Changed**

* The non-blocking event loop runner ``run()`` now defaults to 10 seconds timeout before raising an exception.

**Added**

* Added blocking behavior to ROS API methods, e.g. ``ros.get_topics``.
* Added command-line mode to ROS API, e.g. ``roslibpy topic list``.
* Added blocking behavior to the ``Param`` class.
* Added parameter manipulation methods to ``Ros`` class: ``get_param``, ``set_param``, ``delete_param``.

0.6.0
----------

**Changed**

* For consistency, ``timeout`` parameter of ``Goal.send()`` is now expressed in **seconds**, instead of milliseconds.

**Deprecated**

* The ``timeout`` parameter of ``ActionClient()`` is ignored in favor of blocking until the connection is established.

**Fixed**

* Raise exceptions when timeouts expire on ROS connection or service calls.

**Added**

* Support for calling a function in a thread from the Ros client.
* Added implementation of a Simple Action Server.

0.5.0
----------

**Changed**

* The non-blocking event loop runner now waits for the connection to be established in order to minimize the need for ``on_ready`` handlers.

**Added**

* Support blocking and non-blocking service calls.

**Fixed**

* Fixed an internal unsubscribing issue.

0.4.1
----------

**Fixed**

* Resolve reconnection issues.

0.4.0
----------

**Added**

* Add a non-blocking event loop runner

0.3.0
----------

**Changed**

* Unsubscribing from a listener no longer requires the original callback to be passed.

0.2.1
----------

**Fixed**

* Fix JSON serialization error on TF Client (on Python 3.x)

0.2.0
----------

**Added**

* Add support for IronPython 2.7

**Changed**

* Handler ``on_ready`` now defaults to run the callback in thread

**Deprecated**

* Rename ``run_event_loop`` to the more fitting ``run_forever``

0.1.1
----------

**Fixed**

* Minimal documentation fixes

0.1.0
----------

**Added**

* Initial version


