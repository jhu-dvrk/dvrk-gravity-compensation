Change log
==========

2.0.0 (2021-04-08)
==================

* API changes:
  * None
* Deprecated features:
  * None
* New features:
  * Support for latest dVRK ROS topics and CRTK based dvrk matlab client over ROS
* Bug fixes:
  * Delete `arm` when done to avoid having subscriber callbacks in background

1.0.1 (2019-07-04)
==================

* API changes:
  * None
* Deprecated features:
  * None
* New features:
  * Better support for older Matlab
* Bug fixes:
  * Fixed joint velocity abs bug in GC controller: https://github.com/jhu-dvrk/sawIntuitiveResearchKit/pull/117

1.0.0 (2019-04-09)
==================

* API changes:
  * First release!
* Deprecated features:
  * None
* New features:
  * Matlab tools to collect data from MTM and identify gravity compensation parameters for each arm
  * Requires dVRK stack 1.7.0 released 2019-04-09
  * GC JSON configuration file is using format version 2
* Bug fixes:
  * None
