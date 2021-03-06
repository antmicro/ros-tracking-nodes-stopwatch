Stopwatch
=========

Copyright (c) 2020-2021 `Antmicro <https://www.antmicro.com>`_

To see how this repository can be used, see the `ros-tracking-nodes-policy-examples repository <https://github.com/antmicro/ros-tracking-nodes-policy-examples>`_.
Stopwatch provides a simple interface for measuring the algorithms runtime duration.

Usage
-----

The Stopwatch node includes two interfaces and a service which saves the gathered measurements.

Tic toc
~~~~~~~

The first interface consists of three services: ``createClock``, ``tic`` and ``toc``.

- ``createClock`` takes one parameter: ``description`` and returns ``id`` of created clock
- ``tic`` takes ``id`` as parameter denoting clock id and starts the clock
- ``toc`` also takes ``id`` as its only argument and stops the corresponding clock

Automatic tracking
~~~~~~~~~~~~~~~~~~

Call ``stopwatch/registerPair`` with two topic names, queue size and description.
It is advised to register topics on node creation to easily meet the requirements.

Saving results
~~~~~~~~~~~~~~

``stopwatch/saveRecords`` has to be called in order to save the records gathered from all clocks.
It takes one argument - path to output ``.csv`` file.

Examples
~~~~~~~~

.. code-block:: python

   from stopwatch.srv import registerPairService
   rospy.wait_for_service('stopwatch/registerPair')
   registerPair = rospy.ServiceProxy('stopwatch/registerPair', registerPairService)
   registerPair('fairmot_ros/image_input', 'fairmot_ros/bboxes', 1, 'fairmot_ros_track')

.. code-block:: c++

   #include <stopwatch/ticService.h>
   #include <stopwatch/tocService.h>
   #include <stopwatch/newClockService.h>

   unsigned long clock_id;
   ros::ServiceClient toc_client;
   ros::ServiceClient tic_client;
   ros::ServiceClient new_client;
   
   // setup
   ros::NodeHandle nh;
   toc_client = nh.serviceClient<stopwatch::tocService>("stopwatch/toc");
   tic_client = nh.serviceClient<stopwatch::ticService>("stopwatch/tic");
   new_client = nh.serviceClient<stopwatch::newClockService>("stopwatch/newClock");
   stopwatch::newClockService new_srv;
   new_srv.request.description = "yolo_detect";
   new_client.call(new_srv);
   clock_id = new_srv.response.id;

   stopwatch::ticService tic_srv;
   tic_srv.request.id = clock_id;
   tic_client.call(tic_srv); 

   // an interesting event happens here

   stopwatch::tocService toc_srv;
   toc_srv.request.id = clock_id;
   toc_client.call(toc_srv);


