Stopwatch node
==============

Provides a simple interface for measuring algorithms runtime duration.


Usage
-----

Stopwatch node includes two interfaces and a service which saves gathered measurements.

Tic toc
~~~~~~~

First interface consists of three services: ``createClock``, ``tic`` and ``toc``.

- ``createClock`` takes one parameter: ``description`` and returns ``id`` of created clock
- ``tic`` takes ``id`` as parameter denoting clock id and starts the clock
- ``toc`` also takes ``id`` as its only argument and stops corresponding clock

Automatic tracking
~~~~~~~~~~~~~~~~~~

Call ``stopwatch/registerPair`` with two topic names, queue size and a description.
It is advised to register topics on node creation, to easily meet the requirements.

Saving results
~~~~~~~~~~~~~~

``stopwatch/saveRecords`` has to be called in order to save records gathered from all clocks.
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

   // interesting event happens here

   stopwatch::tocService toc_srv;
   toc_srv.request.id = clock_id;
   toc_client.call(toc_srv);


