.. _user-install-src:

************************
Installation from source
************************

.. _user-install-src-quick:

Source quick-start
==================

1. Install ROS 2 on your system. Refer to the `ROS 2 documentation <https://docs.ros.org/en/rolling/Installation.html>`_
   for available options.


2. Load ROS 2 into the shell environment, e.g. if you installed Foxy using the
   binary packages available for Ubuntu:

   .. code-block:: sh
   
       source /opt/ros/foxy/setup.bash

2. Install RTI Connext DDS Professional and load it in your environment, e.g.
   using ``NDDSHOME``:

   .. code-block:: sh
   
       export NDDSHOME=~/rti_connext_dds-6.0.1

   You can also use variable ``CONNEXTDDS_DIR`` instead of ``NDDSHOME``.
   ``CONNEXTDDS_DIR`` will take precedence if set.

   Skip this step if you don't want to build ``rmw_connextdds`` (e.g. if you
   only want to build ``rmw_connextddsmicro``).

   .. note::
       ``rmw_connextdds`` will only be compiled if RTI Connext DDS Professtional
       5.3.1 or later is installed and configured in the environment.

       Replace the example path with your local Connext installation.

       See :ref:`user-install-req-dds-pro` for more information about
       installing and configuring RTI Connext DDS Professional.

3. Optionally, install and load RTI Connext DDS Micro in your environment, e.g.:

   .. code-block:: sh
   
       export RTIMEHOME=~/rti_connext_dds_micro-3.0.3
  
   Often, ``RTIMEHOME`` can be automatically guessed from the installation
   of RTI Connext DDS Professional.

   This step is only necessary if you are using an "exported" installation
   of RTI Connext DDS Micro 3.x outside of an RTI Connext DDS 6.x
   installation, or if you are planning on building ``rmw_connextddsmicro``
   only (thus you are not loading RTI Connext DDS Professional in your
   environment).
  
   .. note::
       ``rmw_connextddsmicro`` will only be compiled if RTI Connext DDS Micro
       3.0.3 or later is installed and configured in the environment.

       Replace the example path with the location of your Micro installation.
       See :ref:`user-install-req-dds-micro` for more information about
       installing and configuring RTI Connext DDS Micro.

4. Create an overlay directory and clone the repository:

   .. code-block:: sh
   
       mkdir -p ~/ros2_connextdds/src/ros2
       cd ~/ros2_connextdds
       git clone https://github.com/ros2/rmw_connextdds.git src/ros2/rmw_connextdds

5. Build the RMW:

   .. code-block:: sh

       colcon build

6. Load the generated environment script:

   .. code-block:: sh

       source ~/ros2_connextdds/install/setup.bash

7. Run ROS 2 applications with RTI Connext DDS Professional by setting
   ``RMW_IMPLEMENTATION=rmw_connextdds``, e.g.:

   .. code-block:: sh

       RMW_IMPLEMENTATION=rmw_connextdds ros2 run demo_nodes_cpp talker

8. Run ROS 2 applications with RTI Connext DDS Micro by setting
   ``RMW_IMPLEMENTATION=rmw_connextddsmicro``, e.g.:

   .. code-block:: sh

       RMW_IMPLEMENTATION=rmw_connextddsmicro \
       RMW_CONNEXT_INITIAL_PEERS=_shmem:// \
         ros2 run demo_nodes_cpp listener

Supported ROS 2 Releases
========================

Each versions of ROS 2 supported by ``rmw_connextdds`` and
``rmw_connextddsmicro`` is stored in a dedicated branch of
`ros2/rmw_connextdds <https://github.com/ros2/rmw_connextdds>`_.

The following table summarizes the available branches, and the level of
support offered for each ROS 2 release:

+-------------+------------+--------------+
|ROS 2 Release|Branch      |Status        |
+=============+============+==============+
|Rolling      |``master``  |Developed     |
+-------------+------------+--------------+
|Foxy         |``foxy``    |LTS (May 2023)|
+-------------+------------+--------------+
|Eloquent     |``eloquent``|EOL (Nov 2020)|
+-------------+------------+--------------+
|Dashing      |``dashing`` |LTS (May 2021)|
+-------------+------------+--------------+

Branch ``master`` is actively developed and maintained. It is used to create
other branches for specific ROS 2 releases (starting from Galactic).

Branches marked as ``LTS`` will receive updates for critical bug fixes and
important patches only (until they reach ``EOL``).

Branches marked as ``EOL`` will not receive any more updates.
