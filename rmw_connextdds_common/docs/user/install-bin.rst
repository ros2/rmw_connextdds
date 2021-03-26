.. _user-install-bin:

*******************
Binary installation
*******************

.. _user-install-bin-quick:

Binary quick-start
==================

These instructions are for Ubuntu 20.04 running on an x86 64-bit host.

1. Install your desired ROS 2 distributions, e.g. to install Rolling:
      
   .. code-block:: sh
      
       # Download and trust the ROS packaging master key.
       sudo apt update && sudo apt install curl gnupg2 lsb-release
       curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

       # Add the ROS 2 repository to your apt repositories.
       sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'

       # Update package database.
       sudo apt update

       # Use package ros-rolling-ros-base for a more minimal installation.
       sudo apt install ros-rolling-desktop
   
   .. note::
       Refer to the `ROS 2 documentation <https://docs.ros.org/en/rolling/Installation.html>`_
       for more installation options.

2. Install ``rmw_connextdds``, e.g. for Rolling:

   .. code-block:: sh

       sudo apt install ros-rolling-rmw-connextdds

3. Run ROS 2 applications with RTI Connext DDS Professional 5.3.1, e.g.:

   .. code-block:: sh
    
        RMW_IMPLEMENTATION=rmw_connextdds ros2 run demo_nodes_cpp talker


Supported ROS 2 Releases
========================

This method of installation is only available for ``rmw_connextdds``, and only
for ROS 2 releases starting with Galactic. It can also be used by users of the
Rolling distribution.

At the moment, binary packages are only available for Ubuntu 20.04 x86 64-bit.

``rmw_connextdds`` must be built from source on all other platforms (e.g.
aarch64), operating systems (other Linux distributions, macOS, and Windows),
and previous ROS 2 releases (starting with Dashing). See :ref:`user-install-src`
for more information.

``rmw_connextddsmicro`` must always be built from source.

.. warning::
    The binary version of ``rmw_connextdds`` is built with RTI Connext DDS
    Professional 5.3.1.
    You must build it from source in order to use it with a more recent version
    (e.g. 6.0.1).
