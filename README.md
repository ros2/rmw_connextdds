# ROS 2 Middleware Layer for RTI Connext DDS

This repository contains an implementation of the [ROS 2](https://index.ros.org/doc/ros2/)
RMW layer that supports both [RTI Connext DDS Professional](https://www.rti.com/products/connext-dds-professional)
and [RTI Connext DDS Micro](https://www.rti.com/products/connext-dds-micro).

The repository exposes two ROS RMW packages:

- [rmw_connextpro_cpp](#rmw_connextpro_cpp)

- [rmw_connextmicro_cpp](#rmw_connextmicro_cpp)

## rmw_connextpro_cpp

1. Make sure to have RTI Connext DDS Professional 6.x installed on your system.
   The install location must be configured using either `NDDSHOME`, or
   `CONNEXTDDS_DIR`. If you have the old RMW for RTI Connext DDS installed
   (`rmw_connext_cpp`), you should use `CONNEXTDDS_DIR` to override
   the value of `NDDSHOME` exported by that package.
   You can use the `set_env_<architecture>.bash`  script shipped with
   RTI Connext DDS Professional, and derive `CONNEXTDDS_DIR` from `NDDSHOME`.
   E.g. on a Linux sytem with RTI Connext DDS Professional 6.0.1:

   ```sh
    source ~/rti_connext_dds-6.0.1/resource/scripts/rtisetenv_x64Linux4gcc7.3.0.bash
    export CONNEXTDDS_DIR=${NDDSHOME}
   ```

2. Load your ROS environment, e.g.:

    ```sh
    source /opt/ros/foxy/setup.bash
    ```

3. Create an overlay directory and clone the RMW repository:

    ```sh
    mkdir -p ~/ros2_connextdds/src/ros2
    cd ~/ros2_connextdds
    git clone https://github.com/rticommunity/rmw_connextdds.git src/ros2/rmw_connextdds
    ```

4. Build the RMW. If you are using the "rolling" release, there is no need to
   pass additional arguments to cmake.

    ```sh
    colcon build --symlink-install --cmake-args -DRMW_CONNEXT_RELEASE=foxy
    ```

5. Source generated environment script:

    ```sh
    source ~/ros2_connextdds/install/setup.bash
    ```

6. Run ROS applications with RTI Connext DDS Professional:

    ```sh
    RMW_IMPLEMENTATION=rmw_connextpro_cpp ros2 run demo_nodes_cpp talker
    ```

## rmw_connextmicro_cpp

1. Make sure to have RTI Connext DDS Micro 3.x installed on your system.
   The install location can be either specified explicitely via `RTIMEHOME`,
   or it will be automatically detected from the installation of RTI Connext DDS
   Professional, if available (see [rmw_connextpro_cpp](#rmw_connextpro_cpp)
   section).

2. Load your ROS environment, e.g.:

    ```sh
    source /opt/ros/foxy/setup.bash
    ```

3. Create an overlay directory and clone the RMW repository:

    ```sh
    mkdir -p ~/ros2_connextdds/src/ros2
    cd ~/ros2_connextdds
    git clone https://github.com/rticommunity/rmw_connextdds.git src/ros2/rmw_connextdds
    ```

4. `rmw_connextmicro_cpp` requires some custom extensions to RTI Connext DDS
   Micro 3.x which are available from a separate repository. Please contact
   robotics@rti.com to request access to it. Once enabled, clone the repository
   in your overlay directory:

   ```sh
   cd ~/ros2_connextdds
   mkdir src/rti
   git clone https://github.com/rticommunity/rti_connext_dds_micro_ext.git src/rti/rti_connext_dds_micro_ext
   ```

5. Build the RMW. If you are using the "rolling" release, there is no need to
   pass additional arguments to cmake.

    ```sh
    colcon build --symlink-install --cmake-args -DRMW_CONNEXT_RELEASE=foxy
    ```

6. Source generated environment script:

    ```sh
    source ~/ros2_connextdds/install/setup.bash
    ```

7. Run ROS applications with RTI Connext DDS Micro:

    ```sh
    RMW_IMPLEMENTATION=rmw_connextmicro_cpp ros2 run demo_nodes_cpp talker
    ```


