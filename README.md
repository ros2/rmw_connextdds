# ROS 2 Middleware Layer for RTI Connext DDS

This repository contains an implementation of the [ROS 2](https://index.ros.org/doc/ros2/)
RMW layer that supports both [RTI Connext DDS Professional](https://www.rti.com/products/connext-dds-professional)
and [RTI Connext DDS Micro](https://www.rti.com/products/connext-dds-micro).

The repository exposes two ROS RMW packages:

- `rmw_connextpro_cpp`

- `rmw_connextmicro_cpp`

## Quick Start

1. Make sure to have RTI Connext DDS Professional and/or RTI Connext DDS Micro
   installed on your system, and their install location configured via
   variables `CONNEXTDDS_DIR` and `RTIMEHOME`. You can use the `set_env_*` 
   script provided by RTI Connext DDS Professional, and derive `RTIMEHOME`
   from `CONNEXTDDS_DIR`. E.g. on a sytem with RTI Connext DDS Professional 6.0.1
   and RTI Connext DDS Micro 3.0.3 installed at default locations:

   ```sh
    source ~/rti_connext_dds-6.0.1/resource/scripts/rtisetenv_x64Linux4gcc7.3.0.bash
    export CONNEXTDDS_DIR=${NDDSHOME}
    export RTIMEHOME=${NDDSHOME}/rti_connext_dds_micro-3.0.3
   ```

2. Create an overlay directory and clone the RMW repository:

    ```sh
    mkdir -p ~/ros2_connextdds/src
    cd ~/ros2_connextdds
    git clone https://github.com/rticommunity/rmw_connextdds.git src/rmw_connextdds
    ```

3. Build packages with Rolling (binary) release:

    ```sh
    source /opt/ros/rolling/setup.bash
    colcon build --symlink-install
    ```

4. Alternatively, build packages with Foxy (binary) release:

    ```sh
    source /opt/ros/foxy/setup.bash
    colcon build --symlink-install --cmake-args -DRMW_CONNEXT_RELEASE=foxy
    ```

5. Source generated environment script. Make sure to source the
   Connext DDS script again if you have the old Connext RMW installed,
   otherwise the wrong version of Connext DDS libraries might be loaded
   (i.e. 5.3.1):

    ```sh
    source ~/ros2_connextdds/install/setup.bash
    source ~/rti_connext_dds-6.0.1/resource/scripts/rtisetenv_x64Linux4gcc7.3.0.bash
    ```

6. Run ROS applications with RTI Connext DDS Professional:

    ```sh
    RMW_IMPLEMENTATION=rmw_connextpro_cpp ros2 run demo_nodes_cpp talker
    ```


6. Run ROS applications with RTI Connext DDS Micro:

    ```sh
    RMW_IMPLEMENTATION=rmw_connextmicro_cpp ros2 run demo_nodes_py listener
    ```

## Installation

The RMW implementation can be built as part of the [standard ROS build](https://index.ros.org/doc/ros2/Installation/Rolling/#building-from-source).

The code is developed against the ROS "rolling" release, but it should also work with the latest stable release (Foxy).

Only Linux has been tested at the moment (Darwin might work too).

### RTI Connext DDS Installation

The installation of RTI Connext DDS Professional (v6.0.0+) can be 
specified via variables `CONNEXTDDS_DIR`, or `NDDSHOME`. These
variables can be either set in the shell environment, or
specified as explicit arguments to `cmake`.
If specified, `CONNEXTDDS_DIR` takes precedence over `NDDSHOME`,
and so do `cmake` arguments over environment variables.

The installation of RTI Connext DDS Micro (v3.0.0+) can be specified
via variable `RTIMEHOME`. If this variable is not set, then it
defaults to `${NDDSHOME}/rti_connext_dds_micro-3.0.3`.

### Installation from scratch

If you are installing ROS from scratch, follow the [instructions to build the "rolling" release](https://index.ros.org/doc/ros2/Installation/Rolling/#building-from-source).

During the "Get ROS 2 code" step, add the following entry to the end 
of `ros2.repos`, before running `vcs import`:

```yaml
src/ros2/rmw_connextdds:
    type: git
    url: https://bitbucket.rti.com/scm/~asorbini/rmw_connextdds.git
    version: master
```

In order to build `rmw_connextpro_cpp`, make sure to configure 
`CONNEXTDDS_DIR` or `NDDSHOME` (as explained in [RTI Connext DDS Installation](#rti-connext-dds-installation)).

### Existing installation

If you already have ROS installed, clone this repository anywhere
(e.g. in your ROS workspace), then source your ROS installation and
build it with `colcon`:

```bash
source install/setup.bash

git clone https://bitbucket.rti.com/scm/~asorbini/rmw_connextdds.git src/ros2/rmw_connextdds

colcon build --merge-install
```

## Running ROS with RTI Connext DDS

You can select your desired RMW implementation by setting variable
`RMW_IMPLEMENTATION` to either `rmw_connextmicro_cpp`, or `rmw_connextpro_cpp`, e.g.:

```bash
# Run C++ demo talker with RTI Connext DDS Micro
RMW_IMPLEMENTATION=rmw_connextmicro_cpp ros2 run demo_nodes_cpp talker

# Run Python demo listener with RTI Connext DDS Professional
RMW_IMPLEMENTATION=rmw_connextpro_cpp ros2 run demo_nodes_py listener
```

