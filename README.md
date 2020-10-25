# ROS 2 Middleware Layer for RTI Connext DDS

This repository contains a novel implementation of the [ROS 2](https://index.ros.org/doc/ros2/)
RMW layer which enables ROS applciations to use [RTI Connext DDS Professional](https://www.rti.com/products/connext-dds-professional)
and and [RTI Connext DDS Micro](https://www.rti.com/products/connext-dds-micro).

The repository provides two RMW packages:

- `rmw_connextpro_cpp`

- `rmw_connextmicro_cpp`

For any question or feedback, please contact robotics@rti.com.

## Quick Start

1. Configure RTI Connext DDS Professional and/or RTI Connext DDS Micro on your
   system (see [Requirements](#requirements)). Make the installation(s)
   available via environment variables, e.g. by using the provided
   `set_env_<architecture>.bash` script:

   ```sh
    source ~/rti_connext_dds-6.0.1/resource/scripts/rtisetenv_x64Linux4gcc7.3.0.bash
    export CONNEXTDDS_DIR=${NDDSHOME}
    ```

2. Load ROS into the shell environment, e.g.:

    ```sh
    source /opt/ros/rolling/setup.bash
    ```

3. Create an overlay directory and clone the repository:

    ```sh
    mkdir -p ~/ros2_connextdds/src/ros2
    cd ~/ros2_connextdds
    git clone https://github.com/rticommunity/rmw_connextdds.git src/ros2/rmw_connextdds
    ```

4. Build the RMW:

    ```sh
    colcon build --symlink-install
    ```

5. Load the generated environment script:

    ```sh
    source ~/ros2_connextdds/install/setup.bash
    ```

6. Run ROS applications with RTI Connext DDS Professional:

    ```sh
    RMW_IMPLEMENTATION=rmw_connextpro_cpp ros2 run demo_nodes_cpp talker
    ```

7. Run ROS applications with RTI Connext DDS Micro:

    ```sh
    RMW_IMPLEMENTATION=rmw_connextmicro_cpp ros2 run demo_nodes_cpp talker
    ```

## Requirements

Both RMW packages required the appropriate version of RTI Connext DDS to be
available on your system.

`rmw_connextpro_cpp` requires RTI Connext DDS Professional version 6.0.0 or later,
while `rmw_connextmicro_cpp` requires RTI Connext DDS Micro version 3.0.0 or later.

The installations must be made available via environment variables. If no
valid installation is detected, the packages will be skipped and not be built.

|RMW|Product|Environment Variable|Default|
|---|-------|--------|-------|
|`rmw_connextpro_cpp`|RTI Connext DDS Professional 6.x|`CONNEXTDDS_DIR`, or `NDDSHOME`|None|
|`rmw_connextmicro_cpp`|RTI Connext DDS Micro 3.x |`RTIMEHOME`|Guessed from contents of RTI Connext DDS Professional installation.|

### OSRF RMW (rmw_connext_cpp)

Users of the old RMW by OSRF (`rmw_connext_cpp`) who are using the version of
RTI Connext DDS Professional 5.3.1 installed via `apt` should either:

- Uninstall RTI Connext DDS Professional 5.3.1 and the old RMW.

- Avoid pointing `${NDDSHOME}` to RTI Connext DDS Professional 5.3.1,
  e.g. by not sourcing `/opt/rti.com/rti_connext_dds-5.3.1/setenv_ros2rti.bash`.

  - This might not be sufficient since ROS environment hooks installed by the old
    RMW may override the variable and used the `apt` package version.

- Specify an installation of RTI Connext DDS Professional 6.x via `${CONNEXTDDS_DIR}`.

  - Package `rti_connext_dds_vendor` will first check `${CONNEXTDDS_DIR}` and then
    fall back to `${NDDSHOME}` to determine the libraries used by `rmw_connextpro_cpp`.

  - If you use the `set_env_<architecture>.bash` script after loading your ROS
    environment, you might not be able to use the old RMW in the same shell
    (unless you were already using it with RTI Connext DDS Professional 6.x).
