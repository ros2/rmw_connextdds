# ROS 2 Middleware Layer for RTI Connext DDS

This repository contains two novel implementations of the [ROS 2](https://index.ros.org/doc/ros2/)
RMW layer which enable developers to build their ROS applications with [RTI Connext DDS Professional](https://www.rti.com/products/connext-dds-professional)
and [RTI Connext DDS Micro](https://www.rti.com/products/connext-dds-micro).

The repository provides two RMW packages:

- `rmw_connextdds`

- `rmw_connextddsmicro`

For any question or feedback, please contact robotics@rti.com.

## Quick Start

1. Configure RTI Connext DDS Professional and/or RTI Connext DDS Micro on your
   system (see [Requirements](#requirements)). Make the installation(s)
   available via environment variables, e.g. by using the provided
   `rtisetenv_<architecture>.bash` script:

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
    git -b dashing clone https://github.com/rticommunity/rmw_connextdds.git src/ros2/rmw_connextdds
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
    RMW_IMPLEMENTATION=rmw_connextdds ros2 run demo_nodes_cpp talker
    ```

7. Run ROS applications with RTI Connext DDS Micro:

    ```sh
    RMW_IMPLEMENTATION=rmw_connextddsmicro ros2 run demo_nodes_cpp talker
    ```

## Requirements

Both RMW packages require the appropriate version of RTI Connext DDS to be
available on the build and target systems.

`rmw_connextdds` requires RTI Connext DDS Professional (version 6 or later),
while `rmw_connextddsmicro` requires RTI Connext DDS Micro (version 3 or later).

The installations must be made available via environment variables. If no
valid installation is detected, the packages will be skipped and not be built.

|RMW|RTI Product|Environment Variable(s)|Required|Default|
|---|-----------|-----------------------|--------|-------|
|`rmw_connextdds`|RTI Connext DDS Professional 6.x|`CONNEXTDDS_DIR`, or `NDDSHOME`|Yes|None|
|`rmw_connextddsmicro`|RTI Connext DDS Micro 3.x |`RTIMEHOME`|No (if RTI Connext DDS Professional 6.x is available)|Guessed from contents of RTI Connext DDS Professional installation.|

### Compatibility with rmw_connext_cpp

Users of the old RMW developed by OSRF (`rmw_connext_cpp`) might be using
RTI Connext DDS Professional 5.3.1 installed via `apt`, in which
case they should either:

- Uninstall RTI Connext DDS Professional 5.3.1 with `apt`, and manually install
  RTI Connext DDS Professional 6 or later.

- Avoid pointing `${NDDSHOME}` to RTI Connext DDS Professional 5.3.1,
  e.g. by not sourcing `/opt/rti.com/rti_connext_dds-5.3.1/setenv_ros2rti.bash`.

  - This might not be sufficient since ROS environment hooks installed by
    `connext_cmake_module` (used by `rmw_connext-cpp`) may override the
    variable and cause the packaged version to be used.

- Specify an installation of RTI Connext DDS Professional 6 or later via
  `${CONNEXTDDS_DIR}`.

  - Package `rti_connext_dds_vendor` will first check `${CONNEXTDDS_DIR}` and then
    fall back to `${NDDSHOME}` to determine the libraries used by `rmw_connextdds`.

If the `rtisetenv_<architecture>.bash` script provided by RTI Connext DDS Professional
is used after loading the ROS environment, `rmw_connext_cpp` might not be usable
within in the same shell (unless it was already configured to use RTI Connext
DDS Professional 6.x) since the script modifies `${PATH}` and `${LD_LIBRARY_PATH}`
(or `${DYLD_LIBRARY_PATH}`, on Darwin).
