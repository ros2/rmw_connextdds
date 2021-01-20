# ROS 2 Middleware Layer for RTI Connext DDS

This repository contains two novel implementations of the [ROS 2](https://index.ros.org/doc/ros2/)
RMW layer which allow developers to deploy their ROS applications on top of
[RTI Connext DDS Professional](https://www.rti.com/products/connext-dds-professional)
and [RTI Connext DDS Micro](https://www.rti.com/products/connext-dds-micro).

The repository provides two RMW packages:

- `rmw_connextdds`

- `rmw_connextddsmicro`

Package `rmw_connextdds` is meant to be a replacement for [`rmw_connext_cpp`](https://github.com/ros2/rmw_connext).
This new implementation resolves several performance issues, and it improves out-of-the-box
interoperability with DDS applications.

*The repository is undergoing stabilization, with some features still in
active development.
Please consider reporting any [issue](https://github.com/rticommunity/rmw_connextdds/issues)
that you may experience, while monitoring the repository for frequent updates.*

For any questions or feedback, feel free to reach out to robotics@rti.com.

## Quick Start

1. Load ROS into the shell environment, e.g.:

    ```sh
    source /opt/ros/dashing/setup.bash
    ```

2. Configure RTI Connext DDS Professional and/or RTI Connext DDS Micro on your
   system (see [Requirements](#rti-connext-dds-requirements)). Make the installation(s)
   available via environment variables, e.g. by using the provided
   `rtisetenv_<architecture>.bash` script (replace `~/rti_connext_dds-6.0.1` with
   the path of your Connext installation):

   ```sh
    source ~/rti_connext_dds-6.0.1/resource/scripts/rtisetenv_x64Linux4gcc7.3.0.bash
    export CONNEXTDDS_DIR=${NDDSHOME}
    ```

3. Create an overlay directory and clone the repository:

    ```sh
    mkdir -p ~/ros2_connextdds/src/ros2
    cd ~/ros2_connextdds
    git clone -b dashing https://github.com/rticommunity/rmw_connextdds.git src/ros2/rmw_connextdds
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

## Support for different ROS 2 Releases

`rmw_connextdds`, and `rmw_connextddsmicro` support multiple versions of ROS 2.

The following table summarizes which branch of the repository should be
checked out in order to compile the RMW implementations for a specific ROS 2
release:

|ROS 2 Release|Branch|
|-------------|------|
|Rolling      |`master`|
|Foxy         |`master`|
|Dashing      |`dashing`|

## RTI Connext DDS Requirements

Both RMW packages require the appropriate version of RTI Connext DDS to be
available on the build and target systems.

`rmw_connextdds` requires RTI Connext DDS Professional (version 5.3.1 or later),
while `rmw_connextddsmicro` requires RTI Connext DDS Micro (version 3 or later).

The installations must be made available via environment variables. If no
valid installation is detected, the packages will be skipped and not be built.

|RMW|RTI Product|Environment Variable(s)|Required|Default|
|---|-----------|-----------------------|--------|-------|
|`rmw_connextdds`|RTI Connext DDS Professional 5.3.1, or 6.x|`CONNEXTDDS_DIR`, or `NDDSHOME`|Yes|None|
|`rmw_connextddsmicro`|RTI Connext DDS Micro 3.x |`RTIMEHOME`|No (if RTI Connext DDS Professional 6.x is available)|Guessed from contents of RTI Connext DDS Professional installation (6.x only, 5.3.1 users must specify `RTIMEHOME`).|

### Multiple versions of RTI Connext DDS Professional

Package `rti_connext_dds_cmake_module` will first check variable
`${CONNEXTDDS_DIR}`, and then fall back to `${NDDSHOME}` to determine the
location of the RTI Connext DDS Professional libraries used by
`rmw_connextdds`

This behavior allows users of the old Connext RMW (`rmw_connext_cpp`) who
have installed RTI Connext DDS Professional 5.3.1 via the `apt` package
`rti-connext-dds-5.3.1`, to have both that version, and a more recent one
(e.g. 6.0.1) installed on their system, but configured via different variables.

If `rmw_connext_cpp` is installed via debian package
`ros-<version>-rmw-connext-cpp`, variable `${NDDSHOME}` will always be
hard-coded to the install location of the `apt` package
(`/opt/rti.com/rti_connext_dds-5.3.1`).

In this case, you can use `${CONNEXTDDS_DIR}` to point to a Connext 6.x
installation, making sure to source script
`rti_connext_dds-6.x.x/resource/scripts/rtisetenv_<architecture>.bash` after
loading your ROS installation, so that the Connext 6.x libraries and paths will
be found first in the relevant environement variables (e.g. `${LD_LIBRARY_PATH}`).

If you encounter any errors with selecting your desired Connext installation,
consider uninstalling `rmw_connext_cpp` and `connext_cmake_module`
(e.g. `sudo apt remove ros-<version>-rmw-connext-cpp ros-<version>-connext-cmake-module`).
