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

1. Load ROS into the shell environment, e.g. if you are using Foxy:

    ```sh
    source /opt/ros/foxy/setup.bash
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
|Eloquent     |`dashing`|
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

## Runtime Configuration

In addition to standard configuration facilities provided by the ROS2 RMW
interface, `rmw_connextdds`, and `rmw_connextddsmicro` support the additional
configuration of some aspects of their runtime behavior via custom environment
variables.

- [RMW_CONNEXT_CYCLONE_COMPATIBILITY_MODE](#RMW_CONNEXT_CYCLONE_COMPATIBILITY_MODE)
- [RMW_CONNEXT_DISABLE_FAST_ENDPOINT_DISCOVERY](#RMW_CONNEXT_DISABLE_FAST_ENDPOINT_DISCOVERY)
- [RMW_CONNEXT_DO_NOT_OVERRIDE_PUBLISH_MODE](#RMW_CONNEXT_DO_NOT_OVERRIDE_PUBLISH_MODE)
- [RMW_CONNEXT_INITIAL_PEER](#RMW_CONNEXT_INITIAL_PEER)
- [RMW_CONNEXT_OLD_RMW_COMPATIBILITY_MODE](#RMW_CONNEXT_OLD_RMW_COMPATIBILITY_MODE)
- [RMW_CONNEXT_REQUEST_REPLY_MAPPING](#RMW_CONNEXT_REQUEST_REPLY_MAPPING)
- [RMW_CONNEXT_UDP_INTERFACE](#RMW_CONNEXT_UDP_INTERFACE)

### RMW_CONNEXT_CYCLONE_COMPATIBILITY_MODE

Enable different policies to improve interoperability with `rmw_cyclonedds_cpp`.

By default, ROS2 applications using `rmw_connextdds` will be able to communicate
with those using `rmw_cyclonedds_cpp` only via ROS2 publishers and subscribers,
while ROS2 clients and services will not interoperate across vendors.

The reason for this incompatibility lies in `rmw_cyclonedds_cpp`'s use of a custom
mapping for propagating request metadata between clients and services.

When this "compatibility mode" is enabled, `rmw_connextdds` (and `rmw_connextddsmicro`)
will use this non-standard profile in order to interoperate with `rmw_cyclonedds_cpp`,
instead of using one the two standard profiles defined by the DDS-RPC specification
(see [RMW_CONNEXT_REQUEST_REPLY_MAPPING](#rmw_connext_request_reply_mapping)).

### RMW_CONNEXT_DISABLE_FAST_ENDPOINT_DISCOVERY

By default, `rmw_connextdds` modifies the QoS of its DomainParticipant to enable
the optimizations defined by RTI Connext DDS' built-in QoS snippet
`Optimization.Discovery.Endpoint.Fast`.

These optimizations speed up the discovery process between different applications
but they also introduce an overhead in network traffic, which might be undesirable
for larger systems.

Variable `RMW_CONNEXT_DISABLE_FAST_ENDPOINT_DISCOVERY` may be used to disable
these automatic optimizations, and to leave the DomainParticipant's QoS to
its defaults.

### RMW_CONNEXT_DO_NOT_OVERRIDE_PUBLISH_MODE

`rmw_connextdds` will always set `DDS_DataWriterQos::publish_mode::kind` of
any DataWriter it creates to `DDS_ASYNCHRONOUS_PUBLISH_MODE_QOS`, in order to
enable out of the box support for "large data".

This behavior might not be always desirable, and it can be disabled by setting
`RMW_CONNEXT_DO_NOT_OVERRIDE_PUBLISH_MODE` to a non-empty value.

This variable is not used by `rmw_connextddsmicro`, since it doesn't
automatically override `DDS_DataWriterQos::publish_mode::kind`.

### RMW_CONNEXT_INITIAL_PEER

`rmw_connextddsmicro` allows users to specify an initial peer for its
DomainParticipant via `RMW_CONNEXT_INITIAL_PEER`.

The value will be stored in `DDS_DomainParticipantQos::discovery::initial_peers`.

Note that by default, `rmw_connextddsmicro` will use the `lo` network interface,
which will prevent it from accessing the default DDS multicast peer (`239.255.0.1`).

This variable will also be honored by `rmw_connextdds`, although this is less
flexible than [other ways to configure initial peers](https://community.rti.com/static/documentation/connext-dds/6.0.1/doc/manuals/connext_dds/html_files/RTI_ConnextDDS_CoreLibraries_UsersManual/Content/UsersManual/ConfigPeersListUsed_inDiscov.htm)
that are already supported by RTI Connext DDS.

### RMW_CONNEXT_OLD_RMW_COMPATIBILITY_MODE

ROS2 applications using `rmw_connextdds` will not be able to interoperate with
applications using the previous RMW implementation for RTI Connext DDS, `rmw_connext_cpp`,
unless variable `RMW_CONNEXT_OLD_RMW_COMPATIBILITY_MODE` is used to enable
a "compatibility" mode with these older implementation.

In particular, when this mode is enabled, `rmw_connextdds` will revert to adding
a suffix (`_`) to the end of the names of the attributes of the ROS2 data types
propagated via DDS discovery.

### RMW_CONNEXT_REQUEST_REPLY_MAPPING

The [DDS-RPC specification](https://www.omg.org/spec/DDS-RPC/About-DDS-RPC/)
defines two profiles for mapping "request/reply" interactions over DDS messages
(e.g. ROS2 clients and services):

- the *basic* profile conveys information about the originator of a request as
  an inline payload, serialized before the actual request/reply payloads.

- The *extended* profile relies on DDS' metadata to convey request/reply
  information out of band.

By default, `rmw_connextdds` uses the *extended* profile when sending requests
from a ROS2 client to a service, while `rmw_connextddsmicro` uses the *basic* one.

Variable `RMW_CONNEXT_REQUEST_REPLY_MAPPING` can be used to select the actual
profile used at runtime. Either `"basic"` or `"extended"` may be specified.

At the moment, the *extended* profile is only available with `rmw_connextdds`.
In this configuration, `rmw_connextdds` will interoperate with `rmw_fastrtps_cpp`.

When using the *basic* profile, `rmw_connextdds` will interoperate with
`rmw_connextddsmicro`.

Use variable [RMW_CONNEXT_CYCLONE_COMPATIBILITY_MODE](#RMW_CONNEXT_CYCLONE_COMPATIBILITY_MODE)
to enable interoperability with `rmw_cyclonedds_cpp` using a non-standard version
of the *basic* profile.

### RMW_CONNEXT_UDP_INTERFACE

RTI Connext DDS Micro requires applications to explicitly configure the network
interface to use for UDPv4 communication.

`rmw_connextddsmicro` makes the arbitrary decision of using `lo` as the default
interface.

This is undesireable if non-local communication is required, and/or if the
default DDS multicast peer (`239.255.0.1`) is to be used.

Variable `RMW_CONNEXT_UDP_INTERFACE` may be used to customize the network interface
actually used by RTI Connext DDS Micro's UDPv4 transport (e.g. `"eth0"`).

This variable is not used by `rmw_connextdds`.
