# ROS 2 Middleware Layer for RTI Connext DDS

This repository contains two novel implementations of the [ROS 2](https://docs.ros.org/en/rolling)
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

## Table of Contents

- [Quick Start](#quick-start)
- [Support for different ROS 2 Releases](#support-for-different-ros-2-releases)
- [RTI Connext DDS Requirements](#rti-connext-dds-requirements)
  - [Multiple versions of RTI Connext DDS Professional](#multiple-versions-of-rti-connext-dds-professional)
- [Runtime Configuration](#runtime-configuration)
  - [RMW_CONNEXT_CYCLONE_COMPATIBILITY_MODE](#rmw-connext-cyclone-compatibility-mode)
  - [RMW_CONNEXT_DISABLE_LARGE_DATA_OPTIMIZATIONS](#rmw-connext-disable-large-data-optimizations)
  - [RMW_CONNEXT_DISABLE_FAST_ENDPOINT_DISCOVERY](#rmw-connext-disable-fast-endpoint-discovery)
  - [RMW_CONNEXT_ENDPOINT_QOS_OVERRIDE_POLICY](#rmw-connext-endpoint-qos-override-policy)
  - [RMW_CONNEXT_INITIAL_PEERS](#rmw-connext-initial-peers)
  - [RMW_CONNEXT_LEGACY_RMW_COMPATIBILITY_MODE](#rmw-connext-legacy-rmw-compatibility-mode)
  - [RMW_CONNEXT_PARTICIPANT_QOS_OVERRIDE_POLICY](#rmw-connext-participant-qos-override-policy)
  - [RMW_CONNEXT_REQUEST_REPLY_MAPPING](#rmw-connext-request-reply-mapping)
  - [RMW_CONNEXT_UDP_INTERFACE](#rmw-connext-udp-interface)
  - [RMW_CONNEXT_USE_DEFAULT_PUBLISH_MODE](#rmw-connext-use-default-publish-mode)
- [Quality of Service Configuration](#quality-of-service-configuration)
  - [ROS topic mangling conventions](#ros-topic-mangling-conventions)

## Quick Start

1. Load ROS into the shell environment (Rolling if using the `master` branch,
   see [Support for different ROS 2 Releases](#support-for-different-ros-2-releases))

    ```sh
    source /opt/ros/rolling/setup.bash
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

|ROS 2 Release|Branch|Status|
|-------------|------|------|
|Rolling      |`master`|Developed|
|Foxy         |`foxy`|LTS (May 2023)|
|Eloquent     |`eloquent`|EOL (Nov 2020)|
|Dashing      |`dashing`|LTS (May 2021)|

Branch `master` is actively developed and maintained. It is used to create
other branches for specific ROS 2 releases (starting from Galactic).

Branches marked as `LTS` will receive updates for critical bug fixes and
important patches only (until they reach `EOL`).

Branches marked as `EOL` will not receive any future updates.

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
variables:

- [RMW_CONNEXT_CYCLONE_COMPATIBILITY_MODE](#RMW_CONNEXT_CYCLONE_COMPATIBILITY_MODE)
- [RMW_CONNEXT_DISABLE_LARGE_DATA_OPTIMIZATIONS](#RMW_CONNEXT_DISABLE_LARGE_DATA_OPTIMIZATIONS)
- [RMW_CONNEXT_DISABLE_FAST_ENDPOINT_DISCOVERY](#RMW_CONNEXT_DISABLE_FAST_ENDPOINT_DISCOVERY)
- [RMW_CONNEXT_ENDPOINT_QOS_OVERRIDE_POLICY](#RMW_CONNEXT_ENDPOINT_QOS_OVERRIDE_POLICY)
- [RMW_CONNEXT_INITIAL_PEERS](#RMW_CONNEXT_INITIAL_PEERS)
- [RMW_CONNEXT_LEGACY_RMW_COMPATIBILITY_MODE](#RMW_CONNEXT_LEGACY_RMW_COMPATIBILITY_MODE)
- [RMW_CONNEXT_PARTICIPANT_QOS_OVERRIDE_POLICY](#RMW_CONNEXT_PARTICIPANT_QOS_OVERRIDE_POLICY)
- [RMW_CONNEXT_REQUEST_REPLY_MAPPING](#RMW_CONNEXT_REQUEST_REPLY_MAPPING)
- [RMW_CONNEXT_UDP_INTERFACE](#RMW_CONNEXT_UDP_INTERFACE)
- [RMW_CONNEXT_USE_DEFAULT_PUBLISH_MODE](#RMW_CONNEXT_USE_DEFAULT_PUBLISH_MODE)

See also [Quality of Service Configuration](#quality-of-service-configuration) for information on how to customize
the Quality of Service settings using XML files.

### RMW_CONNEXT_CYCLONE_COMPATIBILITY_MODE

*Enable different policies to improve interoperability with `rmw_cyclonedds_cpp`.*

#### Examples

```sh
# Start a `demo_nodes_cpp/talker` with `rmw_connextdds` in Cyclone compatibility mode
RMW_IMPLEMENTATION=rmw_connextdds \
RMW_CONNEXT_CYCLONE_COMPATIBILITY_MODE=y \
ros2 run demo_nodes_cpp talker &

# Query node parameters using `rmw_cyclonedds_cpp`
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
ros2 param list /talker
```

#### Description

By default, ROS2 applications using `rmw_connextdds` will be able to communicate
with those using `rmw_cyclonedds_cpp` only via ROS2 publishers and subscribers,
while ROS2 clients and services will not interoperate across vendors.

The reason for this incompatibility lies in `rmw_cyclonedds_cpp`'s use of a custom
mapping for propagating request metadata between clients and services.

When this "compatibility mode" is enabled, `rmw_connextdds` (and `rmw_connextddsmicro`)
will use this non-standard profile in order to interoperate with `rmw_cyclonedds_cpp`,
instead of using one the two standard profiles defined by the DDS-RPC specification
(see [RMW_CONNEXT_REQUEST_REPLY_MAPPING](#rmw_connext_request_reply_mapping)).

### RMW_CONNEXT_DISABLE_LARGE_DATA_OPTIMIZATIONS

*Disable automatic QoS optimizations applied by `rmw_connextdds` to endpoints using a data type with a serialized size that exceeds the transport's MTU.*

#### Examples

```sh
# Disable "large data" optimizations for `my_package/my_process`
RMW_IMPLEMENTATION=rmw_connextdds \
RMW_CONNEXT_DISABLE_LARGE_DATA_OPTIMIZATIONS=y \
ros2 run my_package my_process
```

#### Description

By default, `rmw_connextdds` will try to detect the use of "large data" types,
and automatically optimize the QoS of DDS DataWriters and DataReaders
using these types, to improve out of the box performance on reliable streams.

`rmw_connextdds` will modify a "large data" endpoint's RTPS reliability
protocol parameters to more quickly recover samples, which typically improves
performance in the presence of very fragmented data, but it might also
end up increasing network traffic unnecessarily, particularly if data is not
exchanged at a fast periodic pace.

Variable `RMW_CONNEXT_DISABLE_LARGE_DATA_OPTIMIZATIONS` may be used to disable
these automatic optimizations, and revert to Connext's default behavior.

Note that these optimizations will only be applied to any endpoint whose type has a
*static* maximum serialized size of at least 1MB. "Unbounded" types (i.e. types
which include at least one variable-length fields without a maximum) will most likely
not be included in these optimizations, since only their "bounded" part will
be taken into account when computing the maximum serialized size. This will generally
end up being a pretty small value (equivalent to having all 0-length "unbounded" fields),
and thus it will typically cause the types not to qualify as "large data" for the purpose
of these optimizations (even though applications may be exchanging "large data"
with them at runtime).

In this case, you should provide custom QoS configuration to manually apply similar
optimizations on the relevant endpoints. See [Quality of Service Configuration](#quality-of-service-configuration)
for more information on how to customize QoS for specific endpoints.

### RMW_CONNEXT_DISABLE_FAST_ENDPOINT_DISCOVERY

*Disable automatic QoS optimizations applied by `rmw_connextdds` to the DomainParticipant in order to speed up discovery of remote DDS endpoints.*

#### Examples

```sh
# Disable "fast endpoint discovery" optimizations for `my_package/my_process`
RMW_IMPLEMENTATION=rmw_connextdds \
RMW_CONNEXT_DISABLE_FAST_ENDPOINT_DISCOVERY=y \
ros2 run my_package my_process
```

#### Description

By default, `rmw_connextdds` modifies the QoS of its DomainParticipant to enable
the optimizations defined by RTI Connext DDS' built-in QoS snippet
`Optimization.Discovery.Endpoint.Fast`.

These optimizations speed up the discovery process between different applications
but they also introduce an overhead in network traffic, which might be undesirable
for larger systems.

Variable `RMW_CONNEXT_DISABLE_FAST_ENDPOINT_DISCOVERY` may be used to disable
these automatic optimizations, and to leave the DomainParticipant's QoS to
its defaults.

### RMW_CONNEXT_ENDPOINT_QOS_OVERRIDE_POLICY

*Control how `rmw_connextdds` overrides a DDS endpoint's default QoS with ROS 2 QoS settings.*

#### Examples

```sh
# Load a custom default QoS profile ("my_application::my_process").
cp rmw_connextdds/resource/xml/USER_QOS_PROFILES.example.xml \
   USER_QOS_PROFILES.xml

# Use ROS 2 QoS on top of default DDS QoS (default behavior if unspecified)
RMW_IMPLEMENTATION=rmw_connextdds \
RMW_CONNEXT_ENDPOINT_QOS_OVERRIDE_POLICY=always \
ros2 run demo_nodes_cpp talker

# Disable ROS 2 QoS for all endpoints and use only the default DDS QoS.
RMW_IMPLEMENTATION=rmw_connextdds \
RMW_CONNEXT_ENDPOINT_QOS_OVERRIDE_POLICY=never \
ros2 run demo_nodes_cpp talker

# Disable ROS 2 QoS only for endpoints on topic "chatter"
RMW_IMPLEMENTATION=rmw_connextdds \
RMW_CONNEXT_ENDPOINT_QOS_OVERRIDE_POLICY="dds_topics: rt/chatter" \
ros2 run demo_nodes_cpp talker
```

These examples load a custom default QoS profile from [USER_QOS_PROFILES.example.xml](rmw_connextdds/resource/xml/USER_QOS_PROFILES.example.xml)
by copying the file to `./USER_QOS_PROFILES.xml`. The same can also be achieved
by specifying `USER_QOS_PROFILES.example.xml`'s path in variable `NDDS_QOS_PROFILES`.

See [Quality of Service Configuration](#quality-of-service-configuration) for more information on how to load QoS profiles.

#### Description

When `RMW_CONNEXT_ENDPOINT_QOS_OVERRIDE_POLICY` is not set, or if it is set to `always`,
in order to determine the QoS used by an endpoint, `rmw_connextdds` will first load Connext's
default QoS profile for the specific type of endpoint (DataWriter or DataReader), and for
the endpoint's own topic (using the "mangled" topic name). The ROS 2 QoS profile will then
be applied on top of these values.

In this mode, any ROS 2 QoS policy not set to `SYSTEM_DEFAULT` will always overwrite
the corresponding DDS QoS policy value obtained from default QoS. See [Quality of Service Configuration](#quality-of-service-configuration)
for more information on how to modify the default QoS used by `rmw_connextdds`
for each endpoint.

When `RMW_CONNEXT_ENDPOINT_QOS_OVERRIDE_POLICY` is set to `never`, the default QoS settings
returned by Connext for an endpoint with be used "as is", and the ROS 2 QoS profile will be
completely ignored.

This policy value will affect *all* endpoints created by `rmw_connextdds`, including
the internal ones that are automatically created for RMW management (`ros_discovery_info`),
and those created by `rcl` for each Node (e.g. `rt/rosout`, `rt/parameter_events`,
and other topics used to manage node parameters). In order to make sure that these
endpoints will continue to communicate with applications using the default QoS, you
may use QoS profile `ros2::rcl.builtin_endpoints` from [ros2_qos_profiles.xml](rmw_connextdds/resource/xml/ros2_qos_profiles.xml)
to configure all of these "built-in endpoints" with the correct QoS settings.

`RMW_CONNEXT_ENDPOINT_QOS_OVERRIDE_POLICY` may also be set to `dds_topics: <regex>`, e.g.:
 `dds_topics: rt/my_topic|rt/my_ns/another_topic`.

In that case, the QoS settings for topics matching the provided regex will be
loaded in the same way as the `never` policy, and the ones that don't match
will be loaded in the same way as the `always` policy.

Be aware that `RMW_CONNEXT_ENDPOINT_QOS_OVERRIDE_POLICY` only controls how
`rmw_connextdds` applies the ROS 2 QoS profiles on top of the default DDS QoS profile,
but it will not affect other QoS optimizations that are automatically
applied by `rmw_connextdds` based on the properties of an endpoint. These
optmizations will be applied even when the `never` policy is selected, and they must
be explicitly disabled using their own dedicated environment variables:

- [RMW_CONNEXT_DISABLE_LARGE_DATA_OPTIMIZATIONS](#RMW_CONNEXT_DISABLE_LARGE_DATA_OPTIMIZATIONS)
- [RMW_CONNEXT_USE_DEFAULT_PUBLISH_MODE](#RMW_CONNEXT_USE_DEFAULT_PUBLISH_MODE)

### RMW_CONNEXT_INITIAL_PEERS

*Specify a list of peer locators to configure `DomainParticipantQos::discovery::initial_peers`.*

#### Examples

```sh
# Configure custom initial peers for `rmw_connextddsmicro`. In this case:
# - enable discovery over the shared memory transport with the first 11 participants
#   on the host (indices 0 to 10).
# - Add unicast UDP locator `192.168.1.1` (with default max index of 4).
# - Include the default multicast UDP locator, which wouldn't be otherwise used
#   because of the other custom locators.
# Note that we must also customize the UDP interface otherwise `lo` will be used
# and the UDP locators would not be reachable by the application.
RMW_IMPLEMENTATION=rmw_connextddsmicro \
RMW_CONNEXT_INITIAL_PEERS="10@_shmem://, 192.168.1.1, 239.255.0.1" \
RMW_CONNEXT_UDP_INTERFACE=eth0 \
ros2 run demo_nodes_cpp talker

# Applications running with `rmw_connextdds` may use `RMW_CONNEXT_INITIAL_PEERS`,
# but they should resort to variable `NDDS_DISCOVERY_PEERS` which is supported
# directly by RTI Connext DDS.
RMW_IMPLEMENTATION=rmw_connextdds \
NDDS_DISCOVERY_PEERS="10@shmem://, 192.168.1.1, 239.255.0.1" \
ros2 run demo_nodes_cpp talker
```

#### Description

Variable `RMW_CONNEXT_INITIAL_PEERS` can be used to specify a list of
comma-separated values of "address locators" that the DomainParticipant created
by the RMW will use to try to make contact with remote peer applications
during the DDS discovery phase.

The values will be parsed, trimmed, and stored in QoS field
`DDS_DomainParticipantQos::discovery::initial_peers`, overwriting any
value it previously contained.

While both `rmw_connextdds` and `rmw_connextddsmicro` will honor this variable,
[equivalent, and more advanced, functionality is already available in RTI Connext DDS](https://community.rti.com/static/documentation/connext-dds/6.0.1/doc/manuals/connext_dds/html_files/RTI_ConnextDDS_CoreLibraries_UsersManual/Content/UsersManual/ConfigPeersListUsed_inDiscov.htm),
for example using variable `NDDS_DISCOVERY_PEERS`.

For this reason, only users of `rmw_connextddsmicro` should consider specifying
`RMW_CONNEXT_INITIAL_PEERS`.

For example, `rmw_connextddsmicro` will use `lo` as its default UDP network
interface (see [RMW_CONNEXT_UDP_INTERFACE](#RMW_CONNEXT_UDP_INTERFACE)),
which will prevent it from accessing the default discovery peer
(multicast address `239.255.0.1`).
The default peer configuration will also prevent the DomainParticipant from
carrying out discovery over the built-in shared-memory transport.
To enable discovery over this transport, in addition to
the default multicast peer:

```sh
RMW_IMPLEMENTATION=rmw_connextddsmicro \
RMW_CONNEXT_INITIAL_PEERS="_shmem://, 239.255.0.1" \
  ros2 run demo_nodes_cpp listener
```

### RMW_CONNEXT_LEGACY_RMW_COMPATIBILITY_MODE

*Enable a compatibility mode which facilitates communication between `rmw_connextdds` and `rmw_connext_cpp`.*

#### Examples

```sh
# Start a `demo_nodes_cpp/listener` with `rmw_connext_cpp`
RMW_IMPLEMENTATION=rmw_connext_cpp \
ros2 run demo_nodes_cpp listener &

# Start a `demo_nodes_cpp/talker` with `rmw_connextdds` in legacy RMW compatibility mode
RMW_IMPLEMENTATION=rmw_connextdds \
RMW_CONNEXT_LEGACY_RMW_COMPATIBILITY_MODE=y \
ros2 run demo_nodes_cpp listener
```

#### Description

ROS2 applications using `rmw_connextdds` will not be able to interoperate with
applications using the previous RMW implementation for RTI Connext DDS, `rmw_connext_cpp`,
unless variable `RMW_CONNEXT_LEGACY_RMW_COMPATIBILITY_MODE` is used to enable
a "compatibility" mode with these older implementation.

In particular, when this mode is enabled, `rmw_connextdds` will revert to adding
a suffix (`_`) to the end of the names of the attributes of the ROS2 data types
propagated via DDS discovery.

This incompatibility can also be overcome using QoS policy [`DDS_TypeConsistencyEnforcementQosPolicy::ignore_member_names`](https://community.rti.com/static/documentation/connext-dds/6.0.1/doc/api/connext_dds/api_c/structDDS__TypeConsistencyEnforcementQosPolicy.html),
available in RTI Connext DDS 6.0.0 or later.

`DDS_TypeConsistencyEnforcementQosPolicy` is a reader-only QoS policy, and it will
enable an application running with `rmw_connextdds` and using RTI Connext DDS 6.x
to be able to match with and receive data from an application running with
`rmw_connext_cpp`, or other applications running with `rmw_connextdds` but with the
legacy RMW compatibility mode enabled.

Since `rmw_connext_cpp` only supports RTI Connext DDS 5.3.1, the only way to receive
data from applications using `rmw_connextdds` is to enable the legacy RMW compatibility mode.

### RMW_CONNEXT_PARTICIPANT_QOS_OVERRIDE_POLICY

*Control how `rmw_connextdds` will override the DomainParticipant's default QoS with settings derived from ROS 2 options and custom QoS optimizations*

#### Examples

```sh
# Load a custom default QoS profile ("my_application::my_process").
cp rmw_connextdds/resource/xml/USER_QOS_PROFILES.example.xml \
   USER_QOS_PROFILES.xml

# Start a `demo_nodes_cpp/listener` without overwriting the default
# DomainParticipant QoS loaded from XML.
RMW_IMPLEMENTATION=rmw_connextdds \
RMW_CONNEXT_PARTICIPANT_QOS_OVERRIDE_POLICY=never \
ros2 run demo_nodes_cpp listener &

# Start a `demo_nodes_cpp/talker` with only basic QoS settings modified
RMW_IMPLEMENTATION=rmw_connextdds \
RMW_CONNEXT_PARTICIPANT_QOS_OVERRIDE_POLICY=basic \
ros2 run demo_nodes_cpp talker
```

These examples load a custom default QoS profile from [USER_QOS_PROFILES.example.xml](rmw_connextdds/resource/xml/USER_QOS_PROFILES.example.xml)
by copying the file to `./USER_QOS_PROFILES.xml`. The same can also be achieved
by specifying `USER_QOS_PROFILES.example.xml`'s path in variable `NDDS_QOS_PROFILES`.

See [Quality of Service Configuration](#quality-of-service-configuration) for more information on how to load QoS profiles.

#### Description

When variable `RMW_CONNEXT_PARTICIPANT_QOS_OVERRIDE_POLICY` is unspecified, or set to `all`,
`rmw_connextdds` will modify the DomainParticipant's default Qos with settings derived from
ROS 2 options such as "localhost only", or "node enclave". Additionally, some optimizations
meant to improve the out of the box experiene (e.g. speed up endpoint discovery, and increase
the size of type information shared via discovery) will also be applied on top
of the default QoS profile.

If `RMW_CONNEXT_PARTICIPANT_QOS_OVERRIDE_POLICY` is set to `basic`, then only those
settings associated with ROS 2 options will be overwritten.

If `RMW_CONNEXT_PARTICIPANT_QOS_OVERRIDE_POLICY` is set to `never`, then no settings
will be modified and the DomainParticipant's default Qos will be used as is.

Note that values `basic` and `never` will automatically disable the QoS
optimizations controlled by [RMW_CONNEXT_DISABLE_FAST_ENDPOINT_DISCOVERY](#RMW_CONNEXT_DISABLE_FAST_ENDPOINT_DISCOVERY).

When using these values, you may replicate the default QoS configuration using
the profiles defined by XML file [ros2_qos_profiles.xml](rmw_connextdds/resource/xml/ros2_qos_profiles.xml)
(e.g. `ros2::rmw_connextdds.base_application`). See [Quality of Service Configuration](#quality-of-service-configuration)
for more information on these built-in profiles.

### RMW_CONNEXT_REQUEST_REPLY_MAPPING

*Control how `rmw_connextdds` maps ROS 2 client/service interactions to DDS messages.*

#### Examples

- Default mapping (DDS-RPC *extended*):

  ```sh
  # Start a `demo_nodes_cpp/listener` with `rmw_connextdds`.
  # Use the default request/reply mapping (equivalent to
  # `RMW_CONNEXT_REQUEST_REPLY_MAPPING=extended`).
  RMW_IMPLEMENTATION=rmw_connextdds \
  ros2 run demo_nodes_cpp listener &

  # Query node parameters using `rmw_fastrtps_cpp`
  RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  ros2 param list /listener
  ```

- Alternative mapping (DDS-RPC *basic*):

  ```sh
  # Start a `demo_nodes_cpp/listener` with `rmw_connextddsmicro`.
  RMW_IMPLEMENTATION=rmw_connextddsmicro \
  ros2 run demo_nodes_cpp listener &

  # Query node parameters using `rmw_connextdds`. Use the `basic` request/reply mapping.
  RMW_IMPLEMENTATION=rmw_connextdds \
  RMW_CONNEXT_REQUEST_REPLY_MAPPING=basic \
  ros2 param list /listener
  ```

- Alternative mapping (Cyclone compatibility mode):

  ```sh
  # Start a `demo_nodes_cpp/listener` with `rmw_cyclonedds_cpp`.
  RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  ros2 run demo_nodes_cpp listener &

  # Query node parameters using `rmw_connextdds`. Use a request/reply mapping
  # compatible with `rmw_cyclonedds_cpp`. RMW_CONNEXT_REQUEST_REPLY_MAPPING is
  # ignored in this case.
  RMW_IMPLEMENTATION=rmw_connextdds \
  RMW_CONNEXT_CYCLONE_COMPATIBILITY_MODE=y \
  ros2 param list /listener
  ```

#### Description

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
In this configuration, `rmw_connextdds` will interoperate with `rmw_fastrtps_cpp`,
e.g.:

```sh
RMW_IMPLEMENTATION=rmw_connextdds \
  ros2 run demo_nodes_cpp add_two_ints_server

RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  ros2 run demo_nodes_cpp add_two_ints_client
```

When using the *basic* profile, `rmw_connextdds` will interoperate with
`rmw_connextddsmicro`, e.g.:

```sh
RMW_IMPLEMENTATION=rmw_connextdds \
RMW_CONNEXT_REQUEST_REPLY_MAPPING=basic \
  ros2 run demo_nodes_cpp add_two_ints_server

RMW_IMPLEMENTATION=rmw_connextddsmicro \
RMW_CONNEXT_INITIAL_PEER=localhost \
  ros2 run demo_nodes_cpp add_two_ints_client
```

Use variable [RMW_CONNEXT_CYCLONE_COMPATIBILITY_MODE](#RMW_CONNEXT_CYCLONE_COMPATIBILITY_MODE)
to enable interoperability with `rmw_cyclonedds_cpp` using a non-standard version
of the *basic* profile, e.g.:

```sh
RMW_IMPLEMENTATION=rmw_connextdds \
RMW_CONNEXT_CYCLONE_COMPATIBILITY_MODE=y \
  ros2 run demo_nodes_cpp add_two_ints_server

RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  ros2 run demo_nodes_cpp add_two_ints_client
```

### RMW_CONNEXT_UDP_INTERFACE

*Select the UDPv4 interface used by `rmw_connextddsmicro` to communicate with other peers.*

#### Examples

```sh
# Start a `demo_nodes_cpp/talker` with `rmw_connextddsmicro` using `eth0` for UDPv4 traffic.
RMW_IMPLEMENTATION=rmw_connextddsmicro \
RMW_CONNEXT_UDP_INTERFACE=eth0 \
ros2 run demo_nodes_cpp talker
```

#### Description

RTI Connext DDS Micro requires applications to explicitly configure the network
interface to use for UDPv4 communication.

`rmw_connextddsmicro` makes the arbitrary decision of using `lo` as the default
interface.

This is undesireable if non-local communication is required, and/or if the
default DDS multicast peer (`239.255.0.1`) is to be used.

Variable `RMW_CONNEXT_UDP_INTERFACE` may be used to customize the network interface
actually used by RTI Connext DDS Micro's UDPv4 transport, e.g. to use `eth0`:

```sh
RMW_IMPLEMENTATION=rmw_connextddsmicro \
RMW_CONNEXT_UDP_INTERFACE=eth0 \
  ros2 run demo_nodes_cpp listener
```

This variable is not used by `rmw_connextdds`.

### RMW_CONNEXT_USE_DEFAULT_PUBLISH_MODE

*Cause `rmw_connextdds` to use the default value for `DataWriterQos::publish_mode::kind`.*

#### Examples

```sh
# Do not ovverride `DataWriterQos::publish_mode::kind` for `my_package/my_process`
RMW_IMPLEMENTATION=rmw_connextdds \
RMW_CONNEXT_USE_DEFAULT_PUBLISH_MODE=y \
ros2 run my_package my_process
```

#### Description

`rmw_connextdds` will always set `DataWriterQos::publish_mode::kind` of
any DataWriter it creates to `DDS_ASYNCHRONOUS_PUBLISH_MODE_QOS`, in order to
enable out of the box support for "large data".
`
This behavior might not be always desirable, and it can be disabled by setting
`RMW_CONNEXT_USE_DEFAULT_PUBLISH_MODE` to a non-empty value.

This variable is not used by `rmw_connextddsmicro`, since it doesn't
automatically override `DDS_DataWriterQos::publish_mode::kind`.

## Quality of Service Configuration

`rmw_connextdds` will use the topic name when querying for the default QoS, so
you may use topic filters in your QoS XML files to have different defaults
for different topics. The only caveat is that you must the mangled topic names
(see [ROS topic mangling conventions](#ros-topic-mangling-conventions)):

```xml
<dds>
  <qos_library name="my_profiles">
    <qos_profile is_default_qos="true">
      <!-- Customize QoS for writing to topic "chatter" -->
      <datawriter_qos topic_filter="rt/chatter">
        ...
      </datawriter_qos>
      <!-- Customize QoS for client requests on topic "describe_parameters" -->
      <datawriter_qos topic_filter="rq/*/describe_parametersRequest">
        ...
      </datawriter_qos>
    <qos_profile>
  </qos_library>
</dds>
```

### ROS topic mangling conventions

ROS mangles topic names in the following way:

- Topics are prefixed with `rt`. e.g.: `/my/fully/qualified/ros/topic` is converted to `rt/my/fully/qualified/ros/topic`.
- The service request topics are prefixed with `rq` and suffixed with `Request`. e.g.: `/my/fully/qualified/ros/service` request topic is `rq/my/fully/qualified/ros/serviceRequest`.
- The service response topics are prefixed with `rr` and suffixed with `Response`. e.g.: `/my/fully/qualified/ros/service` response topic is `rr/my/fully/qualified/ros/serviceResponse`.
