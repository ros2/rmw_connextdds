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

For any questions or feedback, feel free to reach out to robotics@rti.com.

## Table of Contents

- [Quick Start](#quick-start)
- [Support for different ROS 2 Releases](#support-for-different-ros-2-releases)
- [RTI Connext DDS Requirements](#rti-connext-dds-requirements)
  - [Multiple versions of RTI Connext DDS Professional](#multiple-versions-of-rti-connext-dds-professional)
- [RMW Runtime Configuration](#rmw-runtime-configuration)
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
- [DDS Quality of Service Configuration](#dds-quality-of-service-configuration)
  - [Customize DomainParticipant QoS](#customize-domainparticipant-qos)
  - [Customize Endpoint QoS](#customize-endpoint-qos)
  - [Built-in ROS 2 QoS Profiles](#built-in-ros-2-qos-profiles)
  - [Built-in Connext QoS Profiles](#built-in-connext-qos-profiles)
- [DDS Entities Created by the RMW Layer](#dds-entities-created-by-the-rmw-layer)
  - [DomainParticipantFactory](#domainparticipantfactory)
  - [DomainParticipant](#domainparticipant)
  - [Publisher](#publisher)
  - [Subscriber](#subscriber)
  - [Topic](#topic)
  - [DataWriter](#datawriter)
  - [DataReader](#datareader)

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

## RMW Runtime Configuration

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

See also [DDS Quality of Service Configuration](#dds-quality-of-service-configuration) for information on how to customize
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

Note that these optimizations will only be applied to those endpoints whose type has a
*static* maximum serialized size of at least 1MB. "Unbounded" types (i.e. types
which include at least one variable-length fields without a maximum) will most likely
not be included in these optimizations, since only their "bounded" part will
be taken into account when computing the maximum serialized size. This will generally
end up being a pretty small value (equivalent to having all 0-length "unbounded" fields),
and thus it will typically cause the types not to qualify as "large data" for the purpose
of these optimizations (even though applications may be exchanging "large data"
with them at runtime).

In this case, you should provide custom QoS configuration to manually apply similar
optimizations on the relevant endpoints. See [DDS Quality of Service Configuration](#dds-quality-of-service-configuration)
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

See [DDS Quality of Service Configuration](#dds-quality-of-service-configuration) for more information on how to load QoS profiles.

#### Description

When `RMW_CONNEXT_ENDPOINT_QOS_OVERRIDE_POLICY` is not set, or if it is set to `always`,
in order to determine the QoS used by an endpoint, `rmw_connextdds` will first load Connext's
default QoS profile for the specific type of endpoint (DataWriter or DataReader), and for
the endpoint's own topic (using the "mangled" topic name). The ROS 2 QoS profile will then
be applied on top of these values.

In this mode, any ROS 2 QoS policy not set to `SYSTEM_DEFAULT` will always overwrite
the corresponding DDS QoS policy value obtained from default QoS. See [DDS Quality of Service Configuration](#dds-quality-of-service-configuration)
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

See [DDS Quality of Service Configuration](#dds-quality-of-service-configuration) for more information on how to load QoS profiles.

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
(e.g. `ros2::rmw_connextdds.base_application`). See [DDS Quality of Service Configuration](#dds-quality-of-service-configuration)
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

## DDS Quality of Service Configuration

One of the key aspects that a developer must address during design and implementation
of a ROS 2/DDS application is the analysis of the Quality of Service requirements
of their application and overall distributed system.

The DDS specification offers a rich set of standard QoS policies, which are further
expanded by RTI Connext DDS with several proprietary and implementation-specific
extensions (see [Connext's documentation](https://community.rti.com/static/documentation/connext-dds/6.0.1/doc/api/connext_dds/api_c/group__DDSQosTypesModule.html)).

While some of the policies can be modified using the ROS 2 QoS options, for a Connext
application to meet its expected requirements, it will often be necessary to customize
the value of several other policies which are not exposed by the ROS 2 API.

For example, Connext's default settings controlling the heartbeat-based protocol
performed between reliable readers and writers are not tuned for fast-paced
communications and they might cause significant delays in the "repair" of samples
missed by a reader. In this cae, it might be necessary to update parameters in
`DataWriterQos::protocol::reliable_writer` and `DataReadeQos::protocol::reliable_reader`
to make applications using this type of communication more responsive.

Since these parameters are not exposed by the ROS 2 API, users must rely on
the XML-based configuration facility offered by RTI Connext DDS to modify them
when creating DDS entities with this API.

The following sections provide more information about different aspects of
QoS configuration applicable to `rmw_connextdds`.

Since RTI Connext DDS Micro does not support external QoS configuration, most of
these customizations are not yet available to users of `rmw_connextddsmicro`.

### Customize QoS via XML

`rmw_connextdds` supports the specification of most of its initialization parameters,
[including Quality of Service configuration](https://community.rti.com/static/documentation/connext-dds/6.0.1/doc/manuals/connext_dds/html_files/RTI_ConnextDDS_CoreLibraries_UsersManual/index.htm#UsersManual/XMLConfiguration.htm),
in external XML files written (for the most part) using the standard XML grammar
defined by the [DDS-XML specification](https://www.omg.org/spec/DDS-XML/).

The QoS configuration can be expressed in an XML file containing one or more
*QoS libraries* (`<qos_library>`), each one containing one or more *QoS profiles*
(`<qos_profile>`).

Each `<qos_profile>` may contain:

- `0..*` `<participant_qos>` to customize `DomainParticipantQos` policies.
- `0..*` `<publisher_qos>` to customize `PublisherQos` policies.
- `0..*` `<subscriber_qos>` to customize `SubscriberQos` policies.
- `0..*` `<topic_qos>` to customize `TopicQos` policies.
- `0..*` `<datawriter_qos>` to customize `DataWriterQos` policies.
- `0..*` `<datareader_qos>` to customize `DataReaderQos` policies.

All elements (libraries, profiles, and individual policies) may be assigned unique
names, which can later be used to easily extend them and compose them into new
configurations.

For example, profile inheritance may be used to create a single configuration
out of multiple profiles:

```xml
<dds>
  <qos_library name="my_profiles">
    <!-- A profile which sets ReliabilityQosPolicy to RELIABLE -->
    <qos_profile name="reliable_comms">
      <datawriter_qos>
        <reliability>
          <kind>RELIABLE_RELIABILITY_QOS</kind>
        </reliability>
      </datawriter_qos>
      <datareader_qos>
        <reliability>
          <kind>RELIABLE_RELIABILITY_QOS</kind>
        </reliability>
      </datareader_qos>
    <qos_profile>

    <!--
      A profile which sets DurabilityQosPolicy to TRANSIENT_LOCAL.
      For the sake of example, it also inherits from `reliable_comms` to make
      the endpoints "reliable".
    -->
    <qos_profile name="transient_comms" base_name="reliable_comms">
      <datawriter_qos>
        <durability>
          <kind>TRANSIENT_LOCAL_DURABILITY_QOS</kind>
        </durability>
      </datawriter_qos>
      <datareader_qos>
        <durability>
          <kind>TRANSIENT_LOCAL_DURABILITY_QOS</kind>
        </durability>
      </datareader_qos>
    <qos_profile>

    <!--
      A profile which customize the QoS for all DataWriters and DataReaders.
      The profile also selectively applies previous profiles via "topic filters".
    -->
    <qos_profile name="application_topics">
      <!-- Mark all DataWriters and DataReaders as BEST_EFFORT -->
      <datawriter_qos>
        <reliability>
          <kind>BEST_EFFORT_RELIABILITY_QOS</kind>
        </reliability>
      </datawriter_qos>
      <datareader_qos>
        <reliability>
          <kind>BEST_EFFORT_RELIABILITY_QOS</kind>
        </reliability>
      </datareader_qos>
      <!--
        Apply profile `transient_comms` to all endpoints on topic `my_transient_topic`
      -->
      <datawriter_qos base_name="transient_comms" topic_filter="my_transient_topic"/>
      <datareader_qos base_name="transient_comms" topic_filter="my_transient_topic"/>
      <!--
        Apply profile `transient_comms` to all endpoints on topic `my_transient_be_topic`,
        but overwrite ReliabilityQosPolicy to be BEST_EFFORT  
      -->
      <datawriter_qos base_name="transient_comms" topic_filter="my_transient_be_topic">
        <reliability>
          <kind>BEST_EFFORT_RELIABILITY_QOS</kind>
        </reliability>
      </datawriter_qos>
      <datareader_qos base_name="transient_comms" topic_filter="my_transient_be_topic">
        <reliability>
          <kind>BEST_EFFORT_RELIABILITY_QOS</kind>
        </reliability>
      </datareader_qos>
    <qos_profile>

    <!--
      A profile which customizes the EntityNameQosPolicy for DomainParticipants,
      Subscribers, and Publisher.
      The actual value for the property will be dynamically determined by Connext
      at runtime by reading the values of the specified environment variables.
    -->
    <qos_profile name="custom_names">
      <participant_qos>
        <participant_name>
          <name>$(CUSTOM_PARTICIPANT_NAME)</name>
        </participant_name>
      </participant_qos>
      <subscriber_qos>
        <subscriber_name>
          <name>$(CUSTOM_SUBSCRIBER_NAME)</name>
        </subscriber_name>
      </subscriber_qos>
      <publisher_qos>
        <publisher_name>
          <name>$(CUSTOM_PUBLISHER_NAME)</name>
        </publisher_name>
      </publisher_qos>
    <qos_profile>

    <!--
      Since it uses `use_default_qos="true"`, this profile will cause Connext to
      modify the default QoS values returned for DomainParticipants, Publisher,
      Subscriber, DataWriters, and DataReaders (all of the entity QoS policies
      modified by this profile and its inheritance chain).
      
      Since `rmw_connextdds` relies on these default values to determine the QoS used
      by each DDS entity it creates, this technique can be used to effectively modify
      the QoS used by a ROS 2 application running with `rmw_connextdds`
      
      For the sake of example, the profile inherits from `application_topics` to
      inherit all of its topic-based rules, while profile `custom_names` is
      applied selectively by using inheritance on entity-specific elements.
    -->
    <qos_profile name="default_profile" base_name="application_topics" is_default_qos="true">
      <participant_qos base_name="custom_names"/>
      <subscriber_qos base_name="custom_names"/>
      <publisher_qos base_name="custom_names"/>
    <qos_profile>
  </qos_library>
</dds>

```

Refer to Connext's documentation for a complete overview of the rich set of
options available for [QoS Profile Inheritance and Composition](https://community.rti.com/static/documentation/connext-dds/6.0.1/doc/manuals/connext_dds/html_files/RTI_ConnextDDS_CoreLibraries_UsersManual/index.htm#UsersManual/QoS_Profile_Inheritance.htm#19.3.3_QoS_Profile_Inheritance_and_Composition%3FTocPath%3DPart%25203%253A%2520Advanced%2520Concepts%7C19.%2520Configuring%2520QoS%2520with%2520XML%7C19.3%2520QoS%2520Profiles%7C19.3.3%2520QoS%2520Profile%2520Inheritance%2520and%2520Composition%7C_____0)
(please be aware that some options, such as "QoS snippets" are only available in
Connext 6.x).

You might also be interested in learning more about [topic filters](https://community.rti.com/static/documentation/connext-dds/6.0.1/doc/manuals/connext_dds/html_files/RTI_ConnextDDS_CoreLibraries_UsersManual/index.htm#UsersManual/Topic_Filters.htm), and [how to overwrite default QoS values](https://community.rti.com/static/documentation/connext-dds/6.0.1/doc/manuals/connext_dds/html_files/RTI_ConnextDDS_CoreLibraries_UsersManual/index.htm#UsersManual/Overwriting_Default_QoS_Values.htm?Highlight=is_default_qos).

### Customize DomainParticipant QoS

`rmw_connextdds` will create a single DomainParticipant for each ROS context, which
will be shared by all Nodes associated with that context (typically all Nodes created
by a process).

Since only one DomainParticipant will be typically be created by each ROS 2 process,
users may rely on Connext's default QoS configuration to customize the QoS
of the DomainParticipant from an external XML file:

```xml
<dds>
  <qos_library name="my_profiles">
    <qos_profile name="a_profile" is_default_qos="true">
      <participant_qos>
        <participant_name>
          <name>MyRos2DomainParticipant</name>
        </participant_name>
      </participant_qos>
    <qos_profile>
  </qos_library>
</dds>
```

By default, `rmw_connextdds` will always overwrite certain fields of the default
DomainParticipant QoS (see [DomainParticipant creation](#domainparticipant-creation)).
These "hard-coded" customizations may be disabled using the "QoS override" policies
selected with variable [`RMW_CONNEXT_PARTICIPANT_QOS_OVERRIDE_POLICY`](#rmw-connext-participant-qos-override-policy).

The customization may also be replicated in XML by using the QoS profiles contained
in [ros2_qos_profiles.xml](rmw_connextdds/resource/xml/ros2_qos_profiles.xml),
for example, `ros2::rmw_connextdds.base_participant`. See [Built-in ROS 2 QoS Profiles](#built-in-ros-2-qos-profiles) for more information about these QoS profiles
and how to use them.

### Customize Endpoint QoS

`rmw_connextdds` will include the topic name when querying for the default QoS values
to use for a new DDS endpoint. This allows users to take advantage of [topic filters](https://community.rti.com/static/documentation/connext-dds/6.0.1/doc/manuals/connext_dds/html_files/RTI_ConnextDDS_CoreLibraries_UsersManual/index.htm#UsersManual/Topic_Filters.htm)
in QoS XML files to set different defaults for different topics.

The only caveat is that filters must be expressed using the mangled version of the
topic names:

```xml
<dds>
  <qos_library name="my_profiles">
    <qos_profile name="a_profile" is_default_qos="true">
      <!-- Customize QoS for Publishers and Subscriptions on topic "chatter" -->
      <datawriter_qos topic_filter="rt/chatter">
        ...
      </datawriter_qos>
      <!-- Customize QoS for Subscriptions on topic "chatter" -->
      <datareader_qos topic_filter="rt/chatter">
        ...
      </datareader_qos>
      <!-- Customize QoS for Clients on topic "describe_parameters". -->
      <datawriter_qos topic_filter="rq/*/describe_parametersRequest">
        ...
      </datawriter_qos>
      <datareader_qos topic_filter="rr/*/describe_parametersResponse">
        ...
      </datareader_qos>
      <!-- Customize QoS for Services on topic "describe_parameters". -->
      <datawriter_qos topic_filter="rr/*/describe_parametersResponse">
        ...
      </datawriter_qos>
      <datareader_qos topic_filter="rq/*/describe_parametersRequest">
        ...
      </datareader_qos>
    <qos_profile>
  </qos_library>
</dds>
```

The following table summarizes the rules used by ROS 2 to mangle topic names:

| ROS Entities | Mangling Format |
|--------------|-----------------|
| Publisher, Subscription| `rt/<topic>` |
| Client, Service | `rq/<fully-qualified-service-name>/<topic>Request` (request topic)|
|  | `rr/<fully-qualified-service-name>/<topic>Response` (reply topic) |

By default, the default QoS loaded by Connext for a specific endpoint will be
ovewritten with values coming from the ROS QoS profile.

You can disable this behavior altogether or selectively for specific topics by
selecting one of the alternative "QoS override" policies available via
environment variable [RMW_CONNEXT_ENDPOINT_QOS_OVERRIDE_POLICY](#rmw-connext-endpoint-qos-override-policy).

For example, using the XML file above with a `demo_nodes_cpp/talker` instance,
it is possible to completely disable all ROS QoS settings for the customized
topics even if the application was not designed to use the `SYSTEM_DEFAULT` special
value:

```sh
RMW_IMPLEMENTATION=rmw_connextdds \
RMW_CONNEXT_ENDPOINT_QOS_OVERRIDE_POLICY="dds_topics: ^(rt/chatter|(rq|rr)/.*/describe_parameters(Request|Reply))$" \
ros2 run demo_nodes_cpp talker
```

A similar result can also be achieved using the `never` "QoS override" policy,
but the XML QoS profile would require some modifications, for example by making it
extend profile `ros2::rmw_connextdds.base_application`, for all the application
to remain compatible with other applications using the default ROS 2 QoS profiles.

See [Built-in ROS 2 QoS Profiles](#built-in-ros-2-qos-profiles) for more information
about this and other useful QoS profiles contained in file [ros2_qos_profiles.xml](rmw_connextdds/resource/xml/ros2_qos_profiles.xml).

### Built-in ROS 2 QoS Profiles

TODO

### Built-in Connext QoS Profiles

TODO

## DDS Entities Created by the RMW Layer

When an application uses the ROS 2 API to perform operations such as initializing
the ROS context, creating a Node, or subscribing to a topic, `rmw_connextdds` and
`rmw_connextddsmicro` will automatically create any DDS entity required to support
the associated ROS 2 objects.

The following table provides a summary of these DDS entities and their lifecycle
with respect to ROS 2 operations:

| DDS Entity | Associated with ROS... |Created by| Deleted by |
|--------|-----------------|-----------|------------|
|DomainParticipantFactory| Context | `rclcpp::init()` | `rclcpp::shutdown()` |
|DomainParticipant| Context | `rclcpp::init()` | `rclcpp::shutdown()` |
|Publisher| Context | `rclcpp::init()` | `rclcpp::shutdown()` |
|Subscriber| Context | `rclcpp::init()` | `rclcpp::shutdown()` |
|Topic| Publisher, Subscription, Client, Service | `rclcpp::Node::create_publisher()`, `rclcpp::Node::create_subscription()`, `rclcpp::Node::create_client()`, `rclcpp::Node::create_service()` | `rclcpp::Publisher::~Publisher()`, `rclcpp::Subscription::~Subscription()`, `rclcpp::Client::~Client()`, `rclcpp::Service::~Service()` |
|DataWriter| Publisher, Client, Service | `rclcpp::Node::create_publisher()`, `rclcpp::Node::create_client()`, `rclcpp::Node::create_service()` | `rclcpp::Publisher::~Publisher()`, `rclcpp::Client::~Client()`, `rclcpp::Service::~Service()` |
|Subscriber| Subscription, Client, Service | `rclcpp::Node::create_subscription()`, `rclcpp::Node::create_client()`, `rclcpp::Node::create_service()` | `rclcpp::Subscription::~Subscription()`, `rclcpp::Client::~Client()`, `rclcpp::Service::~Service()` |

The rest of this section provides a more detailed description of how each
entity is configured, when they are created, and when they are deleted.

### DomainParticipantFactory

The [DomainParticipantFactory](https://community.rti.com/static/documentation/connext-dds/6.0.1/doc/api/connext_dds/api_c/group__DDSDomainParticipantFactoryModule.html#ga206e77e6b4c2b0db6cefa6ed8d85128d)
is a singleton instance, unique within each DDS process,
whose main purpose is to create DomainParticipants.

This singleton will be configured upon initialization of a ROS context, and it
will only be finalized upon finalization of the last context.

#### DomainParticipantFactory Creation

When the first ROS context is initialized by an application (e.g. by `rclcpp::init()`), 
`rmw_connextdds` and `rmw_connextddsmicro` will both perform the following operations:

- Retrieve the DomainParticipantFactory singleton using
  [`DDS_DomainParticipantFactory_get_instance()`](https://community.rti.com/static/documentation/connext-dds/6.0.1/doc/api/connext_dds/api_c/group__DDSDomainParticipantFactoryModule.html#ga9e3530e68217e932a95a93db6f8da2e6).
- Retrieve the DomainParticipantFactory's QoS using
  [`DDS_DomainParticipantFactory_get_qos()`](https://community.rti.com/static/documentation/connext-dds/6.0.1/doc/api/connext_dds/api_c/group__DDSDomainParticipantFactoryModule.html#gacd908ae7d4885fd0c064f8ef5ed4f75f)
- Set [`DomainParticipantFactoryQos::entity_factory::autoenable_created_entities`](https://community.rti.com/static/documentation/connext-dds/6.0.1/doc/api/connext_dds/api_c/structDDS__EntityFactoryQosPolicy.html#a4b650acc47f1a51eefd90a072d6a70f6)
  to `false` so that DomainParticipants will not be automatically enabled upon    creation,
  but they may instead be first configured and then enabled. This is a "best practice"
  for DDS applications, and it is a required pattern for certain use cases (e.g.
  to make sure that no discovery traffic is missed by a custom user listener).
- Update the DomainParticipantFactory's QoS using
  [`DDS_DomainParticipantFactory_set_qos()`](https://community.rti.com/static/documentation/connext-dds/6.0.1/doc/api/connext_dds/api_c/group__DDSDomainParticipantFactoryModule.html#ga20742ba32637a069e5e328eb1cd5c48f)
  
In the case of `rmw_connextddsmicro`, the some additional initialization
operations will also be performed:

- Register the default Writer History plugin.
- Register the default Reader History plugin.
- Unregister the UDP Transport plugin.
- Configure the UDP Transport plugin to use the interface specified by [`RMW_CONNEXT_UDP_INTERFACE`](#rmw-connext-udp-interface).
- Re-register the UDP Transport plugin.
- Register the Shared-memory Transport plugin.
- Configure the DPDE Discovery plugin with the following settings:
  - `participant_liveliness_assert_period`: 10 seconds.
  - `participant_liveliness_lease_duration`: 60 seconds
- Register the DPDE Discovery plugin.

#### DomainParticipantFactory Deletion

Upon finalization of the last ROS context (e.g. via `rclcpp::shutdown()`), the
DomainParticipantFactory will be finalized using
[`DDS_DomainParticipantFactory_finalize_instance()`](https://community.rti.com/static/documentation/connext-dds/6.0.1/doc/api/connext_dds/api_c/group__DDSDomainParticipantFactoryModule.html#gae06876c2f46093fed7e5a727dcc84343).

In the case of `rmw_connextddsmicro`, all plugins that were registered with
the factory will be unloaded before the factory is finalized, so that all of
their resources may also be finalized.

### DomainParticipant

A [DomainParticipant](https://community.rti.com/static/documentation/connext-dds/6.0.1/doc/api/connext_dds/api_c/group__DDSDomainParticipantModule.html#ga1964274885335d5bcea6855f66b0bfe1)
is the fundamental entity that any DDS application must create
before all others in order to join a DDS domain and take part in DDS communication.

Each DomainParticipant is associated with a single DDS domain, uniquely identified by
an integral "domain id" (default: `0`).

A DomainParticipant will carry out DDS' [Simple Participant Discovery Protocol](https://community.rti.com/static/documentation/connext-dds/6.0.1/doc/manuals/connext_dds/html_files/RTI_ConnextDDS_CoreLibraries_UsersManual/index.htm#UsersManual/Simple_Participant_Discovery.htm)
over its available transports to automatically discover other DomainParticipants in the same domain
and [establish communication paths between matching endpoints](https://community.rti.com/static/documentation/connext-dds/6.0.1/doc/manuals/connext_dds/html_files/RTI_ConnextDDS_CoreLibraries_UsersManual/index.htm#UsersManual/Simple_Endpoint_Discovery.htm#15.1.2_Simple_Endpoint_Discovery%3FTocPath%3DPart%25203%253A%2520Advanced%2520Concepts%7C15.%2520Discovery%7C15.1%2520What%2520is%2520Discovery%253F%7C_____2).

In the case of `rmw_connextdds` and `rmw_connextddsmicro`, a dedicated DomainParticipant
will be created for each ROS context. The DomainParticipant will be shared by all Nodes
associated with a context.

These DomainParticipants will be finalized whenever their associated ROS context is
finalized.

#### DomainParticipant Creation

`rmw_connextdds` and `rmw_connextddsmicro` will create one DomainParticipant
for each ROS context created by an application.

A typical ROS 2 application will use a single context (e.g. initialized
implicitly by `rclcpp::init()`) and thus use a single DomainParticipant to support
all the Nodes it creates.

Upon initialization of a context, the following operations will be performed:

- Determine the default `DomainParticipantQoS` by calling
  [`DDS_DomainParticipantFactory_get_default_participant_qos()`](https://community.rti.com/static/documentation/connext-dds/6.0.1/doc/api/connext_dds/api_c/group__DDSDomainParticipantFactoryModule.html#gaf85e5146fe9f1bd10e11fdf871f66c24).
- In the case of `rmw_connextdds`,  based on [`RMW_CONNEXT_PARTICIPANT_QOS_OVERRIDE_POLICY`](#rmw-connext-participant-qos-override-policy):
  - Tune internal policy `DomainParticipantQos::user_object` to enable sharing of
    DomainParticipants created by `rmw_connextdds` with Connext's C API with C++ applications
    using Connext's C++11 API.
  - Configure ROS 2 options:
    - If "localhost only" communication was requested, set property
      [`"dds.transport.UDPv4.builtin.parent.allow_interfaces_list"`](https://community.rti.com/static/documentation/connext-dds/6.0.1/doc/manuals/connext_dds/html_files/RTI_ConnextDDS_CoreLibraries_UsersManual/index.htm#UsersManual/Setting_Builtin_Transport_Properties_wit.htm#transports_4033894139_848314) to `"127.0.0.1"`.
    - If an "enclave" was specified, store its value in [`DomainParticipantQos::user_data`](https://community.rti.com/static/documentation/connext-dds/6.0.1/doc/api/connext_dds/api_c/structDDS__DomainParticipantQos.html#ad25fbfa462bf8a35bc8f3f97078d9b5f)
      as a string with format `enclave=${ENCLAVE}`, so that it may be shared with
      DDS discovery data.
  - Reduce the chance of RTPS GUID collisions when DomainParticipants are created
    and destroyed in rapid sequence (as in the case of some ROS 2 unit tests).
    - Set [`DomainParticipantQos::wire_protocol::rtps_auto_id_kind`](https://community.rti.com/static/documentation/connext-dds/6.0.1/doc/api/connext_dds/api_c/structDDS__WireProtocolQosPolicy.html#a7818ef96ca05c3f01d7b55fa18a5f6cb) to `RTPS_AUTO_ID_FROM_UUID`.
    - This is the default in Connext 6.x.
  - Allow propagation of longer content-filter expressions with DDS discovery data.
    - Make sure that [`DomainParticipantQos::resource_limits::contentfilter_property_max_length`](https://community.rti.com/static/documentation/connext-dds/6.0.1/doc/api/connext_dds/api_c/structDDS__DomainParticipantResourceLimitsQosPolicy.html#aef1ff851202f5777d31528a52d528e4f)
      is at least `1024`.
  - Disable the legacy "type code" representation format for dissemination of type information with DDS discovery data, in favor of the more modern and standardized "type object" format.
    - Set [`DomainParticipantQos::resource_limits::type_code_max_serialized_length`](https://community.rti.com/static/documentation/connext-dds/6.0.1/doc/api/connext_dds/api_c/structDDS__DomainParticipantResourceLimitsQosPolicy.html#a55d8561093c4ab21484613a4b8de2999) to `0`,
      to disable "type code" propagation.
    - Set [`DomainParticipantQos::resource_limits::type_object_max_serialized_length`](https://community.rti.com/static/documentation/connext-dds/6.0.1/doc/api/connext_dds/api_c/structDDS__DomainParticipantResourceLimitsQosPolicy.html#af7e993465ab759a9032c95bd19a44262) to `65000`
      to support propagation of "larger" types (i.e. consisting of many, possibly nested, members).
  - Reduce Connext's "shutdown period" to speed up finalization of a DomainParticipant.
    - Set [`DomainParticipantQos::database::shutdown_cleanup_period`](https://community.rti.com/static/documentation/connext-dds/6.0.1/doc/api/connext_dds/api_c/structDDS__DatabaseQosPolicy.html#a17dc46e4dd757576e6cc68487379f6fa) to 10ms.
  - Enable quicker endpoint discovery (depending on [RMW_CONNEXT_DISABLE_FAST_ENDPOINT_DISCOVERY](#rmw-connext-disable-fast-endpoint-discovery)).
    - Update [`DomainParticipantQos::discovery_config::publication_writer`](https://community.rti.com/static/documentation/connext-dds/6.0.1/doc/api/connext_dds/api_c/structDDS__DiscoveryConfigQosPolicy.html#a4dd350ea9eb8f9a2cae267bb5e019981):
      - Set `fast_heartbeat_period` to 100ms.
      - Set `late_joiner_heartbeat_period` to 100ms.
      - Set `max_heartbeat_retries` to 300 (30 seconds @ 10Hz).
    - Update [`DomainParticipantQos::discovery_config::subscription_writer`](https://community.rti.com/static/documentation/connext-dds/6.0.1/doc/api/connext_dds/api_c/structDDS__DiscoveryConfigQosPolicy.html#a27ccb8e5002c6f48ab0ac5e1181398eb):
      - Set `fast_heartbeat_period` to 100ms.
      - Set `late_joiner_heartbeat_period` to 100ms.
      - Set `max_heartbeat_retries` to 300 (30 seconds @ 10Hz).
- In the case of `rmw_connextddsmicro`:
  - Enable both the UDPv4 and Shared-Memory transports.
    - Set sequence [`DomainParticipantQos::transports::enabled_transports`](https://community.rti.com/static/documentation/connext-micro/3.0.3/doc/api_c/html/structDDS__TransportQosPolicy.html#ac1367604bd5e1090a1c516624cc3f2f4) to
      `{"_udp", "_shmem"}`.
    - Set sequence [`DomainParticipantQos::user_traffic::enabled_transports`](https://community.rti.com/static/documentation/connext-micro/3.0.3/doc/api_c/html/structDDS__UserTrafficQosPolicy.html#ad38e4eb3463ff0bd1c49765d4f5b8255) to
      `{"_udp://", "_shmem://"}`.
    - Set sequence [`DomainParticipantQos::discovery::enabled_transports`](https://community.rti.com/static/documentation/connext-micro/3.0.3/doc/api_c/html/structDDS__DiscoveryQosPolicy.html#a9ee4ed308f6ef12bbc2299fe242f5f36) to
      `{"_udp://", "_shmem://"}`.
  - Select DPDE as the discovery plugin.
    - Set [`DomainParticipantQos::discovery::discovery`](https://community.rti.com/static/documentation/connext-micro/3.0.3/doc/api_c/html/structDDS__DiscoveryQosPolicy.html#a74d4b6f27bdbc9dd1bd27bcc9fc78e80) to `"dpde"`.
  - Increase default resource limits to allow for the creation of a larger number of
    local entities (types, topics, readers, writers), and the discovery of more
    remote entities (participants, writers, readers):
    - Update [`DomainParticipantQos::resource_limits`](https://community.rti.com/static/documentation/connext-micro/3.0.3/doc/api_c/html/structDDS__DomainParticipantResourceLimitsQosPolicy.html):
      - Set `local_type_allocation` to `32`.
      - Set `local_topic_allocation` to `32`.
      - Set `local_reader_allocation` to `32`.
      - Set `local_writer_allocation` to `32`.
      - Set `local_publisher_allocation` to `1`.
      - Set `local_subscriber_allocation` to `1`.
      - Set `remote_participant_allocation` to `32`.
      - Set `remote_writer_allocation` to `32`.
      - Set `remote_reader_allocation` to `32`.
      - Set `matching_reader_writer_pair_allocation` to `32768`.
      - Set `matching_writer_reader_pair_allocation` to `32768`.
- Determine the DomainParticipant's domain id.
  - Refer to the ROS 2 documentation for information on [how to configure the domain id](https://docs.ros.org/en/rolling/Concepts/About-Domain-ID.html) in a ROS 2 application.
- Create the DomainParticipant using [`DDS_DomainParticipant_create_participant()`](https://community.rti.com/static/documentation/connext-dds/6.0.1/doc/api/connext_dds/api_c/group__DDSDomainParticipantFactoryModule.html#ga325cf8f97a2752b6f486b7b1c3faf5b8).

#### DomainParticipant Deletion

Upon deletion of a ROS context (e.g. via `rclcpp::shutdown()`), both `rmw_connextdds`
and `rmw_connextddsmicro` will delete the DomainParticipant associated with the
context, performing the following operations:

- Make sure that all entities contained in the DomainParticipant are finalized and
  delete by calling [`DDS_DomainParticipant_delete_contained_entities()`](https://community.rti.com/static/documentation/connext-dds/6.0.1/doc/api/connext_dds/api_c/group__DDSDomainParticipantModule.html#gaf407436063900d40a86e359f7e0f6212).
- Delete the DomainParticipant with [`DDS_DomainParticipantFactory_delete_participant()`](https://community.rti.com/static/documentation/connext-dds/6.0.1/doc/api/connext_dds/api_c/group__DDSDomainParticipantFactoryModule.html#ga51725e06d6cd390928ccc2b64dafc1f8).

### Publisher

A [Publisher](https://community.rti.com/static/documentation/connext-dds/6.0.1/doc/api/connext_dds/api_c/group__DDSPublisherModule.html#ga3ecc0192bac0cba03781b70cec3099c9)
is a container of DataWriters, and it is responsible for
disseminating the samples they write to matching DataReaders.

`rmw_connextdds` and `rmw_connextddsmicro` create a single Publisher which is
shared by all DataWriters they create.

The lifecycle of this Publisher is the same as its parent DomainParticipant.

#### Publisher Creation

When `rmw_connextdds` and `rmw_connextddsmicro` create a DomainParticipant
(see [DomainParticipant creation](#domainparticipant-creation)), they will
automatically create a Publisher with the following operations:

- Determine the default QoS for the Publisher using
  [`DDS_DomainParticipant_get_default_publisher_qos()`](https://community.rti.com/static/documentation/connext-dds/6.0.1/doc/api/connext_dds/api_c/group__DDSDomainParticipantModule.html#ga5617e21edf7fadbf70c325e19a7a8f76).
- Create a new Publisher using [`DDS_DomainParticipant_create_publisher()`](https://community.rti.com/static/documentation/connext-dds/6.0.1/doc/api/connext_dds/api_c/group__DDSDomainParticipantModule.html#ga59c1e6c22bb71b5b7d294ca1b669ce63).

#### Publisher Deletion

Upon deletion of the DomainParticipant (see [DomainParticipant deletion](#domainparticipant-deletion)),
both `rmw_connextdds` and `rmw_connextddsmicro` will perform the following operations:

- Make sure that all DataWriters contained in the Publisher are finalized and
  deleted by calling [`DDS_Publisher_delete_contained_entities()](https://community.rti.com/static/documentation/connext-dds/6.0.1/doc/api/connext_dds/api_c/group__DDSPublisherModule.html#gaa4845fff7f12821fd653daaf6f19cdcf).
- Delete the Publisher with [`DDS_DomainParticipant_delete_publisher()`](https://community.rti.com/static/documentation/connext-dds/6.0.1/doc/api/connext_dds/api_c/group__DDSDomainParticipantModule.html#ga53e3cb8272b97abad174f41390809e58).
  
### Subscriber

A [Subscriber](https://community.rti.com/static/documentation/connext-dds/6.0.1/doc/api/connext_dds/api_c/group__DDSSubscriberModule.html#ga2755712602505786a23baef5a42d6782)
is a container of DataReaders, and it is the entity responsible for
delivering received samples into their caches and informing applications of the
availability of new data.

`rmw_connextdds` and `rmw_connextddsmicro` create a single Subscriber which is
shared by all DataReaders they create.

The lifecycle of this Subscriber is the same as its parent DomainParticipant.

#### Subscriber Creation

When `rmw_connextdds` and `rmw_connextddsmicro` create a DomainParticipant
(see [DomainParticipant creation](#domainparticipant-creation)), they will
automatically create a Subscriber with the following operations:

- Determine the default QoS for the Subscriber using
  [`DDS_DomainParticipant_get_default_subscriber_qos()`](https://community.rti.com/static/documentation/connext-dds/6.0.1/doc/api/connext_dds/api_c/group__DDSDomainParticipantModule.html#ga3ac60963418f18e36df5cd0ed70f6089).
- Create a new Subscriber using [`DDS_DomainParticipant_create_subscriber()`](https://community.rti.com/static/documentation/connext-dds/6.0.1/doc/api/connext_dds/api_c/group__DDSDomainParticipantModule.html#ga2f7e89f72d034184c7c02371525da360).

#### Subscriber Deletion

Upon deletion of the DomainParticipant (see [DomainParticipant deletion](#domainparticipant-deletion)),
both `rmw_connextdds` and `rmw_connextddsmicro` will perform the following operations:

- Make sure that all DataReaders contained in the Subscriber are finalized and
  deleted by calling [`DDS_Subscriber_delete_contained_entities()](https://community.rti.com/static/documentation/connext-dds/6.0.1/doc/api/connext_dds/api_c/group__DDSSubscriberModule.html#ga09aa47773904fbfa844b442388482915).
- Delete the Subscriber with [`DDS_DomainParticipant_delete_subscriber()`](https://community.rti.com/static/documentation/connext-dds/6.0.1/doc/api/connext_dds/api_c/group__DDSDomainParticipantModule.html#ga6792b9c4564a2342216994fc90cca793).

### Topic

A [Topic](https://community.rti.com/static/documentation/connext-dds/6.0.1/doc/api/connext_dds/api_c/group__DDSTopicEntityModule.html#ga7f615cc63381c78f535904e370cf1356)
defines a "named, strongly-typed, communication channel" over which a DataWriter and a DataReader
may exchange DDS samples.

A Topic is uniquely identified by its name and by the name of its [data type](https://community.rti.com/static/documentation/connext-dds/6.0.1/doc/manuals/connext_dds/html_files/RTI_ConnextDDS_CoreLibraries_UsersManual/index.htm#UsersManual/DataTypes.htm#datatypes_842270378_397835%3FTocPath%3DPart%25202%253A%2520Core%2520Concepts%7C3.%2520Data%2520Types%2520and%2520DDS%2520Data%2520Samples%7C_____0),
and every DataWriter and DataReader is uniquely associated with a Topic.

For two remote endpoints to match as a result of DDS' [Simple Endpoint Discovery Protocol](https://community.rti.com/static/documentation/connext-dds/6.0.1/doc/manuals/connext_dds/html_files/RTI_ConnextDDS_CoreLibraries_UsersManual/index.htm#UsersManual/Simple_Endpoint_Discovery.htm#15.1.2_Simple_Endpoint_Discovery%3FTocPath%3DPart%25203%253A%2520Advanced%2520Concepts%7C15.%2520Discovery%7C15.1%2520What%2520is%2520Discovery%253F%7C_____2), their Topics must have the same
name.

Additionally, if a machine-consumable description of the Topics' types was shared via
discovery (the default with RTI Connext DDS, but not available in RTI Connext DDS Micro),
then the endpoints will be further compared based on their data types, according to the rules
defined by the [DDS-XTYPES](https://www.omg.org/spec/DDS-XTypes/About-DDS-XTypes/)
specification.

This might cause endpoints to be matched and communicate even if their types are
slightly different (e.g. they have been registered with different names, and/or
they have altogether different, but still "assignable", definitions). See
[Connext's documentation about "type assignability"](https://community.rti.com/static/documentation/connext-dds/6.0.1/doc/manuals/connext_dds/getting_started_extras/html_files/RTI_ConnextDDS_CoreLibraries_GettingStarted_ExtensibleAddendum/index.htm#ExtensibleTypesAddendum/Verifying_Type_Consistency__Type_Assignabilit.htm#2.2_Verifying_Type_Consistency__Type_Assignability%3FTocPath%3D2.%2520Type%2520Safety%2520and%2520System%2520Evolution%7C_____2) for more information
on how different types may still be considered "assignable".

If a description of the types is not available, then only the Topic names will be
considered during endpoint comparison, and matching will occur only if they are exactly
the same.

`rmw_connextdds` and `rmw_connextddsmicro` will automatically create and reuse
Topics based on the DataWriters and DataReaders created by an application (and
the "internal" endpoints they create for RMW management).

The RMWs will also take care of [automatically registering "type plugins"](https://community.rti.com/static/documentation/connext-dds/6.0.1/doc/manuals/connext_dds/html_files/RTI_ConnextDDS_CoreLibraries_UsersManual/index.htm#UsersManual/UsingGenTypeswithoutStandalo.htm#datatypes_842270378_428342%3FTocPath%3DPart%25202%253A%2520Core%2520Concepts%7C3.%2520Data%2520Types%2520and%2520DDS%2520Data%2520Samples%7C3.7%2520Using%2520Generated%2520Types%2520without%2520Connext%2520DDS%2520(Standalone)%7C_____0)
for the data types used by each endpoint.

Topic entities, and their associated "type plugins", will be automatically unregistered
and deleted once every endpoint associated with them has been deleted.

#### Topic Creation

Upon creation of a [DataWriter](#datawriter-creation) or [DataReader](#datareader-creation),
 `rmw_connextdds` and `rmw_connextddsmicro` will perform the following operations:

- Determine the name of the data type used by the endpoint.
  - Data types are registered by appending `::dds_` to the type's package name,
    and suffix `_` to the end type's own name.
  - This means that a data type `<package-name>::<type-name>` (e.g. `std_msgs::msg::String`),
    will be registered as `<package-name>::dds_::<type-name>_` (e.g. `std_msgs::msg::dds_::String_`).
- Register a custom "type plugin" which allows Connext to handle ROS 2 messages
  directly.
  - ROS 2 uses a different memory representation for data types than the one used
    by Connext. For this reason, `rmw_connextdds` and `rmw_connextddsmicro` implement
    custom "type plugins" instead of relying on the code generated by `rtiddsgen`.
  - Thanks to these custom "type plugins", `rmw_connextdds` and `rmw_connextddsmicro`
    are able to let Connext manipulate ROS 2 messages without any additional data
    representation conversion (as was for exaple the case with the previous
    implementation, `rmw_connext_cpp`).
- Determine the mangled version of the topic name (see [Customize Endpoint QoS](#customize-endpoint-qos) 
  for a summary of the name mangling rules).
- Check if a Topic with the specific name and data type has already been created
  by calling [`DDS_DomainParticipant_find_topic()`](https://community.rti.com/static/documentation/connext-dds/6.0.1/doc/api/connext_dds/api_c/group__DDSDomainParticipantModule.html#ga5d03738356707f3375ffddc7d1669161).
- If the Topic doesn't exist, create it using [`DDS_DomainParticipant_create_topic()`](https://community.rti.com/static/documentation/connext-dds/6.0.1/doc/api/connext_dds/api_c/group__DDSDomainParticipantModule.html#ga1b785962ec5a94098875b2268d1655c3).

#### Topic Deletion

Upon deletion of [DataWriter](#datawriter-deletion) or [DataReader](#datareader-deletion),
`rmw_connextdds` and `rmw_connextddsmicro` will perform the following operations:

- Delete the Topic associated with the deleted endpoint by calling
  [`DDS_DomainParticipant_delete_topic()`](https://community.rti.com/static/documentation/connext-dds/6.0.1/doc/api/connext_dds/api_c/group__DDSDomainParticipantModule.html#ga024006f63f750771cddd5f6524ce6034).
  - This function will only decrement the topics internal "reference count" if there
    are still other endpoints using the Topic.
- If the Topic was actually deleted, and no more endpoints are still using the data type
  (via other Topics), then the data type's "type plugin" will be unregistered from
  the enclosing DomainParticipant.

### DataWriter

A [DataWriter](https://community.rti.com/static/documentation/connext-dds/6.0.1/doc/api/connext_dds/api_c/group__DDSWriterModule.html#gafb74fa54675abe1bfb2090265bf4d116)
is the entity used by DDS applications to set the value of the data to be published
under a given [Topic](#topic).

DataWriters are uniquely associated with a single parent [Publisher](#publisher),
and a single Topic.

This means that each DataWriter will only be able to publish samples of a single
data types (the one used by their Topic).

When a sample is written by a DataWriter it is inserted in a local, configurable,
sample "history cache". The size of this cache, and the policy controlling how
long samples are stored in it, can be selected using various DDS QoS policies,
such as [HistoryQosPolicy](https://community.rti.com/static/documentation/connext-dds/6.0.1/doc/api/connext_dds/api_c/group__DDSHistoryQosModule.html), [DurabilityQosPolicy](https://community.rti.com/static/documentation/connext-dds/6.0.1/doc/api/connext_dds/api_c/group__DDSDurabilityQosModule.html),
and [ResourceLimitsQosPolicy](https://community.rti.com/static/documentation/connext-dds/6.0.1/doc/api/connext_dds/api_c/group__DDSResourceLimitsQosModule.html).

Depending on QoS configuration (specifically when DurabilityQosPolicy is set
to a `kind` equal or greater to `TRANSIENT_LOCAL`), a DataWriter may distribute
the samples available in its cache to "late joiner" DataReaders (i.e. DataReaders
that are matched after the samples were added to the cache).

`rmw_connextdds` and `rmw_connextddsmicro` will create a DataWriter to support
several ROS entities:

- A DataWriter will be created for each ROS Publisher.
- A DataWriter will be created on the "request topic" of each ROS Client.
- A DataWriter will be created on the "reply topic" of each ROS Service.

The DataWriters (and their associated Topics) will be automatically finalized
when the associated ROS entity is deleted.

All DataWriters will be created by `rmw_connextdds` and `rmw_connextddsmicro`
in a single parent Publisher.

#### DataWriter Creation

When a ROS Publisher, Client, or Service is created, `rmw_connextdds` and `rmw_connextddsmicro`
will perform the following operations:

- Register a "type plugin" for the DataWriter's data type and create its associated
  Topic if necessary. See [Topic creation](#topic-creation).
- Determine the default QoS for the DataWriter using 
  `DDS_Publisher_get_default_datawriter_qos_w_topic_name()`.
  - This operation is only available in `rmw_connextdds`, and it allows for "topic filters"
    to be taken into consideration when determining the default QoS.
  - Since XML-based configuration is not available with RTI Connext DDS Micro,
    `rmw_connextddsmicro` will fall back to [`DDS_Publisher_get_default_datawriter_qos()`](https://community.rti.com/static/documentation/connext-micro/3.0.3/doc/api_c/html/group__DDSPublisherModule.html#ga777a869f4f8d435be23dea15f949f4bf).
- In the case of `rmw_connextdds`:
  - Based on [`RMW_CONNEXT_ENDPOINT_QOS_OVERRIDE_POLICY`](#rmw-connext-endpoint-qos-override-policy),
    apply the ROS 2 QoS profile on top of the default QoS policy, and overwrite
    them unless the corresponding ROS 2 policies are set to `SYSTEM_DEFAULT`.
  - Based on [`RMW_CONNEXT_DISABLE_LARGE_DATA_OPTIMIZATIONS`](#rmw-connext-disable-large-data-optimizations), and if the DataWriter's data type
    qualifies as "large data", tune the RTPS reliability protocol settings to
    provide better "out of the box" behavior for these types by updating
    [`DataWriterQos::protocol.rtps_reliable_writer`](https://community.rti.com/static/documentation/connext-dds/6.0.1/doc/api/connext_dds/api_c/structDDS__DataWriterProtocolQosPolicy.html#a073f347f5d06730a44c411e5ce25fe6a):
    - Set [`min_send_window_size`](https://community.rti.com/static/documentation/connext-dds/6.0.1/doc/api/connext_dds/api_c/structDDS__RtpsReliableWriterProtocol__t.html#a20be408ce0f25d4631d637e982213d02) to `10`.
    - Set [`max_send_window_size`](https://community.rti.com/static/documentation/connext-dds/6.0.1/doc/api/connext_dds/api_c/structDDS__RtpsReliableWriterProtocol__t.html#a20f5ce40d2c1f5f9c1ab6673d911b0f4) to `100`.
    - Set [`heartbeats_per_max_samples`](https://community.rti.com/static/documentation/connext-dds/6.0.1/doc/api/connext_dds/api_c/structDDS__RtpsReliableWriterProtocol__t.html#a9dd9360fe1f9c8a7f251576c8878a139) to `100`.
    - Set [`heartbeat_period`](https://community.rti.com/static/documentation/connext-dds/6.0.1/doc/api/connext_dds/api_c/structDDS__RtpsReliableWriterProtocol__t.html#a47229dfbfde987f8bbebe67e789e0908) to `200ms`.
    - Set [`late_joiner_heartbeat_period`](https://community.rti.com/static/documentation/connext-dds/6.0.1/doc/api/connext_dds/api_c/structDDS__RtpsReliableWriterProtocol__t.html#a26a476a46b1b4f03171c12c0e4ef784d) to `20ms`.
    - Set [`fast_heartbeat_period`](https://community.rti.com/static/documentation/connext-dds/6.0.1/doc/api/connext_dds/api_c/structDDS__RtpsReliableWriterProtocol__t.html#a0a0198e6baba89dfa5231950814f2a8f) to `20ms`.
    - Set [`max_nack_response_delay`](https://community.rti.com/static/documentation/connext-dds/6.0.1/doc/api/connext_dds/api_c/structDDS__RtpsReliableWriterProtocol__t.html#a7a95278d5204acf040ec49a6348ada46) to `0`.
    - Set [`high_watermark`](https://community.rti.com/static/documentation/connext-dds/6.0.1/doc/api/connext_dds/api_c/structDDS__RtpsReliableWriterProtocol__t.html#a7dc8a8bb8168a5acc3041f8970e95665) to `10`.
    - Set [`low_watermark`](https://community.rti.com/static/documentation/connext-dds/6.0.1/doc/api/connext_dds/api_c/structDDS__RtpsReliableWriterProtocol__t.html#ab603712e435cc4369366afa770700b80) to `0`.
    - Set [`max_heartbeat_retries`](https://community.rti.com/static/documentation/connext-dds/6.0.1/doc/api/connext_dds/api_c/structDDS__RtpsReliableWriterProtocol__t.html#a5be67d978618b5995cb517b51231d39d) to `500` (10s @ 50Hz).
  - Based on [`RMW_CONNEXT_USE_DEFAULT_PUBLISH_MODE`](#rmw-connext-use-default-publish-mode),
    overwrite `DataWriterQos::publish_mode::kind` to `ASYNCHRONOUS_PUBLISH_MODE`.
    - The "asynchronous publish mode" is required by RTI Connext DDS to publish data samples
      which require fragmentation because they exceed the underlying transport's MTU.
    - In "asynchronous publish mode", samples are added to a queue and disseminated to
      DataReaders from a separate thread.
  - If the data type is "unbounded" (i.e. it contains at least one variable-length field
    without a fixed maximum length), then Connext must be configured to always
    dynamically allocate samples for the DataWriter.
    - This is a partial limitation of the custom "type plugin" used by `rmw_connextdds`,
     and it is overcome by setting property `"dds.data_writer.history.memory_manager.fast_pool.pool_buffer_max_size"` to `0`.
- In the case of `rmw_connextddsmicro`:
  - Apply the ROS 2 QoS profile on top of the default QoS values, and overwrite
    them unless the corresponding ROS 2 policies are set to `SYSTEM_DEFAULT`.
  - Based on [`DataWriterQos::history::depth`](https://community.rti.com/static/documentation/connext-micro/3.0.3/doc/api_c/html/structDDS__HistoryQosPolicy.html#aef0fb3fd3579866be17d1a936f5e3729), update [`DataWriterQos::resource_limits`](https://community.rti.com/static/documentation/connext-micro/3.0.3/doc/api_c/html/structDDS__DataWriterQos.html#acba41cd24991ce678ad133bcc276b55c) to be compatible:
    - Update [`max_samples`](https://community.rti.com/static/documentation/connext-micro/3.0.3/doc/api_c/html/structDDS__ResourceLimitsQosPolicy.html#a90db906a3958146c5e22557db648e7ff) to make sure that `max_samples >= MAX(DataWriterQos::history::depth, 10)`.
    - Set [`max_samples_per_instance`](https://community.rti.com/static/documentation/connext-micro/3.0.3/doc/api_c/html/structDDS__ResourceLimitsQosPolicy.html#ae3d9339bebf4c7163cf6dff82882cbef) to `max_samples`.
    - Set [`max_instances`](https://community.rti.com/static/documentation/connext-micro/3.0.3/doc/api_c/html/structDDS__ResourceLimitsQosPolicy.html#ab7339605f314f50c7cb658426ff64cdb) to `1`.
  - Update [`DataWriterQos::writer_resource_limits`](https://community.rti.com/static/documentation/connext-micro/3.0.3/doc/api_c/html/structDDS__DataWriterQos.html#a14ef85955ab7c01c1b4eff56e2479149) to
    increase the default settings, and allow `rmw_connextddsmicro` to support an
    increased number of remote matched applications and endpoints:
    - Set [`max_remote_readers`](https://community.rti.com/static/documentation/connext-micro/3.0.3/doc/api_c/html/group__DDSUserManuals__ResourceModule__dwqos__max__remote__readers.html) to `64`.
    - Set [`max_routes_per_reader`](https://community.rti.com/static/documentation/connext-micro/3.0.3/doc/api_c/html/group__DDSUserManuals__ResourceModule__dwqos__max__routes__per__reader.html) to `1`.
  - Update [`DataWriterQos::protocol::rtps_reliable_writer`](https://community.rti.com/static/documentation/connext-micro/3.0.3/doc/api_c/html/structDDS__DataWriterProtocolQosPolicy.html#a073f347f5d06730a44c411e5ce25fe6a) to speed up the
    the RTPS reliability protocol from the default settings:
    - Set [`heartbeats_per_max_samples`](https://community.rti.com/static/documentation/connext-micro/3.0.3/doc/api_c/html/structDDS__RtpsReliableWriterProtocol__t.html#a9dd9360fe1f9c8a7f251576c8878a139) to `DataWriterQos::resource_limits::max_samples`.
- Create the DataWriter using [`DDS_Publisher_create_datawriter()`](https://community.rti.com/static/documentation/connext-dds/6.0.1/doc/api/connext_dds/api_c/group__DDSPublisherModule.html#ga772b272e1851120ced5e738549cb44a3).

#### DataWriter Deletion

Once the ROS Publisher, Client, or Service associated with a DataWriter are about to be
deleted, `rmw_connextdds` and `rmw_connextddsmicro` will delete the DataWriter
using [`DDS_Publisher_delete_datawriter()`](https://community.rti.com/static/documentation/connext-dds/6.0.1/doc/api/connext_dds/api_c/group__DDSPublisherModule.html#gaa2d341cdc5442fc4aa1fc27f1edd5707).

See [Topic Deletion](#topic-deletion) for information on how the Topic associated
with the DataWriter will be disposed.

### DataReader

A [DataReader](https://community.rti.com/static/documentation/connext-dds/6.0.1/doc/api/connext_dds/api_c/group__DDSReaderModule.html#ga49ce0cab2c1b60ddee4784a1432577a4)
is the entity used by DDS applications to express their interest in receiving data
published on a given [Topic](#topic). DataReaders allow applications to access
received samples, which they store in a configurable "history cache".

DataReaders are uniquely associated with a single parent [Subscriber](#subscriber), and a single
Topic.

This means that each DataReader will only be able to deliver samples of a single
data types (the one used by their Topic) to the application.

Similary to DataWriters, the size of a DataReader's cache, and the policy controlling how
long samples are stored in it, can be selected using various DDS QoS policies,
such as [HistoryQosPolicy](https://community.rti.com/static/documentation/connext-dds/6.0.1/doc/api/connext_dds/api_c/group__DDSHistoryQosModule.html), [DurabilityQosPolicy](https://community.rti.com/static/documentation/connext-dds/6.0.1/doc/api/connext_dds/api_c/group__DDSDurabilityQosModule.html),
and [ResourceLimitsQosPolicy](https://community.rti.com/static/documentation/connext-dds/6.0.1/doc/api/connext_dds/api_c/group__DDSResourceLimitsQosModule.html).

Depending on QoS configuration (specifically when DurabilityQosPolicy is set
to a `kind` equal or greater to `TRANSIENT_LOCAL`), a DataReader may receive
samples that were published by a DataWriter before matching but that are still
available in the DataWriter's cache.

`rmw_connextdds` and `rmw_connextddsmicro` will create a DataReader to support
several ROS entities:

- A DataReader will be created for each ROS Subscription.
- A DataReader will be created on the "reply topic" of each ROS Client.
- A DataReader will be created on the "request topic" of each ROS Service.

The DataReaers (and their associated Topics) will be automatically finalized
when the associated ROS entity is deleted.

All DataReaders will be created by `rmw_connextdds` and `rmw_connextddsmicro`
in a single parent Subscriber.

#### DataReader Creation

When a ROS Publisher, Client, or Service is created, `rmw_connextdds` and `rmw_connextddsmicro`
will perform the following operations:

- Register a "type plugin" for the DataReader's data type and create its associated
  Topic if necessary. See [Topic creation](#topic-creation).
- Determine the default QoS for the DataReader using 
  `DDS_Subscriber_get_default_datawriter_qos_w_topic_name()`.
  - This operation is only available in `rmw_connextdds`, and it allows for "topic filters"
    to be taken into consideration when determining the default QoS.
  - Since XML-based configuration is not available with RTI Connext DDS Micro,
    `rmw_connextddsmicro` will fall back to [`DDS_Subscriber_get_default_datawriter_qos()`]().
- In the case of `rmw_connextdds`:
  - Based on [`RMW_CONNEXT_ENDPOINT_QOS_OVERRIDE_POLICY`](#rmw-connext-endpoint-qos-override-policy),
    apply the ROS 2 QoS profile on top of the default QoS policy, and overwrite
    them unless the corresponding ROS 2 policies are set to `SYSTEM_DEFAULT`.
  - Based on [`RMW_CONNEXT_DISABLE_LARGE_DATA_OPTIMIZATIONS`](#rmw-connext-disable-large-data-optimizations), and if the DataReader's data type
    qualifies as "large data", tune the RTPS reliability protocol settings to
    provide better "out of the box" behavior for these types.
    - Update [`DataReaderQos::protocol.rtps_reliable_reader`](https://community.rti.com/static/documentation/connext-dds/6.0.1/doc/api/connext_dds/api_c/structDDS__DataReaderProtocolQosPolicy.html#ac962c1b10fbf15c0a68d975f9e95d995):
      - Set [`min_heartbeat_response_delay`](https://community.rti.com/static/documentation/connext-dds/6.0.1/doc/api/connext_dds/api_c/structDDS__RtpsReliableReaderProtocol__t.html#ab4d96a016eefca4dcd8ce001e03f7a90) to `0`.
      - Set [`max_heartbeat_response_delay`](https://community.rti.com/static/documentation/connext-dds/6.0.1/doc/api/connext_dds/api_c/structDDS__RtpsReliableReaderProtocol__t.html#aafc2e6ec3a3c039e88355c683a7a622e) to `0`.
    - Set [`DataReaderQos::resource_limits::dynamically_allocate_fragmented_samples`](https://community.rti.com/static/documentation/connext-dds/6.0.1/doc/api/connext_dds/api_c/structDDS__DataReaderResourceLimitsQosPolicy.html#abd03f44d9c89ebe7dd0cd8026c671129) to
     `true`, to force Connext to avoid preallocating large amounts of memory to store
     fragmented samples, but instead allocate memory (and cache for reuse) as needed.
  - If the data type is "unbounded" (i.e. it contains at least one variable-length field
    without a fixed maximum length), then Connext must be configured to always
    dynamically allocate samples for the DataReader.
    - This is a partial limitation of the custom "type plugin" used by `rmw_connextdds`,
     and it is overcome by setting property `"dds.data_reader.history.memory_manager.fast_pool.pool_buffer_max_size"` to `0`.
- In the case of `rmw_connextddsmicro`:
  - Apply the ROS 2 QoS profile on top of the default QoS values, and overwrite
    them unless the corresponding ROS 2 policies are set to `SYSTEM_DEFAULT`.
  - Based on [`DataReaderQos::history::depth`](https://community.rti.com/static/documentation/connext-micro/3.0.3/doc/api_c/html/structDDS__HistoryQosPolicy.html#aef0fb3fd3579866be17d1a936f5e3729), update [`DataReaderQos::resource_limits`](https://community.rti.com/static/documentation/connext-micro/3.0.3/doc/api_c/html/structDDS__DataReaderQos.html#a27f550c1f22be54dfe3b69e007fabad2) to be compatible:
    - Update [`max_samples`](https://community.rti.com/static/documentation/connext-micro/3.0.3/doc/api_c/html/structDDS__ResourceLimitsQosPolicy.html#a90db906a3958146c5e22557db648e7ff) to make sure that `max_samples >= MAX(DataReaderQos::history::depth, 10)`.
    - Set [`max_samples_per_instance`](https://community.rti.com/static/documentation/connext-micro/3.0.3/doc/api_c/html/structDDS__ResourceLimitsQosPolicy.html#ae3d9339bebf4c7163cf6dff82882cbef) to `max_samples`.
    - Set [`max_instances`](https://community.rti.com/static/documentation/connext-micro/3.0.3/doc/api_c/html/structDDS__ResourceLimitsQosPolicy.html#ab7339605f314f50c7cb658426ff64cdb) to `1`.
  - Update [`DataReaderQos::reader_resource_limits`](https://community.rti.com/static/documentation/connext-micro/3.0.3/doc/api_c/html/structDDS__DataReaderQos.html#ac6f4f6e01f59c4d53490717db313bd43) to
    increase the default settings, and allow `rmw_connextddsmicro` to support an
    increased number of remote matched applications and endpoints:
    - Set [`max_remote_writers`](https://community.rti.com/static/documentation/connext-micro/3.0.3/doc/api_c/html/group__DDSUserManuals__ResourceModule__drqos__max__remote__writers.html) to `64`.
    - Set [`max_remote_writers_per_instance`](https://community.rti.com/static/documentation/connext-micro/3.0.3/doc/api_c/html/group__DDSUserManuals__ResourceModule__drqos__max__remote__writers__per__instance.html) to `64`.
    - Set [`max_outstanding_reads`](https://community.rti.com/static/documentation/connext-micro/3.0.3/doc/api_c/html/group__DDSUserManuals__ResourceModule__drqos__max__outstanding__reads.html) to `1`.
    - Set [`max_routes_per_writer`](https://community.rti.com/static/documentation/connext-micro/3.0.3/doc/api_c/html/group__DDSUserManuals__ResourceModule__drqos__max__routes__per__writer.html) to `1`.
  - Update [`DataReaderQos::protocol::rtps_reliable_reader`](https://community.rti.com/static/documentation/connext-micro/3.0.3/doc/api_c/html/structDDS__DataReaderProtocolQosPolicy.html#ac962c1b10fbf15c0a68d975f9e95d995) to speed up the
    the RTPS reliability protocol from the default settings:
    - Set [`nack_period`](https://community.rti.com/static/documentation/connext-micro/3.0.3/doc/api_c/html/structDDS__RtpsReliableReaderProtocol__t.html#a9c299ef4fe921b91a7bdedadcd38c2a8) to `10ms`.
- Create the DataReader using [`DDS_Subscriber_create_datareader()`](https://community.rti.com/static/documentation/connext-dds/6.0.1/doc/api/connext_dds/api_c/group__DDSSubscriberModule.html#gac02a61ced642fe9d9f7466f7ba67010e).

#### DataReader Deletion

Once the ROS Subscription, Client, or Service associated with a DataReader are about to be
deleted, `rmw_connextdds` and `rmw_connextddsmicro` will delete the DataReader
using [`DDS_Subscriber_delete_datareader()`](https://community.rti.com/static/documentation/connext-dds/6.0.1/doc/api/connext_dds/api_c/group__DDSSubscriberModule.html#gaf47ced9dededcc0b95b9cdb5365b2384).

See [Topic Deletion](#topic-deletion) for information on how the Topic associated
with the DataReader will be disposed.
