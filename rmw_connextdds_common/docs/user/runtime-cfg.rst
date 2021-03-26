.. _user-runtime-cfg:

Runtime Configuration
=====================

In addition to standard configuration facilities provided by the ROS2 RMW
interface, ``rmw_connextdds``, and ``rmw_connextddsmicro`` support the
additional configuration of some aspects of their runtime behavior via custom
environment variables.

.. _user-runtime-cfg-cyclone:

RMW_CONNEXT_CYCLONE_COMPATIBILITY_MODE
--------------------------------------

Enable different policies to improve interoperability with
``rmw_cyclonedds_cpp``.

By default, ROS2 applications using ``rmw_connextdds`` will be able to
communicate with those using ``rmw_cyclonedds_cpp`` only via ROS2 publishers and
subscribers, while ROS2 clients and services will not interoperate across
vendors.

The reason for this incompatibility lies in ``rmw_cyclonedds_cpp``'s use of a
custom mapping for propagating request metadata between clients and services.

When this "compatibility mode" is enabled, ``rmw_connextdds`` (and
``rmw_connextddsmicro``) will use this non-standard profile in order to
interoperate with ``rmw_cyclonedds_cpp``, instead of using one the two standard
profiles defined by the DDS-RPC specification
(see :ref:`user-runtime-cfg-reqreply`).

.. _user-runtime-cfg-largedata:

RMW_CONNEXT_DISABLE_LARGE_DATA_OPTIMIZATIONS
--------------------------------------------

By default, ``rmw_connextdds`` will try to detect the use of "large data" types,
and automatically optimize the QoS of DDS DataWriters and DataReaders
using these types, to improve out of the box performance on reliable streams.

These optimizations will be applied to any endpoint whose type has a serialized
size of at least 1MB (configured by a compile-time limit).

``rmw_connextdds`` will modify a "large data" endpoint's RTPS reliability
protocol parameters to more quickly recover samples, which typically improves
performance in the presence of very fragmented data, but it might also
end up increasing network traffic unnecessarily, particularly if data is not
exchanged at a fast periodic pace.

Variable ``RMW_CONNEXT_DISABLE_LARGE_DATA_OPTIMIZATIONS`` may be used to disable
these automatic optimizations, and revert to Connext's default behavior.

.. _user-runtime-cfg-endpoint-discovery:

RMW_CONNEXT_DISABLE_FAST_ENDPOINT_DISCOVERY
-------------------------------------------

By default, ``rmw_connextdds`` modifies the QoS of its DomainParticipant to
enable the optimizations defined by RTI Connext DDS' built-in QoS snippet
``Optimization.Discovery.Endpoint.Fast``.

These optimizations speed up the discovery process between different applications
but they also introduce an overhead in network traffic, which might be undesirable
for larger systems.

Variable ``RMW_CONNEXT_DISABLE_FAST_ENDPOINT_DISCOVERY`` may be used to disable
these automatic optimizations, and to leave the DomainParticipant's QoS to
its defaults.

.. _user-runtime-cfg-endpoint-qos:

RMW_CONNEXT_ENDPOINT_QOS_OVERRIDE_POLICY
----------------------------------------

When this variable is not set or set to ``always``, the QoS settings specified
in the default profile will be used and the ros QoS profile will be applied on
top of it. You can use topic filters in XML profile files to have different
defaults for different topics, but you have to use the mangled topic names
(see [ROS topic mangling conventions](#ros-topic-mangling-conventions)).

In case this variable is set to ``never``, the QoS settings will be loaded from
the default profile as before but the ros QoS profile will be ignored.
Be aware of configuring the QoS of rcl topics (``rt/rosout``,
``rt/parameter_events``, etc.) and the rmw internal topic ``ros_discovery_info``
correctly.

This variable can also be set to ``dds_topics: <regex>``, e.g.:
``dds_topics: rt/my_topic|rt/my_ns/another_topic``. In this case, QoS settings
for topics matching the provided regex will be loaded in the same way as the
``never`` policy, and the ones that don't match will be loaded in the same way
as the ``always`` policy.

ROS topic mangling conventions
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

ROS mangles topic names in the following way:

- Topics are prefixed with ``rt``. e.g.: ``/my/fully/qualified/ros/topic`` is
  converted to ``rt/my/fully/qualified/ros/topic``.
- The service request topics are prefixed with ``rq`` and suffixed with
  ``Request``. e.g.: ``/my/fully/qualified/ros/service`` request topic is
  ``rq/my/fully/qualified/ros/serviceRequest``.
- The service response topics are prefixed with ``rr`` and suffixed with
  ``Response``. e.g.: ``/my/fully/qualified/ros/service`` response topic is
  ``rr/my/fully/qualified/ros/serviceResponse``.

.. _user-runtime-cfg-initialpeers:

RMW_CONNEXT_INITIAL_PEERS
-------------------------

Variable ``RMW_CONNEXT_INITIAL_PEERS`` can be used to specify a list of
comma-separated values of "address locators" that the DomainParticipant created
by the RMW will use to try to make contact with remote peer applications
during the DDS discovery phase.

The values will be parsed, trimmed, and stored in QoS field
``DDS_DomainParticipantQos::discovery::initial_peers``, overwriting any
value it previously contained.

While both ``rmw_connextdds`` and ``rmw_connextddsmicro`` will honor this
variable, `equivalent, and more advanced, functionality is already available in RTI Connext DDS <https://community.rti.com/static/documentation/connext-dds/6.0.1/doc/manuals/connext_dds/html_files/RTI_ConnextDDS_CoreLibraries_UsersManual/Content/UsersManual/ConfigPeersListUsed_inDiscov.htm>`_,
for example using variable ``NDDS_DISCOVERY_PEERS``.

For this reason, only users of ``rmw_connextddsmicro`` should consider
specifying ``RMW_CONNEXT_INITIAL_PEERS``.

For example, ``rmw_connextddsmicro`` will use ``lo`` as its default UDP network
interface (see :ref:`user-runtime-cfg-udp`), which will prevent it from
accessing the default discovery peer (multicast address ``239.255.0.1``).
The default peer configuration will also prevent the DomainParticipant from
carrying out discovery over the built-in shared-memory transport.
To enable discovery over this transport, in addition to
the default multicast peer:

.. code-block:: sh

    RMW_IMPLEMENTATION=rmw_connextddsmicro \
    RMW_CONNEXT_INITIAL_PEERS="_shmem://, 239.255.0.1" \
      ros2 run demo_nodes_cpp listener

.. _user-runtime-cfg-legacy:

RMW_CONNEXT_LEGACY_RMW_COMPATIBILITY_MODE
-----------------------------------------

ROS2 applications using ``rmw_connextdds`` will not be able to interoperate with
applications using the previous RMW implementation for RTI Connext DDS,
``rmw_connext_cpp``, unless variable
``RMW_CONNEXT_LEGACY_RMW_COMPATIBILITY_MODE`` is used to enable a
"compatibility" mode with these older implementation.

In particular, when this mode is enabled, ``rmw_connextdds`` will revert to
adding a suffix (``_``) to the end of the names of the attributes of the ROS2
data types propagated via DDS discovery.

.. _user-runtime-cfg-reqreply:

RMW_CONNEXT_REQUEST_REPLY_MAPPING
---------------------------------

The `DDS-RPC specification <https://www.omg.org/spec/DDS-RPC/About-DDS-RPC/>`_
defines two profiles for mapping "request/reply" interactions over DDS messages
(e.g. ROS2 clients and services):

- the *basic* profile conveys information about the originator of a request as
  an inline payload, serialized before the actual request/reply payloads.

- The *extended* profile relies on DDS' metadata to convey request/reply
  information out of band.

By default, ``rmw_connextdds`` uses the *extended* profile when sending requests
from a ROS2 client to a service, while ``rmw_connextddsmicro`` uses the *basic*
one.

Variable ``RMW_CONNEXT_REQUEST_REPLY_MAPPING`` can be used to select the actual
profile used at runtime. Either ``"basic"`` or ``"extended"`` may be specified.

At the moment, the *extended* profile is only available with ``rmw_connextdds``.
In this configuration, ``rmw_connextdds`` will interoperate with
``rmw_fastrtps_cpp``, e.g.:

.. code-block:: sh

    RMW_IMPLEMENTATION=rmw_connextdds \
      ros2 run demo_nodes_cpp add_two_ints_server

    RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
      ros2 run demo_nodes_cpp add_two_ints_client

When using the *basic* profile, ``rmw_connextdds`` will interoperate with
``rmw_connextddsmicro``, e.g.:

.. code-block:: sh

    RMW_IMPLEMENTATION=rmw_connextdds \
    RMW_CONNEXT_REQUEST_REPLY_MAPPING=basic \
      ros2 run demo_nodes_cpp add_two_ints_server

    RMW_IMPLEMENTATION=rmw_connextddsmicro \
    RMW_CONNEXT_INITIAL_PEER=localhost \
      ros2 run demo_nodes_cpp add_two_ints_client

Use variable :ref:`user-runtime-cfg-cyclone` to enable interoperability with
``rmw_cyclonedds_cpp`` using a non-standard version of the *basic* profile,
e.g.:

.. code-block:: sh

    RMW_IMPLEMENTATION=rmw_connextdds \
    RMW_CONNEXT_CYCLONE_COMPATIBILITY_MODE=y \
      ros2 run demo_nodes_cpp add_two_ints_server

    RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
      ros2 run demo_nodes_cpp add_two_ints_client

.. _user-runtime-cfg-udp:

RMW_CONNEXT_UDP_INTERFACE
-------------------------

RTI Connext DDS Micro requires applications to explicitly configure the network
interface to use for UDPv4 communication.

``rmw_connextddsmicro`` makes the arbitrary decision of using `lo` as the
default interface.

This is undesireable if non-local communication is required, and/or if the
default DDS multicast peer (``239.255.0.1``) is to be used.

Variable ``RMW_CONNEXT_UDP_INTERFACE`` may be used to customize the network
interface actually used by RTI Connext DDS Micro's UDPv4 transport, e.g. to use
``eth0``:

.. code-block:: sh

    RMW_IMPLEMENTATION=rmw_connextddsmicro \
    RMW_CONNEXT_UDP_INTERFACE=eth0 \
      ros2 run demo_nodes_cpp listener

This variable is not used by ``rmw_connextdds``.

.. _user-runtime-cfg-pubmode:

RMW_CONNEXT_USE_DEFAULT_PUBLISH_MODE
------------------------------------

``rmw_connextdds`` will always set ``DDS_DataWriterQos::publish_mode::kind`` of
any DataWriter it creates to ``DDS_ASYNCHRONOUS_PUBLISH_MODE_QOS``, in order to
enable out of the box support for "large data".

This behavior might not be always desirable, and it can be disabled by setting
``RMW_CONNEXT_USE_DEFAULT_PUBLISH_MODE`` to a non-empty value.

This variable is not used by ``rmw_connextddsmicro``, since it doesn't
automatically override ``DDS_DataWriterQos::publish_mode::kind``.
